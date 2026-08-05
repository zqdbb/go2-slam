"""
ICROS 2026 Sim2Sim — MuJoCo + ONNX Runtime (DDS 없이 standalone 실행)

45/225/318/498-dim 정책 자동 지원.

사용법:
    # scan policy (318-dim) + ICROS2025 맵
    python3 scripts/sim2sim/sim2sim_icros2026.py \
        --onnx /path/to/policy.onnx \
        --scene /home/nuri/unitree_mujoco/unitree_robots/go2/scene_icros2025.xml

    # 기본 평지 (scan policy)
    python3 scripts/sim2sim/sim2sim_icros2026.py \
        --onnx /path/to/policy.onnx

    # 체크포인트에서 직접 (play.py로 ONNX export 먼저 필요)
    python3 scripts/sim2sim/sim2sim_icros2026.py \
        --checkpoint logs/rsl_rl/unitree_go2_icros2026_scan/.../model_XXXXX.pt

키보드 조작 (MuJoCo 뷰어):
    W/S : 전진/후진 속도 ±0.2 m/s
    A/D : 좌/우 각속도 ±0.3 rad/s
    Q/E : 좌/우 이동 ±0.2 m/s
    R   : 속도 명령 초기화
    ESC : 종료
    Space: 로봇 리셋
"""

import argparse
import os
import sys
import time
from collections import deque
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

# ─────────────────────────────────────────────────────────────────────────────
# 경로 설정
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
REPO_ROOT  = SCRIPT_DIR.parent.parent
GO2_DIR    = Path("/home/nuri/unitree_mujoco/unitree_robots/go2")
SCENE_FILE = str(GO2_DIR / "scene_icros2026.xml")   # 기본 씬

# ── 맵 단축 이름 → XML 경로 매핑 ──────────────────────────────────────────────
MAP_ALIASES = {
    "icros2025":      str(GO2_DIR / "scene_icros2025.xml"),
    "icros2026":      str(GO2_DIR / "scene_icros2026.xml"),
    "icra2023_easy":  str(GO2_DIR / "scene_icra2023_easy.xml"),
    "icra2023_hard":  str(GO2_DIR / "scene_icra2023_hard.xml"),
    "icra2024_flat":  str(GO2_DIR / "scene_icra2024_flat.xml"),
    "icra2024_sloped":str(GO2_DIR / "scene_icra2024_sloped.xml"),
    "flat":           str(GO2_DIR / "scene.xml"),
}

# ─────────────────────────────────────────────────────────────────────────────
# Joint 매핑
#   IsaacLab 순서: [FL_hip, FR_hip, RL_hip, RR_hip,
#                   FL_thigh, FR_thigh, RL_thigh, RR_thigh,
#                   FL_calf, FR_calf, RL_calf, RR_calf]
#   MuJoCo 순서:   [FR_hip, FR_thigh, FR_calf,
#                   FL_hip, FL_thigh, FL_calf,
#                   RR_hip, RR_thigh, RR_calf,
#                   RL_hip, RL_thigh, RL_calf]
# ─────────────────────────────────────────────────────────────────────────────
IL2MJ = np.array([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])
MJ2IL = np.array([1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10])

DEFAULT_JOINT_POS_IL = np.array([
     0.1, -0.1,  0.1, -0.1,
     0.8,  0.8,  1.0,  1.0,
    -1.5, -1.5, -1.5, -1.5,
], dtype=np.float64)

DEFAULT_JOINT_POS_MJ = DEFAULT_JOINT_POS_IL[MJ2IL]

MJ_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

# ─────────────────────────────────────────────────────────────────────────────
# Height Scan 설정 (icros2026_scan policy — 318-dim)
#   grid: 21×13 = 273 (size=[2.0,1.2]m, resolution=0.1m, 양끝 포함)
#   x: -1.0 ~ +1.0m (21 points)  — 전후
#   y: -0.6 ~ +0.6m (13 points)  — 좌우
# ─────────────────────────────────────────────────────────────────────────────
SCAN_SIZE_X   = 2.0
SCAN_SIZE_Y   = 1.2
SCAN_RES      = 0.1
SCAN_NX       = round(SCAN_SIZE_X / SCAN_RES) + 1   # 21
SCAN_NY       = round(SCAN_SIZE_Y / SCAN_RES) + 1   # 13
SCAN_DIM      = SCAN_NX * SCAN_NY                  # 273
SCAN_CLIP     = (-1.0, 1.0)
SCAN_RAY_DOWN = np.array([0.0, 0.0, -1.0])         # 아래 방향 레이
SCAN_RAY_DIST = 10.0                               # 최대 레이 거리
HEIGHT_SCAN_OFFSET = 0.43                          # MuJoCo/Isaac base-height alignment

# 로봇 기준 상대 좌표 (x: 전후, y: 좌우) — row-major (x 먼저)
_xs = np.linspace(-SCAN_SIZE_X / 2, SCAN_SIZE_X / 2, SCAN_NX)
_ys = np.linspace(-SCAN_SIZE_Y / 2, SCAN_SIZE_Y / 2, SCAN_NY)
SCAN_OFFSETS = np.array([(x, y) for x in _xs for y in _ys], dtype=np.float64)  # (273, 2)

# ─────────────────────────────────────────────────────────────────────────────
# 시뮬레이션 파라미터
# ─────────────────────────────────────────────────────────────────────────────
KP = 25.0
KD = 0.6              # 0.5 → 0.6: damping 증가로 MuJoCo 진동/발 펄쩍 억제
SCALE_ANG_VEL   = 0.2
SCALE_JOINT_VEL = 0.05
ACTION_SCALE    = 0.25
SIM_DT          = 0.005    # 200Hz
POLICY_DT       = 0.02     # 50Hz
POLICY_DECIMATION = int(POLICY_DT / SIM_DT)  # 4

# Action EMA (지수 이동 평균) — Sim2Sim 발 펄쩍 억제
# alpha=0.0: 이전 action 무시 (원본), alpha=1.0: 이전 action만 사용
# 0.3: 새 action 70% + 이전 action 30% → 부드러운 전환
ACTION_EMA_ALPHA = 0.3

# Proprioceptive history (V5/V3 498-dim policy)
# 5 step × 45-dim = 225-dim proprio history + 273-dim height_scan = 498-dim
HISTORY_LEN = 5


# ─────────────────────────────────────────────────────────────────────────────
# 정책 로드
# ─────────────────────────────────────────────────────────────────────────────
def load_policy(checkpoint: str | None, onnx: str | None, cpu: bool = False) -> ort.InferenceSession:
    providers = ["CPUExecutionProvider"] if cpu else ["CUDAExecutionProvider", "CPUExecutionProvider"]
    if onnx and os.path.exists(onnx):
        print(f"[INFO] ONNX 로드: {onnx}")
        return ort.InferenceSession(onnx, providers=providers)
    if checkpoint and os.path.exists(checkpoint):
        onnx_out = Path(checkpoint).parent / "exported" / "policy.onnx"
        if onnx_out.exists():
            print(f"[INFO] 기존 ONNX 발견: {onnx_out}")
            return ort.InferenceSession(str(onnx_out), providers=providers)
        print("[WARN] play.py로 먼저 ONNX export 필요:")
        print(f"  /isaac-sim/python.sh scripts/experiments/icros2026_scan/play.py \\")
        print(f"    --task Unitree-Go2-ICROS2026-Scan --num_envs 8")
        sys.exit(1)
    print("[ERROR] --checkpoint 또는 --onnx 경로를 지정하세요.")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# 유틸 함수
# ─────────────────────────────────────────────────────────────────────────────
def quat_to_rot_mat(quat_wxyz: np.ndarray) -> np.ndarray:
    rot = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rot, quat_wxyz)
    return rot.reshape(3, 3)


def projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    """중력 [0,0,-1] → 바디 프레임 투영 (IsaacLab 방식)."""
    R = quat_to_rot_mat(quat_wxyz)
    return R.T @ np.array([0.0, 0.0, -1.0])


# ─────────────────────────────────────────────────────────────────────────────
# Height Scan (MuJoCo raycasting → 273-dim)
# ─────────────────────────────────────────────────────────────────────────────
def compute_height_scan(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """
    로봇 기준 2.0×1.2m 격자에서 바닥 높이를 레이캐스팅으로 계산.
    IsaacLab height_scan obs와 동일한 방식:
      height = sensor/base_z - hit_z - offset
    Unoise(-0.05, 0.05) 제외 (sim2sim은 노이즈 없음)
    반환: (273,) float32, clip(-1, 1)
    """
    # 로봇 위치 & 자세
    robot_pos  = data.qpos[:3].copy()    # [x, y, z]
    robot_quat = data.qpos[3:7].copy()   # [w, x, y, z]
    R = quat_to_rot_mat(robot_quat)      # 3×3 회전 행렬

    heights = np.zeros(SCAN_DIM, dtype=np.float32)
    ray_start = robot_pos.copy()
    ray_start[2] += 0.5    # 로봇 발 위에서 시작 (바닥 아래로 쏘지 않게)

    for i, (dx, dy) in enumerate(SCAN_OFFSETS):
        # 로봇 전방 기준 오프셋 → 월드 프레임
        local_offset = np.array([dx, dy, 0.0])
        world_offset = R @ local_offset
        world_pos = robot_pos + world_offset

        # 위에서 아래로 레이캐스팅
        ray_origin = np.array([world_pos[0], world_pos[1], world_pos[2] + 1.0])
        geom_id = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(model, data,
                             ray_origin,
                             SCAN_RAY_DOWN,
                             None,        # 모든 geom 포함
                             1,           # flg_static (정적 geom 포함)
                             -1,          # bodyexclude (-1 = 없음)
                             geom_id)

        if dist >= 0 and dist < SCAN_RAY_DIST:
            ground_z = ray_origin[2] - dist
            rel_height = robot_pos[2] - ground_z - HEIGHT_SCAN_OFFSET
        else:
            rel_height = -1.0    # 바닥 없음 → 최저값

        heights[i] = np.clip(rel_height, SCAN_CLIP[0], SCAN_CLIP[1])

    return heights


# ─────────────────────────────────────────────────────────────────────────────
# 관측 계산
# ─────────────────────────────────────────────────────────────────────────────
def get_joint_states(data: mujoco.MjData):
    """MJ 순서 관절 데이터 반환."""
    pos = np.array([data.sensor(f"{n}_pos").data[0] for n in MJ_JOINT_NAMES])
    vel = np.array([data.sensor(f"{n}_vel").data[0] for n in MJ_JOINT_NAMES])
    return pos, vel


def compute_obs_45(data: mujoco.MjData,
                   last_action: np.ndarray,
                   cmd: np.ndarray) -> np.ndarray:
    """45-dim 관측 (icros2026 blind policy)."""
    imu_quat = data.sensor("imu_quat").data.copy()
    imu_gyro = data.sensor("imu_gyro").data.copy()
    pos_mj, vel_mj = get_joint_states(data)
    pos_il = pos_mj[IL2MJ]
    vel_il = vel_mj[IL2MJ]

    return np.concatenate([
        imu_gyro   * SCALE_ANG_VEL,
        projected_gravity(imu_quat),
        cmd.astype(np.float64),
        pos_il - DEFAULT_JOINT_POS_IL,
        vel_il * SCALE_JOINT_VEL,
        last_action,
    ]).astype(np.float32)


def compute_obs_318(model: mujoco.MjModel,
                    data: mujoco.MjData,
                    last_action: np.ndarray,
                    cmd: np.ndarray) -> np.ndarray:
    """318-dim 관측 (icros2026_scan perceptive policy).
    = 45-dim proprioceptive + 273-dim height_scan
    """
    obs_45 = compute_obs_45(data, last_action, cmd)
    scan   = compute_height_scan(model, data)    # (273,)
    return np.concatenate([obs_45, scan]).astype(np.float32)


def compute_obs_225(data: mujoco.MjData,
                    last_action: np.ndarray,
                    cmd: np.ndarray,
                    proprio_history: deque) -> np.ndarray:
    """225-dim 관측 (icros2026_v4 policy).

    layout: proprio_history(5×45=225) — height_scan 없음 (actor는 blind)
      - proprio_history: Oldest→Newest 순서 (IsaacLab _ProprioHistoryTerm과 동일)
      - 각 45-dim step: [ang_vel(3), gravity(3), cmd(3), jpos(12), jvel(12), action(12)]
    """
    obs_45 = compute_obs_45(data, last_action, cmd)
    proprio_history.append(obs_45.copy())
    hist_flat = np.concatenate(list(proprio_history))   # (225,) oldest→newest
    return hist_flat.astype(np.float32)                  # (225,)


def compute_obs_498(model: mujoco.MjModel,
                    data: mujoco.MjData,
                    last_action: np.ndarray,
                    cmd: np.ndarray,
                    proprio_history: deque) -> np.ndarray:
    """498-dim 관측 (V5/V3 history+scan policy).

    layout: proprio_history(5×45=225) + height_scan(273) = 498-dim
      - proprio_history: Oldest→Newest 순서 (IsaacLab _ProprioHistoryTerm과 동일)
      - 각 45-dim step: [ang_vel(3), gravity(3), cmd(3), jpos(12), jvel(12), action(12)]

    [Sim2Real 일관성]
    IsaacSim과 MuJoCo 모두 노이즈 없이 clean obs 사용
    (실제 배포 시에도 noise는 센서 자체 특성이므로 별도 처리)
    """
    obs_45 = compute_obs_45(data, last_action, cmd)
    proprio_history.append(obs_45.copy())
    hist_flat = np.concatenate(list(proprio_history))   # (225,) oldest→newest
    scan = compute_height_scan(model, data)              # (273,)
    return np.concatenate([hist_flat, scan]).astype(np.float32)   # (498,)


# ─────────────────────────────────────────────────────────────────────────────
# PD 제어
# ─────────────────────────────────────────────────────────────────────────────
def apply_pd_action(model: mujoco.MjModel, data: mujoco.MjData,
                    action_il: np.ndarray):
    pos_mj, vel_mj = get_joint_states(data)
    target_il = action_il * ACTION_SCALE + DEFAULT_JOINT_POS_IL
    target_mj = target_il[MJ2IL]
    torque_mj = KP * (target_mj - pos_mj) + KD * (0.0 - vel_mj)
    torque_mj = np.clip(torque_mj, -45.0, 45.0)
    data.ctrl[:] = torque_mj


# ─────────────────────────────────────────────────────────────────────────────
# 로봇 리셋
# ─────────────────────────────────────────────────────────────────────────────
def reset_robot(model: mujoco.MjModel, data: mujoco.MjData):
    mujoco.mj_resetData(model, data)
    # "start" 또는 "stand" 키프레임 시도
    key_id = -1
    for kf_name in ("start", "stand", "home"):
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, kf_name)
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key_id)
            break
    # 속도 명시적 초기화 (튕김 방지)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)
    # warm-up: keyframe qpos → ctrl 목표로 설정해 자세 안정화
    # ctrl에 이미 keyframe ctrl이 있으면 그걸 쓰고, 없으면 qpos에서 읽기
    kf_ctrl = data.ctrl.copy()
    if np.allclose(kf_ctrl, 0.0):
        # ctrl이 전부 0이면 qpos에서 관절 각도 읽기
        kf_ctrl = data.qpos[7:19].copy()
    data.ctrl[:] = kf_ctrl
    # 500 스텝 warm-up (1.0s) — spawn z 낮춰도 완전 안착 보장
    for _ in range(500):
        mujoco.mj_step(model, data)


# ─────────────────────────────────────────────────────────────────────────────
# 키보드
# ─────────────────────────────────────────────────────────────────────────────
class KeyboardState:
    def __init__(self, init_vx: float = 0.5):
        self.lin_vel_x  = init_vx
        self.lin_vel_y  = 0.0
        self.ang_vel_z  = 0.0
        self.reset_flag = False

    def key_callback(self, key: int):
        import glfw
        STEP_LIN = 0.2
        STEP_ANG = 0.3
        # WASD + 방향키 동시 지원
        if   key in (glfw.KEY_W, glfw.KEY_UP):
            self.lin_vel_x = min(self.lin_vel_x + STEP_LIN,  2.0)
        elif key in (glfw.KEY_S, glfw.KEY_DOWN):
            self.lin_vel_x = max(self.lin_vel_x - STEP_LIN, -1.0)
        elif key in (glfw.KEY_A, glfw.KEY_LEFT):
            self.ang_vel_z = min(self.ang_vel_z + STEP_ANG,  2.0)
        elif key in (glfw.KEY_D, glfw.KEY_RIGHT):
            self.ang_vel_z = max(self.ang_vel_z - STEP_ANG, -2.0)
        elif key == glfw.KEY_Q:     self.lin_vel_y = min(self.lin_vel_y + STEP_LIN,  0.5)
        elif key == glfw.KEY_E:     self.lin_vel_y = max(self.lin_vel_y - STEP_LIN, -0.5)
        elif key == glfw.KEY_R:
            self.lin_vel_x = 0.0; self.lin_vel_y = 0.0; self.ang_vel_z = 0.0
        elif key == glfw.KEY_SPACE:
            self.reset_flag = True

    @property
    def command(self) -> np.ndarray:
        return np.array([self.lin_vel_x, self.lin_vel_y, self.ang_vel_z], dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="ICROS 2026 Sim2Sim (MuJoCo + ONNX)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--checkpoint", type=str, default=None,
                        help="model_XXXXX.pt 경로 (exported/policy.onnx 자동 탐색)")
    parser.add_argument("--onnx",       type=str, default=None,
                        help="policy.onnx 직접 경로")
    parser.add_argument("--scene",      type=str, default=None,
                        help=f"MuJoCo scene XML 전체 경로")
    parser.add_argument("--map",        type=str, default=None,
                        choices=list(MAP_ALIASES.keys()),
                        help=(
                            "맵 단축 이름:\n" +
                            "\n".join(f"  {k:18s} → {v}" for k, v in MAP_ALIASES.items())
                        ))
    parser.add_argument("--cmd_vel_x",  type=float, default=0.5,
                        help="초기 전진 속도 [m/s] (기본: 0.5)")
    parser.add_argument("--no_map",     action="store_true",
                        help="기본 평지 씬으로 테스트")
    parser.add_argument("--cpu",        action="store_true",
                        help="ONNX Runtime CPU provider만 사용 (학습 중 smoke test용)")
    parser.add_argument("--headless_steps", type=int, default=0,
                        help="viewer 없이 지정한 policy step 수만큼 smoke test 후 종료")
    args = parser.parse_args()

    # ── 정책 로드 & obs dim 자동 감지 ──────────────────────────────────────
    session     = load_policy(args.checkpoint, args.onnx, cpu=args.cpu)
    input_name  = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    obs_dim     = session.get_inputs()[0].shape[1]   # 45 or 318

    print(f"[INFO] 정책 obs_dim: {obs_dim}")
    print(f"[INFO] 정책 입력: {input_name} {session.get_inputs()[0].shape}")
    print(f"[INFO] 정책 출력: {output_name} {session.get_outputs()[0].shape}")

    if obs_dim == 498:
        print("[INFO] ▶ V5/V3 정책 (498-dim: proprio_history 225 + height_scan 273)")
        use_scan = True
        use_history = True
    elif obs_dim == 225:
        print("[INFO] ▶ icros2026_v4 정책 (225-dim: proprio_history 5×45, blind actor)")
        use_scan = False
        use_history = True
    elif obs_dim == 318:
        print("[INFO] ▶ icros2026_scan 정책 (318-dim: proprioceptive 45 + height_scan 273)")
        use_scan = True
        use_history = False
    elif obs_dim == 45:
        print("[INFO] ▶ icros2026 blind 정책 (45-dim: proprioceptive only)")
        use_scan = False
        use_history = False
    else:
        print(f"[WARN] 알 수 없는 obs_dim={obs_dim}, 45-dim으로 fallback")
        use_scan = False
        use_history = False

    # ── 씬 결정 ───────────────────────────────────────────────────────────
    if args.no_map:
        scene = str(GO2_DIR / "scene.xml")
    elif args.map:
        scene = MAP_ALIASES[args.map]
    elif args.scene:
        scene = args.scene
    else:
        scene = SCENE_FILE
    print(f"[INFO] 씬 로드: {scene}")

    model = mujoco.MjModel.from_xml_path(scene)
    data  = mujoco.MjData(model)
    model.opt.timestep = SIM_DT

    # ── 초기화 ────────────────────────────────────────────────────────────
    kb = KeyboardState(init_vx=args.cmd_vel_x)
    reset_robot(model, data)

    last_action   = np.zeros(12, dtype=np.float32)
    smooth_action = np.zeros(12, dtype=np.float32)  # EMA 스무딩용
    # V5/V3 498-dim: proprio_history 버퍼 (HISTORY_LEN × 45-dim, oldest→newest)
    proprio_history = deque(
        [np.zeros(45, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN
    )
    step_counter  = 0
    episode_steps = 0

    print("\n" + "=" * 60)
    if use_history and use_scan:
        policy_type = "V5/V3-history+scan"
    elif use_history:
        policy_type = "v4-history-blind"
    elif use_scan:
        policy_type = "scan"
    else:
        policy_type = "blind"
    print(f"  ICROS 2026 Sim2Sim  |  obs={obs_dim}-dim  |  {policy_type}")
    print("=" * 60)
    print("  ↑/W : 전진          ↓/S : 후진")
    print("  ←/A : 좌회전        →/D : 우회전")
    print("  Q/E : 좌/우 이동    R   : 속도 초기화")
    print("  Space: 로봇 리셋    ESC : 종료")
    print(f"  초기 명령: vx={kb.lin_vel_x:.1f} m/s")
    if use_history and use_scan:
        print(f"  proprio_history: {HISTORY_LEN}step × 45-dim = {HISTORY_LEN*45}dim")
        print(f"  height_scan: {SCAN_NX}×{SCAN_NY}={SCAN_DIM}점  ({SCAN_SIZE_X}×{SCAN_SIZE_Y}m)")
    elif use_history:
        print(f"  proprio_history: {HISTORY_LEN}step × 45-dim = {HISTORY_LEN*45}dim (blind actor)")
    elif use_scan:
        print(f"  height_scan: {SCAN_NX}×{SCAN_NY}={SCAN_DIM}점  ({SCAN_SIZE_X}×{SCAN_SIZE_Y}m)")
    print("=" * 60 + "\n")

    if args.headless_steps > 0:
        z_min = float(data.qpos[2])
        max_abs_action = 0.0
        for _ in range(args.headless_steps):
            if use_history and use_scan:
                obs = compute_obs_498(model, data, last_action, kb.command, proprio_history)
            elif use_history and not use_scan:
                obs = compute_obs_225(data, last_action, kb.command, proprio_history)
            elif use_scan:
                obs = compute_obs_318(model, data, last_action, kb.command)
            else:
                obs = compute_obs_45(data, last_action, kb.command)

            action_raw = session.run([output_name], {input_name: obs[None, :]})[0][0]
            action_raw = np.clip(action_raw, -1.0, 1.0)
            max_abs_action = max(max_abs_action, float(np.max(np.abs(action_raw))))
            smooth_action = ACTION_EMA_ALPHA * smooth_action + (1.0 - ACTION_EMA_ALPHA) * action_raw
            last_action = action_raw.copy()
            apply_pd_action(model, data, smooth_action)
            for _ in range(POLICY_DECIMATION):
                mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError("MuJoCo state became non-finite")
            z_min = min(z_min, float(data.qpos[2]))

        print(
            "[INFO] headless smoke OK: "
            f"steps={args.headless_steps}, z_final={data.qpos[2]:.3f}, "
            f"z_min={z_min:.3f}, max_abs_action={max_abs_action:.3f}"
        )
        return

    # ── 메인 루프 ─────────────────────────────────────────────────────────
    with mujoco.viewer.launch_passive(model, data,
                                      key_callback=kb.key_callback) as viewer:
        # 카메라: 로봇 스폰 위치 기준으로 설정
        spawn_pos = data.qpos[:3].copy()
        viewer.cam.lookat[:] = spawn_pos          # 로봇 위치 바라보기
        viewer.cam.distance  = 3.5
        viewer.cam.elevation = -25
        viewer.cam.azimuth   = 160

        while viewer.is_running():
            loop_start = time.perf_counter()

            # 리셋
            if kb.reset_flag:
                kb.reset_flag = False
                reset_robot(model, data)
                last_action   = np.zeros(12, dtype=np.float32)
                smooth_action = np.zeros(12, dtype=np.float32)
                proprio_history = deque(
                    [np.zeros(45, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN
                )
                step_counter  = 0
                episode_steps = 0
                print("[INFO] 로봇 리셋")

            # 물리 스텝
            mujoco.mj_step(model, data)
            step_counter  += 1
            episode_steps += 1

            # 50Hz 정책 실행
            if step_counter % POLICY_DECIMATION == 0:
                if use_history and use_scan:
                    # 498-dim: V5/V3 (proprio_history 225 + height_scan 273)
                    obs = compute_obs_498(model, data, last_action, kb.command, proprio_history)
                elif use_history and not use_scan:
                    # 225-dim: V4 (proprio_history 5×45, blind actor)
                    obs = compute_obs_225(data, last_action, kb.command, proprio_history)
                elif use_scan:
                    obs = compute_obs_318(model, data, last_action, kb.command)
                else:
                    obs = compute_obs_45(data, last_action, kb.command)

                action_raw = session.run(
                    [output_name],
                    {input_name: obs[None, :]}
                )[0][0]
                action_raw = np.clip(action_raw, -1.0, 1.0)

                # EMA 스무딩: 급격한 action 변화 완화 → 발 펄쩍 억제
                # smooth = alpha * prev + (1-alpha) * new
                smooth_action = ACTION_EMA_ALPHA * smooth_action + (1.0 - ACTION_EMA_ALPHA) * action_raw
                last_action = action_raw.copy()   # obs에는 raw action (policy 입력 일관성)
                apply_pd_action(model, data, smooth_action)  # PD에는 스무딩된 action
                viewer.sync()

            # 추락 감지
            if data.qpos[2] < -0.5:
                print(f"[WARN] 추락 감지 (z={data.qpos[2]:.2f}m) → 자동 리셋")
                reset_robot(model, data)
                last_action   = np.zeros(12, dtype=np.float32)
                smooth_action = np.zeros(12, dtype=np.float32)
                proprio_history = deque(
                    [np.zeros(45, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN
                )
                step_counter  = 0
                episode_steps = 0

            # 상태 출력 (5초마다)
            if step_counter % (200 * 5) == 0:
                pos = data.qpos[:3]
                cmd = kb.command
                print(f"[{episode_steps:5d}] "
                      f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                      f"cmd=({cmd[0]:.1f},{cmd[1]:.1f},{cmd[2]:.1f}) "
                      f"vx_cmd={kb.lin_vel_x:.1f}")

            # 실시간 속도 유지
            elapsed = time.perf_counter() - loop_start
            if SIM_DT - elapsed > 0:
                time.sleep(SIM_DT - elapsed)


if __name__ == "__main__":
    main()
