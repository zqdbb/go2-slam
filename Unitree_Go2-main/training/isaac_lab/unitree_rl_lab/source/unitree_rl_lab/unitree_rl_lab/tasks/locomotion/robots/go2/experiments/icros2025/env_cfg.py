"""ICROS 2025 Competition environment for Unitree Go2.

legged_gym/ETH Zurich terrain.py 충실 포팅 — 11가지 지형 (9 terrain.py + flat):
  1.  flat            — 평지 워밍업 (5%)
  2.  smooth_slope_up — 완만한 경사 오르막 (9%)  slope 4.6°→20° (row0→9)
  3.  smooth_slope_dn — 완만한 경사 내리막 (9%)  동일 난이도
  4.  rough_slope     — 거친 경사 (9%)           slope + 노이즈 동시 증가
  5.  stairs_up       — 계단 오르막 (13%)        3.5cm→11cm 계단 (row0→9)
  6.  stairs_down     — 계단 내리막 (13%)        동일 난이도
  7.  discrete_obs    — 불연속 장애물 (10%)      볼라드/요철, 4cm→14cm
  8.  bridge_ramp     — 경사 다리 (11%)          도랑 위 오르막·내리막
  9.  stepping_slabs  — 징검다리 슬랩 (11%)      도랑 위 오프셋 발판
  10. zigzag_bridge   — 지그재그 다리 (7%)       방향 전환 강제
  11. checker_blocks  — 체커 블록 (3%)           체스판 패턴 돌출부

난이도 구조 (10 rows × 11 cols = 110 타일):
  Row 0 (쉬움) → Row 9 (최고난도), difficulty=row/9 로 선형 스케일
  예: stairs_up  row0=3.5cm 계단,  row9=11cm 계단
      smooth_slope row0=4.6° 경사, row9=20° 경사

학습 전략 (auto_monitor_competition.py가 자동 관리):
  Phase 1 (iter 0-10k):    max_init_terrain_level=0  → 커리큘럼 from 평지
  Phase 2 (iter 10k-20k):  max_init_terrain_level=5  → 중간 난도 강제 노출
  Phase 3 (iter 20k-30k):  max_init_terrain_level=9  → 전체 난도 강제 노출
  /tmp/training_override.json 파일로 Phase 전환 (재시작 없이 읽음)

주요 설계 결정:
  - num_envs=16384 (RTX 5090 32GB, VRAM ~20GB 추정)
  - max_init_terrain_level=0 from override (Phase 전환 시 5→9로 변경)
  - Actor: 45-dim 고유감각만 (SLAM 연동 대비, 실배포 가능)
  - Critic: 333-dim (45 + height_scan 273) — 특권 정보로 학습 가속
  - DR: 마찰 0.2~1.5, 질량 ±5kg, 외력 ±8N/2Nm, push ±1m/s
"""

import json as _json
import math
import os as _os

# ── auto_monitor override 읽기 ─────────────────────────────────────────────
# /tmp/training_override.json이 있으면 Phase에 따라 max_init_terrain_level 변경
_OVERRIDE_PATH = "/tmp/training_override.json"
_override: dict = {}
if _os.path.exists(_OVERRIDE_PATH):
    try:
        with open(_OVERRIDE_PATH) as _f:
            _override = _json.load(_f)
    except Exception:
        _override = {}

# Phase 전환 시 auto_monitor가 이 값을 변경하고 훈련을 재시작함
_MAX_INIT_TERRAIN_LEVEL: int = int(_override.get("max_init_terrain_level", 0))

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG as ROBOT_CFG
from unitree_rl_lab.tasks.locomotion import mdp
from unitree_rl_lab.tasks.locomotion.robots.go2.experiments.icros2025.terrains import (
    HfBridgeRampTerrainCfg,
    HfCheckerBlocksTerrainCfg,
    HfDiscreteObstaclesTerrainCfg,
    HfPyramidStairsTerrainCfg,
    HfRoughSlopeTerrainCfg,
    HfSmoothSlopeTerrainCfg,
    HfSteppingSlabsTerrainCfg,
    HfZigzagBridgeTerrainCfg,
)

# ---------------------------------------------------------------------------
# 대회용 지형 구성 (10가지) — terrain.py 9가지 충실 포팅 + flat
# proportion 합계 = 1.00
#
# [terrain.py → IsaacLab 매핑]
#   Type1: smooth slope (up)  → HfSmoothSlopeTerrainCfg(inverted=False)
#   Type1: smooth slope (dn)  → HfSmoothSlopeTerrainCfg(inverted=True)
#   Type2: rough slope        → HfRoughSlopeTerrainCfg
#   Type3: stairs up          → HfPyramidStairsTerrainCfg(inverted=False)
#   Type4: stairs down        → HfPyramidStairsTerrainCfg(inverted=True)
#   Type5: discrete obstacles → HfDiscreteObstaclesTerrainCfg
#   Type6: bridge ramp        → HfBridgeRampTerrainCfg
#   Type7: stepping slabs     → HfSteppingSlabsTerrainCfg
#   Type8: zigzag bridge      → HfZigzagBridgeTerrainCfg
#   Type9: checker blocks     → HfCheckerBlocksTerrainCfg
#
# [그리드]  num_rows=10 × num_cols=10 = 100 타일
#           8192 envs → 타일당 ~82마리
# [난이도]  row0=difficulty 0(쉬움), row9=difficulty 1(최고난도)
# [파라미터] terrain.py 기본값 충실히 반영
#   slope: 0.08 ~ 0.36,  step_height: 0.035 ~ 0.11m (terrain.py 값)
# ---------------------------------------------------------------------------
COMPETITION_TERRAIN_CFG = terrain_gen.TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=10,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    difficulty_range=(0.0, 1.0),
    use_cache=False,
    sub_terrains={
        # ── 1. 평지 (5%) — 워밍업 ──────────────────────────────────
        'flat': terrain_gen.MeshPlaneTerrainCfg(proportion=0.05),

        # ── 2. 완만한 경사 오르막 (9%) — terrain.py Type1a ──────────
        # slope: 0.08(row0) ~ 0.36(row9)  [terrain.py 기본값]
        'smooth_slope_up': HfSmoothSlopeTerrainCfg(
            proportion=0.09,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
            slope_min=0.08,
            slope_max=0.36,
            platform_size=2.5,
            inverted=False,
        ),

        # ── 3. 완만한 경사 내리막 (9%) — terrain.py Type1b ─────────
        'smooth_slope_dn': HfSmoothSlopeTerrainCfg(
            proportion=0.09,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
            slope_min=0.08,
            slope_max=0.36,
            platform_size=2.5,
            inverted=True,
        ),

        # ── 4. 거친 경사 (9%) — terrain.py Type2 ───────────────────
        # slope*0.6 + random noise (terrain.py 그대로)
        'rough_slope': HfRoughSlopeTerrainCfg(
            proportion=0.09,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
            slope_min=0.04,
            slope_max=0.20,
            noise_min=0.005,
            noise_max=0.060,
            platform_size=2.5,
        ),

        # ── 5. 계단 오르막 (13%) — terrain.py Type3 ────────────────
        # step_height: 0.035(row0) ~ 0.110(row9)  step_width: 0.30~0.38
        'stairs_up': HfPyramidStairsTerrainCfg(
            proportion=0.13,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
            step_height_min=0.035,
            step_height_max=0.110,
            step_width_min=0.30,
            step_width_max=0.38,
            inverted=False,
        ),

        # ── 6. 계단 내리막 (13%) — terrain.py Type4 ────────────────
        'stairs_down': HfPyramidStairsTerrainCfg(
            proportion=0.13,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
            step_height_min=0.035,
            step_height_max=0.110,
            step_width_min=0.30,
            step_width_max=0.38,
            inverted=True,
        ),

        # ── 7. 불연속 장애물 (10%) — terrain.py Type5 ──────────────
        # 사이드월 포함, 랜덤 직사각형 장애물 + 미세 노이즈
        'discrete_obs': HfDiscreteObstaclesTerrainCfg(
            proportion=0.10,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
            obs_height_min=0.04,
            obs_height_max=0.14,
            obs_size_min=0.25,
            obs_size_max=0.70,
            num_obs_min=10,
            num_obs_max=28,
            platform_size=2.5,
            wall_height=0.20,
            wall_width=0.35,
        ),

        # ── 8. 경사 다리 (11%) — terrain.py Type6 ──────────────────
        # 도랑 위 오르막·내리막 경사로 브리지
        'bridge_ramp': HfBridgeRampTerrainCfg(
            proportion=0.11,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
        ),

        # ── 9. 징검다리 슬랩 (11%) — terrain.py Type7 ──────────────
        # 도랑 위 오프셋 발판들
        'stepping_slabs': HfSteppingSlabsTerrainCfg(
            proportion=0.11,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
        ),

        # ── 10. 지그재그 다리 (7%) — terrain.py Type8 ──────────────
        # 도랑 위 지그재그 브리지 (방향 전환 강제)
        'zigzag_bridge': HfZigzagBridgeTerrainCfg(
            proportion=0.07,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
        ),

        # ── 11. 체커 블록 (3%) — terrain.py Type9 ──────────────────
        # 도랑 위 체스판 패턴 블록
        'checker_blocks': HfCheckerBlocksTerrainCfg(
            proportion=0.03,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.25,
        ),
    },
)


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    """Competition terrain scene with Go2."""

    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=COMPETITION_TERRAIN_CFG,
        max_init_terrain_level=_MAX_INIT_TERRAIN_LEVEL,  # auto_monitor Phase 제어
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )

    robot: ArticulationCfg = ROBOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # LiDAR 대응 height scanner
    # 실제 배포: Go2 탑재 LiDAR/Depth 카메라로 동일한 높이맵 생성
    # size 확대 (1.6x1.0 → 2.0x1.2): 더 먼 장애물 미리 감지
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[2.0, 1.2]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


# ---------------------------------------------------------------------------
# Domain Randomization (Sim-to-Real)
# ---------------------------------------------------------------------------
@configclass
class EventCfg:
    """강화된 Domain Randomization — Sim-to-Real 갭 최소화."""

    # 마찰: 미끄러운 바닥(0.2) ~ 거친 지면(1.5) 전범위 학습
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.2, 1.5),
            "dynamic_friction_range": (0.2, 1.5),
            "restitution_range": (0.0, 0.2),
            "num_buckets": 64,
        },
    )

    # 질량: 바구니+배달물품 최대 ~4kg 추가 고려
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 5.0),
            "operation": "add",
        },
    )

    # 주기적 외력: 장애물 충돌/불규칙 하중 변화 모사 (Sim2Real)
    # mode="interval" → 에피소드 중 랜덤 시점에 짧게 가해짐 (reset 모드 아님)
    # ±8N / ±2Nm: 실내 대회 환경에서 현실적인 외란 범위
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="interval",
        interval_range_s=(6.0, 12.0),
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (-8.0, 8.0),
            "torque_range": (-2.0, 2.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),
            "velocity_range": (-1.0, 1.0),
        },
    )

    # 외부 충격 (실제 장애물 접촉, 밀림 현상 대응)
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(4.0, 8.0),
        params={"velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}},
    )


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
@configclass
class CommandsCfg:
    """대회 주행 속도 범위 반영."""

    base_velocity = mdp.UniformLevelVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,   # 정지 환경 2% (이전 5%)
        debug_vis=True,
        ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 2.0),   # 2.0 m/s: TRG-planner 출력 범위 + 여유
            lin_vel_y=(-0.4, 0.4),  # SLAM 측면 이동 지원
            ang_vel_z=(-1.5, 1.5),  # TRG-planner 회전 명령 커버
        ),
        limit_ranges=mdp.UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 2.5),   # 커리큘럼 최대
            lin_vel_y=(-0.6, 0.6),
            ang_vel_z=(-2.0, 2.0),
        ),
    )


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
@configclass
class ActionsCfg:
    JointPositionAction = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=0.25, use_default_offset=True, clip={".*": (-100.0, 100.0)}
    )


# ---------------------------------------------------------------------------
# Observations
# ---------------------------------------------------------------------------
@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        """Policy observations — Teacher 단계: 고유감각(IMU+관절)만 사용.

        Teacher는 height_scan 없이 고유감각만으로 보행/장애물 통과 학습.
        → 초기 랜덤 정책의 혼돈 최소화, 빠른 수렴 보장.
        → Student 증류 단계에서 height_scan 추가 (Sim2Real 완성).

        총 차원: ang_vel(3) + gravity(3) + cmd(3) + joint_pos(12)
               + joint_vel(12) + action(12) = 45
        """

        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2, clip=(-100, 100), noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-100, 100), noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, clip=(-100, 100), params={"command_name": "base_velocity"}
        )
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, clip=(-100, 100), noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel, scale=0.05, clip=(-100, 100), noise=Unoise(n_min=-1.5, n_max=1.5)
        )
        last_action = ObsTerm(func=mdp.last_action, clip=(-100, 100))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()

    @configclass
    class CriticCfg(ObsGroup):
        """Critic observations (특권 정보 포함 — 학습 시에만 사용)."""

        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, clip=(-100, 100))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2, clip=(-100, 100))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-100, 100))
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, clip=(-100, 100), params={"command_name": "base_velocity"}
        )
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel, clip=(-100, 100))
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel, scale=0.05, clip=(-100, 100))
        joint_effort = ObsTerm(func=mdp.joint_effort, scale=0.01, clip=(-100, 100))
        last_action = ObsTerm(func=mdp.last_action, clip=(-100, 100))
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

    critic: CriticCfg = CriticCfg()


# ---------------------------------------------------------------------------
# Rewards
# ---------------------------------------------------------------------------
@configclass
class RewardsCfg:
    """장애물 극복에 최적화된 보상 함수."""

    # 속도 추적 (핵심 목표)
    # weight 2.0: 전진 보행이 보상의 중심이 되도록 강화 (이전 1.5)
    track_lin_vel_xy = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=3.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )  # 2.0→3.0: 고속 추종 강화 (TRG-planner 2.0 m/s 명령 대응)
    track_ang_vel_z = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=1.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )  # 0.75→1.5: TRG-planner 회전 명령 정확 추종

    # 몸통 안정
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)

    # 에너지 효율
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2e-4)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.15)  # -0.1→-0.15: chattering 억제 (gentle)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
    energy = RewTerm(func=mdp.energy, weight=-2e-5)

    # 자세: 경사면에서 기울어짐 허용 (-1.0), 너무 강하면 언덕 못 오름 (이전 -2.0)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)

    joint_pos = RewTerm(
        func=mdp.joint_position_penalty,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.3,
        },
    )

    # 발 보행: threshold 0.25s → 정상 트로팅 보행에서 달성 가능 (0.5s는 너무 높음)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=1.5,  # 1.0→1.5: terrain 돌파 유도 (iter17000 개입)
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
            "command_name": "base_velocity",
            "threshold": 0.25,
        },
    )
    air_time_variance = RewTerm(
        func=mdp.air_time_variance_penalty,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot"),
        },
    )

    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1,
        params={
            "threshold": 1,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["Head_.*", ".*_hip", ".*_thigh", ".*_calf"]),
        },
    )

    # 쓰러짐에 강한 페널티 — 없으면 policy가 낙상을 학습하지 않음
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)


# ---------------------------------------------------------------------------
# Terminations
# ---------------------------------------------------------------------------
@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )
    # 경사면/장애물 통과 허용: 0.8 → 1.0 rad
    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})


# ---------------------------------------------------------------------------
# Curriculum
# ---------------------------------------------------------------------------
@configclass
class CurriculumCfg:
    # terrain_levels: row0(쉬움)→row9(어려움) 순차 학습
    # 성공 시 harder terrain, 실패 시 easier terrain으로 이동
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)
    lin_vel_cmd_levels = CurrTerm(mdp.lin_vel_cmd_levels)


# ---------------------------------------------------------------------------
# Main Environment
# ---------------------------------------------------------------------------
@configclass
class RobotEnvCfg(ManagerBasedRLEnvCfg):
    """ICROS 2025 Competition environment configuration."""

    scene: RobotSceneCfg = RobotSceneCfg(num_envs=16384, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 80 * 2**15   # 16384 envs (RTX 5090 32GB)
        self.sim.physx.gpu_collision_stack_size = 1024 * 1024 * 1024  # 1GB (16384 envs)

        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # 8마리 GUI play — 모든 지형 타입 체험
        # 10가지 지형 × 10 난이도 = 100 타일 (전체 그리드 표시)
        self.scene.num_envs = 8
        self.scene.terrain.terrain_generator.num_rows = 10
        self.scene.terrain.terrain_generator.num_cols = 10
        # 고난이도 지형부터 시작 (계단, 다리, 슬랩 등 모두 보임)
        self.scene.terrain.max_init_terrain_level = 9
        # 최대 속도 명령으로 play (빠른 주행)
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
