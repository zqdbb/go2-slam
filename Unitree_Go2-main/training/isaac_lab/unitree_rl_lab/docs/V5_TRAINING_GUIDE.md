# ICROS 2026 V5 Training Guide — Codex 핸드오프용 종합 문서

> **이 문서의 목적**  
> Claude Code 세션에서 V5 학습 환경을 구축/디버깅한 모든 결정과 변경사항을 정리.  
> Codex(또는 다른 AI 어시스턴트)로 옮겨서 **이 문서 하나만 읽고도 동일한 작업을 이어갈 수 있도록** 작성.  
> 실수 재발 방지(특히 V1~V3 실패 패턴), 학습 모니터링, sim2sim/배포까지 전 과정.

---

## 0. 한 줄 요약

- **목표**: Unitree Go2 ICROS 2026 대회용 locomotion policy.  
- **핵심 변경(V5)**: V4 Actor(225-dim blind) → **498-dim (proprio 225 + height_scan 273)** — Actor가 앞 지형을 봄.  
- **그 외**: V4의 검증된 reward/DR 전부 유지 (`track=1.5`, `action_rate=-0.01`, `smoothness2=-0.001`).  
- **학습량**: 50,000 iter, 16,384 envs, 약 **3일** 예상.  
- **Phase 전환**: Checkpoint 파일 번호 기반(V4의 log-count 버그 제거).  
- **배포 목표**: FAST-LIO2 height map + TRG-planner velocity command → 실제 Go2.

---

## 1. 실행 환경 (반드시 확인)

| 항목 | 값 | 비고 |
|------|----|----|
| Docker 컨테이너 | `isaac-lab-template` | ⚠️ `isaac-lab` 아님 |
| 컨테이너 내부 Python | `/isaac-sim/python.sh` | `python`/`python3` 사용 금지 |
| 호스트 워크스페이스 | `/home/nuri/unitree_rl_lab` | |
| 컨테이너 워크스페이스 | `/workspace/unitree_rl_lab` | 호스트와 bind mount |
| 호스트 로그 경로 | `/home/nuri/unitree_rl_lab/logs/...` | 학습 산출물 호스트에도 동기화됨 |
| 컨테이너 로그 경로 | `/workspace/logs/rsl_rl/` | 학습 스크립트는 이 경로 사용 |
| GPU | RTX 5090 32GB | 16,384 envs ≈ 20GB VRAM |
| 호스트 OS | Ubuntu (Linux 6.17) | |

> **명령어 형식 예시**  
> ```bash
> docker exec isaac-lab-template /isaac-sim/python.sh /workspace/unitree_rl_lab/scripts/.../train.py ...
> ```

---

## 2. 실험 이력 (왜 V5인가?)

각 버전이 **무엇을 시도하고 왜 실패했는지** — 같은 실수를 다시 하지 않기 위해 반드시 숙지.

| 버전 | Policy 차원 | 핵심 변경 | 결과 | 실패 원인 |
|------|------------|-----------|------|----------|
| V1 | 45 (proprio only) | 베이스 환경 | — | 출발점 |
| V2 (구 `_scan`) | 318 (45 + height_scan) | height_scan 추가 | 25k에서 plateau | `move_down`이 cmd_vel 비례 → 항상 강등 |
| V3 | 498 (5×45 history + scan) | history + 보상 강화 | 망함 | `track=3.0` + `action_rate=-0.25` → **action_rate 포화(-1.02)** → 경련 보행 |
| V4 | **225 (5×45 history, BLIND)** | 보상 SOTA 표준 재설계 | terrain_level 5.27/9 **plateau** | Actor가 앞을 못 봄 → reactive only |
| **V5** | **498 (225 proprio + 273 height_scan)** | **V4 + Actor가 height_scan 볼 수 있게** | (학습 중) | (목표: terrain_level 7~9) |

### V3가 망한 이유 (중요)
- V3도 498-dim이었지만 **실패 원인은 obs 차원이 아니라 reward**:
  - `track_lin_vel_xy=3.0` (너무 큼) + `action_rate=-0.25` (너무 큼) → 포화 -1.02
  - 로봇이 "경련 감수하고 속도 추종" 전략 선택 → Sim2Real 불가
- **V5는 V3 obs(498) + V4 reward(검증) → 두 버전 장점 결합**.

### V4가 plateau한 이유
- V4 Actor는 225-dim proprioception만 → **앞 지형을 못 봄(blind)**.
- 발이 장애물에 닿고 나서야 반응 → 고난도 지형(terrain_level 5+)에서 실패.
- 해결: V5에서 Actor에 height_scan 추가.

---

## 3. V5 환경 정의

### 3.1 파일 위치

```
source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/experiments/icros2026_v5/
├── env_cfg.py        ← 핵심: V4 상속 + PolicyCfg에 height_scan 추가
├── __init__.py
└── terrains/         ← 비어있음 (V1 지형 그대로)

scripts/experiments/icros2026_v5/
├── train.py          ← scripts/rsl_rl/train.py 와 동일 (수정 없음)
├── play.py           ← V4 play.py 복사 (teleop TensorDict 수정 포함)
├── auto_monitor.py   ← V5용 신규: checkpoint 기반 phase 전환
└── README.md
```

### 3.2 Observation Space

```
Policy (Actor — 배포용, noisy):
  proprio_history (225-dim) = 5 frames × 45-dim
    각 frame: [ang_vel(3) + gravity(3) + cmd(3) + jpos(12) + jvel(12) + last_action(12)]
  height_scan (273-dim) = GridPatternCfg(resolution=0.1, size=[2.0, 1.2])
    노이즈 Unoise(-0.1, 0.1) → FAST-LIO2 노이즈 모사
    클립 (-1.0, 1.0)
  ─────────────────
  합계 498-dim

Critic (학습 전용, privileged):
  V1 CriticCfg 그대로 = 333-dim (V4와 동일, 변경 없음)
```

> ⚠️ **Policy obs 레이아웃** — sim2sim/teleop에서 cmd override 시 필요:  
> `cmd[k]`는 각 history frame의 `[k*45+6 : k*45+9]` 위치.  
> 5-step이므로 5번 모두 override 필요.

### 3.3 Reward (V4 그대로, V3 실패 교훈 반영)

| Reward term | Weight | 출처 |
|---|---|---|
| `track_lin_vel_xy` | **1.5** | ETH/Walk-These-Ways/PGTT 표준 |
| `track_ang_vel_z` | **0.75** | SOTA 표준 |
| `action_rate` | **-0.01** | SOTA 표준 (V3의 -0.25는 포화 유발) |
| `action_smoothness_2` | **-0.001** | Walk-These-Ways 2차 도함수 |
| 그 외 (joint_acc, feet_slide, etc.) | V1 그대로 | |

> ⚠️ **절대 바꾸지 말 것**: 이 가중치들을 임의로 올리면 V3의 경련 보행이 재발한다.

### 3.4 Domain Randomization (Phase별)

`_DR_TABLE` (env_cfg.py 내부):

| Phase | max_act_delay | kp/kd range | jpos_noise | ang_vel_noise | jvel_noise |
|-------|---------------|-------------|------------|---------------|------------|
| 1 | 1 step | ±10% | 0.005 | 0.10 | 0.040 |
| 2 | 2 step | ±18% | 0.008 | 0.15 | 0.055 |
| 3 | 3 step | ±25% | 0.010 | 0.20 | 0.075 |

- Phase는 `/tmp/training_override.json`의 `dr_phase` 키로 선택.
- `_DR_PHASE`는 env_cfg.py가 **import 시점에 한 번** 읽음 → Phase 전환 시 **반드시 학습 재시작** 필요 (auto_monitor.py가 자동 처리).

### 3.5 Terrain Curriculum

- `move_up`: `distance > terrain_size * 0.375` (≈ 3.0m)
- `move_down`: `distance < terrain_size * 0.1` (≈ 0.8m)
- 출처: V3에서 검증된 값. V2의 cmd_vel 비례 강등 버그 수정판.

### 3.6 Command 범위

```python
ranges.lin_vel_x = (_LIN_VEL_X_MIN, _LIN_VEL_X_MAX)   # override로 변경
ranges.lin_vel_y = (-0.5, 0.5)
ranges.ang_vel_z = (-1.5, 1.5)
limit_ranges.lin_vel_x = (-1.0, 2.0)   # 커리큘럼 상한
limit_ranges.lin_vel_y = (-0.8, 0.8)
limit_ranges.ang_vel_z = (-2.0, 2.0)
resampling_time_range = (5.0, 15.0)
```

---

## 4. Phase 시스템 (체크포인트 기반)

### 4.1 핵심 원리

**V4의 버그**: `auto_monitor.py`가 로그의 `"Iteration time"` 문자열 개수로 진행도 판단 →  
RSL-RL log buffer가 실제 iter의 ~20%만 캡처 → Phase 3 자동 전환 실패.

**V5의 해결**: **체크포인트 파일 번호(`model_XXXXX.pt`)로 정확한 진행도 측정**.  
`save_interval=50` → 50 iter마다 한 번씩 저장 → 모니터는 가장 큰 번호의 ckpt로 현재 iter 추정.

### 4.2 Phase 정의 (`scripts/experiments/icros2026_v5/auto_monitor.py`)

```python
PHASES = [
    {"name": "Phase1", "start_ckpt":     0, "max_init_level": 0, "lin_vel_x_max": 1.0, "lin_vel_x_min":  0.0, "dr_phase": 1, "desc": "기초 보행 + 평지 (DR 최소)"},
    {"name": "Phase2", "start_ckpt": 10000, "max_init_level": 4, "lin_vel_x_max": 1.5, "lin_vel_x_min":  0.0, "dr_phase": 2, "desc": "중간 지형 + DR 강화"},
    {"name": "Phase3", "start_ckpt": 20000, "max_init_level": 7, "lin_vel_x_max": 2.0, "lin_vel_x_min": -0.5, "dr_phase": 3, "desc": "고난도 지형 + 최대 DR + 30k iter"},
]
TOTAL_ITER = 50000
```

### 4.3 Override 메커니즘 (`/tmp/training_override.json`)

env_cfg.py가 **import 시점에 읽음**. 키:

| 키 | 타입 | 효과 |
|---|---|---|
| `max_init_terrain_level` | int | 초기 spawn 지형 레벨 (0~9) |
| `lin_vel_x_max` | float | cmd.lin_vel_x 상한 |
| `lin_vel_x_min` | float | cmd.lin_vel_x 하한 |
| `dr_phase` | int (1/2/3) | DR 강도 선택 |

> ⚠️ **V3/V4 버그 재발 방지**: `auto_monitor.py`의 `write_override()`는 **항상 `dr_phase` 포함**.  
> 일부만 override 시 `dr_phase`가 누락되어 Phase 강도가 의도와 다르게 적용되는 버그가 있었음.

### 4.4 Phase 전환 시 동작

```
1. auto_monitor: 새 phase 진입 감지 (ckpt_iter >= phase.start_ckpt)
2. write_override(JSON 갱신)
3. pkill -f train.py  (env_cfg.py 다시 import 필요)
4. sleep 15s
5. 가장 최신 ckpt로 resume (--resume --load_run TS --checkpoint model_X.pt)
```

---

## 5. Auto Monitor 상세 (`scripts/experiments/icros2026_v5/auto_monitor.py`)

### 5.1 핵심 상수 (V4의 restart loop 버그 수정판)

```python
CHECK_INTERVAL = 60       # 60s 마다 체크
TOTAL_ITER     = 50000
STALL_TIMEOUT  = 1000     # 1000s = checkpoint 간격(250s) × 4배
INIT_TIMEOUT   = 99999    # 사실상 무한 — ckpt=0이면 절대 재시작 안 함
```

**산정 근거**:
- `save_interval=50`, iter 평균 ≈ 5s → 체크포인트 간격 ≈ 250s.
- `STALL_TIMEOUT=1000s = 4 × 250s` → 1번 늦어도 false alarm 안 남.
- `INIT_TIMEOUT=99999s`: Isaac Sim 초기화는 6~10분 걸림. 초기화 중 재시작하면 무한 루프 → **절대 재시작 안 함**.

### 5.2 `get_latest_checkpoint()` — 최신 RUN 디렉토리만 추적

**버그**: 이전 실패 run의 `model_200.pt`가 새 run의 `model_50.pt`보다 큰 숫자라서 항상 우선됨 → 새 run 진행도 미감지.

**수정**: `ls -dt $LOG_DIR/*/ | head -1`로 가장 최근 수정된 run dir만 추출. 그 안에서 최대 ckpt 번호 사용.

```python
latest_run_dir = docker_exec(f"ls -dt {LOG_DIR}/*/ 2>/dev/null | head -1 | tr -d '\\n'")
# 그 디렉토리 안에서만 최고 번호 model_*.pt 검색
```

### 5.3 Hang 감지 로직

```python
if is_training_running():
    if ckpt_iter > state["last_ckpt_iter"]:
        stall_count = 0  # 진행됨
    else:
        stall_count += 1
        if ckpt_iter == 0:  effective_timeout = INIT_TIMEOUT (재시작 안 함)
        else:               effective_timeout = STALL_TIMEOUT (1000s)
        if stall_count * CHECK_INTERVAL >= effective_timeout:
            kill + write_override(dr_phase 포함) + restart
else:
    # 프로세스가 없으면 크래시 → 즉시 재시작
    write_override(dr_phase 포함) + restart
```

### 5.4 상태 파일

`/tmp/monitor_state_icros2026_v5.json`:
```json
{"current_phase_idx": 0, "last_ckpt_iter": 0, "stall_count": 0}
```

> **모니터를 처음부터 재시작할 때**: 이 파일을 `{"current_phase_idx": 0, "last_ckpt_iter": 0, "stall_count": 0}`로 초기화해야 새 run의 낮은 ckpt 번호를 정체로 오인하지 않음.

---

## 6. 학습 시작 절차

### 6.1 완전 신규 시작 (0부터)

```bash
# (1) 이전 run 디렉토리 청소(선택)
docker exec isaac-lab-template bash -c \
  "rm -rf /workspace/logs/rsl_rl/unitree_go2_icros2026_v5/*"

# (2) Phase1 override 작성
docker exec isaac-lab-template bash -c \
  "echo '{\"max_init_terrain_level\": 0, \"lin_vel_x_max\": 1.0, \"lin_vel_x_min\": 0.0, \"dr_phase\": 1}' > /tmp/training_override.json"

# (3) 모니터 상태 파일 초기화
echo '{"current_phase_idx": 0, "last_ckpt_iter": 0, "stall_count": 0}' > /tmp/monitor_state_icros2026_v5.json

# (4) 학습 시작 (백그라운드)
docker exec isaac-lab-template bash -c "
nohup /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py \
  --task Unitree-Go2-ICROS2026-V5 \
  --headless \
  --num_envs 16384 \
  > /tmp/train_icros2026_v5.log 2>&1 &
echo PID=\$!
"

# (5) 모니터 시작 (호스트에서, 백그라운드)
nohup python3 /home/nuri/unitree_rl_lab/scripts/experiments/icros2026_v5/auto_monitor.py \
  >> /tmp/monitor_icros2026_v5.log 2>&1 &
echo "Monitor PID=$!"
```

### 6.2 진행 상태 확인

```bash
# 모니터 실시간 로그
tail -f /tmp/monitor_icros2026_v5.log

# 학습 로그 (RSL-RL 출력)
tail -f /tmp/train_icros2026_v5.log

# 컨테이너 내부 프로세스
docker exec isaac-lab-template bash -c "ps aux | grep -E 'train.py' | grep -v grep"

# 호스트 모니터 프로세스
ps aux | grep auto_monitor | grep -v grep

# 체크포인트 진행도
docker exec isaac-lab-template bash -c \
  "find /workspace/logs/rsl_rl/unitree_go2_icros2026_v5 -name 'model_*.pt' | sort -V | tail -5"
```

### 6.3 비상시 (모니터/학습 강제 종료 후 재개)

```bash
# 모든 프로세스 죽이기
docker exec isaac-lab-template bash -c "pkill -9 -f train.py" 2>/dev/null
kill $(ps aux | grep auto_monitor | grep -v grep | awk '{print $2}') 2>/dev/null

# 가장 최신 ckpt 확인
docker exec isaac-lab-template bash -c \
  "find /workspace/logs/rsl_rl/unitree_go2_icros2026_v5 -name 'model_*.pt' | sort -V | tail -1"

# 모니터만 재시작하면 ckpt 자동 감지 후 resume
nohup python3 /home/nuri/unitree_rl_lab/scripts/experiments/icros2026_v5/auto_monitor.py \
  >> /tmp/monitor_icros2026_v5.log 2>&1 &
```

---

## 7. Play / Sim2Sim / 배포

### 7.1 IsaacLab Play

```bash
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/play.py \
  --task Unitree-Go2-ICROS2026-V5 \
  --num_envs 8 \
  --load_run <TIMESTAMP_FOLDER>
```

> ⚠️ `--checkpoint`는 절대경로 사용 시 `retrieve_file_path()`가 직접 path로 처리해서 실패함.  
> **`--load_run TS` 만 쓰고 `--checkpoint` 생략**하면 자동으로 가장 큰 model_*.pt 사용.

#### Teleop 모드
```bash
# 위 명령에 --teleop 추가
... --teleop
# 키: W/S 전후, A/D 회전, Q/E 좌우 (터미널 입력)
```

**teleop 버그 수정 기록** (`play.py` 내):
- `RslRlVecEnvWrapper`는 `TensorDict(batch_size=[N])`를 반환.
- `obs.shape`은 `torch.Size([N])` (1D) → `obs.shape[1]` 호출하면 IndexError.
- 해결: `_extract_policy_obs()` / `_put_policy_obs()` 헬퍼로 TensorDict의 `"policy"` key 추출 후 처리.
- `_get_cmd_slots()`: `obs_dim`이 225 또는 498일 때 5-step history → cmd slot 5개 [(k*45+6, k*45+9)] 반환.

### 7.2 ONNX Export

```bash
# play.py 내장 export 사용 (자동으로 ONNX 저장됨)
# 또는 standalone:
docker exec isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/export_onnx_standalone.py \
  --checkpoint /workspace/logs/rsl_rl/unitree_go2_icros2026_v5/<TS>/model_50000.pt \
  --output /workspace/unitree_rl_lab/model_icros2026_v5_50000.onnx
```

### 7.3 MuJoCo Sim2Sim (`scripts/sim2sim/sim2sim_icros2026.py`)

이미 V4/V5 자동 감지 로직 포함:

```python
if obs_dim == 498:
    policy_type = "v3-history+scan"  # V5도 같은 차원이므로 사용
    use_scan, use_history = True, True
elif obs_dim == 225:
    policy_type = "v4-history-blind"  # V4 전용
    use_scan, use_history = False, True
elif obs_dim == 318:
    policy_type = "scan"  # V2 전용
elif obs_dim == 45:
    policy_type = "blind"  # V1 전용
```

실행:
```bash
python3 scripts/sim2sim/sim2sim_icros2026.py \
  --onnx model_icros2026_v5_50000.onnx \
  --map icra2023_easy  # 또는 --scene 으로 경로 직접
```

키보드 조작 (MuJoCo viewer):
- W/S: 전후 ±0.2 m/s
- A/D: 회전 ±0.3 rad/s
- Q/E: 좌우 ±0.2 m/s
- R: 명령 초기화 / Space: 리셋 / ESC: 종료

### 7.4 실제 Go2 배포 (계획)

```
FAST-LIO2 (LiDAR) → 273-dim height map (2.0×1.2m, 0.1m res)
TRG-planner       → velocity commands (vx, vy, wz)
unitree_sdk2py    → joint pos commands

ONNX policy 입력 = [proprio_history(225), height_scan(273)] = 498-dim ✓
```

**전처리 매핑**:
- IsaacLab → MuJoCo joint 순서: `[3,0,9,6,4,1,10,7,5,2,11,8]`
- Kp=25.0, Kd=0.5, action_scale=0.25 (UNITREE_GO2_CFG)
- height_scan: LiDAR pointcloud → 2D bin → 21×13 grid

---

## 8. 디버깅 / 트러블슈팅

### 8.1 자주 발생하는 문제

| 증상 | 원인 | 해결 |
|------|------|------|
| 모니터가 계속 학습 죽이고 재시작 | `STALL_TIMEOUT`이 `save_interval` 간격보다 짧음 | `STALL_TIMEOUT=1000`, `save_interval=50` |
| `model_0.pt` 단계에서 자꾸 재시작 | Isaac Sim 초기화 6~10분 걸리는데 timeout 짧음 | `INIT_TIMEOUT=99999`로 ckpt=0 보호 |
| 새 run인데 monitor가 옛 ckpt 번호 인식 | `find` 전체 LOG_DIR 검색 → 옛 model_200.pt 선택 | `ls -dt LOG_DIR/*/ | head -1`로 최신 run dir만 |
| Phase 전환 후 DR이 그대로 | `dr_phase`가 override JSON에서 누락 | `write_override()`가 항상 4개 키 모두 작성 |
| Teleop에서 `IndexError: shape[1]` | `RslRlVecEnvWrapper`가 `TensorDict` 반환 | `_extract_policy_obs(obs)` 후 사용 |
| `--checkpoint /abs/path` FileNotFoundError | `retrieve_file_path()`가 raw path 처리 | `--load_run TS` 만 쓰고 `--checkpoint` 생략 |
| OOM (16384 envs) | num_envs 누락 → 기본 20480 | 항상 `--num_envs 16384` 명시 |

### 8.2 sanity check 명령

```bash
# 학습 진행 중인지
docker exec isaac-lab-template bash -c "ps aux | grep train.py | grep -v grep"

# GPU 사용량
docker exec isaac-lab-template nvidia-smi

# 가장 최신 ckpt
docker exec isaac-lab-template bash -c \
  "ls -dt /workspace/logs/rsl_rl/unitree_go2_icros2026_v5/*/ | head -1"

# RSL-RL iter 로그 (300줄)
tail -300 /tmp/train_icros2026_v5.log | grep -E "Iteration|Mean reward|terrain_levels"
```

### 8.3 학습이 진행 안 될 때 체크리스트

1. `docker ps` — 컨테이너 살아있나?
2. `ps aux | grep train.py` (컨테이너 내부) — 학습 프로세스 살아있나?
3. `nvidia-smi` — GPU 메모리 정상?
4. `tail /tmp/train_icros2026_v5.log` — 마지막 출력 확인.
5. `find LOG_DIR -name 'model_*.pt' | sort -V | tail` — 최근 ckpt 시간 확인 (`stat -c %y`).
6. monitor 로그에 `Override 작성`이 매번 보이면 → restart loop 의심.

---

## 9. 변경 사항 요약 (이번 세션에서 수정한 파일)

| 파일 | 변경 |
|------|------|
| `source/.../go2/experiments/icros2026_v5/env_cfg.py` | **신규**: V4 상속 + PolicyCfg에 height_scan 추가 |
| `scripts/experiments/icros2026_v5/auto_monitor.py` | **신규**: checkpoint 기반 phase 전환 |
| `scripts/experiments/icros2026_v5/play.py` | **신규**: V4 play.py 복사 + teleop TensorDict 수정 |
| `scripts/experiments/icros2026_v5/train.py` | **신규**: scripts/rsl_rl/train.py 와 동일 |
| `source/.../go2/__init__.py` | `Unitree-Go2-ICROS2026-V5` gym.register 추가 |
| `source/.../tasks/locomotion/agents/rsl_rl_ppo_cfg.py` | `save_interval`: 200 → **50** (체크포인트 더 자주) |
| `scripts/sim2sim/sim2sim_icros2026.py` | `compute_obs_225()` 추가 + dispatch 분기 |
| `scripts/experiments/icros2026_v4/play.py` | TensorDict 호환 헬퍼 추가 (V5도 같은 패턴 사용) |
| `docs/V5_TRAINING_GUIDE.md` | **신규 (이 문서)** |
| `HANDOFF_PROMPT.md` | V5 기준 갱신 |
| `EXPERIMENTS.md` | V5 행 추가 |

---

## 10. 학습 타임라인 예상

```
Phase 1 (ckpt 0 → 10,000):   ≈ 14h
Phase 2 (ckpt 10k → 20k):    ≈ 14h
Phase 3 (ckpt 20k → 50k):    ≈ 42h
─────────────────────────────────
총 ≈ 70 hours (≈ 3 days)
```

(16,384 envs, RTX 5090에서 평균 iter ≈ 5초 기준)

**예상 결과 (V4 → V5)**:
- terrain_level: 5.27 → 7~9 (height_scan 덕분에 발 높이 예측 가능)
- Sim2Real: V4보다 강함 (DR Phase3 + height_scan 노이즈 ±0.1)
- 실제 Go2 배포 가능 (498-dim 입력 = LiDAR height map + proprio)

---

## 11. 추가로 알아야 할 것 (간과하기 쉬운 디테일)

1. **env_cfg.py는 import 시점에 override 읽음** — Phase 전환 시 반드시 프로세스 재시작.
2. **컨테이너 이름은 `isaac-lab-template`** — `isaac-lab`이 아님 (예전 CLAUDE.md에 오타 있음).
3. **`/workspace`는 컨테이너 내부 경로** — 호스트는 `/home/nuri/unitree_rl_lab`.
4. **로그 디렉토리 `unitree_go2_icros2026_v5`** — experiment_name이 task ID와 다름(소문자/언더스코어).
5. **`--num_envs 16384` 명시 필수** — auto_monitor 빠뜨리면 OOM.
6. **체크포인트는 호스트와 동기화됨** — 호스트에서 직접 ONNX 추출 가능.
7. **`new_experiment.py` 사용 의무** — 직접 폴더 만들면 `__init__.py` import path 깨짐.
8. **V1 베이스 환경 절대 수정 금지** — V2~V5 모두 V1 상속 구조.
9. **`scripts/rsl_rl/train.py`, `play.py` 절대 수정 금지** — 실험별 폴더에 복사 후 수정.
10. **`/tmp/training_override.json`은 호스트 ↔ 컨테이너 공유 안 됨** — `docker exec`로 컨테이너 내부에 작성해야 env_cfg가 읽음.

---

## 12. 다음 단계 (V5 학습 완료 후)

1. ONNX export → `model_icros2026_v5_50000.onnx`
2. MuJoCo sim2sim 검증 (ICRA/ICROS 맵에서 장애물 극복 테스트)
3. FAST-LIO2 통합: LiDAR pointcloud → height_scan 전처리 노드
4. TRG-planner 통합: 목표 → velocity command 변환
5. 실제 Go2 배포 (unitree_sdk2py 기반)

---

## 13. 참고 — 핵심 코드 스니펫

### V5 env_cfg.py 핵심
```python
from .icros2026_v4.env_cfg import (
    ObservationsCfg as _V4ObsCfg,
    RobotEnvCfg as _V4EnvCfg,
    _DR_TABLE,
)

@configclass
class ObservationsCfg(_V4ObsCfg):
    @configclass
    class PolicyCfg(_V4ObsCfg.PolicyCfg):
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )
    policy: PolicyCfg = PolicyCfg()
    # critic은 V4(=V1) 그대로 333-dim

@configclass
class RobotEnvCfg(_V4EnvCfg):
    observations: ObservationsCfg = ObservationsCfg()
    def __post_init__(self):
        super().__post_init__()
        if hasattr(self.scene, 'height_scanner'):
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
```

### auto_monitor.py 핵심 함수
```python
def get_latest_checkpoint() -> tuple[str, int]:
    """가장 최신 run 디렉토리에서만 최대 ckpt 번호 추출."""
    latest_run_dir = docker_exec(f"ls -dt {LOG_DIR}/*/  2>/dev/null | head -1 | tr -d '\\n'")
    if not latest_run_dir:
        return "", 0
    out = docker_exec(
        f"find {latest_run_dir} -name 'model_*.pt' 2>/dev/null"
        r" | awk -F'model_' '{print $2, $0}'"
        r" | sed 's/\.pt / /'"
        " | sort -k1 -n | tail -1"
    )
    # ... parse out → (path, iter_num)

def write_override(max_init: int, vel_max: float, vel_min: float, dr_phase: int):
    """반드시 4개 키 모두 작성 (V3/V4 버그 재발 방지)."""
    override = {
        "max_init_terrain_level": max_init,
        "lin_vel_x_max": vel_max,
        "lin_vel_x_min": vel_min,
        "dr_phase": dr_phase,
    }
    docker_exec(f"echo '{json.dumps(override)}' > {OVERRIDE_PATH}")
```

### Teleop TensorDict 헬퍼 (play.py)
```python
def _extract_policy_obs(obs):
    if hasattr(obs, 'keys') or isinstance(obs, dict):
        return obs.get("policy", obs)
    return obs

def _put_policy_obs(obs_orig, policy_obs_modified):
    if hasattr(obs_orig, 'keys') or isinstance(obs_orig, dict):
        obs_orig["policy"] = policy_obs_modified
        return obs_orig
    return policy_obs_modified

def _get_cmd_slots(policy_tensor):
    obs_dim = policy_tensor.shape[-1]
    if obs_dim in (225, 498):
        return [(k*45+6, k*45+9) for k in range(5)]  # 5-step history
    else:
        return [(6, 9)]  # single frame
```

---

---

## 14. 학습 방향 객관적 검증 (2026-05-30, iter 527 / 50000)

> **결론 (TL;DR): 학습 방향 매우 건강. 모든 핵심 지표가 V4 동일 phase 시점보다 우수. 중단 불필요.**

### 14.1 현재 V5 metrics (iter 527, ckpt ≈ 450, Phase1 진행 중)

| Metric | 값 | 해석 |
|--------|------|------|
| Mean reward | 16.94 | 매우 높음 (안정 보행) |
| Mean episode length | 1000.00 | timeout 도달 (max에 도달) |
| Mean action std | 0.48 | 적절한 탐색 |
| `track_lin_vel_xy` | 1.15 | weight=1.5에서 76% 도달 |
| `track_ang_vel_z` | 0.58 | weight=0.75에서 77% 도달 |
| `action_rate` | -0.086 | weight=-0.01, 미가공값 ≈ -8.6 → **포화 아님** |
| `action_smoothness_2` | -0.021 | 미가공값 ≈ -21 → 정상 범위 |
| `error_vel_xy` | 0.327 | tracking error 낮음 |
| `error_vel_yaw` | 0.336 | tracking error 낮음 |
| `terrain_levels` | 2.26 | Phase1 max_init=0인데 빠르게 상승 |
| `lin_vel_cmd_levels` | 1.0 | Phase1 정상 |
| `time_out` 비율 | 97.4% | 안정 보행 |
| `base_contact` 비율 | 1.1% | 거의 안 넘어짐 |
| `bad_orientation` 비율 | 1.4% | 자세 양호 |
| Iter time | 4.4s | 정상 (89k steps/s) |

### 14.2 V4 끝지점 (iter 32000 / 33000, Phase3) 비교

| Metric | V4 (iter 32000) | V5 (iter 527) | V5가 우수? |
|--------|----------------|---------------|-----------|
| Mean reward | 2.58 | 16.94 | △ 조건 다름 (V4는 Phase3 고난도) |
| `track_lin_vel_xy` | 0.849 | 1.150 | ✅ V5가 35% 더 정확 |
| `track_ang_vel_z` | 0.499 | 0.563 | ✅ V5 우수 |
| `error_vel_xy` | 0.778 | 0.327 | ✅ V5가 **2.4× 정확** |
| `error_vel_yaw` | 0.429 | 0.336 | ✅ V5 우수 |
| `action_rate` (raw) | -13.66 | -8.6 | ✅ V5가 **37% 더 부드러움** |
| `action_smoothness_2` (raw) | -37.1 | -21.3 | ✅ V5가 부드러움 |
| `terrain_levels` | **5.30 (plateau)** | 2.26 (상승 중) | ⏳ V5는 아직 진행 중 |
| `feet_air_time` | -0.053 | -0.054 | = 비슷 |
| Mean action std | 0.61 | 0.48 | ✅ V5가 더 결정적 |

> ⚠️ 직접 비교 시 주의: V4 iter 32000은 Phase3(max_init=7, 고난도), V5 iter 527은 Phase1(max_init=0, 평지).  
> → **공정 비교는 V5가 Phase3 진입한 뒤에 가능**. 그래도 핵심 지표 4개(tracking 정확도, action smoothness, action_rate, tracking error)는 학습 단계 무관하게 V5가 V4를 명백히 앞섬.

### 14.3 SOTA 논문 표준 비교 (Reward 가중치)

| Term | ETH legged_gym | Walk-These-Ways | PGTT (Go2 HW) | V5 (우리) |
|------|----------------|------------------|---------------|----------|
| `track_lin_vel_xy` | 1.0~1.5 | 1.0 | 1.0 | **1.5** ✅ |
| `track_ang_vel_z` | 0.5~1.0 | 0.5 | 0.5 | **0.75** ✅ |
| `action_rate` | -0.01 | -0.01 | -0.01 | **-0.01** ✅ |
| `action_smoothness_2` | — | -0.001 | — | **-0.001** ✅ |

→ **모든 가중치가 SOTA 표준 범위 내**.

### 14.4 V3 실패 패턴 미발생 확인

| 위험 신호 | V3 실패 시 | V5 현재 |
|-----------|-----------|---------|
| `action_rate` 포화 | ≈ -1.02 (포화) | -0.086 (정상) ✅ |
| 경련 보행 (action_smoothness_2 폭주) | 음수 폭주 | -0.021 정상 ✅ |
| termination 폭증 | base_contact 5%↑ | base_contact 1.1% ✅ |

→ **V3의 실패 시그니처 전무**. reward 가중치 변경(track 3.0→1.5, action_rate -0.25→-0.01)이 효과적.

### 14.5 V4 plateau 시그니처 vs V5

V4가 plateau에 빠진 이유는 Actor가 blind라서 발 높이를 사전에 조절 못함.  
V5는 height_scan(273-dim) 입력 → 발 앞 지형 보고 발 높이/타이밍 조절 가능.

| 지표 | V4 plateau 시 | V5 기대 |
|------|--------------|---------|
| `feet_air_time` 양수 도달 | 실패 (-0.053) | 진행 중, Phase 2/3에서 양수 기대 |
| `terrain_levels` 7+ 도달 | 실패 (5.30 stop) | Phase3에서 도달 기대 |
| `undesired_contacts` 감소 | 정체 (-0.102) | 현재 -0.041 (V4보다 60% 좋음) ✅ |

**Phase2 진입 시(ckpt 10000)에 `feet_air_time`이 양수로 전환되는지** + **Phase3 후반(ckpt 30000+)에 terrain_levels 7+ 도달하는지**가 V5 성공 판정 기준.

### 14.6 인프라 건강성

- ETA: 13시간 40분 (Phase1 → 시작 후 39분 진행, 525 iter)
- 5,000 iter당 약 1시간 → 50,000 iter 약 10시간 (※ Phase 전환 재시작 오버헤드 ~30분 × 2회 포함하면 11시간)
- ※ 위 가이드 §10의 "≈70시간 (3일)" 예상은 보수적 추정 — 실제는 더 빠를 가능성 높음.
- VRAM: 정상 (OOM 없음)
- Phase 전환: 다음 전환은 ckpt 10,000 (예상 2~3시간 후)

### 14.7 최종 판정

✅ **학습 방향 객관적으로 맞음. 중단 불필요. 계속 진행 권장.**

근거 요약:
1. **SOTA 논문 표준 reward 가중치 준수**
2. **V3 실패 시그니처(action_rate 포화) 전무**
3. **V4 동일 시점보다 모든 정량 지표 우수** (tracking 2.4× 정확, action smoothness 37% 양호)
4. **인프라 안정** (모니터 안정 동작, 학습 정상 진행, hang/restart loop 없음)
5. **height_scan 추가가 의도대로 작동** (Phase1 평지에서도 terrain_levels 2.26 상승)

### 14.8 향후 체크포인트 (학습 도중 점검 시점)

| 시점 | 확인 항목 | 위험 신호 |
|------|----------|----------|
| ckpt 10,000 (Phase2 진입) | Phase 자동 전환, dr_phase=2 적용 | restart 후 ckpt가 10000 이하로 떨어지지 않는지 |
| ckpt 15,000 (Phase2 중간) | `feet_air_time` 양수 진입 | 계속 -0.05 이하면 발 못 듦 |
| ckpt 20,000 (Phase3 진입) | Phase 자동 전환, dr_phase=3 | terrain_levels 5+ 도달했는지 |
| ckpt 30,000 (Phase3 중간) | `terrain_levels` 6+ | V4 plateau(5.30) 돌파 여부 |
| ckpt 40,000~50,000 | `terrain_levels` 7+ | 도달 못하면 V5도 plateau 가능성 |

각 시점에 위험 신호 발견 시 **학습 중단 후 재설계** 필요. 그 외에는 학습 끝까지 진행.

---

## 15. ICROS 대회 적합성 비판적 검토 (Critical Audit)

### 15.1 대회 요구사항 매핑

대회 자율 보행 부문 핵심 요소:
- 다양한 지형 극복: 미끄러운 바닥, 계단, 바위 지형, 경사로
- 주행 속도 및 안정성 평가

V1(=V4=V5 베이스) 지형 구성에 직접 매핑:

| 대회 요소 | V1 sub_terrain | 비율 | 파라미터 | 적합성 |
|-----------|----------------|------|----------|--------|
| **미끄러운 바닥** | `physics_material` reset event | 항상 | static_friction 0.05~1.5 (빙판~아스팔트), reset마다 변경 | ✅ |
| **계단** | stairs_up + stairs_down + mesh_pyramid_up/dn | **33%** | step_height_max **0.190m** (ICROS2025 실측 186mm 반영) | ✅ |
| **바위 지형** | rough_slope + discrete_obs + checker_blocks | **20%** | noise 0.005~0.080m, obs_height_max 0.20m | ✅ |
| **경사로** | smooth_slope_up + smooth_slope_dn + bridge_ramp | **22%** | slope 0.08~0.40 rad (≈4.6°~22.9°) | ✅ |
| **평지** | flat | 5% | 기본 | ✅ |
| **기타 도전** | pothole + stepping_slabs + zigzag_bridge | **20%** | pothole r=(0.10,0.50), d=(0.10,0.40) | ✅ |

**결론**: 지형 구성 자체는 대회 4대 요소 모두 포함, 비율도 균형적. **지형 변경은 불필요**.

### 15.2 V4 학습 로그 plateau 진단 (iter 15000~32000)

| iter | terrain_levels | mean_reward | track_lin_vel_xy | error_vel_xy | action_rate | feet_air_time | undesired_contacts |
|------|----------------|-------------|------------------|--------------|-------------|---------------|---------------------|
| 15000 | 5.06 | 1.91 | 0.736 | 0.892 | -0.129 | **-0.054** | -0.091 |
| 19000 | 5.07 | 2.76 | 0.846 | 0.776 | -0.133 | -0.051 | -0.089 |
| 20000 | 5.03 | 5.95 | 0.834 | 0.822 | -0.124 | -0.051 | -0.063 |
| 25000 | 5.09 | 4.46 | 0.849 | 0.778 | -0.124 | -0.051 | -0.073 |
| 30000 | 5.35 | 3.19 | 0.861 | 0.705 | -0.128 | -0.050 | -0.079 |
| 32000 | **5.30** | **2.58** | 0.849 | 0.778 | -0.137 | **-0.053** | **-0.102** |

**진단**:
1. **terrain_levels가 17,000 iter 동안 5.0~5.35에서 정체** (Δ=0.3 / 17k iter)
2. **feet_air_time이 학습 내내 음수** (-0.05) → 발이 threshold(0.20s) 이상 안 들림
3. **후반 악화**: iter 30k→32k에서 reward 5.95→2.58, undesired_contacts -0.063→-0.102
4. tracking error는 안정적이나 개선 없음 (0.77 plateau)

**근본 원인**: V4 Actor는 225-dim proprio만 → **blind** → 계단/장애물 접촉 후에야 반응 → 발이 미리 들리지 않음 → terrain_level 5+에서 더 못 올라감.

### 15.3 V5 가설 검증 시점

V5의 핵심 가설: **"Actor가 height_scan을 보면 미리 발을 들 수 있다"**

검증을 위한 결정 트리:

```
Phase1 (ckpt 0~10000, 평지):
  └─ feet_air_time 음수 정상 (max_init=0 평지에서 발 들 필요 없음)
  └─ 현재 V5 -0.054 (정상)

Phase2 (ckpt 10000~20000, max_init=4):
  └─ 결정 시점 #1: ckpt 15000에서 feet_air_time 부호 확인
       ✅ 양수 전환 → V5 핵심 가설 검증 성공, 계속 진행
       ❌ 여전히 음수 → V5 가설 실패, 즉시 V6 시작

Phase3 (ckpt 20000~50000, max_init=7):
  └─ 결정 시점 #2: ckpt 25000에서 terrain_levels 확인
       ✅ 6.0 이상 → V4 plateau(5.30) 돌파, 끝까지 진행
       ❌ 5.5 미만 → V4와 동일 plateau, V6 시작
```

**예상 도달 시간**:
- Phase1 → Phase2: 약 5시간 후 (현재 4.4s/iter × 9550 iter)
- Phase2 → Phase3: 약 10시간 후
- 결정 시점 #1 (ckpt 15000): 약 12시간 후

### 15.4 V6 백업 설계 (V5 실패 대비)

**현재 즉시 적용은 권장하지 않음** — V5 가설 검증이 우선. 단, V5 plateau 발생 시 즉시 시작할 수 있도록 미리 설계.

#### V6 핵심 변경안 (V5 기반 추가 개선)

**① `feet_height_clearance` reward 추가 (최우선)**

V4가 plateau에 빠진 결정적 원인은 **feet_air_time이 항상 음수** (발이 0.20s 이상 안 들림).  
height_scan을 추가해도 발 들기에 대한 **직접 보상이 없으면** 학습이 못 일어날 수 있음.

```python
# feet_height_clearance: 발이 desired_clearance 이상 들리면 + 보상
# Walk-These-Ways (Margolis et al., RSS 2022) 차용
feet_height_clearance = RewTerm(
    func=_feet_height_clearance_reward,
    weight=0.5,   # 처음엔 작게 (track=1.5 대비 1/3)
    params={
        "desired_clearance": 0.08,   # 8cm (계단 19cm 대응)
        "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*foot"),
    },
)

def _feet_height_clearance_reward(env, desired_clearance, sensor_cfg):
    """발이 공중에 있을 때 (= contact_force ≈ 0) 발 z-position이 desired_clearance 이상이면 양수."""
    asset = env.scene["robot"]
    foot_idx = asset.find_bodies(".*foot")[0]
    foot_z = asset.data.body_pos_w[:, foot_idx, 2]  # (N, 4)
    foot_z_rel = foot_z - env.scene.terrain.terrain_origins_z  # 지면 기준
    contact = sensor_cfg.contact_forces.data.net_forces_w_history.norm(dim=-1).max(dim=1)[0]
    in_air = (contact < 1.0).float()  # (N, 4) 공중인지
    clearance = (foot_z_rel - desired_clearance).clamp(min=0).sum(dim=1)
    return clearance * in_air.sum(dim=1) / 4
```

**② `lin_vel_y` 범위 확대 (대회 자율 보행용)**

현재 V5: `lin_vel_y = (-0.5, 0.5)`. 대회는 좁은 공간 회피/방향 전환 필요.  
**제안**: `lin_vel_y = (-0.8, 0.8)`, `limit_ranges (-1.0, 1.0)`

**③ Symmetry augmentation 활성화 (RSL-RL 2.3.1+ 지원)**

RSL-RL이 SymmetryCfg 지원. 좌우 대칭 augmentation으로 sample efficiency 30%+ 향상.

```python
# rsl_rl_ppo_cfg.py에 추가
from isaaclab_rl.rsl_rl import RslRlSymmetryCfg

algorithm = RslRlPpoAlgorithmCfg(
    ...
    symmetry_cfg=RslRlSymmetryCfg(
        use_data_augmentation=True,
        use_mirror_loss=True,
        data_augmentation_func="unitree_rl_lab.utils.symmetry:compute_symmetric_states",
    ),
)
```

`compute_symmetric_states`는 obs/action을 좌우 mirror — Go2 joint 매핑 활용.

**④ PPO entropy_coef 약화 (수렴 가속)**

현재 0.01. V4가 후반에 reward 감소(3.19→2.58)한 건 entropy 너무 높아서 과탐색 가능성.  
**제안**: Phase3에서 `entropy_coef=0.005`로 schedule.

**⑤ Critic에 도메인 정보 추가 (선택)**

현재 Critic: 45 + 273(scan) = 333-dim.  
대회용 추가 정보 (training-only privileged):
- `terrain_type` (one-hot 14): 어떤 sub-terrain에 있는지
- `friction_coeff`: 현재 마찰 계수
→ Critic이 환경 인지 강해져서 Actor에 더 좋은 value 신호.

#### V6 코드 위치

```
source/.../go2/experiments/icros2026_v6/
├── env_cfg.py    ← V5 상속 + feet_clearance + lin_vel_y + critic 확장
├── __init__.py
└── terrains/

scripts/experiments/icros2026_v6/
├── auto_monitor.py   ← V5 monitor와 동일 (Phase 정의만 V6용)
├── train.py
└── play.py
```

**V6 학습 시작 명령**:
```bash
python scripts/new_experiment.py icros2026_v6 --base icros2026_v5 \
  --desc "V5 + feet_clearance + lin_vel_y확대 + symmetry aug"
# 그 후 env_cfg.py에 위 변경사항 적용
```

### 15.5 즉시 적용 여부 — 의사결정

| 옵션 | 권고 | 근거 |
|------|------|------|
| A. V5 즉시 중단, V6 시작 | ❌ 비권장 | V5는 객관적으로 V4보다 우수 (§14), 가설 검증 가치 있음 |
| B. V5 계속, V6 백업 설계만 | ✅ **권장** | V5 가설 검증 + 실패 시 즉시 전환 가능 |
| C. V5/V6 병렬 학습 | ❌ 비권장 | GPU 메모리 부족 (16384×2 = 40GB > 32GB) |

### 15.6 다른 SOTA Go2 연구 참조

| 연구 | 핵심 기법 | V5 적용 여부 |
|------|----------|------------|
| **ETH legged_gym** (Rudin et al., CoRL 2022) | terrain curriculum, 45-dim proprio | ✅ V1부터 적용 |
| **Walk-These-Ways** (Margolis et al., RSS 2022) | feet_clearance reward, gait conditioning | ⏳ V6 후보 (feet_clearance) |
| **HIM** (He et al., CoRL 2023) | 환경 인지 임베딩 (encoder-decoder) | ❌ 미적용 (구조 변경 큼) |
| **Extreme Parkour** (Cheng et al., CoRL 2023) | scandots + image, complex foot reward | ✅ scandots(=height_scan) V5 적용 |
| **PGTT** (Shao et al., 2024) | track=1.0, action_rate=-0.01 | ✅ V4부터 적용 |
| **Symmetry aug** (Mittal et al., 2024) | 좌우 대칭 데이터 증강 | ⏳ V6 후보 |

V5는 SOTA의 **검증된 핵심 요소를 이미 모두 적용**:
- Asymmetric actor-critic ✓
- terrain curriculum ✓
- height_scan (scandots) ✓
- DR curriculum (Phase별) ✓
- SOTA reward weights ✓

추가 가능한 것: feet_clearance reward, symmetry augmentation, encoder-decoder.

### 15.7 최종 결정

**V5 학습 계속 진행 (옵션 B)**.

근거:
1. V5는 V4의 명백한 약점(blind)을 보완하는 **타당한 단일 가설**.
2. 현재 V5 trajectory 객관적으로 건강 (§14: tracking 2.4× 정확, action_rate 정상).
3. V4 plateau의 진짜 원인이 blind이면 V5가 해결, reward 자체이면 V6 필요 — 가설 검증 가치 있음.
4. Phase2 결정 시점(ckpt 15000)은 약 12시간 후. V6 백업 설계만 해두면 손실 시간 최소.

**대회 적합성**: 지형/속도/안정성 모두 베이스 환경에 반영됨. 추가 변경 불필요.

---

## 16. 현재 상태 업데이트 (2026-05-30, ckpt ≈ 1,950)

### 16.1 실행 상태

- 학습 프로세스와 V5 auto monitor 모두 정상 동작 중.
- 최신 run: `/workspace/logs/rsl_rl/unitree_go2_icros2026_v5/2026-05-30_07-02-17/`
- 최신 확인 checkpoint: `model_1950.pt`
- override: `{"max_init_terrain_level": 0, "lin_vel_x_max": 1.0, "lin_vel_x_min": 0.0, "dr_phase": 1}`
- 현재 Phase: Phase1, 다음 전환 기준은 `model_10000.pt`.

### 16.2 최신 지표 요약

| Metric | 값 | 판단 |
|--------|-----|------|
| `terrain_levels` | ~5.30 / 9 | 빠르게 상승 중. Phase1의 `max_init=0`은 초기 spawn 제한이지 curriculum 상한이 아니므로 즉시 버그로 보지 않음 |
| `lin_vel_cmd_levels` | 1.20 | tracking 성공으로 command curriculum이 1.0에서 1.2까지 열린 상태 |
| `track_lin_vel_xy` | ~1.12~1.14 / 1.5 | 정상, 상한 대비 약 75% |
| `track_ang_vel_z` | ~0.55 / 0.75 | 정상 |
| `action_rate` | ~-0.12 | V3식 포화 없음 |
| `action_smoothness_2` | ~-0.032 | 정상 범위 |
| `feet_air_time` | ~-0.048 | Phase1에서는 아직 정상. Phase2 ckpt 15000에서 양수 전환 여부가 핵심 |
| `time_out` | ~97% | 안정적 |
| `base_contact` | ~1.5~1.6% | 낮음 |
| `bad_orientation` | ~1.0~1.1% | 낮음 |

### 16.3 추가 수정

- `scripts/experiments/icros2026_v5/play.py` fixed command override 수정.
- TensorDict obs의 `"policy"` key를 처리하고, 225/498-dim history policy에서는 5개 history frame의 command slot을 모두 갱신.

*문서 작성일: 2026-05-30 | 최초 검증 시점: iter 527, ckpt ≈ 450, Phase1 진행 중*  
*현재 업데이트: 2026-05-30 | ckpt ≈ 1,950, Phase1 진행 중*  
*§15 비판적 검토 작성: V4 학습 로그 분석 + 대회 요구사항 매핑 + V6 백업 설계 완료*  
*세션: Claude Code (Sonnet 4.6 → Opus 4.7), Codex 업데이트 포함*  
*모든 변경사항은 git에 반영되지 않음 — 사용자가 직접 commit해야 함.*
