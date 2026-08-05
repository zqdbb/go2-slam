# Experiment: icros2026_v5

## 개요
- **설명**: V4+height_scan_in_actor: 498-dim policy=proprio225+scan273, checkpoint-based curriculum, 16384envs, 50kiter
- **환경 ID**: `Unitree-Go2-ICROS2026-V5`
- **생성일**: 2026-05-30
- **기반 실험**: `icros2025`
- **상태**: 🔄 진행 중

---

## 파일 구성

| 파일 | 위치 | 설명 |
|------|------|------|
| `play.py` | `scripts/experiments/icros2026_v5/` | 커스텀 play 스크립트 |
| `train.py` | `scripts/experiments/icros2026_v5/` | 커스텀 train 스크립트 |
| `env_cfg.py` | `source/.../go2/experiments/icros2026_v5/` | 환경 설정 |
| `terrains/` | `source/.../go2/experiments/icros2026_v5/` | 커스텀 지형 (선택) |

---

## 실행 방법

```bash
# 학습 (Docker 내부)
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py \
  --task Unitree-Go2-ICROS2026-V5 --headless

# 플레이
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/play.py \
  --task Unitree-Go2-ICROS2026-V5 --num_envs 8

# 고정 속도 명령으로 플레이
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/play.py \
  --task Unitree-Go2-ICROS2026-V5 --lin_vel_x 1.0 --num_envs 8
```

---

## 원본 대비 변경 사항

### env_cfg.py
- V4 환경을 상속하고 policy observation에 `height_scan`을 추가했습니다.
- Policy actor 입력은 `proprio_history(225) + height_scan(273) = 498-dim`입니다.
- Critic observation과 reward는 V4를 유지합니다. 핵심 reward는 `track_lin_vel_xy=1.5`, `track_ang_vel_z=0.75`, `action_rate=-0.01`, `action_smoothness_2=-0.001`입니다.
- Phase override는 `/tmp/training_override.json`에서 import 시점에 읽습니다. Phase 전환 시 학습 프로세스를 재시작해야 합니다.

### play.py / train.py
- 기반: `scripts/experiments/icros2025/` 복사
- `train.py`: V5 task 전용 학습 스크립트입니다. 기본 PPO 설정은 `max_iterations=50000`, `save_interval=50`을 사용합니다.
- `play.py`: fixed command override가 TensorDict obs와 5-frame history layout을 처리합니다. 225/498-dim policy에서는 각 history frame의 command slot `[k*45+6:k*45+9]`를 모두 갱신합니다.
- `auto_monitor.py`: checkpoint 번호 기반으로 Phase1/2/3를 전환합니다. `dr_phase`를 포함한 override 4개 키를 항상 기록합니다.

---

## 학습 결과

| Iter | terrain_level | Vel Reward | 비고 |
|------|--------------|-----------|------|
| ~1,950 | ~5.30 / 9 | `track_lin_vel_xy` ~1.13 | Phase1 진행 중, command curriculum 1.2까지 상승 |

**현재 체크포인트**: `logs/rsl_rl/unitree_go2_icros2026_v5/2026-05-30_07-02-17/model_1950.pt`

**최종 체크포인트**: `model_50000.pt` 목표
