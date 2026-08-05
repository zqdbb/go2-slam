# Experiment: icros2025

## 개요
- **목표**: ICROS 2025 대회 Go2 보행 정책 학습
- **환경 ID**: `Unitree-Go2-Competition`
- **시작일**: 2026-05-23
- **상태**: Phase 2까지 진행 (19,200 iter), terrain_level 최고 2.41

---

## 파일 구성

| 파일 | 원본 대비 변경 | 설명 |
|------|--------------|------|
| `play.py` | ✏️ 수정 | `--lin_vel_x/y`, `--ang_vel_z` 고정 속도 명령 추가 |
| `train.py` | ✏️ 수정 | `handle_deprecated_rsl_rl_cfg` 적용, `--task` 자유 문자열 |

**환경 설정** (별도 위치):
- `source/.../go2/experiments/icros2025/env_cfg.py` — 11종 지형, 16384 envs
- `source/.../go2/experiments/icros2025/terrains/` — legged_gym 포팅 커스텀 지형

---

## 실행 방법

```bash
# 학습 (Docker 내부)
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2025/train.py \
  --task Unitree-Go2-Competition --headless

# 플레이 (고정 속도 명령)
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2025/play.py \
  --task Unitree-Go2-Competition --lin_vel_x 1.0 --num_envs 8
```

---

## play.py 주요 변경점

### 추가된 CLI 인자
```
--lin_vel_x   고정 전진 속도 (m/s)
--lin_vel_y   고정 측면 속도 (m/s)
--ang_vel_z   고정 yaw 속도 (rad/s)
```

### override_cmd_and_obs() 패치
GUI 플레이 시 여러 로봇이 동시에 같은 속도 명령을 받도록 수정.
- command_manager tensor와 obs tensor(인덱스 6,7,8)를 동시에 패치
- `obs_tensor.clone()` 로 inference tensor in-place 수정 문제 해결

### API 호환성 패치
- `runner.alg.actor` → `.policy` → `.actor_critic` 3단계 fallback
- `export_policy_as_jit/onnx` 실패 시 경고만 출력 (학습 중단 방지)
- `get_observations()` API 버전 차이 처리

---

## train.py 주요 변경점
- 모듈 import 시 gym.registry 조회 제거 → `--task` 자유 문자열 입력 가능
- `handle_deprecated_rsl_rl_cfg` 적용

---

## 학습 결과 요약

| Iter | terrain_level | Vel Reward | 비고 |
|------|--------------|-----------|------|
| 1,000 | 1.33 / 9 | 2.046 | Phase 1 초기 |
| 5,000 | 2.41 / 9 | 2.052 | **최고점** |
| 10,000 | 1.86 / 9 | 1.969 | Phase 2 전환 |
| 19,200 | 1.43 / 9 | 1.870 | 학습 종료 |

**최종 체크포인트**: `logs/rsl_rl/unitree_go2_competition/2026-05-23_04-36-36/model_19200.pt`

---

## 미해결 문제 및 다음 실험 제안
- terrain_level 정체: `terrain_levels_vel` 커리큘럼이 속도 추종 기준으로 승진 판단 → 고난도 지형 미진입
- 제안: 커리큘럼 성공 기준 완화 또는 anti-curriculum 혼합
- 자세한 내용: [docs/training_issues_log.md](../../../docs/training_issues_log.md)
