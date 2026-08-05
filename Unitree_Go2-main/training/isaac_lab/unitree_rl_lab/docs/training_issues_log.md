# ICROS 2025 Go2 Competition 학습 문제점 정리

> 작성일: 2026-05-23  
> 환경: IsaacLab + RSL-RL PPO, Unitree Go2, RTX 5090 32GB  
> 태스크: `Unitree-Go2-Competition` (16384 envs, 30,000 iter 목표)

---

## ✅ 해결된 문제

---

### 1. `max_init_terrain_level=2` → terrain_levels 하락

| 항목 | 내용 |
|---|---|
| **증상** | terrain_levels가 0.77까지 하락, 로봇이 어려운 지형에서 계속 쓰러짐 |
| **원인** | 레벨 2 지형(경사·계단)부터 스폰 → 초기 랜덤 정책이 감당 불가 |
| **해결** | `max_init_terrain_level=0` → 반드시 평지(레벨 0)부터 시작, 커리큘럼이 점진적으로 올려줌 |
| **파일** | `competition_env_cfg.py` → `RobotSceneCfg.terrain` |

---

### 2. `action_rate` 페널티 너무 강함 → policy 붕괴

| 항목 | 내용 |
|---|---|
| **증상** | weight -0.3으로 올렸더니 학습 붕괴, 로봇이 아무 동작도 안 함 |
| **원인** | action_rate 페널티가 너무 강해 탐색을 막음 |
| **해결** | weight -0.15 (기존 -0.1의 1.5배, gentle 억제) |
| **파일** | `competition_env_cfg.py` → `RewardsCfg.action_rate` |

---

### 3. GUI play 시 로봇 1마리만 움직임

| 항목 | 내용 |
|---|---|
| **증상** | 8마리 스폰했지만 1마리만 직진, 나머지는 멈춤 |
| **원인** | ① step 후 command를 덮어씀 (순서 문제), ② inference tensor in-place 수정 불가 |
| **해결** | `obs_tensor = obs_tensor.clone()` 후 수정, command manager tensor와 obs tensor 동시 패치 |
| **파일** | `scripts/rsl_rl/play.py` (임시 패치) |

---

### 4. terrain 비율 합계 1.01

| 항목 | 내용 |
|---|---|
| **증상** | 11개 지형 proportion 합이 1.01 |
| **원인** | 계산 실수 (`checker_blocks=0.04` 설정) |
| **해결** | `checker_blocks 0.04 → 0.03`, 합계 = 1.00 |
| **파일** | `competition_env_cfg.py` → `COMPETITION_TERRAIN_CFG` |

---

### 5. `HfPotholeTerrainCfg` unused import (F401)

| 항목 | 내용 |
|---|---|
| **증상** | pothole 지형 제거 후 import만 남아 경고 발생 |
| **해결** | import 라인에서 `HfPotholeTerrainCfg` 제거 |
| **파일** | `competition_env_cfg.py` |

---

### 6. Docker 컨테이너 내부 Python 경로

| 항목 | 내용 |
|---|---|
| **증상** | `python`, `python3` 명령어 없음, `ModuleNotFoundError: toml` |
| **원인** | IsaacLab 컨테이너는 표준 Python 경로 미사용 |
| **해결** | 반드시 `/isaac-sim/python.sh` 래퍼 사용 (환경 설정 포함) |
| **적용 위치** | 모든 `docker exec` 훈련/플레이 명령 |

---

### 7. auto_monitor — Python 경로 오류 → `toml` 모듈 없음

| 항목 | 내용 |
|---|---|
| **증상** | Phase 2 전환 후 재시작 실패, `ModuleNotFoundError: No module named 'toml'` |
| **원인** | `start_training()`에서 `/isaac-sim/kit/python/bin/python3` 직접 호출 |
| **해결** | `/isaac-sim/python.sh`로 변경 |
| **파일** | `scripts/auto_monitor_competition.py` → `start_training()` |

---

### 8. auto_monitor — checkpoint 절대경로 오류

| 항목 | 내용 |
|---|---|
| **증상** | `ValueError: No checkpoints match '/workspace/.../model_10000.pt'` |
| **원인** | `get_checkpoint_path()`가 절대경로를 regex 패턴으로 처리 → 매칭 실패 |
| **해결** | `--load_run <타임스탬프디렉토리> --checkpoint <파일명만>` 형식으로 분리 |
| **파일** | `scripts/auto_monitor_competition.py` → `get_latest_checkpoint()`, `start_training()` |

---

### 9. auto_monitor — terrain_levels ×9 스케일 버그

| 항목 | 내용 |
|---|---|
| **증상** | 리포트에 `Level 21.6 / 9` 출력, 개입 로직이 "고난도 정체"로 오판 → 개입 안 함 |
| **원인** | metric이 이미 raw 0~9 값인데 ×9 곱함 |
| **해결** | raw 값 그대로 사용, 표시도 `{terrain:.2f} / 9`로 수정 |
| **파일** | `scripts/auto_monitor_competition.py` → `print_report()`, `decide_intervention()` |

---

### 10. auto_monitor — 재시작 시 Phase 중복 트리거

| 항목 | 내용 |
|---|---|
| **증상** | monitor 재시작할 때마다 Phase 2가 반복 트리거 → 훈련 불필요하게 재시작 |
| **원인** | 재시작 시 `current_phase_idx=0` 초기화 → iter 10k+ 감지 즉시 Phase 2로 전환 |
| **해결** | 시작 시 `/tmp/training_override.json` 읽어 현재 Phase 복원 |
| **파일** | `scripts/auto_monitor_competition.py` → `main()` 초기화 블록 |

---

### 11. auto_monitor — `NameError: name 'checkpoint' is not defined`

| 항목 | 내용 |
|---|---|
| **증상** | Phase 전환 중 monitor 크래시 |
| **원인** | `start_training()` 파라미터를 `run_dir/ckpt_name`으로 바꿨는데 로그 라인에 `checkpoint` 변수명 그대로 남음 |
| **해결** | 로그 라인 변수명 `ckpt_name`으로 수정 |
| **파일** | `scripts/auto_monitor_competition.py` → `start_training()` 로그 라인 |

---

### 12. `joint_pos` weight 너무 강함

| 항목 | 내용 |
|---|---|
| **증상** | 로봇이 기본 자세에서 벗어나지 못해 지형 적응 방해 |
| **원인** | weight -0.7 → 기본 자세 강제가 obstacle traversal과 충돌 |
| **해결** | weight -0.2로 완화 |
| **파일** | `competition_env_cfg.py` → `RewardsCfg.joint_pos` |

---

### 13. `flat_orientation_l2` 너무 강함

| 항목 | 내용 |
|---|---|
| **증상** | 경사면에서 약간 기울어져도 강한 페널티 → 경사 오르기 방해 |
| **원인** | weight -2.5 → 경사 적응 불가 수준 |
| **해결** | weight -1.0으로 완화 |
| **파일** | `competition_env_cfg.py` → `RewardsCfg.flat_orientation_l2` |

---

## ⚠️ 현재 미해결 문제

---

### 🔴 [핵심] terrain_level 정체 및 하락

| 항목 | 내용 |
|---|---|
| **증상** | 19,000 iter 학습 후 terrain_level이 **2.41 → 1.43으로 오히려 하락** |
| **원인** | `terrain_levels_vel` 커리큘럼이 **속도 추종** 성공 기준 사용. 속도 오차 1.23 m/s로 커서 승진 기준 미충족 → 계속 강등 |
| **결과** | 19,000 iter 동안 레벨 1~2에만 머뭄. 계단 11cm, 경사 20° 등 고난도 지형 실제 학습 못 함 |
| **미해결** | Phase 3 전환(iter 20k) 전에 학습 종료 |
| **대응 방향** | ① 커리큘럼 성공 기준 완화 (속도 threshold 낮추기), ② `vel_cmd` 범위 낮추거나 단계적으로 올리기, ③ anti-curriculum (처음부터 모든 레벨 랜덤 스폰) 고려 |

---

### 🟡 auto_monitor 5000-iter 보고 누락

| 항목 | 내용 |
|---|---|
| **증상** | monitor 재시작 반복으로 `last_analysis_iter` 리셋 → 5000 iter 보고 타이밍 어긋남 |
| **원인** | monitor 상태(last_analysis_iter)를 메모리에만 보관, 재시작 시 초기화 |
| **미해결** | 상태 파일(/tmp/monitor_state.json 등)에 영속화 미구현 |

---

### 🟡 checkpoint 파일 알파벳 정렬 문제

| 항목 | 내용 |
|---|---|
| **증상** | `ls model_*.pt \| tail -5`가 `model_10000.pt`를 누락 ("1" < "9" 알파벳 정렬) |
| **원인** | 숫자 자릿수 다를 때 알파벳 정렬은 숫자 순서와 다름 |
| **미해결** | `get_latest_checkpoint()`에서 `sort -V` (version sort) 또는 숫자 추출 정렬 미적용 |
| **임시 방법** | `ls ... \| sort -V \| tail -3` 으로 수동 확인 |

---

## 📊 학습 진행 요약

| Iter | terrain_level | Vel Reward | Timeout | 비고 |
|---|---|---|---|---|
| 1,000 | 1.33 / 9 | 2.046 | 94.4% | Phase 1 초기 |
| 5,000 | 2.41 / 9 | 2.052 | 92.4% | 최고점 |
| 10,000 | 1.86 / 9 | 1.969 | 94.4% | Phase 2 전환 |
| 15,000 | 1.82 / 9 | 1.970 | 94.9% | |
| 19,200 | 1.43 / 9 | 1.870 | 95.2% | 학습 종료 |

> **최종 체크포인트**: `logs/rsl_rl/unitree_go2_competition/2026-05-23_04-36-36/model_19200.pt`

---

## 💡 다음 학습 시 권장 사항

1. **커리큘럼 성공 기준 완화**
   - `terrain_levels_vel` 함수의 속도 threshold 낮추기 (현재 기준이 너무 엄격)
   - 또는 속도 명령 범위 낮추기 (`lin_vel_x: 0~1.5 m/s`로 시작 후 커리큘럼 진행)

2. **Anti-curriculum 혼합**
   - 처음부터 일정 비율(20~30%)의 환경을 높은 레벨에 랜덤 스폰
   - 어려운 지형 데이터를 초반부터 확보

3. **auto_monitor 상태 영속화**
   - `last_analysis_iter`, `current_phase_idx`를 파일에 저장
   - 재시작 시 복원 → 보고/개입 로직 일관성 유지

4. **checkpoint 정렬 수정**
   - `get_latest_checkpoint()`에서 숫자 추출 정렬 적용
   ```python
   files.sort(key=lambda f: int(re.search(r'model_(\d+)\.pt', f).group(1)))
   ```
