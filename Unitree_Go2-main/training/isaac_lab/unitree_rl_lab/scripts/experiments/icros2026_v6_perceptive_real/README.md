# Experiment: icros2026_v6_perceptive_real

## 개요
- **설명**: V5 plateau 분석을 반영해 처음부터 다시 학습하는 level 8+ hard-terrain 모델
- **환경 ID**: `Unitree-Go2-ICROS2026-V6-PerceptiveReal`
- **생성일**: 2026-05-30
- **기반 실험**: `icros2026_v5`
- **상태**: 🔄 학습 중 — V5 40k baseline 완료 후 V6 scratch 50k 학습

---

## 판단

V5는 안정성 자체는 좋다. 36k 근처에서 timeout은 약 97-98%인데,
terrain level은 5.5 근처에서 정체했고 `/cmd_vel` 추종 오차도 남아 있다.
즉 문제는 “넘어짐”보다는 고난도 row 진입/통과와 발 스윙 품질이다.

V6는 V5 checkpoint fine-tuning이 아니라 scratch 학습을 기본 경로로 둔다.
이유는 V5의 낮은 발 스윙/terrain 5.5 plateau가 이미 policy 습관으로 굳었을
가능성이 있어서, 보상과 curriculum이 달라진 V6에서는 처음부터 새로 학습하는
편이 local optimum을 깰 가능성이 높기 때문이다. V5 40k는 baseline/eval용이다.

총 학습 길이는 50k iter로 둔다. 처음 70k로 잡았던 값은 V5 checkpoint에서
이어받아 hard-terrain push를 길게 가져가는 이전 계획의 잔재였고, scratch V6
목적에는 과하다. Phase는 0/8k/16k/28k/38k/45k로 압축한다.

V6는 V5의 핵심 구조를 유지한다.

```text
Actor obs = proprio_history(225) + height_scan(273) = 498 dim
```

그리고 다음을 보수적으로 바꾼다.

| 변경 | 목적 |
|------|------|
| hard terrain min-level phase | 평균 terrain level을 가짜로 올리는 게 아니라 level 5+ 이상 노출 시간을 늘림 |
| V5 Phase3보다 낮은 속도 압박 | 고난도 지형에서 속도 추종보다 통과 안정성 우선 |
| `feet_air_time` threshold 0.20s → 0.14s | 짧고 현실적인 장애물 스텝도 보상되게 함 |
| pothole depth/radius 상한 완화 | 40cm급 과도한 구멍이 curriculum을 지배하지 않게 함 |
| mild height_scan corruption | FAST-LIO/SLAM 기반 `/rl/height_scan` sim2real 차이 일부 반영 |

주의: min-level은 성공 지표가 아니라 노출 제어다. V6 성공 여부는
terrain level과 함께 timeout, base_contact, bad_orientation, velocity error를 같이 본다.

## 파일 구성

| 파일 | 위치 | 설명 |
|------|------|------|
| `play.py` | `scripts/experiments/icros2026_v6_perceptive_real/` | 커스텀 play 스크립트 |
| `train.py` | `scripts/experiments/icros2026_v6_perceptive_real/` | 커스텀 train 스크립트 |
| `auto_monitor.py` | `scripts/experiments/icros2026_v6_perceptive_real/` | V6용 checkpoint 기반 phase monitor |
| `env_cfg.py` | `source/.../go2/experiments/icros2026_v6_perceptive_real/` | 환경 설정 |
| `terrains/` | `source/.../go2/experiments/icros2026_v6_perceptive_real/` | 커스텀 지형 (선택) |

---

## 실행 방법

```bash
# 학습 (Docker 내부)
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v6_perceptive_real/train.py \
  --task Unitree-Go2-ICROS2026-V6-PerceptiveReal --headless --num_envs 16384

# 플레이
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v6_perceptive_real/play.py \
  --task Unitree-Go2-ICROS2026-V6-PerceptiveReal --num_envs 8

# 고정 속도 명령으로 플레이
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v6_perceptive_real/play.py \
  --task Unitree-Go2-ICROS2026-V6-PerceptiveReal --lin_vel_x 1.0 --num_envs 8
```

Monitor로 실행:

```bash
python3 scripts/experiments/icros2026_v6_perceptive_real/auto_monitor.py
```

V5 40k baseline이 끝난 뒤 V6 scratch를 자동 시작:

```bash
setsid scripts/experiments/icros2026_v6_perceptive_real/start_scratch_after_v5.sh \
  >> /tmp/start_v6_scratch_after_v5.log 2>&1 < /dev/null &
```

V5 40k checkpoint를 V6 seed로 쓰는 것은 비교 실험 옵션이다:

```bash
scripts/experiments/icros2026_v6_perceptive_real/handoff_from_v5.sh
```

---

## 원본 대비 변경 사항

### env_cfg.py
- V5 전체 상속.
- `PolicyCfg.height_scan`을 mild `_RealisticHeightScanTerm`으로 교체.
- terrain curriculum을 `terrain_levels_vel_v6`로 교체해서 phase별 min-level/move-up threshold 지원.
- `feet_air_time` threshold/weight와 `air_time_variance`를 V5 plateau 지표에 맞게 조정.
- pothole range를 competition scale에 더 가깝게 제한.

### play.py / train.py
- 기반: `scripts/experiments/icros2026_v5/` 복사
- `play.py`는 V5의 TensorDict/498-dim command override fix를 그대로 사용.
- `auto_monitor.py`는 V6 task/log path로 분리.

---

## 학습 결과

| Iter | terrain_level | Vel Reward | 비고 |
|------|--------------|-----------|------|
| - | - | - | scratch 학습 대기 |

**최종 체크포인트**: (학습 후 기록)
