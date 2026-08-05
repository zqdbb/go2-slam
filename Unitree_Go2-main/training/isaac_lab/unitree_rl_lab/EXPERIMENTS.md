# Experiments 인덱스

이 레포는 `unitree_rl_lab` 원본을 베이스로, 실험별로 커스텀 파일을 분리 관리합니다.

---

## 📁 구조 설명

```
unitree_rl_lab/
├── scripts/
│   ├── rsl_rl/               ← ✅ 원본 그대로 (건드리지 마세요)
│   │   ├── play.py
│   │   └── train.py
│   ├── new_experiment.py     ← 🛠️ 새 실험 생성 자동화 도구
│   └── experiments/          ← 커스텀 스크립트
│       └── <실험명>/
│           ├── play.py
│           ├── train.py
│           └── README.md
│
└── source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/
    ├── __init__.py           ← 모든 실험 환경 등록 (여기에만 gym.register 추가)
    ├── velocity_env_cfg.py   ← ✅ 원본 그대로
    └── experiments/          ← 커스텀 환경 설정
        └── <실험명>/
            ├── __init__.py
            ├── env_cfg.py
            └── terrains/
                ├── __init__.py
                └── custom_terrains.py
```

---

## 🧪 실험 목록

| 실험명 | 환경 ID | 설명 | 상태 |
|--------|---------|------|------|
| [icros2025](scripts/experiments/icros2025/README.md) | `Unitree-Go2-Competition` | ICROS 2025 대회용, 11종 지형, 16384 envs | ✅ 완료 (19,200 iter) |
| [icros2026](scripts/experiments/icros2026/README.md) | `Unitree-Go2-ICROS2026` | ICROS 2026 대회용 — 실제 맵 분석 기반, 커리큘럼 개선, Sim2Real 강화 | 🔄 진행 중 |
| [icros2026_scan](scripts/experiments/icros2026_scan/README.md) | `Unitree-Go2-ICROS2026-Scan` | height_scan in actor (297-dim), improved feet_air_time & action_rate for obstacle traversal | 🔄 진행 중 |
| [icros2026_v4](scripts/experiments/icros2026_v4/README.md) | `Unitree-Go2-ICROS2026-V4` | reward 전면 재설계: 논문 표준 적용 (tracking=1.5, action_rate=-0.01, smoothness-2, PGTT swing penalty) | 🔄 진행 중 |
| [icros2026_v5](scripts/experiments/icros2026_v5/README.md) | `Unitree-Go2-ICROS2026-V5` | **V4+height_scan_in_actor (498-dim): proprio_history(225)+scan(273), checkpoint-based phase curriculum, 16384 envs.** 36k 기준 terrain 5.5 plateau 확인, 40k baseline까지 수집 | 🔄 학습 중 (Phase 3, 40k stop 예정) |
| [icros2026_v6_perceptive_real](scripts/experiments/icros2026_v6_perceptive_real/README.md) | `Unitree-Go2-ICROS2026-V6-PerceptiveReal` | V5 plateau 분석 기반 scratch 재학습: level 8+ 목표, feet_air_time/terrain exposure/Sim2Real height_scan corruption 보수 조정 | 🔄 준비 중 |
<!-- EXPERIMENTS_TABLE_END -->

---

## ➕ 새 실험 추가 방법 (자동화)

> **⚠️ 중요**: 파일을 직접 만들지 말고 반드시 `new_experiment.py`를 먼저 실행하세요.

```bash
# 기본 사용
python scripts/new_experiment.py <이름>

# 예시
python scripts/new_experiment.py v2_curriculum
python scripts/new_experiment.py distillation --base icros2025 --desc "Teacher-Student distillation"
python scripts/new_experiment.py v3_anticurriculum --env-id Unitree-Go2-AntiCurriculum
```

한 번 실행하면 자동으로:
- `scripts/experiments/<이름>/play.py, train.py, README.md` 생성
- `source/.../go2/experiments/<이름>/env_cfg.py, terrains/` 생성
- `go2/__init__.py`에 `gym.register()` 추가
- 이 파일(EXPERIMENTS.md)의 실험 목록 업데이트

---

## ⚠️ 규칙

1. `scripts/rsl_rl/play.py`, `train.py` — **절대 수정 금지** (원본 그대로)
2. 커스텀 수정은 반드시 `scripts/experiments/<실험명>/` 에서만
3. 환경 등록은 `go2/__init__.py` 에만 (파일 분산 방지)
4. **새 실험 시작 = `new_experiment.py` 실행** (직접 파일 생성 금지)
5. 각 실험 폴더의 `README.md`에 변경 내용과 결과 반드시 기록
