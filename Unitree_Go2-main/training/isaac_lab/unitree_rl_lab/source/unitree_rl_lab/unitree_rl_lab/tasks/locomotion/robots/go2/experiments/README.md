# Experiment Environments

각 실험별 커스텀 환경 설정이 여기에 있습니다.
원본 환경은 `../velocity_env_cfg.py` (수정 금지).

## 실험 목록
| 폴더 | 환경 ID | 상태 |
|------|---------|------|
| [icros2025/](icros2025/) | `Unitree-Go2-Competition` | ✅ 완료 |

## 각 실험 폴더 구조
```
<실험명>/
├── __init__.py
├── env_cfg.py          ← 메인 환경 설정 (RobotEnvCfg, RobotPlayEnvCfg)
└── terrains/           ← 커스텀 지형 (있을 경우)
    ├── __init__.py
    └── custom_terrains.py
```

→ 새 실험 추가 방법은 [EXPERIMENTS.md](../../../../../../EXPERIMENTS.md) 참고
