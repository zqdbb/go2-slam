import gymnasium as gym

gym.register(
    id="Unitree-Go2-Velocity",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.velocity_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# ---------------------------------------------------------------------------
# Experiment environments
# 새 실험 추가 시 여기에 gym.register() 블록을 추가하세요.
# env_cfg_entry_point 경로: {__name__}.experiments.<실험명>.env_cfg:<클래스명>
# ---------------------------------------------------------------------------

# [icros2025] ICROS 2025 대회용 Go2 환경 — 11가지 커스텀 지형, 16384 envs
gym.register(
    id="Unitree-Go2-Competition",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2025.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2025.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# [test_scaffold] Unitree-Go2-TestScaffold
gym.register(
    id="Unitree-Go2-TestScaffold",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.test_scaffold.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.test_scaffold.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# ─────────────────────────────────────────────────────────────────────────────
# ICROS 2026 Competition Series
# ─────────────────────────────────────────────────────────────────────────────

# [icros2026_v1] 45-dim proprioceptive actor — 베이스 환경 (scan 없음)
# 체크포인트: 없음 (베이스 전용, v2/v3의 부모)
gym.register(
    id="Unitree-Go2-ICROS2026-V1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v1.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v1.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# [icros2026_v2] 318-dim (45 proprioceptive + 273 height_scan) actor
# 문제: feet_air_time threshold=0.35 (2.0m/s에서 penalty), terrain plateau 5.4
# 체크포인트: model_25000.pt (unitree_go2_icros2026_v2 또는 icros2026_scan)
gym.register(
    id="Unitree-Go2-ICROS2026-V2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v2.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v2.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# [icros2026_v3] 318-dim — 커리큘럼/보상 수정판 ★ 현재 실험
# 수정: feet_air_time 0.35→0.20, terrain move_up 비례 기준, Phase3 vel 1.5 시작
gym.register(
    id="Unitree-Go2-ICROS2026-V3",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v3.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v3.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# ── 구버전 ID 호환 (기존 체크포인트 play용) ──────────────────────────────────
gym.register(
    id="Unitree-Go2-ICROS2026",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v1.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v1.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)
gym.register(
    id="Unitree-Go2-ICROS2026-Scan",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v2.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v2.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# [icros2026_v4] Unitree-Go2-ICROS2026-V4
gym.register(
    id="Unitree-Go2-ICROS2026-V4",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v4.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v4.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# [icros2026_v5] Unitree-Go2-ICROS2026-V5
gym.register(
    id="Unitree-Go2-ICROS2026-V5",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v5.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v5.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)

# [icros2026_v6_perceptive_real] Unitree-Go2-ICROS2026-V6-PerceptiveReal
gym.register(
    id="Unitree-Go2-ICROS2026-V6-PerceptiveReal",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.icros2026_v6_perceptive_real.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.icros2026_v6_perceptive_real.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)
