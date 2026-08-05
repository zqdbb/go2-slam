# Claude Code Instructions — unitree_rl_lab

## Project Overview
- Unitree Go2 quadruped RL training with IsaacLab + RSL-RL PPO
- Goal: Sim2Real locomotion for ICROS 2025 and beyond
- Docker-based: all training runs inside `isaac-lab` container via `/isaac-sim/python.sh`

---

## CRITICAL: Experiment Management Rules

**NEVER create experiment files directly.** Always scaffold first:

```bash
python scripts/new_experiment.py <name> [--base icros2025] [--desc "..."] [--env-id Unitree-Go2-X]
```

This auto-creates the full structure AND registers the gym environment.
Only then modify the generated files.

### What counts as a "new experiment"
- New environment config (different rewards, terrains, obs space)
- New policy architecture or training hyperparameters
- New domain randomization settings
- Any change that would make checkpoints incompatible with other experiments

### What does NOT need a new experiment
- Bug fixes within an existing experiment
- README/comment updates
- Minor numerical tweaks being iterated quickly (document in existing README)

---

## File Protection Rules

| File | Rule |
|------|------|
| `scripts/rsl_rl/play.py` | **READ-ONLY** — original, never modify |
| `scripts/rsl_rl/train.py` | **READ-ONLY** — original, never modify |
| `source/.../velocity_env_cfg.py` | **READ-ONLY** — original base env |
| `scripts/experiments/<name>/` | Editable — experiment-specific scripts |
| `source/.../go2/experiments/<name>/` | Editable — experiment-specific env |
| `source/.../go2/__init__.py` | Only add `gym.register()` blocks, never remove |

---

## Directory Structure

```
unitree_rl_lab/
├── EXPERIMENTS.md                    ← experiment index
├── scripts/
│   ├── new_experiment.py             ← scaffold tool (run this first!)
│   ├── rsl_rl/                       ← ORIGINAL scripts (do not modify)
│   └── experiments/<name>/           ← custom scripts per experiment
└── source/.../go2/
    ├── velocity_env_cfg.py           ← ORIGINAL base env (do not modify)
    └── experiments/<name>/           ← custom env configs per experiment
        ├── env_cfg.py
        └── terrains/
```

---

## Docker Commands

```bash
# Training
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/<name>/train.py \
  --task <env-id> --headless

# Play
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/<name>/play.py \
  --task <env-id> --num_envs 8

# List registered environments
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/list_envs.py
```

---

## Gym Environment Registration

All environments are registered in:
`source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/__init__.py`

Pattern:
```python
gym.register(
    id="Unitree-Go2-<ExperimentName>",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.experiments.<name>.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.experiments.<name>.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)
```

`new_experiment.py` adds this automatically — do not add manually.

---

## Key Technical Facts

- **Python path**: `/isaac-sim/python.sh` (NOT `python` or `python3`)
- **Phase override**: `/tmp/training_override.json` → `{"max_init_terrain_level": N}`
- **Auto monitor**: `scripts/auto_monitor_competition.py` (phase-based curriculum)
- **Checkpoint sort**: use `sort -V` or numeric extraction (not `ls | sort`)
- **VRAM**: RTX 5090 32GB → 16384 envs ≈ 20GB
- **obs layout (policy)**: `[ang_vel(0:3), gravity(3:6), cmd(6:9), joint_pos(9:21), joint_vel(21:33), action(33:45)]`
- **Actor**: 45-dim proprioceptive only (SLAM-deployable)
- **Critic**: 45 + height_scan dims (privileged, train only)
