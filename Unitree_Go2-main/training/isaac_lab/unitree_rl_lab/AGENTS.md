# AI Agent Instructions — unitree_rl_lab

This file is read by AI coding assistants (ChatGPT/Codex, Gemini, Cursor, Copilot, etc.)
to understand the project conventions. Follow these rules strictly.

---

## Project Purpose
Unitree Go2 quadruped robot RL training (IsaacLab + RSL-RL PPO).
Goal: robust locomotion policy for Sim2Real deployment at ICROS 2025 competition.

---

## RULE #1: Always Scaffold New Experiments

Before creating any new environment config, policy, or training script:

```bash
python scripts/new_experiment.py <name>
```

Options:
- `--base <existing_experiment>` : copy scripts from this experiment (default: icros2025)
- `--desc "description"` : one-line experiment description
- `--env-id Unitree-Go2-X` : custom gym env ID (auto-generated if omitted)

Example:
```bash
python scripts/new_experiment.py v2_curriculum --desc "Relaxed curriculum threshold"
python scripts/new_experiment.py distillation --base icros2025 --desc "Teacher-Student"
```

This creates all files and registers the gym environment automatically.
**Do not create experiment files manually.**

---

## RULE #2: Protected Files — Never Modify

- `scripts/rsl_rl/play.py` — original, read-only
- `scripts/rsl_rl/train.py` — original, read-only
- `source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg.py` — original base env

Customize only inside `scripts/experiments/<name>/` and `source/.../go2/experiments/<name>/`.

---

## RULE #3: One Experiment = One Folder

Each experiment lives in two mirrored folders:
1. `scripts/experiments/<name>/` — training/play scripts
2. `source/.../go2/experiments/<name>/` — environment configuration

These are linked via `go2/__init__.py` gym registration.

---

## Project File Map

```
unitree_rl_lab/
├── EXPERIMENTS.md              ← List of all experiments (auto-updated by new_experiment.py)
├── CLAUDE.md                   ← Claude Code specific instructions
├── AGENTS.md                   ← This file
├── scripts/
│   ├── new_experiment.py       ← Scaffold tool
│   ├── auto_monitor_competition.py  ← Phase-based training monitor
│   ├── rsl_rl/                 ← ORIGINAL scripts
│   │   ├── play.py
│   │   └── train.py
│   └── experiments/
│       └── icros2025/          ← Example: ICROS 2025 experiment
│           ├── play.py
│           ├── train.py
│           └── README.md
└── source/unitree_rl_lab/unitree_rl_lab/
    └── tasks/locomotion/robots/go2/
        ├── __init__.py         ← Gym registrations (auto-updated)
        ├── velocity_env_cfg.py ← ORIGINAL base environment
        └── experiments/
            └── icros2025/
                ├── env_cfg.py
                └── terrains/
                    └── custom_terrains.py
```

---

## Docker / Runtime

```bash
# Python runtime inside container
/isaac-sim/python.sh <script.py> [args]

# Train
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/<name>/train.py \
  --task <env-id> --headless

# Play (8 robots, GUI)
docker exec -it isaac-lab /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/<name>/play.py \
  --task <env-id> --num_envs 8 --lin_vel_x 1.0
```

---

## Observation Space Convention

Policy actor inputs (45-dim, proprioceptive only — deployable on real robot):
```
[0:3]   base_ang_vel
[3:6]   projected_gravity
[6:9]   velocity_commands (vx, vy, wz)
[9:21]  joint_pos_rel
[21:33] joint_vel_rel
[33:45] last_action
```

Critic inputs: 45-dim above + height_scan (privileged, training only).

---

## After Adding an Experiment

1. Edit `source/.../go2/experiments/<name>/env_cfg.py` (rewards, terrain, obs)
2. Update `scripts/experiments/<name>/README.md` with what changed and why
3. Update `EXPERIMENTS.md` status when training completes
