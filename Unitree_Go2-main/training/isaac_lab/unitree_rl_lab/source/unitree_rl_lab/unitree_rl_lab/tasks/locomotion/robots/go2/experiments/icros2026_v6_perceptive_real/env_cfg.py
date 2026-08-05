"""ICROS 2026 V6 — V5 hard-terrain breakthrough fine-tuning.

Objective
---------
V5 is a stable perceptive locomotion baseline, but analysis around 20k-36k
showed terrain level plateauing near 5.5 while timeout stayed around 97%.
That means the main failure is not basic stability; it is insufficient hard-row
progression and gait quality on 15-19 cm obstacles.

V6 keeps the V5 policy layout and reward design:

    proprio_history(225) + height_scan(273) = 498-dim actor input

and changes the curriculum/reward surface conservatively:

    - hard terrain exposure through a minimum terrain level floor per phase
    - lower hard-terrain speed pressure than V5 Phase3
    - feet_air_time threshold relaxed so short obstacle steps can be rewarded
    - mild FAST-LIO-like height_scan corruption retained for Sim2Real
"""

from __future__ import annotations

import json as _json
import os as _os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ManagerTermBase
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from unitree_rl_lab.tasks.locomotion.robots.go2.experiments.icros2026_v5.env_cfg import (
    ObservationsCfg as _V5ObsCfg,
    RobotEnvCfg as _V5EnvCfg,
)

_OVERRIDE_PATH = "/tmp/training_override.json"
_v6_override: dict = {}
if _os.path.exists(_OVERRIDE_PATH):
    try:
        with open(_OVERRIDE_PATH) as _f:
            _v6_override = _json.load(_f)
    except Exception:
        _v6_override = {}

_MIN_TERRAIN_LEVEL: int = int(_v6_override.get("min_terrain_level", 0))
_MOVE_UP_M: float = float(_v6_override.get("move_up_m", 2.4))
_MOVE_DOWN_M: float = float(_v6_override.get("move_down_m", 0.6))


def terrain_levels_vel_v6(
    env: "ManagerBasedRLEnv",
    env_ids: Sequence[int],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Hard-terrain curriculum.

    V5 required 3.0 m progress to level up and allowed reset down to row 0.
    V6 lowers the progression threshold and optionally applies a phase-controlled
    minimum terrain row. This is exposure control, not success fabrication:
    timeout/contact metrics still reveal whether the policy survives those rows.
    """
    from isaaclab.assets import Articulation
    from isaaclab.terrains import TerrainImporter

    asset: Articulation = env.scene[asset_cfg.name]
    terrain: TerrainImporter = env.scene.terrain
    distance = torch.norm(
        asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1
    )

    move_up = distance > _MOVE_UP_M
    move_down = (distance < _MOVE_DOWN_M) & ~move_up
    terrain.update_env_origins(env_ids, move_up, move_down)

    if _MIN_TERRAIN_LEVEL > 0 and getattr(terrain, "terrain_origins", None) is not None:
        min_level = min(_MIN_TERRAIN_LEVEL, terrain.max_terrain_level - 1)
        terrain.terrain_levels[env_ids] = torch.clamp(terrain.terrain_levels[env_ids], min=min_level)
        terrain.env_origins[env_ids] = terrain.terrain_origins[
            terrain.terrain_levels[env_ids], terrain.terrain_types[env_ids]
        ]

    return torch.mean(terrain.terrain_levels.float())


class _RealisticHeightScanTerm(ManagerTermBase):
    """Corrupt IsaacLab height_scan to resemble FAST-LIO-derived local maps."""

    _NX = 21
    _NY = 13
    _DIM = _NX * _NY

    def __init__(self, cfg: ObsTerm, env: "ManagerBasedRLEnv"):
        super().__init__(cfg, env)
        self._lag_steps = int(cfg.params.get("lag_steps", 2))
        self._empty_value = float(cfg.params.get("empty_value", -1.0))
        self._buf = torch.full(
            (env.num_envs, self._lag_steps + 1, self._DIM),
            self._empty_value,
            device=env.device,
            dtype=torch.float32,
        )
        self._initialized = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        if env_ids is None:
            self._buf[:] = self._empty_value
            self._initialized[:] = False
        else:
            self._buf[env_ids] = self._empty_value
            self._initialized[env_ids] = False

    def __call__(
        self,
        env: "ManagerBasedRLEnv",
        sensor_cfg: SceneEntityCfg,
        noise_range: float = 0.12,
        z_bias_range: float = 0.03,
        quantize_m: float = 0.02,
        cell_dropout_prob: float = 0.08,
        row_dropout_prob: float = 0.015,
        col_dropout_prob: float = 0.015,
        full_dropout_prob: float = 0.015,
        empty_value: float = -1.0,
        lag_steps: int = 2,
    ) -> torch.Tensor:
        scan = mdp.height_scan(env, sensor_cfg=sensor_cfg).clamp(-1.0, 1.0)

        if noise_range > 0.0:
            scan = scan + torch.empty_like(scan).uniform_(-noise_range, noise_range)
        if z_bias_range > 0.0:
            z_bias = torch.empty(scan.shape[0], 1, device=scan.device).uniform_(
                -z_bias_range, z_bias_range
            )
            scan = scan + z_bias
        if quantize_m > 0.0:
            scan = torch.round(scan / quantize_m) * quantize_m

        scan = scan.clamp(-1.0, 1.0)
        grid = scan.view(-1, self._NX, self._NY)

        if cell_dropout_prob > 0.0:
            grid = torch.where(
                torch.rand_like(grid) < cell_dropout_prob,
                torch.full_like(grid, empty_value),
                grid,
            )
        if row_dropout_prob > 0.0:
            row_mask = torch.rand(grid.shape[0], self._NX, 1, device=grid.device) < row_dropout_prob
            grid = torch.where(row_mask.expand_as(grid), torch.full_like(grid, empty_value), grid)
        if col_dropout_prob > 0.0:
            col_mask = torch.rand(grid.shape[0], 1, self._NY, device=grid.device) < col_dropout_prob
            grid = torch.where(col_mask.expand_as(grid), torch.full_like(grid, empty_value), grid)
        if full_dropout_prob > 0.0:
            full_mask = torch.rand(grid.shape[0], 1, 1, device=grid.device) < full_dropout_prob
            grid = torch.where(full_mask.expand_as(grid), torch.full_like(grid, empty_value), grid)

        corrupted = grid.reshape(-1, self._DIM).clamp(-1.0, 1.0)

        fresh_envs = ~self._initialized
        if torch.any(fresh_envs):
            self._buf[fresh_envs] = corrupted[fresh_envs].unsqueeze(1)
            self._initialized[fresh_envs] = True

        self._buf = torch.roll(self._buf, shifts=-1, dims=1)
        self._buf[:, -1] = corrupted

        effective_lag = max(0, min(int(lag_steps), self._lag_steps))
        return self._buf[:, self._lag_steps - effective_lag]


@configclass
class ObservationsCfg(_V5ObsCfg):
    @configclass
    class PolicyCfg(_V5ObsCfg.PolicyCfg):
        height_scan = ObsTerm(
            func=_RealisticHeightScanTerm,
            params={
                "sensor_cfg": SceneEntityCfg("height_scanner"),
                "noise_range": 0.06,
                "z_bias_range": 0.02,
                "quantize_m": 0.02,
                "cell_dropout_prob": 0.04,
                "row_dropout_prob": 0.008,
                "col_dropout_prob": 0.008,
                "full_dropout_prob": 0.005,
                "empty_value": -1.0,
                "lag_steps": 1,
            },
            clip=(-1.0, 1.0),
        )

    policy: PolicyCfg = PolicyCfg()


@configclass
class RobotEnvCfg(_V5EnvCfg):
    """V6 train env: V5 locomotion retuned for level 8+ terrain exposure."""

    observations: ObservationsCfg = ObservationsCfg()

    def __post_init__(self):
        super().__post_init__()

        # V4/V5 expanded potholes to 40 cm depth, which is useful for stress
        # testing but can dominate the row curriculum beyond the competition's
        # 18-19 cm obstacle scale. Keep them hard, but realistic.
        tgen = self.scene.terrain.terrain_generator
        if "pothole" in tgen.sub_terrains:
            tgen.sub_terrains["pothole"].pothole_radius_range = (0.08, 0.28)
            tgen.sub_terrains["pothole"].pothole_depth_range = (0.06, 0.24)
            tgen.sub_terrains["pothole"].num_potholes = (4, 18)

        # V5 logs showed feet_air_time stayed negative around -0.047, meaning
        # many contacts happened before the 0.20 s threshold. A slightly lower
        # threshold rewards short, realistic obstacle steps without encouraging
        # exaggerated hopping.
        self.rewards.feet_air_time.params["threshold"] = 0.14
        self.rewards.feet_air_time.weight = 1.8
        self.rewards.air_time_variance.weight = -0.6

        self.curriculum.terrain_levels = CurrTerm(func=terrain_levels_vel_v6)

        print("[icros2026_v6_perceptive_real] Policy obs remains 498-dim")
        print("[icros2026_v6_perceptive_real] objective: terrain level 8+ hard-row fine-tuning")
        print(
            f"[icros2026_v6_perceptive_real] terrain curriculum: "
            f"move_up={_MOVE_UP_M:.2f}m, move_down={_MOVE_DOWN_M:.2f}m, min_level={_MIN_TERRAIN_LEVEL}"
        )
        print("[icros2026_v6_perceptive_real] feet_air_time threshold=0.14, weight=1.8")
        print("[icros2026_v6_perceptive_real] mild height_scan corruption retained for Sim2Real")


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 8
        self.episode_length_s = 40.0
        self.commands.base_velocity.ranges.lin_vel_x = (0.3, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.25, 0.25)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.8, 0.8)
