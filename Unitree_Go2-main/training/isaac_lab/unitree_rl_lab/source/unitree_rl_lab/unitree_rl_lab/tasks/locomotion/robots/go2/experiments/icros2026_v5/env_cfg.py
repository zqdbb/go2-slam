"""ICROS 2026 V5 — V4 + height_scan in actor (single key change).

## V4 한계 분석
  Actor가 앞을 못 봄 (225-dim proprio_history only, blind)
  → 장애물을 발이 닿고서야 반응 (reactive only)
  → terrain_level 5.27/9에서 정체

## V5 변경 사항 (최소 변경 원칙)

### [ONLY CHANGE] height_scan을 Policy obs에 추가
  Policy: 225 (proprio_history) + 273 (height_scan) = 498-dim
  Critic: V1 CriticCfg = 333-dim (변경 없음)

  [근거]
  - V4의 검증된 reward 가중치 전부 유지 (track=1.5, action_rate=-0.01)
  - V3의 498-dim obs가 실패한 이유 = reward 문제 (action_rate 포화), obs 차원 아님
  - V5 = V3 obs + V4 reward → 두 버전의 장점 결합
  - FAST-LIO2 + TRG-planner 통합 시: height_scan = FAST-LIO2 local height map
  - Go2 탑재 LiDAR → 실제 배포 가능

### 유지 사항 (V4에서 검증된 것들)
  - track_lin_vel_xy: 1.5 (SOTA 표준)
  - action_rate: -0.01 (SOTA 표준)
  - action_smoothness_2: -0.001 (Walk-These-Ways)
  - 5-frame proprio history (225-dim)
  - Phase별 curriculum DR (Phase1 ±10%, Phase2 ±18%, Phase3 ±25%)
  - terrain curriculum: move_up=3.0m, move_down=0.8m
  - feet_air_time: weight=1.5, threshold=0.20s (V4 검증값)

## 학습 설정
  num_envs: 16384 (RTX 5090 32GB, ≈20GB)
  total_iter: 50000
  Phase1: ckpt 0~10k    (max_init=0, vel=1.0, dr=1)
  Phase2: ckpt 10k~20k  (max_init=4, vel=1.5, dr=2)
  Phase3: ckpt 20k~50k  (max_init=7, vel=2.0, dr=3, 30k충분)

## SLAM 통합 계획 (참고)
  FAST-LIO2 → local height map (2.0×1.2m, 0.1m res) → 273-dim height_scan
  TRG-planner → velocity commands (vx, vy, wz)
  Policy input = proprio_history(225) + height_scan(273) = 498-dim ✓
"""

from __future__ import annotations

import json as _json
import os as _os
from typing import TYPE_CHECKING

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# ── Override 읽기 ──────────────────────────────────────────────────────────────
_OVERRIDE_PATH = "/tmp/training_override.json"
_v5_override: dict = {}
if _os.path.exists(_OVERRIDE_PATH):
    try:
        with open(_OVERRIDE_PATH) as _f:
            _v5_override = _json.load(_f)
    except Exception:
        _v5_override = {}

_DR_PHASE: int = int(_v5_override.get("dr_phase", 1))

# ---------------------------------------------------------------------------
# V4 임포트 (전부 상속)
# ---------------------------------------------------------------------------
from unitree_rl_lab.tasks.locomotion.robots.go2.experiments.icros2026_v4.env_cfg import (
    ObservationsCfg as _V4ObsCfg,
    RobotEnvCfg as _V4EnvCfg,
    _DR_TABLE,
)

_DR = _DR_TABLE[max(1, min(3, _DR_PHASE))]


# ---------------------------------------------------------------------------
# Observations
# V4 PolicyCfg (proprio_history 225-dim) + height_scan (273-dim) = 498-dim
# Critic: V1 CriticCfg 그대로 (333-dim, 변경 없음)
# ---------------------------------------------------------------------------
@configclass
class ObservationsCfg(_V4ObsCfg):
    @configclass
    class PolicyCfg(_V4ObsCfg.PolicyCfg):
        """V4 proprio_history(225) + height_scan(273) = 498-dim.

        Height scan: GridPatternCfg(resolution=0.1, size=[2.0, 1.2])
          → 2.0m forward × 1.2m lateral @ 0.1m = 273 rays
        LiDAR noise: Unoise(-0.1, 0.1) — FAST-LIO2 noise 모사, Sim2Real 강화
        """
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )

    policy: PolicyCfg = PolicyCfg()
    # critic: _V4ObsCfg의 critic 그대로 상속 (V1 CriticCfg = 333-dim)


# ---------------------------------------------------------------------------
# RobotEnvCfg
# ---------------------------------------------------------------------------
@configclass
class RobotEnvCfg(_V4EnvCfg):
    """V5 학습 환경.

    V4 전체 상속 + ObservationsCfg만 교체 (PolicyCfg에 height_scan 추가).
    """

    observations: ObservationsCfg = ObservationsCfg()
    # rewards: _V4EnvCfg 그대로 상속 (변경 없음, 검증된 V4 reward)

    def __post_init__(self):
        super().__post_init__()

        # height_scanner update period 설정 (V1 scene에 이미 있음)
        if hasattr(self.scene, 'height_scanner'):
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        print('[icros2026_v5] ★ Policy obs: proprio_history(225) + height_scan(273) = 498-dim')
        print('[icros2026_v5]   Critic obs: V1 CriticCfg = 333-dim (변경 없음)')
        print('[icros2026_v5]   V4 rewards 전부 유지: track=1.5, action_rate=-0.01')
        print(f'[icros2026_v5]   DR Phase {_DR_PHASE}: kp/kd ±{int(round((1.0-_DR["kp_range"][0])*100))}%')


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.episode_length_s = 40.0
        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.3, 0.3)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
