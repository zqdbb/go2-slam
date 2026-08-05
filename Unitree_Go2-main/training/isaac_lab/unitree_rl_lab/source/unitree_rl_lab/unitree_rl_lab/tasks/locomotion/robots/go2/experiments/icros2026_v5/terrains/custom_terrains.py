"""Custom terrain generators for icros2026_v5 experiment.

참고: experiments/icros2025/terrains/custom_terrains.py
IsaacLab HfTerrainBaseCfg 문서: https://isaac-sim.github.io/IsaacLab

구현 방법:
  1. terrain 함수 정의: (difficulty: float, cfg: MyCfg) -> np.ndarray
  2. Cfg 클래스 정의: HfTerrainBaseCfg 상속, function 필드 설정
  3. terrains/__init__.py 에서 export
  4. env_cfg.py 에서 TerrainGeneratorCfg에 포함
"""
from __future__ import annotations

from dataclasses import MISSING

import numpy as np

from isaaclab.terrains.height_field import HfTerrainBaseCfg
from isaaclab.utils import configclass


# ---------------------------------------------------------------------------
# 📝 여기에 커스텀 terrain 구현
# ---------------------------------------------------------------------------

# 예시 (주석 해제 후 수정):
#
# def flat_terrain(difficulty: float, cfg: FlatTerrainCfg) -> np.ndarray:
#     """단순 평지 (난이도 무관)."""
#     hf = np.zeros((cfg.size[0], cfg.size[1]), dtype=np.int16)
#     return hf
#
# @configclass
# class FlatTerrainCfg(HfTerrainBaseCfg):
#     function = flat_terrain
#     size: tuple[float, float] = MISSING
#     horizontal_scale: float = 0.1
#     vertical_scale: float = 0.005
