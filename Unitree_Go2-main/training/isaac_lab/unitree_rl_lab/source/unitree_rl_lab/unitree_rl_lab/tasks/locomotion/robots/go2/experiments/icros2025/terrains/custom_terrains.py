"""Custom terrain generators ported from legged_gym terrain.py (ETH Zurich / NVIDIA).

9 terrain types:
  1. pothole         — circular negative obstacles
  2. smooth_slope    — pyramid sloped terrain
  3. rough_slope     — pyramid slope + random uniform noise
  4. stairs_up       — pyramid stairs ascending
  5. stairs_down     — pyramid stairs descending
  6. discrete_obs    — discrete obstacles with side walls
  7. bridge_ramp     — ramp bridge over ditch
  8. stepping_slabs  — offset stepping slabs over ditch
  9. zigzag_bridge   — zigzag bridge over ditch
 10. checker_blocks  — checker-pattern blocks over ditch
"""

from __future__ import annotations

from dataclasses import MISSING

import numpy as np

from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.utils import configclass


# ---------------------------------------------------------------------------
# Internal helpers  (operate directly on numpy int16 height field arrays)
# ---------------------------------------------------------------------------

def _px(h_scale: float, meters: float, size_px: int) -> int:
    return max(0, min(size_px, int(round(meters / h_scale))))


def _h(v_scale: float, meters: float) -> int:
    return int(round(meters / v_scale))


def _add_box(hf, xs, ys, h_scale, v_scale, x0m, x1m, y0m, y1m, hm):
    x0, x1 = _px(h_scale, x0m, xs), _px(h_scale, x1m, xs)
    y0, y1 = _px(h_scale, y0m, ys), _px(h_scale, y1m, ys)
    if x1 <= x0 or y1 <= y0:
        return
    h = _h(v_scale, hm)
    hf[x0:x1, y0:y1] = np.maximum(hf[x0:x1, y0:y1], h)


def _set_box(hf, xs, ys, h_scale, v_scale, x0m, x1m, y0m, y1m, hm):
    x0, x1 = _px(h_scale, x0m, xs), _px(h_scale, x1m, xs)
    y0, y1 = _px(h_scale, y0m, ys), _px(h_scale, y1m, ys)
    if x1 <= x0 or y1 <= y0:
        return
    h = _h(v_scale, hm)
    hf[x0:x1, y0:y1] = h


def _add_ramp_x(hf, xs, ys, h_scale, v_scale, x0m, x1m, y0m, y1m, h0m, h1m):
    x0, x1 = _px(h_scale, x0m, xs), _px(h_scale, x1m, xs)
    y0, y1 = _px(h_scale, y0m, ys), _px(h_scale, y1m, ys)
    if x1 <= x0 or y1 <= y0:
        return
    h0, h1 = _h(v_scale, h0m), _h(v_scale, h1m)
    length = x1 - x0
    if length <= 1:
        hf[x0:x1, y0:y1] = np.maximum(hf[x0:x1, y0:y1], h1)
        return
    vals = np.linspace(h0, h1, length, endpoint=True).astype(np.int16)
    for i in range(length):
        hf[x0 + i, y0:y1] = np.maximum(hf[x0 + i, y0:y1], vals[i])


def _add_side_walls(hf, xs, ys, lm, wm, h_scale, v_scale, wall_h=0.28, wall_w=0.40):
    _add_box(hf, xs, ys, h_scale, v_scale, 0.0, lm, 0.0, wall_w, wall_h)
    _add_box(hf, xs, ys, h_scale, v_scale, 0.0, lm, wm - wall_w, wm, wall_h)


# ---------------------------------------------------------------------------
# 1. Pothole terrain
# ---------------------------------------------------------------------------

@height_field_to_mesh
def pothole_terrain(difficulty: float, cfg: "HfPotholeTerrainCfg") -> np.ndarray:
    """Circular negative obstacles (potholes) on flat ground."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((xs, ys), dtype=np.float32)

    num = int(round(cfg.num_potholes[0] + difficulty * (cfg.num_potholes[1] - cfg.num_potholes[0])))
    r_max = cfg.pothole_radius_range[0] + difficulty * (cfg.pothole_radius_range[1] - cfg.pothole_radius_range[0])
    d_max = cfg.pothole_depth_range[0] + difficulty * (cfg.pothole_depth_range[1] - cfg.pothole_depth_range[0])
    r_min = cfg.pothole_radius_range[0]
    d_min = cfg.pothole_depth_range[0]

    plat_r_px = int((cfg.platform_width * 0.5) / cfg.horizontal_scale)
    cx0, cy0 = xs // 2, ys // 2
    margin_px = max(2, int(r_max / cfg.horizontal_scale))

    for _ in range(max(0, num)):
        r_m = float(np.random.uniform(r_min, r_max))
        r_px = max(1, int(r_m / cfg.horizontal_scale))
        d_m = float(np.random.uniform(d_min, d_max))
        d_step = max(1, int(d_m / cfg.vertical_scale))
        for _t in range(16):
            cx = int(np.random.randint(margin_px, max(margin_px + 1, xs - margin_px)))
            cy = int(np.random.randint(margin_px, max(margin_px + 1, ys - margin_px)))
            if (cx - cx0) ** 2 + (cy - cy0) ** 2 > (plat_r_px + r_px + margin_px) ** 2:
                break
        else:
            continue
        x0, x1 = max(0, cx - r_px), min(xs, cx + r_px + 1)
        y0, y1 = max(0, cy - r_px), min(ys, cy + r_px + 1)
        xg = np.arange(x0, x1)[:, None]
        yg = np.arange(y0, y1)[None, :]
        dist = np.sqrt((xg - cx) ** 2 + (yg - cy) ** 2)
        mask = dist <= r_px
        profile = np.clip(1.0 - dist / max(r_px, 1), 0.0, 1.0)
        local = hf[x0:x1, y0:y1]
        local[mask] -= d_step * profile[mask]
        hf[x0:x1, y0:y1] = local

    return np.rint(hf).astype(np.int16)


@configclass
class HfPotholeTerrainCfg(HfTerrainBaseCfg):
    function = pothole_terrain
    num_potholes: tuple[int, int] = MISSING
    pothole_radius_range: tuple[float, float] = MISSING
    pothole_depth_range: tuple[float, float] = MISSING
    platform_width: float = 1.5


# ---------------------------------------------------------------------------
# 2. Smooth slope terrain
# ---------------------------------------------------------------------------

@height_field_to_mesh
def smooth_slope_terrain(difficulty: float, cfg: "HfSmoothSlopeTerrainCfg") -> np.ndarray:
    """Pyramid sloped terrain. slope scales with difficulty."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((xs, ys), dtype=np.float32)

    slope = cfg.slope_min + difficulty * (cfg.slope_max - cfg.slope_min)
    if cfg.inverted:
        slope = -slope

    # pyramid: height = slope * min(dist_to_center_x, dist_to_center_y)
    cx, cy = xs / 2.0, ys / 2.0
    xg = np.arange(xs)[:, None]
    yg = np.arange(ys)[None, :]
    dist_x = np.abs(xg - cx) * cfg.horizontal_scale
    dist_y = np.abs(yg - cy) * cfg.horizontal_scale
    dist = np.minimum(dist_x, dist_y)
    height_m = -slope * dist  # negative: lower at edges, higher at center (pyramid up)
    hf = (height_m / cfg.vertical_scale).astype(np.float32)

    # flat platform at center
    plat_px = int((cfg.platform_size * 0.5) / cfg.horizontal_scale)
    cx_i, cy_i = xs // 2, ys // 2
    x0, x1 = max(0, cx_i - plat_px), min(xs, cx_i + plat_px)
    y0, y1 = max(0, cy_i - plat_px), min(ys, cy_i + plat_px)
    center_h = float(hf[cx_i, cy_i])
    hf[x0:x1, y0:y1] = center_h

    return np.rint(hf).astype(np.int16)


@configclass
class HfSmoothSlopeTerrainCfg(HfTerrainBaseCfg):
    function = smooth_slope_terrain
    slope_min: float = 0.08
    slope_max: float = 0.36
    platform_size: float = 2.5
    inverted: bool = False


# ---------------------------------------------------------------------------
# 3. Rough slope terrain (slope + random uniform noise)
# ---------------------------------------------------------------------------

@height_field_to_mesh
def rough_slope_terrain(difficulty: float, cfg: "HfRoughSlopeTerrainCfg") -> np.ndarray:
    """Pyramid slope + random uniform noise."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)

    slope = (cfg.slope_min + difficulty * (cfg.slope_max - cfg.slope_min)) * 0.6
    cx, cy = xs / 2.0, ys / 2.0
    xg = np.arange(xs)[:, None]
    yg = np.arange(ys)[None, :]
    dist = np.minimum(np.abs(xg - cx), np.abs(yg - cy)) * cfg.horizontal_scale
    hf = (-slope * dist / cfg.vertical_scale).astype(np.float32)

    # flat platform
    plat_px = int((cfg.platform_size * 0.5) / cfg.horizontal_scale)
    cx_i, cy_i = xs // 2, ys // 2
    x0, x1 = max(0, cx_i - plat_px), min(xs, cx_i + plat_px)
    y0, y1 = max(0, cy_i - plat_px), min(ys, cy_i + plat_px)
    center_h = float(hf[cx_i, cy_i])
    hf[x0:x1, y0:y1] = center_h

    # add noise
    noise_max = cfg.noise_min + difficulty * (cfg.noise_max - cfg.noise_min)
    noise_m = np.random.uniform(-noise_max, noise_max, (xs, ys)).astype(np.float32)
    hf = hf + noise_m / cfg.vertical_scale

    return np.rint(hf).astype(np.int16)


@configclass
class HfRoughSlopeTerrainCfg(HfTerrainBaseCfg):
    function = rough_slope_terrain
    slope_min: float = 0.04
    slope_max: float = 0.20
    noise_min: float = 0.005
    noise_max: float = 0.060
    platform_size: float = 2.5


# ---------------------------------------------------------------------------
# 4 & 5. Stairs up / down
# ---------------------------------------------------------------------------

@height_field_to_mesh
def pyramid_stairs_terrain(difficulty: float, cfg: "HfPyramidStairsTerrainCfg") -> np.ndarray:
    """Pyramid staircase. step_height scales with difficulty."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((xs, ys), dtype=np.int16)

    step_width = cfg.step_width_max - difficulty * (cfg.step_width_max - cfg.step_width_min)
    step_height = cfg.step_height_min + difficulty * (cfg.step_height_max - cfg.step_height_min)
    if cfg.inverted:
        step_height = -step_height

    step_w_px = max(1, int(step_width / cfg.horizontal_scale))
    step_h_int = _h(cfg.vertical_scale, abs(step_height))
    cx, cy = xs // 2, ys // 2

    num_steps = max(xs, ys) // (2 * step_w_px) + 2
    current_h = 0
    for i in range(num_steps):
        r = i * step_w_px
        x0, x1 = max(0, cx - r), min(xs, cx + r)
        y0, y1 = max(0, cy - r), min(ys, cy + r)
        if step_height > 0:
            hf[x0:x1, y0:y1] = current_h
        else:
            hf[:, :] = np.maximum(hf[:, :], current_h)
            hf[x0:x1, y0:y1] = np.minimum(hf[x0:x1, y0:y1], current_h)
        if step_height > 0:
            current_h -= step_h_int
        else:
            current_h += step_h_int

    # normalize so center = 0
    hf -= hf[cx, cy]

    return hf


@configclass
class HfPyramidStairsTerrainCfg(HfTerrainBaseCfg):
    function = pyramid_stairs_terrain
    step_width_min: float = 0.30
    step_width_max: float = 0.38
    step_height_min: float = 0.035
    step_height_max: float = 0.110
    inverted: bool = False


# ---------------------------------------------------------------------------
# 6. Discrete obstacles with side walls
# ---------------------------------------------------------------------------

@height_field_to_mesh
def discrete_obstacles_terrain(difficulty: float, cfg: "HfDiscreteObstaclesTerrainCfg") -> np.ndarray:
    """Random rectangular obstacles with side walls."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    lm = xs * cfg.horizontal_scale
    wm = ys * cfg.horizontal_scale
    hf = np.zeros((xs, ys), dtype=np.int16)

    _add_side_walls(hf, xs, ys, lm, wm, cfg.horizontal_scale, cfg.vertical_scale,
                    wall_h=cfg.wall_height, wall_w=cfg.wall_width)

    obs_height = cfg.obs_height_min + difficulty * (cfg.obs_height_max - cfg.obs_height_min)
    num_obs = int(cfg.num_obs_min + difficulty * (cfg.num_obs_max - cfg.num_obs_min))
    plat_px = int((cfg.platform_size * 0.5) / cfg.horizontal_scale)
    cx, cy = xs // 2, ys // 2

    for _ in range(num_obs):
        obs_w = float(np.random.uniform(cfg.obs_size_min, cfg.obs_size_max))
        obs_l = float(np.random.uniform(cfg.obs_size_min, cfg.obs_size_max))
        h_val = float(np.random.uniform(0.0, obs_height))

        ox_px = int(np.random.randint(0, max(1, xs - int(obs_l / cfg.horizontal_scale))))
        oy_px = int(np.random.randint(0, max(1, ys - int(obs_w / cfg.horizontal_scale))))

        # skip if overlapping center platform
        ox_e = ox_px + int(obs_l / cfg.horizontal_scale)
        oy_e = oy_px + int(obs_w / cfg.horizontal_scale)
        if (ox_px < cx + plat_px and ox_e > cx - plat_px and
                oy_px < cy + plat_px and oy_e > cy - plat_px):
            continue

        _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
                 ox_px * cfg.horizontal_scale,
                 ox_e * cfg.horizontal_scale,
                 oy_px * cfg.horizontal_scale,
                 oy_e * cfg.horizontal_scale,
                 h_val)

    # slight noise
    noise_scale = 0.005 + 0.015 * difficulty
    noise = np.random.uniform(-noise_scale, noise_scale, (xs, ys))
    hf = hf + (noise / cfg.vertical_scale).astype(np.int16)

    return hf


@configclass
class HfDiscreteObstaclesTerrainCfg(HfTerrainBaseCfg):
    function = discrete_obstacles_terrain
    obs_height_min: float = 0.04
    obs_height_max: float = 0.14
    obs_size_min: float = 0.25
    obs_size_max: float = 0.70
    num_obs_min: int = 10
    num_obs_max: int = 28
    platform_size: float = 2.0
    wall_height: float = 0.22
    wall_width: float = 0.40


# ---------------------------------------------------------------------------
# 7. Bridge ramp course
# ---------------------------------------------------------------------------

@height_field_to_mesh
def bridge_ramp_terrain(difficulty: float, cfg: "HfBridgeRampTerrainCfg") -> np.ndarray:
    """Ramp bridge over a ditch. Ported from legged_gym bridge_ramp_course."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    lm = xs * cfg.horizontal_scale
    wm = ys * cfg.horizontal_scale
    hf = np.zeros((xs, ys), dtype=np.int16)

    _add_side_walls(hf, xs, ys, lm, wm, cfg.horizontal_scale, cfg.vertical_scale,
                    wall_h=0.28, wall_w=0.45)

    cy = wm / 2.0
    ditch_depth = -0.07 - 0.08 * difficulty
    _set_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
             0.7, 5.5, cy - 1.80, cy + 1.80, ditch_depth)

    width = 1.35
    height = 0.10 + 0.08 * difficulty
    y0, y1 = cy - width / 2, cy + width / 2

    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 0.6, 1.0, y0, y1, 0.02)
    _add_ramp_x(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 1.0, 2.2, y0, y1, 0.02, height)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 2.2, 3.5, y0, y1, height)
    _add_ramp_x(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 3.5, 4.8, y0, y1, height, 0.02)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 4.8, 5.4, y0, y1, 0.02)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 5.6, 6.3, cy - 1.2, cy - 0.2, 0.04 + 0.03 * difficulty)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 6.3, 7.0, cy + 0.2, cy + 1.2, 0.04 + 0.03 * difficulty)

    return hf


@configclass
class HfBridgeRampTerrainCfg(HfTerrainBaseCfg):
    function = bridge_ramp_terrain


# ---------------------------------------------------------------------------
# 8. Stepping slabs course
# ---------------------------------------------------------------------------

@height_field_to_mesh
def stepping_slabs_terrain(difficulty: float, cfg: "HfSteppingSlabsTerrainCfg") -> np.ndarray:
    """Offset stepping slabs over a ditch."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    lm = xs * cfg.horizontal_scale
    wm = ys * cfg.horizontal_scale
    hf = np.zeros((xs, ys), dtype=np.int16)

    _add_side_walls(hf, xs, ys, lm, wm, cfg.horizontal_scale, cfg.vertical_scale,
                    wall_h=0.28, wall_w=0.45)

    cy = wm / 2.0
    ditch_depth = -0.08 - 0.10 * difficulty
    _set_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
             0.6, 6.4, cy - 1.55, cy + 1.55, ditch_depth)

    slab_len = 0.85
    slab_w = 1.20
    gap = 0.08 + 0.05 * difficulty
    base_h = 0.02 + 0.035 * difficulty

    offsets = [-0.65, 0.45, -0.35, 0.65, -0.45, 0.35]
    heights = [base_h * 0.5, base_h * 1.0, base_h * 0.7, base_h * 1.2, base_h * 0.8, base_h * 1.1]

    x = 0.8
    for k in range(len(offsets)):
        yc = cy + offsets[k]
        y0, y1 = yc - slab_w / 2, yc + slab_w / 2
        _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, x, x + slab_len, y0, y1, heights[k])
        x += slab_len + gap

    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 0.35, 0.75, cy - 1.25, cy + 1.25, 0.02)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 5.95, 6.50, cy - 1.25, cy + 1.25, 0.02)

    return hf


@configclass
class HfSteppingSlabsTerrainCfg(HfTerrainBaseCfg):
    function = stepping_slabs_terrain


# ---------------------------------------------------------------------------
# 9. Zigzag bridge course
# ---------------------------------------------------------------------------

@height_field_to_mesh
def zigzag_bridge_terrain(difficulty: float, cfg: "HfZigzagBridgeTerrainCfg") -> np.ndarray:
    """Zigzag narrow bridge over a ditch."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    lm = xs * cfg.horizontal_scale
    wm = ys * cfg.horizontal_scale
    hf = np.zeros((xs, ys), dtype=np.int16)

    _add_side_walls(hf, xs, ys, lm, wm, cfg.horizontal_scale, cfg.vertical_scale,
                    wall_h=0.30, wall_w=0.45)

    cy = wm / 2.0
    ditch_depth = -0.07 - 0.09 * difficulty
    _set_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
             0.6, 6.7, cy - 1.70, cy + 1.70, ditch_depth)

    seg_len = 1.00
    seg_w = 1.05
    height = 0.04 + 0.06 * difficulty
    offsets = [-0.85, -0.35, 0.35, 0.85, 0.30, -0.35]

    x = 0.7
    prev_yc = cy + offsets[0]
    for idx, off in enumerate(offsets):
        yc = cy + off
        y0, y1 = yc - seg_w / 2, yc + seg_w / 2
        _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, x, x + seg_len, y0, y1, height)
        if idx > 0:
            conn_y0 = min(prev_yc, yc) - seg_w / 2
            conn_y1 = max(prev_yc, yc) + seg_w / 2
            _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
                     x - 0.20, x + 0.25, conn_y0, conn_y1, height)
        prev_yc = yc
        x += seg_len * 0.9

    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 0.35, 0.70, cy - 1.30, cy + 1.30, 0.02)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 6.10, 6.70, cy - 1.30, cy + 1.30, 0.02)

    return hf


@configclass
class HfZigzagBridgeTerrainCfg(HfTerrainBaseCfg):
    function = zigzag_bridge_terrain


# ---------------------------------------------------------------------------
# 10. Checker blocks course
# ---------------------------------------------------------------------------

@height_field_to_mesh
def checker_blocks_terrain(difficulty: float, cfg: "HfCheckerBlocksTerrainCfg") -> np.ndarray:
    """Checker-pattern elevated blocks over a ditch."""
    xs = int(cfg.size[0] / cfg.horizontal_scale)
    ys = int(cfg.size[1] / cfg.horizontal_scale)
    lm = xs * cfg.horizontal_scale
    wm = ys * cfg.horizontal_scale
    hf = np.zeros((xs, ys), dtype=np.int16)

    _add_side_walls(hf, xs, ys, lm, wm, cfg.horizontal_scale, cfg.vertical_scale,
                    wall_h=0.30, wall_w=0.45)

    cy = wm / 2.0
    tile = 0.68
    gap = 0.08
    start_x = 0.8
    start_y = cy - 1.45
    base_h = 0.02 + 0.025 * difficulty
    high_h = 0.07 + 0.06 * difficulty
    rows, cols = 4, 6

    ditch_depth = -0.06 - 0.08 * difficulty
    total_x0 = start_x - 0.10
    total_x1 = start_x + cols * (tile + gap) + 0.10
    total_y0 = start_y - 0.10
    total_y1 = start_y + rows * (tile + gap) + 0.10
    _set_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
             total_x0, total_x1, total_y0, total_y1, ditch_depth)

    for r in range(rows):
        for c in range(cols):
            x0 = start_x + c * (tile + gap)
            x1 = x0 + tile
            y0 = start_y + r * (tile + gap)
            y1 = y0 + tile
            h = base_h if (r + c) % 2 == 0 else high_h
            _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, x0, x1, y0, y1, h)

    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale, 0.35, 0.75, cy - 1.40, cy + 1.40, 0.02)
    _add_box(hf, xs, ys, cfg.horizontal_scale, cfg.vertical_scale,
             total_x1, total_x1 + 0.55, cy - 1.40, cy + 1.40, 0.02)

    return hf


@configclass
class HfCheckerBlocksTerrainCfg(HfTerrainBaseCfg):
    function = checker_blocks_terrain
