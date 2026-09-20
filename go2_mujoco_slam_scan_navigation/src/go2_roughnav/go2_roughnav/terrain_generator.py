#!/usr/bin/env python3
"""Generate MuJoCo rough-terrain MJCF scenes inspired by ICRA 2023 QRC."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET


def _geom(parent, name, geom_type, pos, size, rgba, **kwargs):
    attrs = {
        "name": name,
        "type": geom_type,
        "pos": " ".join(f"{v:.4f}" for v in pos),
        "size": " ".join(f"{v:.4f}" for v in size),
        "rgba": " ".join(f"{v:.3f}" for v in rgba),
    }
    attrs.update({k: str(v) for k, v in kwargs.items()})
    return ET.SubElement(parent, "geom", attrs)


def _box(parent, name, x, y, z, sx, sy, sz, rgba, yaw=0.0, roll=0.0, pitch=0.0, friction="0.9 0.03 0.003"):
    quat = _euler_to_quat(roll, pitch, yaw)
    return _geom(
        parent,
        name,
        "box",
        (x, y, z),
        (sx, sy, sz),
        rgba,
        quat=" ".join(f"{v:.6f}" for v in quat),
        friction=friction,
    )


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _add_crossing_ramps(world, x0, hard):
    angle = math.radians(15.0 if hard else 8.0)
    _box(world, "cross_ramp_left", x0, -0.45, 0.10, 1.25, 0.45, 0.04, (0.55, 0.55, 0.50, 1), roll=angle)
    _box(world, "cross_ramp_right", x0 + 1.55, 0.45, 0.10, 1.25, 0.45, 0.04, (0.55, 0.55, 0.50, 1), roll=-angle)


def _add_soft_floor(world, x0, hard):
    friction = "0.45 0.02 0.002" if hard else "0.65 0.02 0.002"
    _box(world, "soft_floor", x0, 0.0, 0.015, 1.45, 0.85, 0.015, (0.70, 0.62, 0.42, 1), friction=friction)
    for i, y in enumerate((-0.45, 0.0, 0.45)):
        _box(world, f"soft_stepover_{i}", x0 - 0.35 + 0.35 * i, y, 0.065, 0.04, 0.33, 0.05, (0.42, 0.30, 0.18, 1))


def _add_pallet_pipes(world, x0, hard):
    step_h = 0.15 if hard else 0.10
    offsets = (-0.28, 0.18, -0.16) if hard else (0.0, 0.0, 0.0)
    for i, off in enumerate(offsets):
        _box(world, f"pallet_step_{i}", x0 + i * 0.72, off, step_h / 2, 0.34, 0.78, step_h / 2, (0.50, 0.37, 0.22, 1))
        _geom(
            world,
            f"rolling_pipe_{i}",
            "cylinder",
            (x0 + i * 0.72 + 0.34, off, step_h + 0.035),
            (0.035, 0.78),
            (0.72, 0.72, 0.70, 1),
            euler="1.5708 0 0",
            friction="0.35 0.01 0.001",
        )


def _add_k_rails(world, x0, hard):
    rail_h = 0.10 if hard else 0.06
    spacing = 0.32 if hard else 0.45
    for i in range(6 if hard else 4):
        _box(
            world,
            f"k_rail_{i}",
            x0 - 0.75 + i * spacing,
            0.0,
            rail_h / 2,
            0.05,
            0.95,
            rail_h / 2,
            (0.80, 0.80, 0.78, 1),
            yaw=math.radians(45),
            friction="0.55 0.02 0.002",
        )


def _add_negative_obstacles(world, x0, hard):
    gap_w = 0.28 if hard else 0.16
    _box(world, "crate_field_floor_a", x0 - 0.55, 0.0, 0.05, 0.45, 0.85, 0.05, (0.34, 0.34, 0.30, 1))
    _box(world, "crate_field_floor_b", x0 + 0.55, 0.0, 0.05, 0.45, 0.85, 0.05, (0.34, 0.34, 0.30, 1))
    _box(world, "gap_visual_dark", x0, 0.0, -0.02, gap_w, 0.78, 0.01, (0.02, 0.02, 0.02, 1), yaw=math.radians(35 if hard else 20))
    _box(world, "diagonal_hill", x0 + 1.15, 0.0, 0.08, 0.38, 0.85, 0.04, (0.45, 0.45, 0.40, 1), roll=math.radians(10 if hard else 5))


def build_scene(terrain: str, difficulty: str) -> ET.Element:
    hard = difficulty == "hard"
    root = ET.Element("mujoco", {"model": f"go2_{terrain}_{difficulty}_roughnav"})
    ET.SubElement(root, "compiler", {"angle": "radian", "coordinate": "local"})
    ET.SubElement(root, "option", {"timestep": "0.002", "gravity": "0 0 -9.81"})
    ET.SubElement(root, "include", {"file": "go2.xml"})
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {"name": "grid", "type": "2d", "builtin": "checker", "rgb1": ".18 .20 .22", "rgb2": ".24 .25 .26", "width": "512", "height": "512"})
    ET.SubElement(asset, "material", {"name": "groundmat", "texture": "grid", "texrepeat": "6 6", "reflectance": "0.08"})
    world = ET.SubElement(root, "worldbody")
    _geom(world, "ground", "plane", (0, 0, 0), (18, 3, 0.02), (0.22, 0.24, 0.24, 1), material="groundmat", friction="0.8 0.03 0.003")
    ET.SubElement(world, "light", {"name": "key", "pos": "0 -3 4", "dir": "0 0 -1", "diffuse": "0.8 0.8 0.8"})
    ET.SubElement(world, "camera", {"name": "track", "pos": "-2.5 -4.5 2.2", "xyaxes": "1 0 0 0 0.45 0.89"})

    sections = {
        "crossing_ramps": _add_crossing_ramps,
        "soft_floor": _add_soft_floor,
        "pallet_pipes": _add_pallet_pipes,
        "k_rails": _add_k_rails,
        "negative_obstacles": _add_negative_obstacles,
    }
    if terrain == "mixed_course":
        for x0, fn in zip((-4.5, -2.3, 0.0, 2.25, 4.5), sections.values()):
            fn(world, x0, hard)
    else:
        sections[terrain](world, 0.0, hard)
    return root


def write_scene(root: ET.Element, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    rough = ET.tostring(root, encoding="utf-8")
    pretty = minidom.parseString(rough).toprettyxml(indent="  ")
    out.write_text(pretty, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--terrain", default="mixed_course", choices=["crossing_ramps", "soft_floor", "pallet_pipes", "k_rails", "negative_obstacles", "mixed_course"])
    parser.add_argument("--difficulty", default="hard", choices=["easy", "hard"])
    parser.add_argument("--out", default="mujoco/icra2023_go2_rough_course.xml")
    args = parser.parse_args()
    out = Path(args.out).expanduser()
    write_scene(build_scene(args.terrain, args.difficulty), out)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
