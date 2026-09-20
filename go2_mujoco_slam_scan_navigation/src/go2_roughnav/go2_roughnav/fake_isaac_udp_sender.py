#!/usr/bin/env python3
"""Send Isaac-like UDP packets for ROS-side debugging without GPU/Isaac Gym."""

from __future__ import annotations

import math
import pickle
import socket
import time
import zlib

import numpy as np


def _terrain_points(t: float) -> np.ndarray:
    xs = np.linspace(0.2, 6.0, 80, dtype=np.float32)
    ys = np.linspace(-2.5, 2.5, 50, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys)
    zz = 0.08 * np.sin(2.0 * xx + 0.3 * t) + 0.05 * np.cos(3.0 * yy)
    obstacle = (np.abs(xx - 2.4) < 0.12) & (np.abs(yy) < 1.2)
    zz[obstacle] += 0.35
    intensity = np.clip(1.0 - xx / 8.0, 0.0, 1.0)
    pts = np.stack([xx.reshape(-1), yy.reshape(-1), zz.reshape(-1), intensity.reshape(-1)], axis=1)
    return pts[::3].astype(np.float32)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", 5010)
    start = time.time()
    print("[fake_isaac_udp] sending packets to udp://127.0.0.1:5010")
    while True:
        now = time.time()
        t = now - start
        x = 0.15 * t
        yaw = 0.15 * math.sin(0.3 * t)
        packet = {
            "stamp": now,
            "points": _terrain_points(t),
            "root_pos": np.array([x, 0.0, 0.42], dtype=np.float32),
            "root_quat_xyzw": np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)], dtype=np.float32),
            "lin_vel": np.array([0.15, 0.0, 0.0], dtype=np.float32),
            "ang_vel": np.array([0.0, 0.0, 0.045 * math.cos(0.3 * t)], dtype=np.float32),
            "lin_acc": np.array([0.0, 0.0, 9.81], dtype=np.float32),
        }
        payload = zlib.compress(pickle.dumps(packet, protocol=pickle.HIGHEST_PROTOCOL), level=1)
        sock.sendto(payload, target)
        time.sleep(0.1)


if __name__ == "__main__":
    main()
