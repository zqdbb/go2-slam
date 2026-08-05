#!/usr/bin/env python3
"""Accumulate MuJoCo LiDAR scans along teleop odometry into a saved PCD map."""

from __future__ import annotations

import argparse
import math
import os
import signal
import struct
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3, dtype=np.float32)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float32,
    )


def read_xyz(msg: PointCloud2) -> np.ndarray:
    offsets = {field.name: field.offset for field in msg.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        return np.zeros((0, 3), dtype=np.float32)
    count = msg.width * msg.height
    pts = np.empty((count, 3), dtype=np.float32)
    for i in range(count):
        base = i * msg.point_step
        pts[i, 0] = struct.unpack_from("<f", msg.data, base + offsets["x"])[0]
        pts[i, 1] = struct.unpack_from("<f", msg.data, base + offsets["y"])[0]
        pts[i, 2] = struct.unpack_from("<f", msg.data, base + offsets["z"])[0]
    return pts[np.isfinite(pts).all(axis=1)]


def make_cloud(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(len(points))
    msg.is_dense = False
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    xyzi = np.zeros((len(points), 4), dtype=np.float32)
    if len(points):
        xyzi[:, :3] = points.astype(np.float32, copy=False)
        xyzi[:, 3] = 1.0
    msg.data = xyzi.tobytes()
    return msg


class TeleopMapAccumulator(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("teleop_map_accumulator")
        self.args = args
        self.frame_id = args.frame_id
        self.out_path = Path(args.output).expanduser()
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self.base_pos: np.ndarray | None = None
        self.base_rot: np.ndarray | None = None
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.last_save = time.monotonic()
        self.last_pub = time.monotonic()
        self.cloud_count = 0

        self.pub = self.create_publisher(PointCloud2, args.out_topic, 1)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 10)
        self.create_subscription(PointCloud2, args.cloud_topic, self._cloud_cb, 5)
        self.create_timer(1.0 / max(args.publish_rate, 0.1), self._publish)
        self.get_logger().info(
            f"teleop map: cloud={args.cloud_topic}, odom={args.odom_topic}, "
            f"out={args.out_topic}, save={self.out_path}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.base_pos = np.array([p.x, p.y, p.z], dtype=np.float32)
        self.base_rot = quat_xyzw_to_rot(q.x, q.y, q.z, q.w)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if self.base_pos is None or self.base_rot is None:
            return
        local = read_xyz(msg)
        if len(local) == 0:
            return
        if self.args.stride > 1:
            local = local[:: self.args.stride]
        if self.args.max_range > 0:
            local = local[np.linalg.norm(local[:, :2], axis=1) <= self.args.max_range]
        if len(local) == 0:
            return
        lidar_offset = np.array([0.0, 0.0, self.args.lidar_z], dtype=np.float32)
        world = self.base_pos + (self.base_rot @ (local + lidar_offset).T).T
        if self.args.voxel > 0:
            world = np.unique(np.round(world / self.args.voxel).astype(np.int32), axis=0).astype(np.float32) * self.args.voxel
        self.points = np.vstack([self.points, world])
        if len(self.points) > self.args.max_points:
            self.points = self.points[-self.args.max_points :]
        self.cloud_count += 1
        now = time.monotonic()
        if now - self.last_save >= self.args.save_period:
            self.save()
            self.last_save = now

    def _publish(self) -> None:
        if len(self.points) == 0:
            return
        self.pub.publish(make_cloud(self.points, self.frame_id, self.get_clock().now().to_msg()))

    def save(self) -> None:
        pts = self.points
        tmp = self.out_path.with_suffix(self.out_path.suffix + ".tmp")
        with tmp.open("w", encoding="ascii") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n")
            for x, y, z in pts:
                f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")
        tmp.replace(self.out_path)
        self.get_logger().info(f"saved {len(pts)} pts -> {self.out_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=42)
    parser.add_argument("--cloud-topic", default="/utlidar/cloud")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--out-topic", default="/teleop_map")
    parser.add_argument("--frame-id", default="camera_init")
    parser.add_argument("--output", default="/workspace/Unitree_Go2/artifacts/slam_maps/teleop_map_latest.pcd")
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--voxel", type=float, default=0.02)
    parser.add_argument("--lidar-z", type=float, default=0.15)
    parser.add_argument("--max-range", type=float, default=8.0)
    parser.add_argument("--max-points", type=int, default=600000)
    parser.add_argument("--publish-rate", type=float, default=2.0)
    parser.add_argument("--save-period", type=float, default=5.0)
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = TeleopMapAccumulator(args)

    stop = {"value": False}

    def _stop(_signum, _frame):
        stop["value"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        while rclpy.ok() and not stop["value"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
