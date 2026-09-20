#!/usr/bin/env python3
"""Build a local traversability OccupancyGrid from FAST-LIO registered clouds."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class TraversabilityNode(Node):
    def __init__(self):
        super().__init__("traversability_node")
        self.declare_parameter("pointcloud_topic", "/cloud_registered")
        self.declare_parameter("map_topic", "/traversability_map")
        self.declare_parameter("terrain_flags_topic", "/roughnav/terrain_flags")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("width_m", 12.0)
        self.declare_parameter("height_m", 6.0)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("step_height_warn_m", 0.08)
        self.declare_parameter("step_height_block_m", 0.18)
        self.declare_parameter("unknown_is_risky", True)

        self.resolution = float(self.get_parameter("resolution").value)
        self.width_m = float(self.get_parameter("width_m").value)
        self.height_m = float(self.get_parameter("height_m").value)
        self.grid_w = int(round(self.width_m / self.resolution))
        self.grid_h = int(round(self.height_m / self.resolution))
        self.min_points = int(self.get_parameter("min_points_per_cell").value)
        self.warn_step = float(self.get_parameter("step_height_warn_m").value)
        self.block_step = float(self.get_parameter("step_height_block_m").value)
        self.unknown_is_risky = bool(self.get_parameter("unknown_is_risky").value)

        cloud_topic = str(self.get_parameter("pointcloud_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        flag_topic = str(self.get_parameter("terrain_flags_topic").value)
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, 1)
        self.flag_pub = self.create_publisher(String, flag_topic, 1)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, 1)
        self.get_logger().info(f"Traversability from {cloud_topic} to {map_topic}, {self.grid_w}x{self.grid_h}")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        points = self._read_xyz(msg)
        if points.size == 0:
            return

        half_w = self.width_m / 2.0
        half_h = self.height_m / 2.0
        ix = np.floor((points[:, 0] + half_w) / self.resolution).astype(np.int32)
        iy = np.floor((points[:, 1] + half_h) / self.resolution).astype(np.int32)
        valid = (ix >= 0) & (ix < self.grid_w) & (iy >= 0) & (iy < self.grid_h)
        ix, iy, z = ix[valid], iy[valid], points[:, 2][valid]
        if z.size == 0:
            return

        z_min = np.full((self.grid_h, self.grid_w), np.inf, dtype=np.float32)
        z_max = np.full((self.grid_h, self.grid_w), -np.inf, dtype=np.float32)
        count = np.zeros((self.grid_h, self.grid_w), dtype=np.int32)
        np.minimum.at(z_min, (iy, ix), z)
        np.maximum.at(z_max, (iy, ix), z)
        np.add.at(count, (iy, ix), 1)

        observed = count >= self.min_points
        z_span = np.where(observed, z_max - z_min, 0.0)
        cost = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)
        if self.unknown_is_risky:
            cost[:] = 65
        cost[observed] = 0

        ramp = observed & (z_span >= self.warn_step) & (z_span < self.block_step)
        blocked = observed & (z_span >= self.block_step)
        cost[ramp] = np.clip(30 + 70 * (z_span[ramp] - self.warn_step) / (self.block_step - self.warn_step), 30, 95).astype(np.int8)
        cost[blocked] = 100

        msg_out = OccupancyGrid()
        msg_out.header = msg.header
        msg_out.info.resolution = self.resolution
        msg_out.info.width = self.grid_w
        msg_out.info.height = self.grid_h
        msg_out.info.origin.position.x = -half_w
        msg_out.info.origin.position.y = -half_h
        msg_out.info.origin.orientation.w = 1.0
        msg_out.data = cost.reshape(-1).astype(int).tolist()
        self.map_pub.publish(msg_out)
        self._publish_flags(cost, observed, z_span)

    def _publish_flags(self, cost: np.ndarray, observed: np.ndarray, z_span: np.ndarray) -> None:
        blocked_ratio = float(np.mean(cost == 100))
        rough_ratio = float(np.mean((cost >= 30) & (cost < 100)))
        unknown_ratio = float(np.mean(~observed))
        max_step = float(np.max(z_span)) if np.any(observed) else 0.0
        flags = {
            "blocked_ratio": round(blocked_ratio, 3),
            "rough_ratio": round(rough_ratio, 3),
            "unknown_ratio": round(unknown_ratio, 3),
            "max_step_m": round(max_step, 3),
        }
        self.flag_pub.publish(String(data=str(flags)))

    @staticmethod
    def _read_xyz(msg: PointCloud2) -> np.ndarray:
        rows: Iterable[tuple[float, float, float]] = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = [(float(x), float(y), float(z)) for x, y, z in rows if math.isfinite(x) and math.isfinite(y) and math.isfinite(z)]
        return np.asarray(pts, dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
