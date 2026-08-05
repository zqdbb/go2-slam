#!/usr/bin/env python3
"""FAST-LIO point cloud to V5 273-dim height_scan bridge."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


SCAN_SIZE_X = 2.0
SCAN_SIZE_Y = 1.2
SCAN_RES = 0.1
SCAN_NX = round(SCAN_SIZE_X / SCAN_RES) + 1
SCAN_NY = round(SCAN_SIZE_Y / SCAN_RES) + 1
SCAN_DIM = SCAN_NX * SCAN_NY
SCAN_CLIP = (-1.0, 1.0)


def quat_to_yaw(q) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_rot(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


class HeightScanBridge(Node):
    def __init__(self):
        super().__init__("height_scan_bridge")
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("out_topic", "/rl/height_scan")
        self.declare_parameter("empty_value", -1.0)
        self.declare_parameter("point_stride", 1)

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)
        self.empty_value = float(self.get_parameter("empty_value").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        self.robot_pos = None
        self.robot_yaw_rot = None
        self.pub = self.create_publisher(Float32MultiArray, out_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, 5)

        self.get_logger().info(
            "height_scan bridge: "
            f"cloud={cloud_topic}, odom={odom_topic}, out={out_topic}, "
            f"empty={self.empty_value}, stride={self.point_stride}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.robot_yaw_rot = yaw_to_rot(quat_to_yaw((q.x, q.y, q.z, q.w)))

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if self.robot_pos is None or self.robot_yaw_rot is None:
            return

        heights = np.full((SCAN_NX, SCAN_NY), self.empty_value, dtype=np.float32)
        counts = np.zeros((SCAN_NX, SCAN_NY), dtype=np.int32)

        for idx, (x, y, z) in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if idx % self.point_stride != 0:
                continue

            p_w = np.array([x, y, z], dtype=np.float64)
            p_b = self.robot_yaw_rot.T @ (p_w - self.robot_pos)

            if not (-SCAN_SIZE_X / 2 <= p_b[0] <= SCAN_SIZE_X / 2):
                continue
            if not (-SCAN_SIZE_Y / 2 <= p_b[1] <= SCAN_SIZE_Y / 2):
                continue

            ix = int(round((p_b[0] + SCAN_SIZE_X / 2) / SCAN_RES))
            iy = int(round((p_b[1] + SCAN_SIZE_Y / 2) / SCAN_RES))
            ix = max(0, min(SCAN_NX - 1, ix))
            iy = max(0, min(SCAN_NY - 1, iy))

            height = float(np.clip(p_b[2], SCAN_CLIP[0], SCAN_CLIP[1]))
            heights[ix, iy] = max(float(heights[ix, iy]), height)
            counts[ix, iy] += 1

        out = Float32MultiArray()
        out.data = heights.reshape(SCAN_DIM).astype(np.float32).tolist()
        self.pub.publish(out)

        if not hasattr(self, "_log_count"):
            self._log_count = 0
        self._log_count += 1
        if self._log_count % 20 == 0:
            filled = int((counts > 0).sum())
            self.get_logger().info(f"height_scan published: filled={filled}/{SCAN_DIM}")


def main(args=None):
    rclpy.init(args=args)
    node = HeightScanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
