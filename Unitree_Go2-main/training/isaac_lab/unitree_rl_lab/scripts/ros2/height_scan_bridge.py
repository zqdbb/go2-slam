#!/usr/bin/env python3
"""FAST-LIO point cloud to V5 273-dim height_scan bridge.

Input:
  /cloud_registered or /livox/lidar  (sensor_msgs/PointCloud2)
  /Odometry                          (nav_msgs/Odometry)

Output:
  /rl/height_scan                    (std_msgs/Float32MultiArray, 273 values)

The output layout matches the V5 policy convention used by IsaacLab:
  21 x 13 grid, x=[-1.0, 1.0], y=[-0.6, 0.6], row-major with x first.
  Each value is base_z - terrain_z - offset, clipped to [-1, 1].

IsaacLab's RayCasterCfg for this project uses ``ray_alignment="yaw"``. The real
bridge therefore projects the FAST-LIO cloud into a yaw-only base frame instead
of the full roll/pitch body frame. That keeps the deployed height_scan closer to
the V5 training distribution.
"""

from __future__ import annotations

import argparse
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
HEIGHT_SCAN_OFFSET = 0.43


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
    def __init__(
        self,
        cloud_topic: str,
        odom_topic: str,
        out_topic: str,
        empty_value: float,
        point_stride: int,
    ):
        super().__init__("height_scan_bridge")
        self.robot_pos = None
        self.robot_yaw_rot = None
        self.empty_value = float(empty_value)
        self.point_stride = max(1, int(point_stride))

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

        # FAST-LIO cloud is typically in the map/camera_init frame. Convert to base.
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

            # IsaacLab mdp.height_scan shape, adjusted for MuJoCo/Isaac base-height offset.
            # p_b[2] is hit_z - base_z, so the matching value is -p_b[2] - offset.
            h = float(np.clip(-p_b[2] - HEIGHT_SCAN_OFFSET, SCAN_CLIP[0], SCAN_CLIP[1]))
            if counts[ix, iy] == 0:
                heights[ix, iy] = h
            else:
                heights[ix, iy] = min(float(heights[ix, iy]), h)
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--out-topic", default="/rl/height_scan")
    parser.add_argument("--empty-value", type=float, default=-1.0)
    parser.add_argument(
        "--point-stride",
        type=int,
        default=1,
        help="Use every Nth point to reduce CPU load.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = HeightScanBridge(
        args.cloud_topic,
        args.odom_topic,
        args.out_topic,
        args.empty_value,
        args.point_stride,
    )
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
