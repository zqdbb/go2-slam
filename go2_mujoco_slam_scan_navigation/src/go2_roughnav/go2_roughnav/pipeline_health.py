#!/usr/bin/env python3
"""Print a compact health report for Go2 FAST-LIO + TRG + RL pipeline."""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32MultiArray


@dataclass
class TopicState:
    count: int = 0
    last_time: float = 0.0
    detail: str = "-"


class PipelineHealth(Node):
    def __init__(self):
        super().__init__("pipeline_health")
        self.declare_parameter("report_period_s", 2.0)
        self.declare_parameter("stale_after_s", 3.0)
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        self.states = {
            "/utlidar/cloud": TopicState(),
            "/utlidar/imu": TopicState(),
            "/Odometry": TopicState(),
            "/cloud_registered": TopicState(),
            "/rl/height_scan": TopicState(),
            "/goal_pose": TopicState(),
            "/path": TopicState(),
            "/cmd_vel": TopicState(),
        }
        self.create_subscription(PointCloud2, "/utlidar/cloud", lambda m: self._cloud("/utlidar/cloud", m), 10)
        self.create_subscription(Imu, "/utlidar/imu", lambda m: self._touch("/utlidar/imu", f"az={m.linear_acceleration.z:.2f}"), 10)
        self.create_subscription(Odometry, "/Odometry", self._odom, 10)
        self.create_subscription(PointCloud2, "/cloud_registered", lambda m: self._cloud("/cloud_registered", m), 10)
        self.create_subscription(Float32MultiArray, "/rl/height_scan", self._height, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal, 10)
        self.create_subscription(Path, "/path", self._path, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.create_timer(float(self.get_parameter("report_period_s").value), self._report)
        self.get_logger().info("Pipeline health monitor started")

    def _now(self) -> float:
        return time.monotonic()

    def _touch(self, topic: str, detail: str = "-") -> None:
        state = self.states[topic]
        state.count += 1
        state.last_time = self._now()
        state.detail = detail

    def _cloud(self, topic: str, msg: PointCloud2) -> None:
        self._touch(topic, f"points={msg.width * msg.height}")

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._touch("/Odometry", f"x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}")

    def _height(self, msg: Float32MultiArray) -> None:
        self._touch("/rl/height_scan", f"n={len(msg.data)}")

    def _goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._touch("/goal_pose", f"x={p.x:.2f}, y={p.y:.2f}")

    def _path(self, msg: Path) -> None:
        self._touch("/path", f"poses={len(msg.poses)}")

    def _cmd(self, msg: Twist) -> None:
        self._touch("/cmd_vel", f"vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")

    def _report(self) -> None:
        now = self._now()
        parts = []
        for topic, state in self.states.items():
            if state.count == 0:
                status = "MISSING"
            elif now - state.last_time > self.stale_after:
                status = "STALE"
            else:
                status = "OK"
            parts.append(f"{topic}:{status}({state.detail})")
        self.get_logger().info(" | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = PipelineHealth()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

