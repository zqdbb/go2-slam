#!/usr/bin/env python3
"""Publish simple cyclic goals for autonomous loop smoke tests."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class AutoGoalPublisher(Node):
    def __init__(self):
        super().__init__("auto_goal_publisher")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("goal_radius_m", 2.0)
        self.declare_parameter("publish_period_s", 8.0)
        self.declare_parameter("goal_tolerance_m", 0.6)
        self.declare_parameter("enable", False)
        self.enable = bool(self.get_parameter("enable").value)
        self.frame = str(self.get_parameter("fixed_frame").value)
        self.radius = float(self.get_parameter("goal_radius_m").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance_m").value)
        self.odom = None
        self.goal = None
        self.goal_idx = 0
        self.offsets = [(1.0, 0.0), (0.7, 0.7), (0.0, 1.0), (-0.7, 0.7), (1.0, 0.0)]
        self.pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10)
        self.create_timer(float(self.get_parameter("publish_period_s").value), self._timer_cb)
        self.get_logger().info(f"Auto goal publisher enable={self.enable}")

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        if self.enable and self.goal is not None:
            p = msg.pose.pose.position
            gp = self.goal.pose.position
            if math.hypot(gp.x - p.x, gp.y - p.y) < self.goal_tol:
                self._publish_next_goal()

    def _timer_cb(self) -> None:
        if self.enable and self.odom is not None:
            self._publish_next_goal()

    def _publish_next_goal(self) -> None:
        p = self.odom.pose.pose.position
        ox, oy = self.offsets[self.goal_idx % len(self.offsets)]
        self.goal_idx += 1
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.pose.position.x = float(p.x + ox * self.radius)
        msg.pose.position.y = float(p.y + oy * self.radius)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal = msg
        self.pub.publish(msg)
        self.get_logger().info(f"Published goal x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = AutoGoalPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
