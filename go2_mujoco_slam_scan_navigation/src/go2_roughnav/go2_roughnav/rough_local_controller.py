#!/usr/bin/env python3
"""Terrain-aware path follower with simple stuck recovery for Go2."""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class RoughLocalController(Node):
    def __init__(self):
        super().__init__("rough_local_controller")
        self.declare_parameter("path_topic", "/trg_path")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("traversability_topic", "/traversability_map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/roughnav/gait_mode")
        self.declare_parameter("lookahead_m", 0.75)
        self.declare_parameter("max_linear_mps", 0.45)
        self.declare_parameter("max_angular_rps", 0.8)
        self.declare_parameter("rough_speed_scale", 0.45)
        self.declare_parameter("goal_tolerance_m", 0.25)
        self.declare_parameter("stuck_window_s", 1.5)
        self.declare_parameter("stuck_min_progress_m", 0.05)

        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.max_v = float(self.get_parameter("max_linear_mps").value)
        self.max_w = float(self.get_parameter("max_angular_rps").value)
        self.rough_scale = float(self.get_parameter("rough_speed_scale").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance_m").value)
        self.stuck_window = float(self.get_parameter("stuck_window_s").value)
        self.stuck_progress = float(self.get_parameter("stuck_min_progress_m").value)

        self.path = []
        self.odom = None
        self.grid = None
        self.pose_hist = deque()
        self.recovery_until = None
        self.last_cmd = Twist()

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.mode_pub = self.create_publisher(String, str(self.get_parameter("mode_topic").value), 10)
        self.create_subscription(Path, str(self.get_parameter("path_topic").value), self._path_cb, 1)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("traversability_topic").value), self._grid_cb, 1)
        self.create_timer(0.05, self._tick)
        self.get_logger().info("Rough local controller ready")

    def _path_cb(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        self.pose_hist.append((now, p.x, p.y))
        while self.pose_hist and now - self.pose_hist[0][0] > self.stuck_window:
            self.pose_hist.popleft()

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        self.grid = msg

    def _tick(self) -> None:
        if self.odom is None:
            return
        now = self.get_clock().now()
        if self.recovery_until is not None and now < self.recovery_until:
            self._publish_recovery()
            return
        self.recovery_until = None

        if not self.path:
            self._publish_stop("idle")
            return

        p = self.odom.pose.pose.position
        yaw = _yaw_from_quat(self.odom.pose.pose.orientation)
        target = self._select_target(p.x, p.y)
        if target is None:
            self._publish_stop("goal")
            return

        risk = self._local_risk()
        mode = self._mode_from_risk(risk)
        if self._is_stuck() and self.last_cmd.linear.x > 0.08:
            self.recovery_until = now + rclpy.duration.Duration(seconds=0.9)
            self._publish_recovery()
            return

        dx, dy = target[0] - p.x, target[1] - p.y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        err = _wrap(heading - yaw)
        speed_scale = self.rough_scale if risk >= 45 else 1.0
        cmd = Twist()
        cmd.linear.x = float(np.clip(self.max_v * speed_scale * max(0.0, math.cos(err)), 0.0, self.max_v))
        cmd.angular.z = float(np.clip(1.6 * err, -self.max_w, self.max_w))
        if dist < self.goal_tol:
            cmd.linear.x = 0.0
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.mode_pub.publish(String(data=f"{mode}: risk={risk:.1f}"))

    def _select_target(self, x: float, y: float):
        dists = [math.hypot(px - x, py - y) for px, py in self.path]
        if not dists:
            return None
        nearest = int(np.argmin(dists))
        if nearest == len(self.path) - 1 and dists[nearest] < self.goal_tol:
            return None
        for px, py in self.path[nearest:]:
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return self.path[-1]

    def _local_risk(self) -> float:
        if self.grid is None or not self.grid.data:
            return 60.0
        arr = np.asarray(self.grid.data, dtype=np.int16)
        arr = arr[arr >= 0]
        if arr.size == 0:
            return 60.0
        return float(np.percentile(arr, 80))

    @staticmethod
    def _mode_from_risk(risk: float) -> str:
        if risk >= 75:
            return "high_clearance_probe"
        if risk >= 45:
            return "high_clearance_slow"
        return "normal_trot"

    def _is_stuck(self) -> bool:
        if len(self.pose_hist) < 2:
            return False
        _, x0, y0 = self.pose_hist[0]
        _, x1, y1 = self.pose_hist[-1]
        return math.hypot(x1 - x0, y1 - y0) < self.stuck_progress

    def _publish_recovery(self) -> None:
        cmd = Twist()
        cmd.linear.x = -0.12
        cmd.angular.z = 0.35
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.mode_pub.publish(String(data="recovery_backstep_turn"))

    def _publish_stop(self, mode: str) -> None:
        self.last_cmd = Twist()
        self.cmd_pub.publish(self.last_cmd)
        self.mode_pub.publish(String(data=mode))


def main(args=None):
    rclpy.init(args=args)
    node = RoughLocalController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
