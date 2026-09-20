#!/usr/bin/env python3
"""Convert TRG nav_msgs/Path into conservative Go2 /cmd_vel commands.

`angular.z` is always yaw rate in rad/s. Absolute heading targets must be
converted to yaw rate before publishing Twist.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PathToCmdVel(Node):
    def __init__(self):
        super().__init__("path_to_cmd_vel")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("height_scan_topic", "/rl/height_scan")
        self.declare_parameter("lookahead_m", 0.8)
        self.declare_parameter("stop_dist_m", 0.35)
        self.declare_parameter("stale_after_s", 1.0)
        self.declare_parameter("max_lin_x", 0.35)
        self.declare_parameter("max_lin_y", 0.20)
        self.declare_parameter("max_ang_z", 0.70)
        self.declare_parameter("yaw_gain", 1.5)
        self.declare_parameter("body_x_gain", 0.6)
        self.declare_parameter("body_y_gain", 0.5)
        self.declare_parameter("use_height_scan_speed_limit", True)
        self.declare_parameter("height_step_slow_m", 0.08)
        self.declare_parameter("height_step_crawl_m", 0.16)
        self.declare_parameter("unknown_slow_ratio", 0.45)

        self.path_topic = str(self.get_parameter("path_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.height_scan_topic = str(self.get_parameter("height_scan_topic").value)
        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.stop_dist = float(self.get_parameter("stop_dist_m").value)
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        self.max_x = float(self.get_parameter("max_lin_x").value)
        self.max_y = float(self.get_parameter("max_lin_y").value)
        self.max_z = float(self.get_parameter("max_ang_z").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.body_x_gain = float(self.get_parameter("body_x_gain").value)
        self.body_y_gain = float(self.get_parameter("body_y_gain").value)
        self.use_scan_limit = bool(self.get_parameter("use_height_scan_speed_limit").value)
        self.height_step_slow = float(self.get_parameter("height_step_slow_m").value)
        self.height_step_crawl = float(self.get_parameter("height_step_crawl_m").value)
        self.unknown_slow_ratio = float(self.get_parameter("unknown_slow_ratio").value)

        self.path: list[tuple[float, float]] = []
        self.path_time = 0.0
        self.odom: Odometry | None = None
        self.odom_time = 0.0
        self.height_scan: np.ndarray | None = None
        self.height_time = 0.0

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Path, self.path_topic, self._path_cb, 1)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Float32MultiArray, self.height_scan_topic, self._height_cb, 10)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            "path_to_cmd_vel: "
            f"path={self.path_topic}, odom={self.odom_topic}, cmd={self.cmd_vel_topic}, "
            f"height={self.height_scan_topic}"
        )

    def _path_cb(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_time = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_time = time.monotonic()

    def _height_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) == 273:
            self.height_scan = np.asarray(msg.data, dtype=np.float32)
            self.height_time = time.monotonic()

    def _publish_stop(self) -> None:
        self.pub.publish(Twist())

    def _height_speed_scale(self, now: float) -> float:
        if not self.use_scan_limit or self.height_scan is None:
            return 1.0
        if now - self.height_time > self.stale_after:
            return 0.65

        # Match the V5/V6 scan layout: 21 x 13, x-major flatten. Use the
        # front-center band for speed limiting so rear/side artifacts do not
        # unnecessarily stop the robot.
        scan = self.height_scan.reshape(21, 13)[11:21, 3:10].reshape(-1)
        unknown = scan <= -0.99
        unknown_ratio = float(np.mean(unknown))
        valid = scan[~unknown]
        if valid.size == 0:
            return 0.45

        max_h = float(np.max(valid))
        roughness = float(np.max(valid) - np.min(valid))
        scale = 1.0
        if unknown_ratio > self.unknown_slow_ratio:
            scale = min(scale, 0.65)
        if max_h > self.height_step_crawl or roughness > 0.22:
            scale = min(scale, 0.45)
        elif max_h > self.height_step_slow or roughness > 0.14:
            scale = min(scale, 0.65)
        return scale

    def _tick(self) -> None:
        now = time.monotonic()
        if self.odom is None or not self.path:
            self._publish_stop()
            return
        if now - self.odom_time > self.stale_after or now - self.path_time > self.stale_after:
            self._publish_stop()
            return

        pose = self.odom.pose.pose
        p = pose.position
        yaw = yaw_from_quat(pose.orientation)
        dists = [math.hypot(x - p.x, y - p.y) for x, y in self.path]
        nearest = int(np.argmin(dists))

        if dists[-1] < self.stop_dist:
            self._publish_stop()
            return

        target = self.path[-1]
        for candidate in self.path[nearest:]:
            if math.hypot(candidate[0] - p.x, candidate[1] - p.y) >= self.lookahead:
                target = candidate
                break

        dx = target[0] - p.x
        dy = target[1] - p.y
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        bx = c * dx - s * dy
        by = s * dx + c * dy
        heading_err = wrap_angle(math.atan2(dy, dx) - yaw)

        speed_scale = self._height_speed_scale(now)
        yaw_slow = max(0.0, 1.0 - min(abs(heading_err), 0.8) / 0.8)

        cmd = Twist()
        cmd.linear.x = float(np.clip(self.body_x_gain * bx, -0.10, self.max_x * speed_scale * yaw_slow))
        cmd.linear.y = float(np.clip(self.body_y_gain * by, -self.max_y * speed_scale, self.max_y * speed_scale))
        cmd.angular.z = float(np.clip(self.yaw_gain * heading_err, -self.max_z, self.max_z))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathToCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
