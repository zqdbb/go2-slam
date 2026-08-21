#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class StuckRecovery(Node):
    """Arbitrate navigation velocity and execute bounded narrow-space recovery."""

    def __init__(self):
        super().__init__('stuck_recovery')
        self.declare_parameter('stuck_window_s', 2.5)
        self.declare_parameter('stuck_distance_m', 0.055)
        self.declare_parameter('stuck_yaw_rad', 0.10)
        self.declare_parameter('backup_duration_s', 2.2)
        self.declare_parameter('turn_duration_s', 1.8)
        self.declare_parameter('backup_speed', 0.55)
        self.declare_parameter('turn_walk_speed', 0.50)
        self.declare_parameter('turn_yaw_speed', 1.0)
        self.declare_parameter('cmd_timeout_s', 0.35)
        self.declare_parameter('recovery_cooldown_s', 3.0)
        self.declare_parameter('max_recovery_attempts', 3)

        p = lambda name: self.get_parameter(name).value
        self.stuck_window = float(p('stuck_window_s'))
        self.stuck_distance = float(p('stuck_distance_m'))
        self.stuck_yaw = float(p('stuck_yaw_rad'))
        self.backup_duration = float(p('backup_duration_s'))
        self.turn_duration = float(p('turn_duration_s'))
        self.backup_speed = float(p('backup_speed'))
        self.turn_walk_speed = float(p('turn_walk_speed'))
        self.turn_yaw_speed = float(p('turn_yaw_speed'))
        self.cmd_timeout = float(p('cmd_timeout_s'))
        self.cooldown = float(p('recovery_cooldown_s'))
        self.max_attempts = int(p('max_recovery_attempts'))

        self.nav_cmd = Twist()
        self.nav_cmd_time = 0.0
        self.pose_history = deque()
        self.have_odom = False
        self.left_clearance = 10.0
        self.right_clearance = 10.0
        self.front_clearance = 10.0
        self.rear_clearance = 10.0
        self.recovery_phase = 'idle'
        self.phase_start = 0.0
        self.turn_sign = 1.0
        self.attempts = 0
        self.cooldown_until = 0.0
        self.blocked = False
        self.last_goal = None
        self.self_goal_until = 0.0
        self.waiting_for_fresh_traj = False
        self.replan_deadline = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.status_pub = self.create_publisher(Bool, '/planning/stuck_recovery_active', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_callback, 20)
        self.create_subscription(Odometry, '/quad_0/body_pose', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bspline, '/planning/bspline', self.trajectory_callback, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('Stuck recovery ready: /cmd_vel_nav -> /cmd_vel')

    def cmd_callback(self, msg: Twist):
        self.nav_cmd = msg
        self.nav_cmd_time = time.monotonic()

    def goal_callback(self, msg: PoseStamped):
        self.last_goal = msg
        if time.monotonic() >= self.self_goal_until:
            self.attempts = 0
            self.blocked = False
            self.recovery_phase = 'idle'
            self.waiting_for_fresh_traj = False
            self.pose_history.clear()

    def trajectory_callback(self, _msg: Bspline):
        if self.waiting_for_fresh_traj:
            self.waiting_for_fresh_traj = False
            self.pose_history.clear()
            self.cooldown_until = time.monotonic() + self.cooldown
            self.get_logger().info('Fresh SCAN trajectory received; navigation resumed')

    def odom_callback(self, msg: Odometry):
        now = time.monotonic()
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose_history.append((now, float(p.x), float(p.y), yaw))
        while self.pose_history and now - self.pose_history[0][0] > self.stuck_window:
            self.pose_history.popleft()
        self.have_odom = True

    @staticmethod
    def sector_clearance(scan: LaserScan, low: float, high: float) -> float:
        values = []
        angle = scan.angle_min
        limit = min(len(scan.ranges), int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 2)
        for index in range(limit):
            wrapped = normalize_angle(angle)
            in_sector = low <= wrapped <= high
            if in_sector:
                value = scan.ranges[index]
                if math.isinf(value):
                    value = scan.range_max
                if math.isfinite(value) and value >= scan.range_min:
                    values.append(min(float(value), float(scan.range_max)))
            angle += scan.angle_increment
        if not values:
            return 0.0
        values.sort()
        return values[max(0, int(0.2 * (len(values) - 1)))]

    def scan_callback(self, msg: LaserScan):
        self.front_clearance = self.sector_clearance(msg, -0.55, 0.55)
        self.left_clearance = self.sector_clearance(msg, 0.70, 2.35)
        self.right_clearance = self.sector_clearance(msg, -2.35, -0.70)
        rear_a = self.sector_clearance(msg, 2.55, math.pi)
        rear_b = self.sector_clearance(msg, -math.pi, -2.55)
        self.rear_clearance = min(rear_a, rear_b) if rear_a and rear_b else max(rear_a, rear_b)

    def commanded_motion(self) -> bool:
        return (
            abs(self.nav_cmd.linear.x) > 0.24
            or abs(self.nav_cmd.linear.y) > 0.20
            or abs(self.nav_cmd.angular.z) > 0.45
        )

    def is_stuck(self, now: float) -> bool:
        if (
            not self.have_odom
            or self.blocked
            or now < self.cooldown_until
            or not self.commanded_motion()
            or now - self.nav_cmd_time > self.cmd_timeout
            or len(self.pose_history) < 2
        ):
            return False
        first = self.pose_history[0]
        last = self.pose_history[-1]
        if last[0] - first[0] < self.stuck_window * 0.85:
            return False
        displacement = math.hypot(last[1] - first[1], last[2] - first[2])
        yaw_change = abs(normalize_angle(last[3] - first[3]))
        return displacement < self.stuck_distance and yaw_change < self.stuck_yaw

    def start_recovery(self, now: float):
        if self.attempts >= self.max_attempts:
            self.blocked = True
            self.get_logger().error(
                'Recovery failed three times; stopped. Send a new 2D Goal after checking clearance.'
            )
            return
        self.attempts += 1
        self.turn_sign = 1.0 if self.left_clearance >= self.right_clearance else -1.0
        self.recovery_phase = 'backup' if self.rear_clearance > 0.45 else 'turn'
        self.phase_start = now
        self.pose_history.clear()
        side = 'left' if self.turn_sign > 0.0 else 'right'
        self.get_logger().warn(
            f'Stuck detected; recovery {self.attempts}/{self.max_attempts}: '
            f'rear={self.rear_clearance:.2f} m, turning {side}'
        )

    def recovery_command(self, now: float) -> Twist:
        cmd = Twist()
        elapsed = now - self.phase_start
        if self.recovery_phase == 'backup':
            if elapsed < self.backup_duration and self.rear_clearance > 0.35:
                cmd.linear.x = -self.backup_speed
                cmd.angular.z = self.turn_sign * 0.8 * self.turn_yaw_speed
                return cmd
            self.recovery_phase = 'turn'
            self.phase_start = now
            elapsed = 0.0
        if self.recovery_phase == 'turn':
            if elapsed < self.turn_duration:
                if max(self.front_clearance, self.rear_clearance) > 0.38:
                    cmd.linear.x = (
                        self.turn_walk_speed
                        if self.front_clearance >= self.rear_clearance
                        else -self.turn_walk_speed
                    )
                elif max(self.left_clearance, self.right_clearance) > 0.38:
                    cmd.linear.y = self.turn_sign * self.turn_walk_speed
                else:
                    self.recovery_phase = 'idle'
                    self.blocked = True
                    self.get_logger().error('No safe clearance for recovery motion; stopped')
                    return cmd
                cmd.angular.z = self.turn_sign * self.turn_yaw_speed
                return cmd
            self.recovery_phase = 'idle'
            self.pose_history.clear()
            self.get_logger().info('Recovery motion complete; requesting a fresh global/local plan')
            if self.last_goal is not None:
                self.waiting_for_fresh_traj = True
                self.replan_deadline = now + 5.0
                self.last_goal.header.stamp = self.get_clock().now().to_msg()
                self.self_goal_until = now + 1.0
                self.goal_pub.publish(self.last_goal)
            else:
                self.blocked = True
                self.get_logger().error('Recovery completed without a stored goal; stopped')
        return cmd

    def timer_callback(self):
        now = time.monotonic()
        if self.recovery_phase != 'idle':
            cmd = self.recovery_command(now)
            self.cmd_pub.publish(cmd)
            self.status_pub.publish(Bool(data=True))
            return
        if self.waiting_for_fresh_traj:
            if now > self.replan_deadline:
                self.waiting_for_fresh_traj = False
                self.blocked = True
                self.get_logger().error('No fresh SCAN trajectory after recovery; stopped')
            self.cmd_pub.publish(Twist())
            self.status_pub.publish(Bool(data=True))
            return
        if self.is_stuck(now):
            self.start_recovery(now)
            if self.recovery_phase != 'idle':
                self.cmd_pub.publish(self.recovery_command(now))
                self.status_pub.publish(Bool(data=True))
                return
        if self.blocked or now - self.nav_cmd_time > self.cmd_timeout:
            self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(self.nav_cmd)
        self.status_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = StuckRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
