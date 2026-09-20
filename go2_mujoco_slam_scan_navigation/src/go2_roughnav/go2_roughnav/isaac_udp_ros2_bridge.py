#!/usr/bin/env python3
"""Receive Isaac Gym UDP packets and publish ROS 2 sensor topics."""

from __future__ import annotations

import pickle
import socket
import zlib

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


class IsaacUdpRos2Bridge(Node):
    def __init__(self):
        super().__init__("isaac_udp_ros2_bridge")
        self.declare_parameter("udp_ip", "127.0.0.1")
        self.declare_parameter("udp_port", 5010)
        self.declare_parameter("lidar_topic", "/lidar/points")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("isaac_cmd_ip", "127.0.0.1")
        self.declare_parameter("isaac_cmd_port", 5011)
        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("imu_frame", "imu_link")

        ip = str(self.get_parameter("udp_ip").value)
        port = int(self.get_parameter("udp_port").value)
        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_target = (
            str(self.get_parameter("isaac_cmd_ip").value),
            int(self.get_parameter("isaac_cmd_port").value),
        )

        self.cloud_pub = self.create_publisher(PointCloud2, str(self.get_parameter("lidar_topic").value), 10)
        self.imu_pub = self.create_publisher(Imu, str(self.get_parameter("imu_topic").value), 10)
        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self._cmd_cb, 10)
        self.create_timer(0.005, self._poll)
        self.get_logger().info(f"Listening for Isaac Gym UDP packets on udp://{ip}:{port}")
        self.get_logger().info(f"Forwarding /cmd_vel to Isaac Gym on udp://{self.cmd_target[0]}:{self.cmd_target[1]}")

    def _cmd_cb(self, msg: Twist) -> None:
        packet = {
            "lin_x": float(msg.linear.x),
            "lin_y": float(msg.linear.y),
            "ang_z": float(msg.angular.z),
        }
        payload = zlib.compress(pickle.dumps(packet, protocol=pickle.HIGHEST_PROTOCOL), level=1)
        self.cmd_sock.sendto(payload, self.cmd_target)

    def _poll(self) -> None:
        while True:
            try:
                payload, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                packet = pickle.loads(zlib.decompress(payload))
            except Exception as exc:
                self.get_logger().warn(f"Failed to decode Isaac UDP packet: {exc}")
                continue
            self._publish_packet(packet)

    def _stamp(self, packet) -> Time:
        t = float(packet.get("stamp", 0.0))
        sec = int(t)
        return Time(sec=sec, nanosec=int((t - sec) * 1e9))

    def _publish_packet(self, packet) -> None:
        stamp = self._stamp(packet)
        points = np.asarray(packet.get("points", []), dtype=np.float32)
        if points.ndim == 2 and points.shape[1] >= 3:
            self.cloud_pub.publish(self._make_cloud(stamp, points))

        pos = np.asarray(packet.get("root_pos", [0.0, 0.0, 0.0]), dtype=np.float32)
        quat = np.asarray(packet.get("root_quat_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=np.float32)
        lin_vel = np.asarray(packet.get("lin_vel", [0.0, 0.0, 0.0]), dtype=np.float32)
        ang_vel = np.asarray(packet.get("ang_vel", [0.0, 0.0, 0.0]), dtype=np.float32)
        lin_acc = np.asarray(packet.get("lin_acc", [0.0, 0.0, 0.0]), dtype=np.float32)
        self.imu_pub.publish(self._make_imu(stamp, quat, ang_vel, lin_acc))
        self.odom_pub.publish(self._make_odom(stamp, pos, quat, lin_vel, ang_vel))
        self.tf_pub.publish(TFMessage(transforms=[self._make_tf(stamp, pos, quat)]))

    def _make_cloud(self, stamp: Time, points: np.ndarray) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        n = max(1, points.shape[0])
        if points.shape[1] == 3:
            pts = [(float(x), float(y), float(z), 1.0, int(i % 16), float(i) / n * 0.1) for i, (x, y, z) in enumerate(points)]
        else:
            pts = [
                (float(x), float(y), float(z), float(intensity), int(idx % 16), float(idx) / n * 0.1)
                for idx, (x, y, z, intensity) in enumerate(points[:, :4])
            ]
        return point_cloud2.create_cloud(Header(stamp=stamp, frame_id=self.lidar_frame), fields, pts)

    def _make_imu(self, stamp: Time, quat, ang_vel, lin_acc) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        msg.orientation.x = float(quat[0])
        msg.orientation.y = float(quat[1])
        msg.orientation.z = float(quat[2])
        msg.orientation.w = float(quat[3])
        msg.angular_velocity.x = float(ang_vel[0])
        msg.angular_velocity.y = float(ang_vel[1])
        msg.angular_velocity.z = float(ang_vel[2])
        msg.linear_acceleration.x = float(lin_acc[0])
        msg.linear_acceleration.y = float(lin_acc[1])
        msg.linear_acceleration.z = float(lin_acc[2])
        return msg

    def _make_odom(self, stamp: Time, pos, quat, lin_vel, ang_vel) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.fixed_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.x = float(quat[0])
        msg.pose.pose.orientation.y = float(quat[1])
        msg.pose.pose.orientation.z = float(quat[2])
        msg.pose.pose.orientation.w = float(quat[3])
        msg.twist.twist.linear.x = float(lin_vel[0])
        msg.twist.twist.linear.y = float(lin_vel[1])
        msg.twist.twist.linear.z = float(lin_vel[2])
        msg.twist.twist.angular.x = float(ang_vel[0])
        msg.twist.twist.angular.y = float(ang_vel[1])
        msg.twist.twist.angular.z = float(ang_vel[2])
        return msg

    def _make_tf(self, stamp: Time, pos, quat) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.fixed_frame
        msg.child_frame_id = self.base_frame
        msg.transform.translation.x = float(pos[0])
        msg.transform.translation.y = float(pos[1])
        msg.transform.translation.z = float(pos[2])
        msg.transform.rotation.x = float(quat[0])
        msg.transform.rotation.y = float(quat[1])
        msg.transform.rotation.z = float(quat[2])
        msg.transform.rotation.w = float(quat[3])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = IsaacUdpRos2Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
