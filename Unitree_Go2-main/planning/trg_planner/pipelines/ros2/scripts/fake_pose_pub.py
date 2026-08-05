#!/usr/bin/env python3

import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def signal_handler(sig, frame):
    print('Exiting and plotting data...')
    sys.exit(0)


class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'fake_robot_pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'fake_goal', 10)

        self.frame_id = 'map'

        # Subscriber
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose',
                                 self.initial_pose_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback,
                                 10)

    def initial_pose_callback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.frame_id

        # Copy pose information
        p = msg.pose.pose
        odom_msg.pose.pose = p

        # Publish
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f'initial pose: {p.position.x}, {p.position.y}')

    def goal_callback(self, msg):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.frame_id

        goal_msg.pose = msg.pose

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(
            f'goal pose: {msg.pose.position.x}, {msg.pose.position.y}')

    def run(self):
        rclpy.spin(self)


def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    pose_publisher = PosePublisher()

    try:
        pose_publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
