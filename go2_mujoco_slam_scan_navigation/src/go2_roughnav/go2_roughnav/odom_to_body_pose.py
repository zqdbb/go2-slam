#!/usr/bin/env python3
"""将 /Odometry 重发布为 /quad_0/body_pose 和 /quad_0/lidar_pose。"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class OdomToBodyPose(Node):
    def __init__(self):
        super().__init__("odom_to_body_pose")
        self.pub_body  = self.create_publisher(Odometry, "/quad_0/body_pose",  10)
        self.pub_lidar = self.create_publisher(Odometry, "/quad_0/lidar_pose", 10)
        self.create_subscription(Odometry, "/Odometry", self._cb, 10)

    def _cb(self, msg: Odometry):
        # body_pose
        body = Odometry()
        body.header = msg.header
        body.header.frame_id = "world"
        body.child_frame_id  = "base"
        body.pose = msg.pose
        body.twist = msg.twist
        self.pub_body.publish(body)

        # lidar_pose: same as body but offset +0.18m in x and +0.18m in z
        lidar = Odometry()
        lidar.header = msg.header
        lidar.header.frame_id = "world"
        lidar.child_frame_id  = "lidar_link"
        lidar.pose.pose.position.x = msg.pose.pose.position.x + 0.18
        lidar.pose.pose.position.y = msg.pose.pose.position.y
        lidar.pose.pose.position.z = msg.pose.pose.position.z + 0.18
        lidar.pose.pose.orientation = msg.pose.pose.orientation
        self.pub_lidar.publish(lidar)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdomToBodyPose())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
