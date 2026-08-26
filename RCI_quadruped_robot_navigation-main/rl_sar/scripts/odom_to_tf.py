#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.get_logger().info('Odom to TF broadcaster started')

    def odom_callback(self, msg):
        # Create transform message
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # 'odom'
        t.child_frame_id = msg.child_frame_id    # 'base_link'

        # Copy position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Normalize quaternion to fix orientation issues
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)

        if norm > 0.0:
            t.transform.rotation.x = q.x / norm
            t.transform.rotation.y = q.y / norm
            t.transform.rotation.z = q.z / norm
            t.transform.rotation.w = q.w / norm
        else:
            # Identity quaternion if norm is zero
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()