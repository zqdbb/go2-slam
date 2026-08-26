#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(TFMessage, '/tf', self.cb, 50)
    def cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id.strip('/') == 'odom' and t.child_frame_id.strip('/') == 'base_link':
                o = Odometry()
                o.header = t.header; o.header.frame_id = 'odom'; o.child_frame_id = 'base_link'
                o.pose.pose.position.x = t.transform.translation.x
                o.pose.pose.position.y = t.transform.translation.y
                o.pose.pose.position.z = t.transform.translation.z
                o.pose.pose.orientation = t.transform.rotation
                self.pub.publish(o)
def main():
    rclpy.init(); n=GroundTruthOdom(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
