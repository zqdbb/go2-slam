#!/usr/bin/env python3

import signal
import sys

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def signal_handler(sig, frame):
    print('Exiting and plotting data...')
    sys.exit(0)


class PosePublisher:

    def __init__(self):
        rospy.init_node('pose_publisher')

        # Publisher
        self.odom_pub = rospy.Publisher('fake_robot_pose',
                                        Odometry,
                                        queue_size=10)
        self.goal_pub = rospy.Publisher('fake_goal',
                                        PoseStamped,
                                        queue_size=10)

        # Subscriber
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped,
                         self.initial_pose_callback)
        rospy.Subscriber('move_base_simple/goal', PoseStamped,
                         self.goal_callback)

    def initial_pose_callback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"

        # Copy pose information
        p = msg.pose.pose
        odom_msg.pose.pose = p

        # Publish
        self.odom_pub.publish(odom_msg)
        print(f'egoPose: {p.position.x}, {p.position.y}')

    def goal_callback(self, msg):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"

        goal_msg.pose = msg.pose

        self.goal_pub.publish(goal_msg)
        print(f'goal pose: {msg.pose.position.x}, {msg.pose.position.y}')

    def run(self):
        try:
            while not rospy.is_shutdown():
                rospy.spin()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    pose_publisher = PosePublisher()
    signal.signal(signal.SIGINT, signal_handler)
    pose_publisher.run()
