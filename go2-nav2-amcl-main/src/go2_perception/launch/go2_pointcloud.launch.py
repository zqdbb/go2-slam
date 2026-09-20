from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # cloud_base is the current frame in the robot body frame.  The Unitree
    # timestamp can use a different clock from the ROS host, so the converter
    # normalizes the outgoing LaserScan timestamp below.
    input_topic = DeclareLaunchArgument('input_topic', default_value='/utlidar/cloud_base')
    return LaunchDescription([input_topic,
        Node(
            package='go2_perception', executable='pointcloud_to_laserscan_node',
            remappings=[
                # Do not use cloud_accumulation: it mixed historical frames
                # without transforming each frame before concatenation.
                ('cloud_in', LaunchConfiguration('input_topic')),
                ('scan', '/scan')
                ],
            parameters=[{
                # cloud_base already uses base_link; leaving target_frame empty
                # disables MessageFilter and avoids rejecting device-clock stamps.
                'target_frame': '',
                'transform_tolerance': 0.15,
                'normalize_timestamp': True,
                # odom TF arrives up to about 50 ms after the lidar callback.
                # Date scans slightly into the past so slam_toolbox can resolve TF immediately.
                # Keep the host timestamp.  The SLAM filter handles the
                # small TF publication latency; dating scans into the past
                # causes extrapolation at the beginning of the TF cache.
                'timestamp_offset': 0.0,
                # 高度按需调整
                'min_height': 0.1,
                'max_height': 0.5,

                'angle_min': -3.14,  
                'angle_max': 3.14,  
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.00,          
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0

            }],
            name='pointcloud_to_laserscan_node'
        )
    ])
