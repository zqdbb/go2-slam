from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("go2_roughnav"))
    config = LaunchConfiguration("config")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(share / "config" / "isaac_bridge.yaml")),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz" / "isaac_debug.rviz")),
            Node(
                package="go2_roughnav",
                executable="isaac_udp_ros2_bridge",
                name="isaac_udp_ros2_bridge",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
