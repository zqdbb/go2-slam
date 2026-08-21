from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("go2_roughnav"))
    trg_config = LaunchConfiguration("trg_config")
    trg_map = LaunchConfiguration("trg_map")
    return LaunchDescription(
        [
            DeclareLaunchArgument("trg_config", default_value=str(share / "config" / "trg_ros2_params_isaac.yaml")),
            DeclareLaunchArgument("trg_map", default_value="mountain"),
            Node(
                package="trg_planner_ros",
                executable="trg_ros2_node",
                name="trg_ros2_node",
                output="screen",
                parameters=[trg_config, {"mapConfig": trg_map}],
            ),
        ]
    )
