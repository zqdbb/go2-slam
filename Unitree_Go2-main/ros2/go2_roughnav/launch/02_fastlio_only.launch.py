import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("go2_roughnav"))
    fastlio_launch = LaunchConfiguration("fastlio_launch")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fastlio_launch",
                default_value=os.path.expanduser("~/fastlio_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "use_sim_time": "false",
                    "rviz": "false",
                    "config_path": str(share / "config"),
                    "config_file": "fastlio_isaac.yaml",
                }.items(),
            ),
        ]
    )
