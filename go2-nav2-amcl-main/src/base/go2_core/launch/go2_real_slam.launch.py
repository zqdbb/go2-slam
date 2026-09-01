"""Real Go2 SLAM entry point.  This launch intentionally starts no simulator."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    core = get_package_share_directory("go2_core")
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/utlidar/cloud_deskewed"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(core, "launch", "go2_start.launch.py")),
            launch_arguments={
                "use_slamtoolbox": "true",
                "input_topic": LaunchConfiguration("input_topic"),
            }.items(),
        )
    ])
