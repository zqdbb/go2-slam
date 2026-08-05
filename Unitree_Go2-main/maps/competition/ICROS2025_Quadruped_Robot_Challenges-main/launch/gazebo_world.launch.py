import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    wname = "QRC_map"
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(get_package_share_directory("ICROS2025_QRC_Simulation_Map"), "worlds", wname + ".world"),
        }.items(),
    )

    return LaunchDescription([
        gazebo,
    ])
