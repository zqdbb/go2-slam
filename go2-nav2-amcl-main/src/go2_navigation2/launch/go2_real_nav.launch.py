"""Real Go2 navigation entry point; no Gazebo/Mujoco processes are started."""
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("go2_navigation2")
    map_arg = DeclareLaunchArgument(
        "map", default_value=os.path.join(pkg, "maps", "my_room.yaml"),
        description="地图 YAML 文件路径")
    params_arg = DeclareLaunchArgument(
        "params_file", default_value=os.path.join(pkg, "config", "nav2_params.yaml"),
        description="Nav2 参数文件路径")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    return LaunchDescription([
        map_arg, params_arg, use_rviz_arg, use_sim_time_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "go2_nav2.launch.py")),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        )
    ])
