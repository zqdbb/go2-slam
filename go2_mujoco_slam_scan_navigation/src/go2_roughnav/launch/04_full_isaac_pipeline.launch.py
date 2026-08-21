import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.substitutions


def generate_launch_description():
    share = Path(get_package_share_directory("go2_roughnav"))
    config = LaunchConfiguration("config")
    launch_fastlio = LaunchConfiguration("launch_fastlio")
    launch_trg = LaunchConfiguration("launch_trg")
    launch_cmd = LaunchConfiguration("launch_cmd")
    launch_auto_goal = LaunchConfiguration("launch_auto_goal")
    launch_health = LaunchConfiguration("launch_health")
    launch_rviz = LaunchConfiguration("launch_rviz")
    fastlio_launch = LaunchConfiguration("fastlio_launch")
    trg_config = LaunchConfiguration("trg_config")
    trg_map = LaunchConfiguration("trg_map")
    rviz_config = LaunchConfiguration("rviz_config")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    launch_robot_model = LaunchConfiguration("launch_robot_model")
    launch_bag = LaunchConfiguration("launch_bag")
    bag_path = LaunchConfiguration("bag_path")

    go2_urdf = launch_ros.substitutions.FindPackageShare("go2_description").find("go2_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(share / "config" / "isaac_bridge.yaml")),
            DeclareLaunchArgument("launch_fastlio", default_value="true"),
            DeclareLaunchArgument("launch_trg", default_value="true"),
            DeclareLaunchArgument("launch_cmd", default_value="true"),
            DeclareLaunchArgument("launch_auto_goal", default_value="false"),
            DeclareLaunchArgument("launch_health", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "fastlio_launch",
                default_value=os.path.expanduser("~/fastlio_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py"),
            ),
            DeclareLaunchArgument("trg_config", default_value=str(share / "config" / "trg_ros2_params_isaac.yaml")),
            DeclareLaunchArgument("trg_map", default_value="mountain"),
            DeclareLaunchArgument("fastlio_config_file", default_value="fastlio_isaac.yaml"),
            DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz" / "isaac_debug.rviz")),
            DeclareLaunchArgument("launch_robot_model", default_value="false"),
            DeclareLaunchArgument("launch_bag", default_value="false"),
            DeclareLaunchArgument("bag_path", default_value=os.path.expanduser("~/fastlio_ws/slam_map3/")),
            Node(
                package="go2_roughnav",
                executable="isaac_udp_ros2_bridge",
                name="isaac_udp_ros2_bridge",
                output="screen",
                parameters=[config],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "use_sim_time": "false",
                    "rviz": "false",
                    "config_path": str(share / "config"),
                    "config_file": fastlio_config_file,
                }.items(),
                condition=IfCondition(launch_fastlio),
            ),
            Node(
                package="trg_planner_ros",
                executable="trg_ros2_node",
                name="trg_ros2_node",
                output="screen",
                parameters=[trg_config, {"mapConfig": trg_map}],
                condition=IfCondition(launch_trg),
            ),
            Node(
                package="go2_roughnav",
                executable="path_to_cmd_vel",
                name="path_to_cmd_vel",
                output="screen",
                parameters=[config],
                condition=IfCondition(launch_cmd),
            ),
            Node(
                package="go2_roughnav",
                executable="auto_goal_publisher",
                name="auto_goal_publisher",
                output="screen",
                parameters=[config, {"enable": True}],
                condition=IfCondition(launch_auto_goal),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
            Node(
                package="go2_roughnav",
                executable="pipeline_health",
                name="pipeline_health",
                output="screen",
                condition=IfCondition(launch_health),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": Command(["xacro ", str(go2_urdf), "/xacro/robot.xacro"])},
                    {"use_tf_static": False},
                    {"publish_frequency": 50.0},
                ],
                condition=IfCondition(launch_robot_model),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_base_link",
                arguments=["--frame-id", "body", "--child-frame-id", "base_link"],
                condition=IfCondition(launch_robot_model),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play", bag_path,
                    "--topics", "/utlidar/cloud", "/utlidar/imu",
                    "--remap", "/utlidar/cloud:=/lidar/points", "/utlidar/imu:=/imu/data",
                    "--loop",
                ],
                output="screen",
                condition=IfCondition(launch_bag),
            ),
        ]
    )
