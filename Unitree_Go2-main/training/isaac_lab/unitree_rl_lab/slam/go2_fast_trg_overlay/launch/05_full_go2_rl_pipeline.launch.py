from pathlib import Path
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("go2_roughnav"))

    launch_isaac_bridge = LaunchConfiguration("launch_isaac_bridge")
    launch_fastlio = LaunchConfiguration("launch_fastlio")
    launch_trg = LaunchConfiguration("launch_trg")
    launch_cmd = LaunchConfiguration("launch_cmd")
    launch_height_scan = LaunchConfiguration("launch_height_scan")
    launch_health = LaunchConfiguration("launch_health")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_robot_model = LaunchConfiguration("launch_robot_model")
    launch_bag = LaunchConfiguration("launch_bag")

    fastlio_launch = LaunchConfiguration("fastlio_launch")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    trg_config = LaunchConfiguration("trg_config")
    trg_map = LaunchConfiguration("trg_map")
    rl_config = LaunchConfiguration("rl_config")
    rviz_config = LaunchConfiguration("rviz_config")
    bag_path = LaunchConfiguration("bag_path")
    bag_cloud_topic = LaunchConfiguration("bag_cloud_topic")
    bag_imu_topic = LaunchConfiguration("bag_imu_topic")

    try:
        go2_urdf = Path(get_package_share_directory("go2_description"))
    except PackageNotFoundError:
        go2_urdf = None

    actions = [
        DeclareLaunchArgument("launch_isaac_bridge", default_value="false"),
        DeclareLaunchArgument("launch_fastlio", default_value="true"),
        DeclareLaunchArgument("launch_trg", default_value="true"),
        DeclareLaunchArgument("launch_cmd", default_value="true"),
        DeclareLaunchArgument("launch_height_scan", default_value="true"),
        DeclareLaunchArgument("launch_health", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_robot_model", default_value="false"),
        DeclareLaunchArgument("launch_bag", default_value="false"),
        DeclareLaunchArgument(
            "fastlio_launch",
            default_value="/fastlio_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py",
        ),
        DeclareLaunchArgument("fastlio_config_file", default_value="fastlio_go2_hw.yaml"),
        DeclareLaunchArgument("trg_config", default_value=str(share / "config" / "trg_ros2_params_isaac.yaml")),
        DeclareLaunchArgument("trg_map", default_value="competition"),
        DeclareLaunchArgument("rl_config", default_value=str(share / "config" / "rl_interface.yaml")),
        DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz" / "isaac_debug.rviz")),
        DeclareLaunchArgument("bag_path", default_value=os.path.expanduser("~/fastlio_ws/slam_map3/")),
        DeclareLaunchArgument("bag_cloud_topic", default_value="/utlidar/cloud"),
        DeclareLaunchArgument("bag_imu_topic", default_value="/utlidar/imu"),
        Node(
            package="go2_roughnav",
            executable="isaac_udp_ros2_bridge",
            name="isaac_udp_ros2_bridge",
            output="screen",
            condition=IfCondition(launch_isaac_bridge),
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
            executable="height_scan_bridge",
            name="height_scan_bridge",
            output="screen",
            parameters=[rl_config],
            condition=IfCondition(launch_height_scan),
        ),
        Node(
            package="go2_roughnav",
            executable="path_to_cmd_vel",
            name="path_to_cmd_vel",
            output="screen",
            parameters=[rl_config],
            condition=IfCondition(launch_cmd),
        ),
        Node(
            package="go2_roughnav",
            executable="pipeline_health",
            name="pipeline_health",
            output="screen",
            parameters=[rl_config],
            condition=IfCondition(launch_health),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(launch_rviz),
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                bag_path,
                "--topics",
                bag_cloud_topic,
                bag_imu_topic,
                "--loop",
            ],
            output="screen",
            condition=IfCondition(launch_bag),
        ),
    ]

    if go2_urdf is not None:
        actions.extend(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    parameters=[
                        {"robot_description": Command(["xacro ", str(go2_urdf / "xacro" / "robot.xacro")])},
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
            ]
        )

    return LaunchDescription(actions)
