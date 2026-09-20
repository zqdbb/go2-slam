import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.substitutions


def generate_launch_description():
    share = Path(get_package_share_directory("go2_roughnav"))
    config = LaunchConfiguration("config")
    launch_mujoco = LaunchConfiguration("launch_mujoco")
    launch_fastlio = LaunchConfiguration("launch_fastlio")
    launch_trg = LaunchConfiguration("launch_trg")
    launch_controller = LaunchConfiguration("launch_controller")
    launch_traversability = LaunchConfiguration("launch_traversability")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rl = LaunchConfiguration("launch_rl")
    fastlio_launch = LaunchConfiguration("fastlio_launch")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    trg_config = LaunchConfiguration("trg_config")
    trg_map = LaunchConfiguration("trg_map")
    rviz_config = LaunchConfiguration("rviz_config")
    rl_model_path = LaunchConfiguration("rl_model_path")

    go2_urdf = launch_ros.substitutions.FindPackageShare("go2_description").find("go2_description")

    sim_time = {"use_sim_time": True}

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(share / "config" / "roughnav.yaml")),
            DeclareLaunchArgument("launch_mujoco", default_value="true"),
            DeclareLaunchArgument("launch_fastlio", default_value="true"),
            DeclareLaunchArgument("launch_trg", default_value="true"),
            DeclareLaunchArgument("launch_controller", default_value="true"),
            DeclareLaunchArgument("launch_traversability", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("launch_rl", default_value="true"),
            DeclareLaunchArgument(
                "rl_model_path",
                default_value=os.path.expanduser("~/go2_sim_ws/src/go2_rl_deploy/models/actor_jit.pt"),
            ),
            DeclareLaunchArgument(
                "fastlio_launch",
                default_value=os.path.expanduser("~/fastlio_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py"),
            ),
            DeclareLaunchArgument("fastlio_config_file", default_value="fastlio_roughnav.yaml"),
            DeclareLaunchArgument("trg_config", default_value=str(share / "config" / "trg_ros2_params_roughnav.yaml")),
            DeclareLaunchArgument("trg_map", default_value="mountain"),
            DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz" / "roughnav.rviz")),

            # MuJoCo simulation bridge — publishes /points_raw, /imu, /clock
            Node(
                package="go2_roughnav",
                executable="mujoco_go2_bridge",
                name="mujoco_go2_bridge",
                output="screen",
                parameters=[config],
                condition=IfCondition(launch_mujoco),
            ),

            # FAST-LIO SLAM — subscribes /points_raw + /imu, publishes /cloud_registered + /Odometry
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "rviz": "false",
                    "config_path": str(share / "config"),
                    "config_file": fastlio_config_file,
                }.items(),
                condition=IfCondition(launch_fastlio),
            ),

            # TRG-planner — subscribes /cloud_registered + /Odometry + /goal_pose → /trg_path
            Node(
                package="trg_planner_ros",
                executable="trg_ros2_node",
                name="trg_ros2_node",
                output="screen",
                parameters=[trg_config, {"mapConfig": trg_map}, sim_time],
                condition=IfCondition(launch_trg),
            ),

            # Path follower — /trg_path + /Odometry → /cmd_vel
            Node(
                package="go2_roughnav",
                executable="rough_local_controller",
                name="rough_local_controller",
                output="screen",
                parameters=[config, sim_time],
                condition=IfCondition(launch_controller),
            ),

            # Traversability map — /cloud_registered → /traversability_map
            Node(
                package="go2_roughnav",
                executable="traversability_node",
                name="traversability_node",
                output="screen",
                parameters=[config, sim_time],
                condition=IfCondition(launch_traversability),
            ),

            # TF: FAST-LIO publishes camera_init→body. Connect body→base_link for URDF chain.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_base_link",
                arguments=["--frame-id", "body", "--child-frame-id", "base_link"],
                parameters=[sim_time],
            ),

            # Go2 URDF joint states → TF for all leg links
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": Command(["xacro ", str(go2_urdf), "/xacro/robot.xacro"])},
                    {"use_tf_static": False},
                    {"publish_frequency": 50.0},
                    sim_time,
                ],
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                parameters=[sim_time],
                condition=IfCondition(launch_rviz),
            ),

            # Height scan: /velodyne_points → /height_scan (17×11 grid for RL policy)
            Node(
                package="go2_rl_deploy",
                executable="height_scan_node",
                name="height_scan_node",
                output="screen",
                parameters=[sim_time],
                condition=IfCondition(launch_rl),
            ),

            # RL policy: /joint_states + /imu/data + /cmd_vel + /height_scan → joint targets
            # Delayed 5 s to let MuJoCo stabilize first
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="go2_rl_deploy",
                        executable="rl_policy_node",
                        name="rl_policy_node",
                        output="screen",
                        parameters=[
                            sim_time,
                            {"model_path": rl_model_path, "use_scan": True, "action_scale": 0.25},
                        ],
                        condition=IfCondition(launch_rl),
                    ),
                ],
            ),
        ]
    )
