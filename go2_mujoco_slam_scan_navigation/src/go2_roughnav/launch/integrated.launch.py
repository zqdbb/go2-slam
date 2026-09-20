"""integrated.launch.py — Mujoco + SLAM Toolbox + SCAN-Planner"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command


def generate_launch_description():
    ws = "/workspace"
    mujoco_scene  = "/tmp/go2_mujoco/scene_icra2024_flat.xml"
    server_script = os.path.join(ws, "mujoco_server.py")
    policy_path = os.path.join(ws, "policies", "model_40000.pt")
    slam_config   = os.path.join(
        get_package_share_directory("go2_slam"), "config", "mapper_params_online_async.yaml")
    planner_yaml  = os.path.join(
        get_package_share_directory("scan_planner"), "config", "planner.yaml")
    ctrl_yaml     = os.path.join(
        get_package_share_directory("scan_planner"), "config", "controllers.yaml")
    rviz_cfg      = os.path.join(
        get_package_share_directory("go2_roughnav"), "rviz", "mapping.rviz")
    xacro_file = os.path.join(
        get_package_share_directory("go2_description"), "xacro", "robot.xacro")

    # 1. Mujoco bridge
    mujoco_bridge = Node(
        package="go2_roughnav", executable="mujoco_go2_bridge",
        name="mujoco_go2_bridge", output="screen",
        parameters=[{
            "model_path":               mujoco_scene,
            "server_script":            server_script,
            "policy_path":              policy_path,
            "mujoco_container":         "mujoco-go2-mapping",
            "mujoco_python":            "/lerobot/.venv/bin/python3",
            "publish_rate_hz":          50.0,
            "lidar_rate_hz":            10.0,
            "lidar_horizontal_samples": 360,
            "lidar_vertical_samples":   16,
            "lidar_vertical_fov_deg":   30.0,
            "lidar_range_m":            10.0,
            "base_frame":               "base_link",
            "odom_frame":               "odom",
            "lidar_frame":              "lidar_link",
            "mujoco_viewer":             True,
        }],
    )

    # 2. 静态 TF
    tf_lidar = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x","0.18","--y","0","--z","0.18",
                   "--frame-id","base_link","--child-frame-id","lidar_link"],
    )
    tf_footprint = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x","0","--y","0","--z","0",
                   "--frame-id","base_link","--child-frame-id","base_footprint"],
    )

    tf_description_root = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x","0","--y","0","--z","0",
                   "--frame-id","base_link","--child-frame-id","base"],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": Command([
            "xacro ", xacro_file, " use_gazebo:=false"
        ])}],
        remappings=[("robot_description", "/go2/robot_description")],
    )

    # 3. pointcloud_to_laserscan: /points_raw → /scan
    pc2scan = Node(
        package="go2_perception", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", output="screen",
        remappings=[("cloud_in", "/points_raw"), ("scan", "/scan")],
        parameters=[{
            "target_frame":       "base_footprint",
            "transform_tolerance": 0.2,
            "min_height":         -0.2,
            "max_height":          0.5,
            "angle_min":          -3.14159,
            "angle_max":           3.14159,
            "angle_increment":     0.0087,
            "scan_time":           0.1,
            "range_min":           0.1,
            "range_max":          10.0,
            "use_inf":            True,
        }],
    )

    # 4. SLAM Toolbox (建图模式)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": slam_config, "use_sim_time": "true"}.items(),
    )

    # 5. Odometry bridge: /Odometry → /quad_0/body_pose
    odom_bridge = Node(
        package="go2_roughnav", executable="odom_to_body_pose",
        name="odom_to_body_pose", output="screen",
    )

    # 6. SCAN-Planner (is_real_world=true, 接外部里程计)
    scan_planner = Node(
        package="scan_planner", executable="scan_planner_node",
        name="scan_planner_node", output="screen",
        parameters=[planner_yaml, {"use_sim_time": True, "grid_map.sensor_type": "lidar", "fsm.navi_mode": 1}],
        remappings=[
            ("body_pose",              "/quad_0/body_pose"),
            ("sensor_pose",            "/quad_0/lidar_pose"),
            ("cloud",                  "/points_raw"),
            ("move_base_simple/goal",  "/move_base_simple/goal"),
        ],
    )
    controller = Node(
        package="scan_planner", executable="closed_loop_controller",
        name="closed_loop_controller", output="screen",
        parameters=[ctrl_yaml, {"use_sim_time": True}],
        remappings=[("body_pose", "/quad_0/body_pose"), ("cmd_vel", "/cmd_vel")],
    )

    # 7. RViz
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        mujoco_bridge,
        tf_lidar,
        tf_footprint,
        tf_description_root,
        robot_state_publisher,
        rviz,
        TimerAction(period=2.0, actions=[pc2scan, odom_bridge]),
        TimerAction(period=3.0, actions=[slam]),
        TimerAction(period=4.0, actions=[scan_planner, controller]),
    ])
