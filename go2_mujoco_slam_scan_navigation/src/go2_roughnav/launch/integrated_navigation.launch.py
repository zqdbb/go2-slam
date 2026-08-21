import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    ws = '/workspace'
    share = get_package_share_directory('go2_roughnav')
    scene = '/tmp/go2_mujoco/scene_icra2024_flat.xml'
    slam_cfg = os.path.join(share, 'config', 'mapper_params_localization.yaml')
    nav_cfg = os.path.join(share, 'config', 'nav2_planner_params.yaml')
    planner_yaml = os.path.join(get_package_share_directory('scan_planner'), 'config', 'planner.yaml')
    ctrl_yaml = os.path.join(get_package_share_directory('scan_planner'), 'config', 'controllers.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'mapping.rviz')
    xacro = os.path.join(get_package_share_directory('go2_description'), 'xacro', 'robot.xacro')
    bridge = Node(package='go2_roughnav', executable='mujoco_go2_bridge', name='mujoco_go2_bridge', output='screen', parameters=[{
        'model_path': scene, 'server_script': os.path.join(ws, 'mujoco_server.py'),
        'policy_path': os.path.join(ws, 'policies', 'model_40000.pt'),
        'mujoco_container': 'mujoco-go2-mapping', 'mujoco_python': '/lerobot/.venv/bin/python3',
        'publish_rate_hz': 50.0, 'lidar_rate_hz': 5.0, 'lidar_horizontal_samples': 180,
        'lidar_vertical_samples': 8, 'lidar_vertical_fov_deg': 30.0, 'lidar_range_m': 10.0,
        'base_frame': 'base_link', 'odom_frame': 'odom', 'lidar_frame': 'lidar_link', 'mujoco_viewer': True,
    }])
    tf_lidar = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x','0.18','--y','0','--z','0.18','--frame-id','base_link','--child-frame-id','lidar_link'])
    tf_foot = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x','0','--y','0','--z','0','--frame-id','base_link','--child-frame-id','base_footprint'])
    tf_base = Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x','0','--y','0','--z','0','--frame-id','base_link','--child-frame-id','base'])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[{'robot_description': Command(['xacro ', xacro, ' use_gazebo:=false'])}], remappings=[('robot_description','/go2/robot_description')])
    pc2scan = Node(package='go2_perception', executable='pointcloud_to_laserscan_node', name='pointcloud_to_laserscan', output='screen', remappings=[('cloud_in','/points_raw'),('scan','/scan')], parameters=[{'target_frame':'base_footprint','transform_tolerance':0.2,'min_height':-0.2,'max_height':0.5,'angle_min':-3.14159,'angle_max':3.14159,'angle_increment':0.0087,'scan_time':0.1,'range_min':0.1,'range_max':10.0,'use_inf':True}])
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource('/opt/ros/humble/share/slam_toolbox/launch/localization_launch.py'), launch_arguments={'slam_params_file':slam_cfg,'use_sim_time':'true'}.items())
    odom = Node(package='go2_roughnav', executable='odom_to_body_pose', name='odom_to_body_pose', output='screen')
    planner = Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[nav_cfg])
    lm = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[nav_cfg])
    lifecycle_activator = Node(package='go2_roughnav', executable='nav2_lifecycle_activator', name='nav2_lifecycle_activator', output='screen', parameters=[{'use_sim_time':True}])
    path_bridge = Node(package='go2_roughnav', executable='nav2_path_bridge', name='nav2_path_bridge', output='screen', parameters=[{'use_sim_time':True}])
    scan_planner = Node(package='scan_planner', executable='scan_planner_node', name='scan_planner_node', output='screen', parameters=[planner_yaml, {'use_sim_time':True,'grid_map.sensor_type':'lidar','fsm.navi_mode':3}], remappings=[('body_pose','/quad_0/body_pose'),('sensor_pose','/quad_0/lidar_pose'),('cloud','/points_raw'),('initial_path','/initial_path')])
    controller = Node(package='scan_planner', executable='closed_loop_controller', name='closed_loop_controller', output='screen', parameters=[ctrl_yaml, {'use_sim_time':True}], remappings=[('body_pose','/quad_0/body_pose'),('cmd_vel','/cmd_vel_nav')])
    stuck_recovery = Node(package='go2_roughnav', executable='stuck_recovery', name='stuck_recovery', output='screen', parameters=[{'use_sim_time':True}])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d',rviz_cfg], parameters=[{'use_sim_time':True}])
    return LaunchDescription([bridge,tf_lidar,tf_foot,tf_base,rsp,rviz,TimerAction(period=2.0,actions=[pc2scan,odom]),TimerAction(period=3.0,actions=[slam]),TimerAction(period=5.0,actions=[planner]),TimerAction(period=7.0,actions=[lm,path_bridge,scan_planner,controller,stuck_recovery]),TimerAction(period=10.0,actions=[lifecycle_activator])])
