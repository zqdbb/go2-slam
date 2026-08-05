from setuptools import find_packages, setup

package_name = "go2_roughnav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/roughnav.yaml",
                "config/trg_ros2_params_roughnav.yaml",
                "config/fastlio_roughnav.yaml",
                "config/isaac_bridge.yaml",
                "config/fastlio_isaac.yaml",
                "config/fastlio_real_go2.yaml",
                "config/fastlio_go2_hw.yaml",
                "config/fastlio_mujoco.yaml",
                "config/trg_ros2_params_isaac.yaml",
                "config/rl_interface.yaml",
            ],
        ),
        (
            f"share/{package_name}/launch",
            [
                "launch/roughnav.launch.py",
                "launch/01_isaac_bridge_rviz.launch.py",
                "launch/02_fastlio_only.launch.py",
                "launch/03_trg_only.launch.py",
                "launch/04_full_isaac_pipeline.launch.py",
                "launch/05_full_go2_rl_pipeline.launch.py",
            ],
        ),
        (
            f"share/{package_name}/rviz",
            [
                "rviz/roughnav.rviz",
                "rviz/isaac_debug.rviz",
                "rviz/icros2025_trg_debug.rviz",
                "rviz/teleop_map_debug.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeonbotdae",
    maintainer_email="user@example.com",
    description="Rough-terrain autonomy glue for MuJoCo Go2, FAST-LIO, and TRG-planner.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "isaac_udp_ros2_bridge = go2_roughnav.isaac_udp_ros2_bridge:main",
            "path_to_cmd_vel = go2_roughnav.path_to_cmd_vel:main",
            "height_scan_bridge = go2_roughnav.height_scan_bridge:main",
            "auto_goal_publisher = go2_roughnav.auto_goal_publisher:main",
            "fake_isaac_udp_sender = go2_roughnav.fake_isaac_udp_sender:main",
            "pipeline_health = go2_roughnav.pipeline_health:main",
            "mujoco_go2_bridge = go2_roughnav.mujoco_go2_bridge:main",
            "traversability_node = go2_roughnav.traversability_node:main",
            "rough_local_controller = go2_roughnav.rough_local_controller:main",
            "generate_mujoco_terrain = go2_roughnav.terrain_generator:main",
        ],
    },
)
