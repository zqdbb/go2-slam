#!/bin/bash
# 已建图定位与导航模式：MuJoCo + SLAM Toolbox localization + Nav2 global planner + SCAN-Planner local planner
set -e
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select go2_roughnav --symlink-install
source install/setup.bash
printf '%s\n' '========================================' ' 已建图导航模式' ' 地图: /workspace/maps/go2_mapping_20260820_105700.posegraph/.data' ' RViz 发送 2D Goal 后由 Nav2 生成全局路径，SCAN-Planner 负责局部避障' '========================================'
ros2 launch go2_roughnav integrated_navigation.launch.py
