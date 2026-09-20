#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/unitree_ws/install/setup.bash
source /opt/go2_ws/install/setup.bash
set -u

PARAMS=/opt/go2_ws/src/go2-nav2-amcl-main/src/go2_slam/config/mapper_params_online_async.yaml

if ! ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then
  nohup ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$PARAMS" \
    use_sim_time:=false >/tmp/go2_slam_only.log 2>&1 &
fi

sleep 5
echo '--- slam nodes ---'
ros2 node list | grep -E 'slam|sync' || true
echo '--- map info ---'
ros2 topic info /map || true
timeout 5 ros2 topic hz /map || true
