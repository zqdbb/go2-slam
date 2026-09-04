#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
if [ -f /opt/unitree_ws/install/setup.bash ]; then
  source /opt/unitree_ws/install/setup.bash
fi
if [ -f /opt/go2_ws/install/setup.bash ]; then
  source /opt/go2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

ros2 run go2_driver driver >/tmp/go2_driver.log 2>&1 &
PID_DRIVER=$!
ros2 run go2_driver footprint_to_link >/tmp/go2_foot.log 2>&1 &
PID_FOOT=$!
ros2 run go2_driver lowstate_to_imu >/tmp/go2_imu.log 2>&1 &
PID_IMU=$!
ros2 launch go2_perception go2_pointcloud.launch.py >/tmp/go2_pointcloud.log 2>&1 &
PID_PC=$!

cleanup() {
  kill "$PID_DRIVER" "$PID_FOOT" "$PID_IMU" "$PID_PC" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 10

echo '--- nodes'
ros2 node list || true

echo '--- topics'
ros2 topic list | grep -E '^(\/tf|\/tf_static|\/imu|\/scan|\/odom|\/lf\/|\/utlidar\/)' || true

echo '--- tf odom base_link'
timeout 5 ros2 run tf2_ros tf2_echo odom base_link || true

echo '--- hz imu'
timeout 5 ros2 topic hz /imu || true

echo '--- hz scan'
timeout 5 ros2 topic hz /scan || true

echo '--- info scan'
ros2 topic info /scan || true

echo '--- info imu'
ros2 topic info /imu || true

echo '--- logs tail'
tail -n 20 /tmp/go2_driver.log || true
tail -n 20 /tmp/go2_foot.log || true
tail -n 20 /tmp/go2_imu.log || true
tail -n 20 /tmp/go2_pointcloud.log || true
