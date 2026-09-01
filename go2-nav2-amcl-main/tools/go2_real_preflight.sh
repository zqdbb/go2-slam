#!/usr/bin/env bash
set -u

echo '[Go2 real preflight] ROS_DOMAIN_ID='"${ROS_DOMAIN_ID:-unset}"
echo '[Go2 real preflight] RMW_IMPLEMENTATION='"${RMW_IMPLEMENTATION:-unset}"
command -v ros2 >/dev/null || { echo 'ERROR: ros2 not found'; exit 1; }

check_topic() {
  local topic="$1"
  if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
    echo "OK   $topic"
  else
    echo "MISS $topic"
  fi
}

echo 'Checking Unitree and sensor topics (robot must be powered on and connected)...'
check_topic /lf/lowstate
check_topic /lf/sportmodestate
check_topic /utlidar/cloud_deskewed
check_topic /utlidar/cloud
check_topic /utlidar/robot_pose
check_topic /api/sport/request

echo 'TF snapshot:'
ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -20 || true
echo 'Preflight finished. Resolve MISS/TF errors before enabling navigation.'
