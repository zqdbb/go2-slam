#!/usr/bin/env bash
set -euo pipefail
CONTAINER=go2_mapping
printf '%s\n' 'Stopping SCAN-Planner and closed-loop controller for manual mapping...'
docker exec "$CONTAINER" bash -lc "pkill -TERM -x scan_planner_node || true; pkill -TERM -x closed_loop_controller || true"
printf '%s\n' 'Starting keyboard teleop. Use the keys shown by teleop_twist_keyboard; Ctrl-C exits.'
docker exec -it "$CONTAINER" bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel'
# Ensure a stop command remains after teleop exits.
docker exec "$CONTAINER" bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; timeout 1 ros2 topic pub --qos-reliability reliable --qos-durability volatile --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"' >/dev/null 2>&1 || true
printf '%s\n' 'Manual teleop ended. Restart integrated launch to restore SCAN-Planner.'
