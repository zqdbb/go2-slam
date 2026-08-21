#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
MAP_DIR="${1:-/workspace/maps}"
NAME="${2:-go2_map}"
mkdir -p "$MAP_DIR"
BASE="$MAP_DIR/$NAME"
echo "Serializing SLAM pose graph to ${BASE}.posegraph/.data ..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$BASE'}"
echo "Saving occupancy grid to ${BASE}.pgm/.yaml ..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$BASE'}}"
echo "Saved map artifacts under $MAP_DIR"
ls -lh "${BASE}"* 2>/dev/null || true
