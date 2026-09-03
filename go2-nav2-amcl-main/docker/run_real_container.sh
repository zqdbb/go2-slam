#!/usr/bin/env bash
set -euo pipefail
IMAGE="${GO2_IMAGE:-go2-slam:humble-arm64}"
DOMAIN="${ROS_DOMAIN_ID:-0}"
mkdir -p "${HOME}/go2_maps"
exec sudo docker run --rm -it --network host --ipc host --privileged \
  --env ROS_DOMAIN_ID="${DOMAIN}" --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --env DISPLAY="${DISPLAY:-}" --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /home/unitree/unitree_ros2/cyclonedds_ws/src:/opt/unitree_src:ro \
  --volume "${HOME}/go2_maps:/root/go2_maps" "${IMAGE}" "$@"
