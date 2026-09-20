#!/usr/bin/env bash
set -euo pipefail

IMAGE="${GO2_IMAGE:-go2-slam:humble-arm64}"
PROJECT_ROOT="${GO2_PROJECT_ROOT:-/home/unitree/go2-slam-real/go2-nav2-amcl-main}"
CONTAINER_NAME="${GO2_RVIZ_CONTAINER:-go2-rviz-real}"
DISPLAY_VALUE="${DISPLAY:-:99}"
RVIZ_CONFIG="${GO2_RVIZ_CONFIG:-real_sensing.rviz}"
case "$RVIZ_CONFIG" in
  real_sensing.rviz|real_navigation.rviz) ;;
  *)
    echo "Unsupported RViz config: $RVIZ_CONFIG" >&2
    exit 1
    ;;
esac
XAUTH_HOST="${XAUTHORITY_HOST:-}"
if [ -z "$XAUTH_HOST" ] || [ ! -r "$XAUTH_HOST" ]; then
  XAUTH_HOST="$(find /home/unitree/.nx/node -maxdepth 2 -type f -name authority -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {sub(/^[^ ]+ /, ""); print}')"
fi
if [ -z "$XAUTH_HOST" ] || [ ! -r "$XAUTH_HOST" ]; then
  echo "No readable NoMachine Xauthority found" >&2
  exit 1
fi

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

exec docker run -d --name "$CONTAINER_NAME" --restart unless-stopped \
  --network host --ipc host --privileged \
  -e DISPLAY="$DISPLAY_VALUE" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e GO2_RVIZ_CONFIG="$RVIZ_CONFIG" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH_HOST:/tmp/.Xauthority:ro" \
  -v "$PROJECT_ROOT:/opt/go2_ws/src/go2-nav2-amcl-main:ro" \
  -v go2-nav2-amcl-main_go2_build:/opt/go2_ws/install \
  -v go2-nav2-amcl-main_unitree_build:/opt/unitree_ws/install \
  -v /home/unitree/unitree_ros2/cyclonedds_ws/src:/opt/unitree_src:ro \
  --entrypoint /bin/bash "$IMAGE" -lc '
    set +u
    source /opt/ros/humble/setup.bash
    source /opt/unitree_ws/install/setup.bash
    source /opt/go2_ws/install/setup.bash
    exec rviz2 -d "/opt/go2_ws/src/go2-nav2-amcl-main/src/base/go2_core/rviz2/$GO2_RVIZ_CONFIG"
  '
