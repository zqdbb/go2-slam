#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash

if [ -d /opt/unitree_src ] && [ ! -f /opt/unitree_ws/install/setup.bash ]; then
  mkdir -p /opt/unitree_ws/src
  cp -a /opt/unitree_src/. /opt/unitree_ws/src/
  # The Unitree source tree contains a Cyclone DDS fork that conflicts with
  # Humble's packaged RMW; use the tested Humble RMW and build only messages.
  colcon build --base-paths /opt/unitree_ws/src --install-base /opt/unitree_ws/install \
    --packages-skip cyclonedds rmw_cyclonedds_cpp
fi
if [ -f /opt/unitree_ws/install/setup.bash ]; then
  source /opt/unitree_ws/install/setup.bash
fi

cd /opt/go2_ws
if [ ! -f install/setup.bash ]; then
  rosdep update --rosdistro humble 2>/dev/null || true
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true
  colcon build --symlink-install
fi
source install/setup.bash
exec "$@"
