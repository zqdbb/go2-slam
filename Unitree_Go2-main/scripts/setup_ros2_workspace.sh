#!/usr/bin/env bash
set -euo pipefail

REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

mkdir -p "${HOME}/go2_roughnav_ws/src" "${HOME}/go2_sim_ws/src"

copy_source() {
  local src="$1"
  local dst="$2"
  rm -rf "${dst}"
  mkdir -p "$(dirname "${dst}")"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --exclude build/ --exclude install/ --exclude log/ --exclude logs/ "${src}/" "${dst}/"
  else
    cp -a "${src}" "${dst}"
  fi
}

copy_source "${REPO}/ros2/go2_roughnav" "${HOME}/go2_roughnav_ws/src/go2_roughnav"
copy_source "${REPO}/planning/trg_planner" "${HOME}/go2_sim_ws/src/TRG-planner-1"

cat <<EOF
ROS2 workspace source copies are ready.

go2_roughnav:
  ${HOME}/go2_roughnav_ws/src/go2_roughnav

TRG-planner:
  ${HOME}/go2_sim_ws/src/TRG-planner-1

Next:
  source /opt/ros/humble/setup.bash
  cd ${HOME}/go2_sim_ws/src/TRG-planner-1
  cmake -B cpp/trg_planner/build -S cpp/trg_planner -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME}/go2_sim_ws/install/trg_planner_core
  cmake --build cpp/trg_planner/build -j\$(nproc)
  cmake --install cpp/trg_planner/build

  cd ${HOME}/go2_sim_ws
  export CMAKE_PREFIX_PATH=${HOME}/go2_sim_ws/install/trg_planner_core:\${CMAKE_PREFIX_PATH:-}
  colcon build --base-paths src/TRG-planner-1/pipelines/ros2 --cmake-args -DCMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH

  source ${HOME}/go2_sim_ws/install/setup.bash
  cd ${HOME}/go2_roughnav_ws
  colcon build --packages-select go2_roughnav
EOF
