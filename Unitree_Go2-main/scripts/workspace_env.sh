#!/usr/bin/env bash

# Source this file from ~/.bashrc to reproduce the local robotics workspace layout.
export PATH="$HOME/.pixi/bin:$PATH"

workspace_find_dir() {
    local name="$1"
    local preferred="$2"
    if [ -d "$preferred" ]; then
        printf '%s\n' "$preferred"
    else
        local found
        found="$(find "$HOME" -maxdepth 4 -type d -name "$name" -print -quit 2>/dev/null)"
        printf '%s\n' "${found:-$preferred}"
    fi
}

[ -f "$HOME/.config/robotics_workspace.env" ] && . "$HOME/.config/robotics_workspace.env"

export PROJECTS_DIR="${PROJECTS_DIR:-$(workspace_find_dir Project "$HOME/Project")}"
export UNITREE_GO2_ROOT="${UNITREE_GO2_ROOT:-$(workspace_find_dir Unitree_Go2 "$PROJECTS_DIR/Unitree_Go2")}"
export REPO="${REPO:-$UNITREE_GO2_ROOT}"
export ISAACLAB_ROOT="${ISAACLAB_ROOT:-$(workspace_find_dir IsaacLab "$PROJECTS_DIR/IsaacLab")}"
export UNITREE_RL_LAB_ROOT="${UNITREE_RL_LAB_ROOT:-$UNITREE_GO2_ROOT/training/isaac_lab/unitree_rl_lab}"
export OMX_F_ISAACLAB_ROOT="${OMX_F_ISAACLAB_ROOT:-$ISAACLAB_ROOT/omx_f_isaaclab}"
export OPEN_MANIPULATOR_ROOT="${OPEN_MANIPULATOR_ROOT:-$(workspace_find_dir open_manipulator "$PROJECTS_DIR/open_manipulator")}"
export ROS2_WS="${ROS2_WS:-$(workspace_find_dir ros2_ws "$PROJECTS_DIR/ros2_ws")}"
export GO2_ROUGHNAV_ROOT="${GO2_ROUGHNAV_ROOT:-$UNITREE_GO2_ROOT/ros2/go2_roughnav}"

source_if_exists() {
    [ -f "$1" ] && . "$1"
}

source_if_exists /opt/ros/jazzy/setup.bash
source_if_exists "$ROS2_WS/install/setup.bash"
source_if_exists "$GO2_ROUGHNAV_ROOT/install/setup.bash"

unalias go2 go2repo go2rl go2ros omxlab omxsrc killisaac 2>/dev/null || true

go2() {
    cd "$UNITREE_RL_LAB_ROOT/docker" || return
    docker compose --env-file .env.base up -d
    docker compose --env-file .env.base exec isaac-lab-template bash
}

go2repo() { cd "$UNITREE_GO2_ROOT"; }
go2rl() { cd "$UNITREE_RL_LAB_ROOT"; }
go2ros() { cd "$GO2_ROUGHNAV_ROOT"; }
omxlab() { cd "$OMX_F_ISAACLAB_ROOT"; }
omxsrc() { cd "$OPEN_MANIPULATOR_ROOT"; }

killisaac() {
    sudo pkill -9 -f "kit"
    sudo pkill -9 -f "isaac"
    sudo pkill -9 -f "train.py"
}
