# Unitree_Go2

Snapshot date: 2026-06-02

This is the consolidated Unitree Go2 rough-terrain workspace. It packages the current IsaacLab/RSL-RL training backend, MuJoCo sim2sim, ROS2 teleop, FAST-LIO2/TRG integration glue, competition maps, policy artifacts, and Unitree SDK sources needed for later Sim2Real work.

The repository is not meant to freeze one final model. Policies, maps, training backends, and planners are selected at runtime through environment variables.

## Layout

```text
Unitree_Go2/
├── training/
│   └── isaac_lab/unitree_rl_lab/     # Current IsaacLab + RSL-RL backend
├── simulation/
│   └── mujoco/go2/                  # Go2 MuJoCo model and competition scenes
├── ros2/
│   └── go2_roughnav/                # FAST-LIO2/TRG/RL ROS2 glue package
├── planning/
│   └── trg_planner/                 # TRG-planner source, ROS2 pipeline, configs
├── maps/
│   └── competition/                 # ICRA2023, ICRA2024, ICROS2025 map assets
├── third_party/
│   ├── unitree_sdk2/                # Unitree SDK2 source for Sim2Real
│   └── unitree_sdk2_python/         # Python Unitree SDK2 bindings
├── artifacts/
│   ├── policies/v5_model_40000/     # Packaged V5 40k policy
│   └── slam_maps/                   # Saved map/debug artifacts
├── scripts/                         # Workspace helper scripts
└── docker-compose.yaml              # Root Docker entry point
```

Excluded by design: old V2/V3/V4 training logs, build/install outputs, caches, TensorBoard files, generated ROS workspaces, and prebuilt install folders.

## Clone On Another Desktop

```bash
mkdir -p ~/Project
git clone https://github.com/Kimz1xq/Unitree_Go2.git ~/Project/Unitree_Go2
cd ~/Project/Unitree_Go2
scripts/install_workspace_env.sh
source ~/.bashrc
```

Required runtime dependencies depend on what you run:

```text
Training:
  - Docker + NVIDIA runtime
  - IsaacLab base image that provides /isaac-sim/python.sh

Sim2Sim / SLAM / Planning:
  - ROS2 Humble recommended
  - MuJoCo Python package
  - ONNX Runtime
  - colcon, CMake, PCL/Eigen
  - FAST-LIO2 workspace if running FAST-LIO2
```

## Select Model, Map, And Backend

Set these once per shell. Change them whenever you want a different model or scene.

```bash
export REPO="${REPO:-$HOME/Project/Unitree_Go2}"
export TRAINING_BACKEND="${TRAINING_BACKEND:-isaac_lab}"

export POLICY_NAME="${POLICY_NAME:-v5_model_40000}"
export POLICY_ITER="${POLICY_ITER:-40000}"
export POLICY_ONNX="$REPO/artifacts/policies/$POLICY_NAME/exported/policy.onnx"
export POLICY_PT="$REPO/artifacts/policies/$POLICY_NAME/model_${POLICY_ITER}.pt"

export MUJOCO_SCENE="${MUJOCO_SCENE:-$REPO/simulation/mujoco/go2/scene_icra2024_sloped.xml}"
export TRG_MAP="${TRG_MAP:-icra2024_sloped_route_robot30}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
```

Packaged policy:

| Policy | Backend | Path | Notes |
| --- | --- | --- | --- |
| `v5_model_40000` | IsaacLab/RSL-RL | `artifacts/policies/v5_model_40000/` | Current packaged sim2sim policy. Keep `policy.onnx` and `policy.onnx.data` together. |

Future policies should follow:

```text
artifacts/policies/<policy_name>/
├── model_<iteration>.pt
├── exported/policy.onnx
├── exported/policy.onnx.data
└── params/
```

## Docker

Build the IsaacLab training image from the repository root:

```bash
cd "$REPO"
docker compose build isaac-lab-template
```

Start the IsaacLab container:

```bash
cd "$REPO"
docker compose up -d isaac-lab-template
```

Build the ROS2 Humble SLAM/RL runtime:

```bash
cd "$REPO"
docker compose build go2-slam-rl-humble
```

Open a ROS2 runtime shell:

```bash
cd "$REPO"
docker compose run --rm go2-slam-rl-humble
```

The compose file mounts:

```text
/workspace/Unitree_Go2     -> full consolidated repository
/workspace/unitree_rl_lab  -> IsaacLab backend path for compatibility
```

## IsaacLab Training

Inside the `isaac-lab-template` container, the IsaacLab backend is mounted at `/workspace/unitree_rl_lab`.

Train V5:

```bash
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py \
  --task Unitree-Go2-ICROS2026-V5 \
  --headless
```

Train V6 perceptive-real:

```bash
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v6_perceptive_real/train.py \
  --task Unitree-Go2-ICROS2026-V6-PerceptiveReal \
  --headless
```

Play/evaluate V5:

```bash
docker exec -it isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/play.py \
  --task Unitree-Go2-ICROS2026-V5 \
  --num_envs 8 \
  --lin_vel_x 1.0
```

Isaac Gym is not packaged yet. When added, keep it as a separate backend under `training/isaac_gym/` and export policies to `artifacts/policies/<policy_name>/` so the same sim2sim/ROS2 commands can select them through `POLICY_ONNX`.

## MuJoCo Sim2Sim

Run without ROS:

```bash
cd "$REPO"
python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \
  --onnx "$POLICY_ONNX" \
  --map "$MUJOCO_SCENE" \
  --no-ros
```

Run with ROS2 topics, LiDAR, and MuJoCo-generated height scan:

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \
  --onnx "$POLICY_ONNX" \
  --map "$MUJOCO_SCENE" \
  --lidar \
  --domain-id "$ROS_DOMAIN_ID" \
  --publish-height-scan
```

Useful packaged MuJoCo scenes:

```text
simulation/mujoco/go2/scene_icra2023_easy.xml
simulation/mujoco/go2/scene_icra2023_hard.xml
simulation/mujoco/go2/scene_icra2024_flat.xml
simulation/mujoco/go2/scene_icra2024_sloped.xml
simulation/mujoco/go2/scene_icros2025.xml
simulation/mujoco/go2/scene_icros2026.xml
```

## Teleop

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/teleop_cmd_vel.py \
  --domain-id "$ROS_DOMAIN_ID" \
  --max-lin 0 \
  --max-strafe 0 \
  --max-yaw 0
```

Key behavior:

```text
W/S: increase/decrease forward velocity
A/D: increase/decrease lateral velocity
Q/E: increase/decrease yaw rate
K:   zero velocity command
Space: publish robot reset on /go2/reset
```

Unlimited teleop caps are for simulation debugging only.

## Teleop Map Accumulation

Terminal 1, run sim2sim with LiDAR:

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \
  --onnx "$POLICY_ONNX" \
  --map "$MUJOCO_SCENE" \
  --lidar \
  --domain-id "$ROS_DOMAIN_ID" \
  --publish-height-scan
```

Terminal 2, run teleop:

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/teleop_cmd_vel.py \
  --domain-id "$ROS_DOMAIN_ID" \
  --max-lin 0 \
  --max-strafe 0 \
  --max-yaw 0
```

Terminal 3, accumulate map:

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/teleop_map_accumulator.py \
  --domain-id "$ROS_DOMAIN_ID" \
  --cloud-topic /utlidar/cloud \
  --odom-topic /Odometry \
  --out-topic /teleop_map \
  --output "$REPO/artifacts/slam_maps/teleop_map_latest.pcd"
```

Terminal 4, view in RViz:

```bash
source /opt/ros/humble/setup.bash
rviz2 -d "$REPO/ros2/go2_roughnav/rviz/teleop_map_debug.rviz"
```

## ROS2 Workspace Setup

Create ROS2 workspace source copies from this consolidated repo:

```bash
cd "$REPO"
bash scripts/setup_ros2_workspace.sh
```

Build TRG core and ROS2 pipeline:

```bash
source /opt/ros/humble/setup.bash

cd ~/go2_sim_ws/src/TRG-planner-1
cmake -B cpp/trg_planner/build -S cpp/trg_planner \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/go2_sim_ws/install/trg_planner_core
cmake --build cpp/trg_planner/build -j$(nproc)
cmake --install cpp/trg_planner/build

cd ~/go2_sim_ws
export CMAKE_PREFIX_PATH=$HOME/go2_sim_ws/install/trg_planner_core:${CMAKE_PREFIX_PATH:-}
colcon build \
  --base-paths src/TRG-planner-1/pipelines/ros2 \
  --cmake-args -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
```

Build the Go2 ROS2 glue package:

```bash
source /opt/ros/humble/setup.bash
source ~/go2_sim_ws/install/setup.bash

cd ~/go2_roughnav_ws
colcon build --packages-select go2_roughnav
source install/setup.bash
```

FAST-LIO2 is expected in a separate workspace, typically:

```text
/fastlio_ws/src/FAST_LIO_ROS2
/fastlio_ws/install/setup.bash
```

The Docker ROS2 runtime builds FAST-LIO2 into `/fastlio_ws` by default.

## FAST-LIO2 Only

Terminal 1, sim publishes LiDAR/IMU and leaves odometry to FAST-LIO2:

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \
  --onnx "$POLICY_ONNX" \
  --map "$MUJOCO_SCENE" \
  --lidar \
  --domain-id "$ROS_DOMAIN_ID" \
  --no-fastlio-odom \
  --publish-height-scan
```

Terminal 2, launch FAST-LIO2:

```bash
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash

ros2 launch go2_roughnav 02_fastlio_only.launch.py \
  fastlio_launch:=/fastlio_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py
```

## TRG-Planner Only

```bash
source /opt/ros/humble/setup.bash
source ~/go2_sim_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash

ros2 launch go2_roughnav 03_trg_only.launch.py \
  trg_map:="$TRG_MAP"
```

Useful TRG maps:

```text
icra2023_easy_route_robot30
icra2023_hard_route_robot30
icra2024_flat_route_robot30
icra2024_sloped_route_robot30
icros2025_obstacle_left_robot30
icros2025_obstacle_right_robot30
```

## Full Sim Pipeline: MuJoCo + FAST-LIO2 + TRG + RL

Terminal 1:

```bash
cd "$REPO"
source /opt/ros/humble/setup.bash

python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \
  --onnx "$POLICY_ONNX" \
  --map "$MUJOCO_SCENE" \
  --lidar \
  --domain-id "$ROS_DOMAIN_ID" \
  --no-fastlio-odom \
  --require-height-scan
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_sim_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash

ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py \
  launch_fastlio:=true \
  launch_trg:=true \
  launch_cmd:=true \
  launch_height_scan:=true \
  launch_health:=true \
  launch_rviz:=true \
  fastlio_config_file:=fastlio_mujoco.yaml \
  trg_map:="$TRG_MAP" \
  rviz_config:="$REPO/ros2/go2_roughnav/rviz/isaac_debug.rviz"
```

## V5 Runtime Contract

The current V5 policy input is:

```text
proprio_history(225) + height_scan(273) = 498
```

Required runtime inputs:

```text
/cmd_vel          geometry_msgs/msg/Twist
/rl/height_scan   std_msgs/msg/Float32MultiArray, length 273
Go2 lowstate      IMU, joint positions, joint velocities
```

`/cmd_vel` alone is not enough for V5. In sim, `--publish-height-scan` generates the scan from MuJoCo raycasts. In the FAST-LIO2 pipeline, `height_scan_bridge` converts `/cloud_registered` and `/Odometry` into `/rl/height_scan`.

## Smoke Tests

Check ONNX:

```bash
python3 - <<'PY'
import os
import onnx
p = os.environ.get("POLICY_ONNX", os.path.expanduser("~/Project/Unitree_Go2/artifacts/policies/v5_model_40000/exported/policy.onnx"))
m = onnx.load(p)
onnx.checker.check_model(m)
print("ONNX OK")
print("inputs:", [(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input])
print("outputs:", [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
PY
```

Compile Python entry points:

```bash
cd "$REPO"
python3 -m py_compile \
  training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \
  training/isaac_lab/unitree_rl_lab/scripts/ros2/teleop_cmd_vel.py \
  training/isaac_lab/unitree_rl_lab/scripts/ros2/teleop_map_accumulator.py \
  training/isaac_lab/unitree_rl_lab/scripts/ros2/height_scan_bridge.py \
  training/isaac_lab/unitree_rl_lab/scripts/ros2/trg_path_follower.py \
  ros2/go2_roughnav/setup.py
```

## Local Folder Cleanup Policy

The consolidated workspace is `~/Project/Unitree_Go2`. Local incoming/downloaded files go to `~/Inbox`, research material goes to `~/Research`, screenshots/images go to `~/Images`, and general documents stay in `~/Documents`.

Do not keep extra compatibility symlinks in `~/`; use the real folder paths in shells, editors, Docker mounts, and scripts. Older sibling folders such as `~/unitree_rl_lab`, `~/unitree_mujoco`, `~/go2_sim_ws`, `~/go2_roughnav_ws`, `~/ICRA2023_Quadruped_Robot_Challenges`, and `~/ICRA2024_Quadruped_Robot_Challenges` were reviewed as source material. They should be treated as legacy copies after this repository is validated on the target desktop.

See `docs/WORKSPACE_CONSOLIDATION.md` for what was integrated, what was excluded, and which old local folders can be archived after validation.
