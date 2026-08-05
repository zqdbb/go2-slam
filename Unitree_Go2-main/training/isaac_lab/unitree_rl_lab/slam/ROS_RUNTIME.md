# ROS Runtime Split

## 결론

`isaac-lab-template` Docker는 IsaacLab 학습/플레이용이다. 이 컨테이너에는
ROS2가 없다.

```text
isaac-lab-template:
  - IsaacLab / RSL-RL training
  - /isaac-sim/python.sh
  - no ros2, no rclpy

ROS runtime:
  - FAST-LIO2
  - TRG-planner
  - go2_roughnav
  - sim2sim_ros2.py
  - height_scan_bridge
```

현재 호스트에는 ROS 2 Jazzy가 설치되어 있지만, 대회/실기체 런타임은 ROS 2
Humble로 고정한다. Jazzy는 로컬 빠른 확인용으로만 쓴다.

## ROS 버전 원칙

한 번의 ROS graph 안에 들어가는 노드들은 같은 ROS distro로 맞춘다.

확정:

```text
dedicated ROS 2 Humble container
  go2_roughnav + FAST-LIO2 + TRG + sim2sim_ros2 + Unitree bridge
```

Jazzy와 Humble 노드를 섞어 통신시키는 방식은 메시지가 단순하면 될 때도 있지만,
대회 런타임 기준으로는 피한다. `/cmd_vel`, `/Odometry`, `PointCloud2` 같은 표준
메시지라도 DDS/RMW와 패키지 ABI 차이 때문에 디버깅 비용이 커질 수 있다.

## 실행 배치

### 1. 학습

```bash
docker exec isaac-lab-template /isaac-sim/python.sh \
  /workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py \
  --task Unitree-Go2-ICROS2026-V5 --headless --num_envs 16384
```

### 2. ROS2 Humble 컨테이너

Build:

```bash
cd /home/nuri/unitree_rl_lab
docker compose -f docker/docker-compose.yaml build go2-slam-rl-humble
```

Run:

```bash
docker compose -f docker/docker-compose.yaml run --rm go2-slam-rl-humble
```

Inside:

```bash
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/go2_roughnav_ws
colcon build --symlink-install
source install/setup.bash
```

## 현재 상태

- Host ROS distro: Jazzy, but competition runtime target: Humble
- `isaac-lab-template`: ROS 없음
- `go2-slam-rl-humble`: ROS 2 Humble + CycloneDDS + Livox-SDK2 + `livox_ros_driver2` + `fast_lio` 빌드 성공
- `~/go2_sim_ws/src/TRG-planner-1`: ROS2 package 로컬 패치 후 `trg_planner_ros` 빌드 성공
- `~/go2_roughnav_ws/src/go2_roughnav`: overlay 적용됨
- `colcon build --symlink-install --packages-select go2_roughnav`: Humble 컨테이너에서 성공
- `ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py --show-args`: Humble 컨테이너에서 성공

TRG-planner는 core CMake library를 먼저 local prefix로 설치한 뒤 ROS2 package를
빌드했다.

```bash
cd ~/go2_sim_ws/src/TRG-planner-1
cmake -B cpp/trg_planner/build -S cpp/trg_planner \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/go2_sim_ws/install/trg_planner_core
cmake --build cpp/trg_planner/build -j$(nproc)
cmake --install cpp/trg_planner/build

cd ~/go2_sim_ws
export CMAKE_PREFIX_PATH=$HOME/go2_sim_ws/install/trg_planner_core:${CMAKE_PREFIX_PATH:-}
colcon build --symlink-install \
  --base-paths src/TRG-planner-1/pipelines/ros2 \
  --cmake-args -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
```

참고: `go2_description`은 없을 수 있으므로 `launch_robot_model:=false` 기본값에서는
launch 로딩을 막지 않게 했다. 로봇 모델 TF까지 RViz에 띄우려면 별도로
`go2_description`을 빌드하고 source한 뒤 `launch_robot_model:=true`를 쓴다.
