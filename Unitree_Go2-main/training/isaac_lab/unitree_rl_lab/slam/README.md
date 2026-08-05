# Go2 SLAM/TRG Implementation Guide

이 문서는 SLAM팀 자료(`go2_fast_trg`, FAST-LIO, TRG-planner)를 V5 RL
locomotion과 붙이기 위한 실행 기준이다.

ROS 런타임 분리는 [`ROS_RUNTIME.md`](ROS_RUNTIME.md)를 먼저 본다.

## 최종 결정

대회 목표가 장애물 회피가 아니라 계단, 경사, 구멍, 험지 극복이므로
`/cmd_vel`만 RL에 넘기는 구조는 메인 전략으로 부족하다.

최종 파이프라인은 다음처럼 간다.

```text
Go2 LiDAR/IMU
  -> FAST-LIO2                         -> /cloud_registered + /Odometry
  -> TRG-planner                       -> /path
  -> path_to_cmd_vel                   -> /cmd_vel
  -> height_scan_bridge                -> /rl/height_scan
  -> V5 RL deploy                      -> joint targets
  -> Unitree SDK2 low-level control
```

## `go2_fast_trg`에서 고칠 점

원본 repo는 FAST-LIO + TRG + `/cmd_vel`까지는 좋다. 우리가 추가/수정해야 할
부분은 RL policy 입력에 맞춘 부분이다.

| 항목 | 원본 상태 | 개선 |
|------|-----------|------|
| `/cmd_vel` | path follower가 publish | `angular.z`를 yaw rate로 명확히 고정 |
| height input | 없음 | `/cloud_registered + /Odometry -> /rl/height_scan` 추가 |
| launch | Isaac bridge가 기본 포함 | 실기체 launch에서는 Isaac bridge 기본 off |
| setup.py | 신규 config 일부 미설치 가능 | `fastlio_go2_hw.yaml`, `rl_interface.yaml`, 새 launch 설치 |
| health | `/cmd_vel`까지 확인 | `/utlidar/*`, `/rl/height_scan`까지 확인 |

## 적용 방법

```bash
cd /home/nuri/unitree_rl_lab
bash slam/go2_fast_trg_overlay/apply_overlay.sh \
  ~/go2_roughnav_ws/src/go2_roughnav

cd ~/go2_roughnav_ws
colcon build --symlink-install
source install/setup.bash
```

현재 머신에서는 위 overlay를 `~/go2_roughnav_ws/src/go2_roughnav`에 적용했다.
대회 런타임은 Humble로 고정한다. `go2-slam-rl-humble` 컨테이너에서
`go2_roughnav`, `livox_ros_driver2`, `fast_lio` 빌드와 launch argument 로딩을
확인했다.

TRG-planner도 `~/go2_sim_ws/src/TRG-planner-1`에 설치했다. ROS2 package에
`project(trg_planner_ros)`와 `TRG_ROS_DIR` 정의가 빠져 있어 로컬 패치 후
`trg_planner_ros trg_ros2_node` 등록까지 확인했다.

컨테이너 기본 FAST-LIO launch 경로:

```text
/fastlio_ws/src/FAST_LIO_ROS2/launch/mapping.launch.py
```

## Day 1 Mapping

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_sim_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash

ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py \
  launch_trg:=false \
  launch_cmd:=false \
  launch_height_scan:=false \
  fastlio_config_file:=fastlio_go2_hw.yaml
```

확인:

```bash
ros2 topic list | grep utlidar
ros2 topic echo /Odometry --once
ros2 topic echo /cloud_registered --once
```

맵 저장 후에는 `slam_maps/real_go2_fastlio.pcd`를 백업한다.

## Day 2/3 Traversal

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_sim_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash

ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py \
  fastlio_config_file:=fastlio_go2_hw.yaml \
  trg_map:=competition
```

확인:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /rl/height_scan --once
```

`pipeline_health`에서 최소한 다음이 OK여야 RL을 켠다.

```text
/utlidar/cloud
/utlidar/imu
/Odometry
/cloud_registered
/path
/cmd_vel
/rl/height_scan
```

## `/cmd_vel` 계약

```text
linear.x   전후 속도, m/s
linear.y   좌우 속도, m/s
angular.z  yaw 회전 속도, rad/s
```

`angular.z`는 z축 방향 상승이 아니다. 목표 heading도 아니다. 목표 각도가
필요하면 planner가 heading error를 yaw rate로 변환해서 넣어야 한다.

## `/rl/height_scan` 계약

```text
type:     std_msgs/msg/Float32MultiArray
length:   273
layout:   21 x 13 grid, x-major flatten
x range:  [-1.0, 1.0] m
y range:  [-0.6, 0.6] m
value:    base z 기준 상대 지형 높이
clip:     [-1.0, 1.0]
```

V5 IsaacLab sensor가 `ray_alignment="yaw"`를 쓰므로 실기체 bridge도 roll/pitch를
제외한 yaw-only base frame으로 cloud를 변환한다.

## 테스트 순서

1. Bag으로 FAST-LIO 재생: `/cloud_registered`, `/Odometry` 확인.
2. `height_scan_bridge`: `/rl/height_scan` 길이 273 확인.
3. TRG goal 입력: `/path` 확인.
4. `path_to_cmd_vel`: `/cmd_vel`의 `angular.z`가 rad/s 범위인지 확인.
5. RL deploy: `/cmd_vel + /rl/height_scan + lowstate`로 ONNX policy 실행.
6. 실기체 첫 주행은 `max_lin_x=0.20~0.25`로 낮춰서 시작.
