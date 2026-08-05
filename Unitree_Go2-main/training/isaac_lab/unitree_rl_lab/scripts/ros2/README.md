# Go2 RL + go2_fast_trg 통합 가이드

이 디렉터리는 RL팀 쪽 어댑터를 관리한다. SLAM팀의 기준 repo는
`SeEEEun/go2_fast_trg`이며, 그 repo가 FAST-LIO + TRG-planner + `/cmd_vel`
출력까지 담당한다고 본다.

중요: `isaac-lab-template` 컨테이너에는 ROS2가 없다. 이 디렉터리의 ROS2
스크립트는 **ROS 2 Humble 컨테이너**에서 실행한다.
런타임 분리는 [`slam/ROS_RUNTIME.md`](../../slam/ROS_RUNTIME.md)를 기준으로 한다.

## 토픽 계약

```
Go2 Mid-360/IMU
  → go2_fast_trg FAST-LIO        → /cloud_registered, /Odometry
  → go2_fast_trg TRG-planner     → /path
  → go2_fast_trg path_to_cmd_vel → /cmd_vel
  → unitree_rl_lab RL adapter    → ONNX policy → joint targets
```

`/cmd_vel.angular.z`는 z축 방향 이동이 아니다. ROS `Twist`에서
`angular.z`는 수직 z축을 중심으로 도는 yaw rate(rad/s)다. 목표 heading을
그대로 넣지 말고, planner/path follower가 heading error를 yaw rate로 변환한
값만 `/cmd_vel`에 넣어야 한다.

V5 policy는 `/cmd_vel`만으로 실행할 수 없다. Actor input이
`proprio_history(225) + height_scan(273) = 498`이므로, 실기체에서는
FAST-LIO point cloud를 273-dim height scan으로 변환하는 어댑터가 필요하다.
자세한 계약은 [`slam/RL_SLAM_INTERFACE.md`](../../slam/RL_SLAM_INTERFACE.md)를 기준으로 한다.

## 파일 구성

| 파일 | 역할 |
|------|------|
| `sim2sim_ros2.py` | MuJoCo sim2sim + ROS2 bridge. 45/225/318/498-dim policy 자동 지원 |
| `trg_path_follower.py` | TRG-planner path 또는 direct goal을 `/cmd_vel`로 변환하는 fallback |
| `height_scan_bridge.py` | `/cloud_registered` + `/Odometry` → `/rl/height_scan` 273-dim 변환 |
| `setup_fastlio_jazzy.sh` | 구버전 실험용 FAST-LIO2 빌드 스크립트. SLAM팀은 `go2_fast_trg` 기준 사용 권장 |

SLAM 구현/적용 파일은 RL 어댑터와 섞지 않기 위해 top-level [`slam/`](../../slam) 폴더로 분리했다.
SLAM 적용 순서는 [`slam/README.md`](../../slam/README.md)를 기준으로 한다.

## 권장 개발 순서

### 1. RL 모델 ONNX 준비

V5 학습 완료 후 `model_50000.pt`를 ONNX로 export한다.

```bash
python3 scripts/export_onnx_standalone.py \
  --checkpoint logs/rsl_rl/unitree_go2_icros2026_v5/<RUN>/model_50000.pt \
  --out model_icros2026_v5_50000.onnx
```

### 2. SLAM/TRG 없이 ROS2 sim2sim smoke test

```bash
source /opt/ros/humble/setup.bash
python3 scripts/ros2/sim2sim_ros2.py \
  --onnx "$REPO/artifacts/policies/v5_model_40000/exported/policy.onnx" \
  --map icra2023_easy \
  --domain-id 42 \
  --publish-height-scan
```

확인:
- ONNX obs_dim이 `498 (V5/V3 history+scan)`으로 표시되는지
- `/odom`, `/Odometry`, `/imu/data`, `/utlidar/imu`, `/cmd_vel`, `/rl/height_scan` 토픽이 보이는지

`--publish-height-scan`은 MuJoCo raycast를 `/rl/height_scan`으로 발행한다. 이
모드는 SLAM 없이 V5 policy가 `/cmd_vel + /rl/height_scan` 결합 입력을 받는지
확인하는 smoke test용이다.

### 3. go2_fast_trg와 연결한 sim test

통합 repo 기준으로 ROS2 workspace symlink를 만든다.

```bash
cd ~/Unitree_Go2
export REPO=$PWD
bash scripts/setup_ros2_workspace.sh
```

터미널 구성:

```bash
# Terminal 1: RL sim bridge
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_sim_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash
export ROS_DOMAIN_ID=42
python3 "$REPO/training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py" \
  --onnx "$REPO/artifacts/policies/v5_model_40000/exported/policy.onnx" \
  --map icra2023_easy \
  --domain-id 42 \
  --height-scan-topic /rl/height_scan \
  --require-height-scan

# Terminal 2: go2_fast_trg pipeline
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
source ~/go2_sim_ws/install/setup.bash
source ~/go2_roughnav_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py \
  launch_bag:=false \
  launch_fastlio:=false \
  launch_height_scan:=false \
  launch_trg:=true \
  launch_cmd:=true
```

이 구성에서는 Terminal 1이 `/rl/height_scan`이 들어오기 전까지 V5 action을
정지시킨다. 실제 SLAM 없이 테스트할 때는 Terminal 1에 `--publish-height-scan`을
추가하거나, 별도 `height_scan_bridge`가 `/rl/height_scan`을 발행해야 한다.

### 4. 실기체용 height_scan bridge

```bash
source /opt/ros/humble/setup.bash
python3 scripts/ros2/height_scan_bridge.py \
  --cloud-topic /cloud_registered \
  --odom-topic /Odometry \
  --out-topic /rl/height_scan
```

RL deploy node는 다음 입력을 동시에 사용해야 한다.

| 입력 | 출처 |
|------|------|
| `base_ang_vel`, `projected_gravity` | Go2 lowstate IMU |
| `joint_pos_rel`, `joint_vel_rel` | Go2 lowstate motor state |
| `last_action` | RL node 내부 |
| `velocity_commands` | `/cmd_vel` |
| `height_scan` | `/rl/height_scan` |

## 학습을 다시 해야 하나?

메인 노선 기준으로는 **지금 다시 학습하지 않는다. V5를 그대로 SLAM과 결합한다.**

- V5를 쓰려면 반드시 실시간 273-dim height scan을 만들어 actor에 넣어야 한다.
- `/cmd_vel`만 받고 height scan을 0 또는 -1로 채우는 방식은 권장하지 않는다. 학습 분포와 달라져서 장애물 앞에서 성능이 크게 흔들릴 수 있다.
- V6는 V5와 같은 498-dim 구조의 optional 강건화 실험이다. Bag/실기체 테스트에서 FAST-LIO height_scan의 빈 셀/지연/스파스함이 문제로 확인될 때만 검토한다.

따라서 현재 최선은 **V5 학습 완료 → FAST-LIO point cloud를 V5 height_scan으로 변환 → V5 ONNX로 sim2sim/실기체 테스트**이다.

## go2_fast_trg repo 개선 제안

아래 개선은 `slam/go2_fast_trg_overlay/`로 구현했고,
`~/go2_roughnav_ws/src/go2_roughnav`에 적용 및 `colcon build --symlink-install`
까지 확인했다.

- `setup.py`의 `data_files`에 `fastlio_go2_hw.yaml`, `trg_ros2_params_isaac.yaml`, `competition.yaml/pre.yaml` 등 실제 launch에서 쓰는 신규 config가 모두 포함되어야 한다.
- `/path`, `/cmd_vel`, `/Odometry`, `/cloud_registered` 토픽 이름을 README에 “RL팀 계약”으로 고정해두는 것이 좋다.
- `path_to_cmd_vel.py`의 속도 제한은 실기체 초기 테스트에서 지금처럼 보수적인 값이 좋다: `max_lin_x=0.35`, `max_lin_y=0.20`, `max_ang_z=0.7`.
- Day 1 PCD 저장 위치와 Day 2/3 prebuilt map path를 launch argument로 노출하면 컴퓨터마다 경로 수정이 줄어든다.
- pipeline_health 출력에 `/rl/height_scan`도 추가하면 RL팀 준비 상태까지 한 줄로 볼 수 있다.

## 공통 주의

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```

실기체 네트워크는 SLAM팀 문서 기준으로 `192.168.123.x/24` 대역을 맞춘다.
