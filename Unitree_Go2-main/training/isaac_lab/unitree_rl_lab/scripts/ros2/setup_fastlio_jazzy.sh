#!/bin/bash
# =============================================================================
# FAST-LIO2 + livox_ros_driver2 ROS2 Jazzy 빌드 스크립트
# 호스트 Ubuntu 24.04 + ROS2 Jazzy 기준
#
# 실행: bash scripts/ros2/setup_fastlio_jazzy.sh
# =============================================================================
set -e

echo "================================================="
echo "  FAST-LIO2 ROS2 Jazzy 빌드 시작"
echo "  Ubuntu 24.04 + ROS2 Jazzy"
echo "================================================="

# ── 1. ROS2 Jazzy 확인 ────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash 2>/dev/null || {
    echo "[ERROR] ROS2 Jazzy가 설치되어 있지 않습니다."
    echo "  sudo apt install ros-jazzy-desktop"
    exit 1
}
echo "[OK] ROS2 Jazzy 확인"

# ── 2. 의존성 설치 ─────────────────────────────────────────────────────────────
echo "[1/5] 의존성 설치..."
sudo apt update -q
sudo apt install -y \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-rviz2 \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    libpcl-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    git
echo "[OK] 의존성 설치 완료"

# ── 3. 워크스페이스 생성 ───────────────────────────────────────────────────────
WS=~/fastlio_ws
mkdir -p ${WS}/src
echo "[2/5] 워크스페이스: ${WS}"

# ── 4. livox_ros_driver2 (Mid-360 드라이버) ────────────────────────────────────
if [ ! -d "${WS}/src/livox_ros_driver2" ]; then
    echo "[3/5] livox_ros_driver2 클론..."
    cd ${WS}/src
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    echo "[OK] livox_ros_driver2 클론 완료"
else
    echo "[SKIP] livox_ros_driver2 이미 있음"
fi

# ── 5. FAST-LIO ROS2 ──────────────────────────────────────────────────────────
if [ ! -d "${WS}/src/FAST_LIO_ROS2" ]; then
    echo "[4/5] FAST_LIO_ROS2 클론..."
    cd ${WS}/src
    git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git
    echo "[OK] FAST_LIO_ROS2 클론 완료"
else
    echo "[SKIP] FAST_LIO_ROS2 이미 있음"
fi

# ── 6. Mid-360 설정 파일 확인 및 생성 ─────────────────────────────────────────
CONFIG_DIR=${WS}/src/FAST_LIO_ROS2/config
MID360_YAML=${CONFIG_DIR}/mid360.yaml

echo "[5/5] Mid-360 설정 파일 확인..."
if [ ! -f "${MID360_YAML}" ]; then
    echo "  mid360.yaml 생성 중..."
    cat > ${MID360_YAML} << 'YAMLEOF'
# FAST-LIO2 Mid-360 설정 파일
# Go2 탑재 Mid-360 + Go2 내장 IMU 사용

common:
    lid_topic:  "/livox/lidar"     # Mid-360 포인트클라우드
    imu_topic:  "/go2/imu"         # Go2 내장 IMU (고성능)
    time_sync_en: false

preprocess:
    lidar_type: 6                   # LIVOX (Mid-360)
    scan_line: 4                    # Mid-360 pseudo scan lines
    timestamp_unit: 3               # 0-ms, 1-us, 2-10us, 3-ns
    blind: 0.5                      # 블라인드 거리 (m)

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360.0               # Mid-360 전방위
    det_range: 40.0                 # 최대 탐지 거리 (m)
    extrinsic_est_en: false         # 외부 파라미터 추정 OFF
    extrinsic_T: [0.0, 0.0, 0.15]  # LiDAR 위치 (로봇 기준, m)
    extrinsic_R: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]

publish:
    path_en: true
    scan_publish_en: true
    dense_publish_en: true
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: false
    interval: -1
YAMLEOF
    echo "[OK] mid360.yaml 생성 완료"
else
    echo "[OK] mid360.yaml 이미 있음"
fi

# 시뮬레이션용 설정 (MuJoCo sim2sim_ros2.py와 연동)
SIM_YAML=${CONFIG_DIR}/mid360_sim.yaml
cat > ${SIM_YAML} << 'YAMLEOF'
# FAST-LIO2 시뮬레이션 설정
# sim2sim_ros2.py가 발행하는 토픽에 맞춤

common:
    lid_topic:  "/livox/lidar"    # sim2sim_ros2.py 발행
    imu_topic:  "/imu/data"        # sim2sim_ros2.py 발행
    time_sync_en: false

preprocess:
    lidar_type: 6
    scan_line: 4
    timestamp_unit: 3
    blind: 0.2

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360.0
    det_range: 40.0
    extrinsic_est_en: false
    extrinsic_T: [0.0, 0.0, 0.0]
    extrinsic_R: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]

publish:
    path_en: true
    scan_publish_en: true
    dense_publish_en: false
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: false
    interval: -1
YAMLEOF
echo "[OK] mid360_sim.yaml 생성 완료"

# ── 7. 빌드 ──────────────────────────────────────────────────────────────────
echo ""
echo "빌드 시작 (5~15분 소요)..."
cd ${WS}
source /opt/ros/jazzy/setup.bash

# rosdep 초기화 (처음 한 번만)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || true
    rosdep update
fi

# 의존성 해결
rosdep install --from-paths src --ignore-src -y 2>/dev/null || true

# 빌드 (livox_ros_driver2 먼저)
colcon build --packages-select livox_ros_driver2 --symlink-install
source install/setup.bash

# FAST-LIO2 빌드
colcon build --packages-select fast_lio --symlink-install

echo ""
echo "================================================="
echo "  빌드 완료!"
echo "================================================="
echo ""
echo "실행 방법:"
echo ""
echo "  [터미널 1] MuJoCo sim2sim + ROS2 브릿지:"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    cd ~/Unitree_Go2"
echo "    export REPO=\$PWD"
echo "    python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \\"
echo "        --onnx \"\$REPO/artifacts/policies/v5_model_40000/exported/policy.onnx\" \\"
echo "        --map icra2023_easy \\"
echo "        --lidar  # FAST-LIO2 사용 시 추가"
echo ""
echo "  [터미널 2] FAST-LIO2 (선택, --lidar 플래그 사용 시):"
echo "    source ~/fastlio_ws/install/setup.bash"
echo "    ros2 launch fast_lio mapping.launch.py \\"
echo "        config_file:=mid360_sim.yaml"
echo ""
echo "  [터미널 3] Goal Follower:"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    cd ~/Unitree_Go2"
echo "    python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/trg_path_follower.py --mode direct"
echo ""
echo "  [터미널 4] RViz2 (목표 설정):"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    rviz2 -d ~/Unitree_Go2/ros2/go2_roughnav/rviz/isaac_debug.rviz"
echo ""
echo "  FAST-LIO2 없이 테스트 (sim ground truth /odom 사용):"
echo "    trg_path_follower.py --mode direct  # /odom 기본"
echo "  FAST-LIO2 사용 시:"
echo "    trg_path_follower.py --mode trg"
