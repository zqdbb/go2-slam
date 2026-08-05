"""
sim2sim_ros2.py — MuJoCo sim2sim + ROS2 Jazzy 브릿지

기존 sim2sim_icros2026.py에 ROS2 인터페이스를 추가:
  Subscribe: /cmd_vel (geometry_msgs/Twist)  → RL policy velocity command 주입
  Subscribe: /rl/height_scan (Float32MultiArray) → V5 policy terrain input 주입
  Publish:   /odom    (nav_msgs/Odometry)    → 시뮬레이션 ground truth 위치
  Publish:   /imu/data (sensor_msgs/Imu)     → 시뮬레이션 IMU (FAST-LIO2 optional)
  Publish:   /livox/lidar (PointCloud2)       → 시뮬 LiDAR (FAST-LIO2 optional, --lidar 플래그)

실행:
  source /opt/ros/jazzy/setup.bash
  cd ~/Unitree_Go2
  export REPO=$PWD
  python3 training/isaac_lab/unitree_rl_lab/scripts/ros2/sim2sim_ros2.py \\
      --onnx "$REPO/artifacts/policies/v5_model_40000/exported/policy.onnx" \\
      --map  icra2023_easy

SLAM팀과 연동 시:
  동일 ROS_DOMAIN_ID 설정 후 trg_path_follower.py 별도 실행
  → /cmd_vel 구독하여 RL 정책에 주입
"""

import argparse
import math
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

# ROS2
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from std_msgs.msg import Empty, Float32MultiArray, Header
    import tf2_ros
    HAS_ROS2 = True
except ImportError:
    print("[WARN] rclpy 없음. ROS2 기능 비활성화. headless 시뮬레이션으로 실행.")
    HAS_ROS2 = False

# ─────────────────────────────────────────────────────────────────────────────
# 경로 설정
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent

def find_repo_root() -> Path:
    env_repo = os.environ.get("REPO")
    candidates = []
    if env_repo:
        candidates.append(Path(env_repo).expanduser())
    candidates.extend(SCRIPT_DIR.parents)
    for candidate in candidates:
        if (candidate / "simulation/mujoco/go2").exists():
            return candidate
    return SCRIPT_DIR.parents[4]

REPO_ROOT = find_repo_root()
GO2_DIR = Path(os.environ.get("GO2_MUJOCO_DIR", REPO_ROOT / "simulation/mujoco/go2")).expanduser()

MAP_ALIASES = {
    "icros2025":       str(GO2_DIR / "scene_icros2025.xml"),
    "icros2026":       str(GO2_DIR / "scene_icros2026.xml"),
    "icra2023_easy":   str(GO2_DIR / "scene_icra2023_easy.xml"),
    "icra2023_hard":   str(GO2_DIR / "scene_icra2023_hard.xml"),
    "icra2024_flat":   str(GO2_DIR / "scene_icra2024_flat.xml"),
    "icra2024_sloped": str(GO2_DIR / "scene_icra2024_sloped.xml"),
    "flat":            str(GO2_DIR / "scene.xml"),
}

# ─────────────────────────────────────────────────────────────────────────────
# 상수 (sim2sim_icros2026.py와 동일)
# ─────────────────────────────────────────────────────────────────────────────
MJ_JOINT_NAMES = ["FR_hip","FR_thigh","FR_calf","FL_hip","FL_thigh","FL_calf",
                   "RR_hip","RR_thigh","RR_calf","RL_hip","RL_thigh","RL_calf"]
IL2MJ = np.array([3,0,9,6,4,1,10,7,5,2,11,8])
MJ2IL = np.array([1,5,9,0,4,8,3,7,11,2,6,10])
DEFAULT_JOINT_POS_IL = np.array(
    [0.1,-0.1,0.1,-0.1, 0.8,0.8,1.0,1.0, -1.5,-1.5,-1.5,-1.5], dtype=np.float64)
DEFAULT_JOINT_POS_MJ = DEFAULT_JOINT_POS_IL[MJ2IL]

POLICY_FREQ   = 50       # Hz
PHYSICS_FREQ  = 200      # Hz (MuJoCo, standalone sim2sim과 동일)
DECIMATION    = PHYSICS_FREQ // POLICY_FREQ   # = 4
ACTION_SCALE  = 0.25
KP, KD        = 25.0, 0.6
SCALE_ANG_VEL   = 0.2
SCALE_JOINT_VEL = 0.05
ACTION_EMA_ALPHA = 0.3
HISTORY_LEN = 5

# Height scan (273-dim, 21×13 grid, 2.0×1.2m)
SCAN_NX, SCAN_NY = 21, 13
SCAN_SIZE_X, SCAN_SIZE_Y = 2.0, 1.2
SCAN_DIM = SCAN_NX * SCAN_NY
xs = np.linspace(-SCAN_SIZE_X/2, SCAN_SIZE_X/2, SCAN_NX)
ys = np.linspace(-SCAN_SIZE_Y/2, SCAN_SIZE_Y/2, SCAN_NY)
SCAN_OFFSETS = [(x, y) for x in xs for y in ys]
SCAN_RAY_DOWN = np.array([0.0, 0.0, -1.0])
SCAN_RAY_DIST = 3.0
SCAN_CLIP = (-1.0, 1.0)
HEIGHT_SCAN_OFFSET = 0.43

# Mid-360 LiDAR 시뮬레이션 파라미터
LIDAR_RINGS     = 16      # 스캔 라인 수 (단순화)
LIDAR_PTS_RING  = 360     # 라인당 점 수
LIDAR_EL_MIN    = math.radians(-17.5)   # 최저 앙각
LIDAR_EL_MAX    = math.radians(52.0)    # 최고 앙각
LIDAR_RANGE_MAX = 40.0    # 최대 거리 (m)
LIDAR_FREQ      = 10      # Hz


# ─────────────────────────────────────────────────────────────────────────────
# 유틸
# ─────────────────────────────────────────────────────────────────────────────
def quat_mj_to_ros(q_wxyz):
    """MuJoCo [w,x,y,z] → ROS [x,y,z,w]"""
    return q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]

def quat_to_rot_mat(q_wxyz):
    w,x,y,z = q_wxyz
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)  ],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)  ],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])

def projected_gravity(q_wxyz):
    R = quat_to_rot_mat(q_wxyz)
    g_world = np.array([0.0, 0.0, -1.0])
    return R.T @ g_world


# ─────────────────────────────────────────────────────────────────────────────
# ONNX 로드
# ─────────────────────────────────────────────────────────────────────────────
def load_policy(onnx_path: str, cpu: bool = False) -> ort.InferenceSession:
    providers = ["CPUExecutionProvider"] if cpu else ["CUDAExecutionProvider","CPUExecutionProvider"]
    sess = ort.InferenceSession(onnx_path, providers=providers)
    return sess


# ─────────────────────────────────────────────────────────────────────────────
# 관측 계산
# ─────────────────────────────────────────────────────────────────────────────
def compute_height_scan(model, data):
    robot_pos  = data.qpos[:3].copy()
    robot_quat = data.qpos[3:7].copy()
    # IsaacLab RayCasterCfg uses ray_alignment="yaw" for this scanner.
    yaw = math.atan2(
        2.0 * (robot_quat[0] * robot_quat[3] + robot_quat[1] * robot_quat[2]),
        1.0 - 2.0 * (robot_quat[2] * robot_quat[2] + robot_quat[3] * robot_quat[3]),
    )
    cy, sy = math.cos(yaw), math.sin(yaw)
    R_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    heights = np.zeros(SCAN_DIM, dtype=np.float32)
    for i, (dx, dy) in enumerate(SCAN_OFFSETS):
        world_offset = R_yaw @ np.array([dx, dy, 0.0])
        world_pos    = robot_pos + world_offset
        ray_origin   = np.array([world_pos[0], world_pos[1], robot_pos[2] + 2.0])
        geom_id = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(model, data, ray_origin, SCAN_RAY_DOWN,
                             None, 1, -1, geom_id)
        if 0 <= dist < SCAN_RAY_DIST:
            hit_z = ray_origin[2] - dist
            # IsaacLab mdp.height_scan shape, adjusted for MuJoCo/Isaac base-height offset.
            rel_h = robot_pos[2] - hit_z - HEIGHT_SCAN_OFFSET
        else:
            rel_h = -1.0
        heights[i] = np.clip(rel_h, *SCAN_CLIP)
    return heights


def get_joint_states(data):
    pos = np.array([data.sensor(f"{n}_pos").data[0] for n in MJ_JOINT_NAMES])
    vel = np.array([data.sensor(f"{n}_vel").data[0] for n in MJ_JOINT_NAMES])
    return pos, vel


def compute_obs_45(data, last_action, cmd):
    imu_quat = data.sensor("imu_quat").data.copy()
    imu_gyro = data.sensor("imu_gyro").data.copy()
    pos_mj, vel_mj = get_joint_states(data)
    pos_il = pos_mj[IL2MJ]
    vel_il = vel_mj[IL2MJ]
    return np.concatenate([
        imu_gyro * SCALE_ANG_VEL,
        projected_gravity(imu_quat),
        cmd.astype(np.float64),
        pos_il - DEFAULT_JOINT_POS_IL,
        vel_il * SCALE_JOINT_VEL,
        last_action,
    ]).astype(np.float32)


def compute_obs(model, data, last_action, cmd, obs_dim, proprio_history, external_height_scan=None):
    """Compute policy observation for 45/225/318/498-dim Go2 policies."""
    obs45 = compute_obs_45(data, last_action, cmd)

    if obs_dim == 45:
        return obs45

    if obs_dim == 225:
        proprio_history.append(obs45.copy())
        return np.concatenate(list(proprio_history)).astype(np.float32)

    if obs_dim == 318:
        scan = external_height_scan if external_height_scan is not None else compute_height_scan(model, data)
        return np.concatenate([obs45, scan]).astype(np.float32)

    if obs_dim == 498:
        proprio_history.append(obs45.copy())
        hist = np.concatenate(list(proprio_history))
        scan = external_height_scan if external_height_scan is not None else compute_height_scan(model, data)
        return np.concatenate([hist, scan]).astype(np.float32)

    # Conservative fallback for old blind policies.
    return obs45


def apply_pd_action(model, data, action_il):
    pos_mj, vel_mj = get_joint_states(data)
    target_il = action_il * ACTION_SCALE + DEFAULT_JOINT_POS_IL
    target_mj = target_il[MJ2IL]
    torque = KP * (target_mj - pos_mj) + KD * (0.0 - vel_mj)
    data.ctrl[:] = np.clip(torque, -45.0, 45.0)


def reset_robot(model, data):
    mujoco.mj_resetData(model, data)
    for kf in ("start","stand","home"):
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, kf)
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
            break
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)
    kf_ctrl = data.ctrl.copy()
    if np.allclose(kf_ctrl, 0.0):
        kf_ctrl = data.qpos[7:19].copy()
    data.ctrl[:] = kf_ctrl
    for _ in range(500):
        mujoco.mj_step(model, data)


# ─────────────────────────────────────────────────────────────────────────────
# Mid-360 LiDAR 시뮬레이션 (PointCloud2)
# ─────────────────────────────────────────────────────────────────────────────
def simulate_lidar(model, data) -> np.ndarray:
    """
    MuJoCo raycasting으로 Mid-360 LiDAR 시뮬레이션.
    반환: (N, 3) xyz points in LiDAR frame
    """
    robot_pos  = data.qpos[:3].copy()
    robot_quat = data.qpos[3:7].copy()
    R = quat_to_rot_mat(robot_quat)

    el_angles  = np.linspace(LIDAR_EL_MIN, LIDAR_EL_MAX, LIDAR_RINGS)
    az_angles  = np.linspace(0, 2*math.pi, LIDAR_PTS_RING, endpoint=False)
    points = []

    for el in el_angles:
        cos_el = math.cos(el)
        sin_el = math.sin(el)
        for az in az_angles:
            # 방향 벡터 (로봇 로컬 프레임)
            dx_l = cos_el * math.cos(az)
            dy_l = cos_el * math.sin(az)
            dz_l = sin_el
            # 월드 프레임으로 변환
            d_local = np.array([dx_l, dy_l, dz_l])
            d_world = R @ d_local
            lidar_offset = np.array([0.0, 0.0, 0.15])
            ray_orig = robot_pos + R @ lidar_offset

            geom_id = np.zeros(1, dtype=np.int32)
            dist = mujoco.mj_ray(model, data, ray_orig, d_world,
                                 None, 1, -1, geom_id)
            if 0 < dist < LIDAR_RANGE_MAX:
                hit_world = ray_orig + dist * d_world
                # FAST-LIO 입력은 LiDAR frame 기준이어야 한다.
                hit_local = R.T @ (hit_world - ray_orig)
                points.append(hit_local.astype(np.float32))

    return np.array(points, dtype=np.float32) if points else np.zeros((0, 3), dtype=np.float32)


def make_pointcloud2_msg(points_xyz: np.ndarray, frame_id: str, stamp) -> "PointCloud2":
    """numpy (N,3) → sensor_msgs/PointCloud2 with x/y/z/intensity fields."""
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width  = len(points_xyz)
    msg.is_dense   = False
    msg.is_bigendian = False
    msg.point_step = 16   # 4 × float32
    msg.row_step   = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points_xyzi = np.zeros((len(points_xyz), 4), dtype=np.float32)
    if len(points_xyz) > 0:
        points_xyzi[:, :3] = points_xyz.astype(np.float32, copy=False)
        points_xyzi[:, 3] = 1.0
    msg.data = points_xyzi.tobytes()
    return msg


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 노드
# ─────────────────────────────────────────────────────────────────────────────
class Go2RLNode(Node):
    """MuJoCo sim2sim ↔ ROS2 브릿지 노드."""

    def __init__(
        self,
        enable_lidar: bool = False,
        height_scan_topic: str = "/rl/height_scan",
        publish_height_scan: bool = False,
        publish_fastlio_odom: bool = True,
    ):
        super().__init__("go2_rl_controller")
        self.enable_lidar = enable_lidar
        self.height_scan_topic = height_scan_topic
        self.publish_height_scan_enabled = publish_height_scan
        self.publish_fastlio_odom_enabled = publish_fastlio_odom

        # 공유 상태 (thread-safe: GIL로 충분)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.height_scan = None
        self.height_scan_stamp = 0.0
        self.reset_requested = False

        # Subscriber
        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel", self._cb_cmd_vel, 10)
        self.sub_reset = self.create_subscription(
            Empty, "/go2/reset", self._cb_reset, 10)
        self.sub_height_scan = self.create_subscription(
            Float32MultiArray, self.height_scan_topic, self._cb_height_scan, 10)

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        if self.publish_fastlio_odom_enabled:
            self.pub_fastlio_odom = self.create_publisher(Odometry, "/Odometry", 10)
        self.pub_imu  = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_utlidar_imu = self.create_publisher(Imu, "/utlidar/imu", 10)
        if self.enable_lidar:
            self.pub_lidar = self.create_publisher(
                PointCloud2, "/livox/lidar", 10)
            self.pub_utlidar = self.create_publisher(
                PointCloud2, "/utlidar/cloud", 10)
        if self.publish_height_scan_enabled:
            self.pub_height_scan = self.create_publisher(
                Float32MultiArray, self.height_scan_topic, 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            f"Go2 RL Controller 노드 시작  "
            f"(LiDAR={'ON' if enable_lidar else 'OFF'}, "
            f"height_scan_topic={height_scan_topic}, "
            f"publish_height_scan={'ON' if publish_height_scan else 'OFF'}, "
            f"publish_fastlio_odom={'ON' if publish_fastlio_odom else 'OFF'})")

    def _cb_cmd_vel(self, msg: Twist):
        self.cmd_vx = float(msg.linear.x)
        self.cmd_vy = float(msg.linear.y)
        self.cmd_wz = float(msg.angular.z)

    def _cb_reset(self, _msg: Empty):
        self.reset_requested = True
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.get_logger().info("MuJoCo reset requested via /go2/reset")

    def _cb_height_scan(self, msg: Float32MultiArray):
        if len(msg.data) != SCAN_DIM:
            self.get_logger().warn(
                f"height_scan 길이 오류: expected={SCAN_DIM}, got={len(msg.data)}",
                throttle_duration_sec=2.0,
            )
            return
        self.height_scan = np.asarray(msg.data, dtype=np.float32).clip(*SCAN_CLIP)
        self.height_scan_stamp = time.monotonic()

    @property
    def cmd(self) -> np.ndarray:
        return np.array([self.cmd_vx, self.cmd_vy, self.cmd_wz], dtype=np.float32)

    def get_height_scan(self, timeout_s: float) -> np.ndarray | None:
        if self.height_scan is None:
            return None
        if time.monotonic() - self.height_scan_stamp > timeout_s:
            return None
        return self.height_scan.copy()

    def publish_odom(self, data: mujoco.MjData):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_mj_to_ros(data.qpos[3:7])
        vel = data.qvel  # [vx, vy, vz, wx, wy, wz] in world frame

        msg = Odometry()
        msg.header.stamp    = now
        msg.header.frame_id = "odom"
        msg.child_frame_id  = "base_link"
        p = msg.pose.pose
        p.position.x, p.position.y, p.position.z = data.qpos[:3]
        p.orientation.x = qx; p.orientation.y = qy
        p.orientation.z = qz; p.orientation.w = qw
        t = msg.twist.twist
        t.linear.x  = float(vel[0])
        t.linear.y  = float(vel[1])
        t.linear.z  = float(vel[2])
        t.angular.x = float(vel[3])
        t.angular.y = float(vel[4])
        t.angular.z = float(vel[5])
        self.pub_odom.publish(msg)
        if self.publish_fastlio_odom_enabled:
            self.pub_fastlio_odom.publish(msg)

        # TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = "odom"
        tf.child_frame_id  = "base_link"
        tf.transform.translation.x = data.qpos[0]
        tf.transform.translation.y = data.qpos[1]
        tf.transform.translation.z = data.qpos[2]
        tf.transform.rotation.x = qx; tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz; tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def publish_imu(self, data: mujoco.MjData):
        now  = self.get_clock().now().to_msg()
        gyro = data.sensor("imu_gyro").data
        acc  = data.sensor("imu_acc").data if "imu_acc" in [
            data.model.sensor(i).name for i in range(data.model.nsensor)
        ] else np.zeros(3)
        qx, qy, qz, qw = quat_mj_to_ros(data.sensor("imu_quat").data)

        msg = Imu()
        msg.header.stamp    = now
        msg.header.frame_id = "imu_link"
        msg.orientation.x = qx; msg.orientation.y = qy
        msg.orientation.z = qz; msg.orientation.w = qw
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        self.pub_imu.publish(msg)
        self.pub_utlidar_imu.publish(msg)

    def publish_lidar(self, model: mujoco.MjModel, data: mujoco.MjData):
        if not self.enable_lidar:
            return
        pts = simulate_lidar(model, data)
        if len(pts) > 0:
            now = self.get_clock().now().to_msg()
            msg = make_pointcloud2_msg(pts, "lidar_link", now)
            self.pub_lidar.publish(msg)
            self.pub_utlidar.publish(msg)

    def publish_height_scan(self, scan: np.ndarray):
        if not self.publish_height_scan_enabled:
            return
        msg = Float32MultiArray()
        msg.data = scan.astype(np.float32).clip(*SCAN_CLIP).tolist()
        self.pub_height_scan.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="MuJoCo sim2sim + ROS2 bridge")
    parser.add_argument("--onnx",       required=True,  help="ONNX 정책 파일")
    parser.add_argument("--map",        default="flat",  help="맵 이름 또는 XML 경로")
    parser.add_argument("--lidar",      action="store_true",
                        help="LiDAR PointCloud2 publish (FAST-LIO2 용, 느림)")
    parser.add_argument("--domain-id",  type=int, default=0,
                        help="ROS_DOMAIN_ID (SLAM팀과 맞출 것)")
    parser.add_argument("--height-scan-topic", default="/rl/height_scan",
                        help="External V5 height_scan topic.")
    parser.add_argument("--height-scan-timeout", type=float, default=0.25,
                        help="External height_scan stale timeout in seconds.")
    parser.add_argument("--require-height-scan", action="store_true",
                        help="For 318/498-dim policies, stop actions until external height_scan is fresh.")
    parser.add_argument("--ignore-external-height-scan", action="store_true",
                        help="Use MuJoCo raycast height_scan for policy input even if /rl/height_scan is available.")
    parser.add_argument("--publish-height-scan", action="store_true",
                        help="Publish MuJoCo raycast height_scan to --height-scan-topic for integration tests.")
    parser.add_argument("--no-fastlio-odom", action="store_true",
                        help="Do not publish ground-truth /Odometry; leave it for FAST-LIO.")
    parser.add_argument("--no-ros",     action="store_true",
                        help="ROS2 없이 실행 (단독 시뮬레이션)")
    parser.add_argument("--cpu",        action="store_true",
                        help="ONNX Runtime CPU provider만 사용 (학습 중 smoke test용)")
    parser.add_argument("--headless-steps", type=int, default=0,
                        help="viewer 없이 지정한 policy step 수만큼 ROS2 smoke test 후 종료")
    parser.add_argument("--debug-motion", action="store_true",
                        help="Print cmd, action norm, and base pose once per second.")
    parser.add_argument("--fixed-cmd", type=float, nargs=3, metavar=("VX", "VY", "WZ"),
                        default=None,
                        help="Override /cmd_vel with a fixed command for policy smoke tests.")
    parser.add_argument("--fixed-height-scan", type=float, default=None,
                        help="Use a constant height_scan value for 318/498-dim policy smoke tests.")
    args = parser.parse_args()

    # ── 씬 경로 해석 ──────────────────────────────────────────────────────────
    scene_path = MAP_ALIASES.get(args.map, args.map)
    if not Path(scene_path).exists():
        scene_path = str(GO2_DIR / "scene.xml")
        print(f"[WARN] 씬 파일 없음, 기본 사용: {scene_path}")
    print(f"[INFO] 씬: {scene_path}")

    # ── ONNX 로드 ──────────────────────────────────────────────────────────────
    session  = load_policy(args.onnx, cpu=args.cpu)
    obs_dim  = session.get_inputs()[0].shape[1]
    in_name  = session.get_inputs()[0].name
    out_name = session.get_outputs()[0].name
    if obs_dim == 498:
        policy_type = "V5/V3 history+scan"
    elif obs_dim == 225:
        policy_type = "V4 history blind"
    elif obs_dim == 318:
        policy_type = "scan"
    elif obs_dim == 45:
        policy_type = "blind"
    else:
        policy_type = "unknown, fallback=45"
    print(f"[INFO] ONNX obs_dim={obs_dim}  ({policy_type})")

    # ── MuJoCo 로드 ───────────────────────────────────────────────────────────
    model = mujoco.MjModel.from_xml_path(scene_path)
    data  = mujoco.MjData(model)
    model.opt.timestep = 1.0 / PHYSICS_FREQ
    reset_robot(model, data)

    # ── ROS2 초기화 ───────────────────────────────────────────────────────────
    use_ros = HAS_ROS2 and not args.no_ros
    ros_node = None
    ros_thread = None
    executor = None

    if use_ros:
        import os
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
        rclpy.init()
        ros_node = Go2RLNode(
            enable_lidar=args.lidar,
            height_scan_topic=args.height_scan_topic,
            publish_height_scan=args.publish_height_scan,
            publish_fastlio_odom=not args.no_fastlio_odom,
        )
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(ros_node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        print(f"[INFO] ROS2 노드 시작 (domain={args.domain_id})")
        print("[INFO] /cmd_vel, /go2/reset 구독 중 | /odom, /imu/data 발행 중")
        fastlio_odom_status = "/Odometry 발행 중" if not args.no_fastlio_odom else "/Odometry 발행 안 함(FAST-LIO 전용)"
        print(f"[INFO] {args.height_scan_topic} 구독 중 | {fastlio_odom_status} | /utlidar/imu 발행 중")
        if args.lidar:
            print("[INFO] /livox/lidar, /utlidar/cloud 발행 중 (FAST-LIO2 연동)")
        if args.publish_height_scan:
            print(f"[INFO] MuJoCo height_scan → {args.height_scan_topic} 발행 중")
    else:
        print("[INFO] ROS2 없이 실행 — 키보드 전용")

    # ── 키보드 폴백 (ROS2 없을 때 또는 수동 제어) ─────────────────────────────
    kb_vx, kb_vy, kb_wz = [0.0], [0.0], [0.0]

    # ── 시뮬레이션 루프 ───────────────────────────────────────────────────────
    last_action = np.zeros(12, dtype=np.float32)
    smooth_action = np.zeros(12, dtype=np.float32)
    proprio_history = deque(
        [np.zeros(45, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN
    )
    lidar_counter = 0
    LIDAR_EVERY = POLICY_FREQ // LIDAR_FREQ  # 5 policy steps마다 LiDAR
    policy_step_count = 0

    print("\n" + "="*60)
    print("  Go2 RL + ROS2 브릿지 시뮬레이션")
    print(f"  맵: {args.map}  |  obs: {obs_dim}-dim")
    print("  MuJoCo 뷰어에서 Space: 리셋")
    if use_ros:
        print("  /cmd_vel 토픽으로 속도 명령 가능")
        print("  /go2/reset 토픽으로 MuJoCo 상태 리셋 가능")
        print(f"  ROS_DOMAIN_ID={args.domain_id}")
    print("="*60 + "\n")

    def do_reset(source: str) -> None:
        nonlocal last_action, smooth_action
        reset_robot(model, data)
        last_action[:] = 0.0
        smooth_action[:] = 0.0
        proprio_history.clear()
        proprio_history.extend(
            [np.zeros(45, dtype=np.float32)] * HISTORY_LEN
        )
        if ros_node is not None:
            ros_node.reset_requested = False
            ros_node.cmd_vx = 0.0
            ros_node.cmd_vy = 0.0
            ros_node.cmd_wz = 0.0
        print(f"[INFO] MuJoCo reset ({source})", flush=True)

    def run_policy_step() -> None:
        nonlocal last_action, smooth_action, lidar_counter, policy_step_count

        # ── 속도 명령 결정 ──────────────────────────────────────────────
        if use_ros and ros_node is not None:
            cmd = ros_node.cmd
        else:
            cmd = np.array([kb_vx[0], kb_vy[0], kb_wz[0]], dtype=np.float32)
        if args.fixed_cmd is not None:
            cmd = np.array(args.fixed_cmd, dtype=np.float32)

        # ── RL 추론 ─────────────────────────────────────────────────────
        needs_scan = obs_dim in (318, 498)
        external_scan = None
        if use_ros and ros_node is not None and needs_scan and not args.ignore_external_height_scan:
            external_scan = ros_node.get_height_scan(args.height_scan_timeout)
        fixed_scan = None
        if args.fixed_height_scan is not None and needs_scan:
            fixed_scan = np.full(
                SCAN_DIM,
                float(np.clip(args.fixed_height_scan, *SCAN_CLIP)),
                dtype=np.float32,
            )
        if args.require_height_scan and needs_scan and external_scan is None:
            action_raw = np.zeros(12, dtype=np.float32)
            smooth_action[:] = 0.0
        else:
            obs = compute_obs(
                model,
                data,
                last_action,
                cmd,
                obs_dim,
                proprio_history,
                external_height_scan=fixed_scan if fixed_scan is not None else external_scan,
            )
            action_raw = session.run([out_name], {in_name: obs[None]})[0][0]
            action_raw = np.clip(action_raw, -1.0, 1.0)
            smooth_action = (
                ACTION_EMA_ALPHA * smooth_action
                + (1.0 - ACTION_EMA_ALPHA) * action_raw
            )

        # ── MuJoCo 물리 스텝 (decimation) ──────────────────────────────
        apply_pd_action(model, data, smooth_action)
        for _ in range(DECIMATION):
            mujoco.mj_step(model, data)
        last_action = action_raw.copy()

        # ── ROS2 publish ────────────────────────────────────────────────
        if use_ros and ros_node is not None:
            ros_node.publish_odom(data)
            ros_node.publish_imu(data)
            if args.publish_height_scan and needs_scan:
                ros_node.publish_height_scan(compute_height_scan(model, data))
            if args.lidar:
                lidar_counter += 1
                if lidar_counter >= LIDAR_EVERY:
                    ros_node.publish_lidar(model, data)
                    lidar_counter = 0

        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError("MuJoCo state became non-finite")

        policy_step_count += 1
        if args.debug_motion and policy_step_count % POLICY_FREQ == 0:
            action_norm = float(np.linalg.norm(smooth_action))
            print(
                "[DEBUG] "
                f"cmd=({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}) "
                f"action_norm={action_norm:.3f} "
                f"base=({data.qpos[0]:+.2f},{data.qpos[1]:+.2f},{data.qpos[2]:+.2f}) "
                f"vel=({data.qvel[0]:+.2f},{data.qvel[1]:+.2f},{data.qvel[2]:+.2f})",
                flush=True,
            )

    if args.headless_steps > 0:
        z_min = float(data.qpos[2])
        for _ in range(args.headless_steps):
            t0 = time.perf_counter()
            run_policy_step()
            z_min = min(z_min, float(data.qpos[2]))
            sleep_t = 1.0 / POLICY_FREQ - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)
        print(
            "[INFO] ROS2 headless smoke OK: "
            f"steps={args.headless_steps}, z_final={data.qpos[2]:.3f}, z_min={z_min:.3f}"
        )
    else:
        reset_requested = False

        def key_callback(key: int):
            nonlocal reset_requested
            try:
                import glfw
            except ImportError:
                return
            if key == glfw.KEY_SPACE:
                reset_requested = True

        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = 1 if model.nbody > 1 else 0
            viewer.cam.distance = 3.5
            viewer.cam.elevation = -25
            viewer.cam.azimuth = 160

            step_count = 0
            while viewer.is_running():
                t0 = time.perf_counter()
                if reset_requested or (ros_node is not None and ros_node.reset_requested):
                    reset_requested = False
                    do_reset("viewer/ros")
                run_policy_step()
                viewer.sync()
                step_count += 1

                # 50Hz 유지
                elapsed = time.perf_counter() - t0
                sleep_t = 1.0/POLICY_FREQ - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

    # ── 종료 ──────────────────────────────────────────────────────────────────
    if use_ros:
        if executor is not None:
            executor.shutdown()
        if ros_node is not None:
            ros_node.destroy_node()
        rclpy.shutdown()
        if ros_thread is not None:
            ros_thread.join(timeout=1.0)
    print("[INFO] 시뮬레이션 종료")


if __name__ == "__main__":
    main()
