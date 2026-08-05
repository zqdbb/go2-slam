#!/usr/bin/env python3
"""ROS 2 bridge for a MuJoCo Go2 scene.

The bridge publishes the minimum sensor/state contract needed by this workspace:
FAST-LIO inputs (`/points_raw`, `/imu`), RL inputs (`/joint_states`, `/imu/data`),
odometry (`/Odometry`), `/tf`, and `/clock`. It also accepts RL joint trajectory
commands and optional `/cmd_vel` kinematic fallback for early pipeline testing.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory


def _quat_wxyz_to_xyzw(q):
    return float(q[1]), float(q[2]), float(q[3]), float(q[0])


def _quat_to_yaw_wxyz(q) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_to_quat_wxyz(yaw: float):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)], dtype=np.float64)


class MujocoGo2Bridge(Node):
    def __init__(self):
        super().__init__("mujoco_go2_bridge")
        self.declare_parameter(
            "model_path",
            "/workspace/Unitree_Go2/simulation/mujoco/go2/scene_icra2024_sloped.xml",
        )
        self.declare_parameter("base_body", "base")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("lidar_rate_hz", 10.0)
        self.declare_parameter("lidar_range_m", 8.0)
        self.declare_parameter("lidar_horizontal_samples", 180)
        self.declare_parameter("lidar_vertical_samples", 16)
        self.declare_parameter("lidar_vertical_fov_deg", 30.0)
        self.declare_parameter("apply_cmd_vel_to_freejoint", True)
        self.declare_parameter("publish_champ_joint_names", True)
        self.declare_parameter("enable_viewer", False)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("joint_command_topic", "/joint_group_effort_controller/joint_trajectory")
        self.declare_parameter("use_pd_control", False)
        self.declare_parameter("pd_kp", 40.0)
        self.declare_parameter("pd_kp_hip", 20.0)
        self.declare_parameter("pd_kd", 1.0)

        self.model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        self.base_body = str(self.get_parameter("base_body").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        self.apply_cmd_vel = bool(self.get_parameter("apply_cmd_vel_to_freejoint").value)
        self.publish_champ_names = bool(self.get_parameter("publish_champ_joint_names").value)
        self.enable_viewer = bool(self.get_parameter("enable_viewer").value)
        self.cmd = Twist()
        self.joint_targets: dict[str, float] = {}
        self._last_lidar_t = -1.0

        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("Python package 'mujoco' is required. Install it in the active ROS Python environment.") from exc
        self.mujoco = mujoco
        if not self.model_path.exists():
            raise FileNotFoundError(f"MuJoCo model does not exist: {self.model_path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.viewer = None
        if self.enable_viewer:
            try:
                import mujoco.viewer

                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.get_logger().info("MuJoCo passive viewer enabled")
            except Exception as exc:
                self.get_logger().error(f"Failed to start MuJoCo viewer: {exc}")

        self.base_body_id = self._body_id_or_warn(self.base_body)
        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt) if self.model.joint(i).name]
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        self.ros_to_mujoco_joint = {self._to_champ_joint_name(name): name for name in self.joint_names}

        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu_fastlio = self.create_publisher(Imu, "/imu", 10)
        self.pub_imu_rl = self.create_publisher(Imu, "/imu/data", 10)
        self.pub_odom = self.create_publisher(Odometry, "/Odometry", 10)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/points_raw", 2)
        self.pub_cloud_rl = self.create_publisher(PointCloud2, "/velodyne_points", 2)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(JointTrajectory, str(self.get_parameter("joint_command_topic").value), self._joint_cmd_cb, 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._step)
        self.get_logger().info(f"Loaded MuJoCo model {self.model_path}")

    def _body_id_or_warn(self, name: str) -> int:
        body_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            self.get_logger().warn(f"Body {name!r} not found; using world-relative qpos fallback")
        return body_id

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd = msg

    def _joint_cmd_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        point = msg.points[-1]
        for name, pos in zip(msg.joint_names, point.positions):
            mujoco_name = self.ros_to_mujoco_joint.get(name, name)
            self.joint_targets[mujoco_name] = float(pos)

    def _step(self) -> None:
        self._apply_controls()
        self.mujoco.mj_step(self.model, self.data)
        stamp = self._stamp()
        self._publish_clock(stamp)
        self._publish_joint_state(stamp)
        imu = self._make_imu(stamp)
        self.pub_imu_fastlio.publish(imu)
        self.pub_imu_rl.publish(imu)
        self._publish_odom_tf(stamp)
        if self.viewer is not None:
            self.viewer.sync()
        lidar_period = 1.0 / float(self.get_parameter("lidar_rate_hz").value)
        if self.data.time - self._last_lidar_t >= lidar_period:
            cloud = self._make_lidar_cloud(stamp)
            self.pub_cloud.publish(cloud)
            self.pub_cloud_rl.publish(cloud)
            self._last_lidar_t = self.data.time

    def _apply_controls(self) -> None:
        use_pd = bool(self.get_parameter("use_pd_control").value)
        kp     = float(self.get_parameter("pd_kp").value)
        kp_hip = float(self.get_parameter("pd_kp_hip").value)
        kd     = float(self.get_parameter("pd_kd").value)

        for actuator_idx, actuator_name in enumerate(self.actuator_names):
            joint_like = actuator_name.replace("_motor", "").replace("_actuator", "")
            joint_name = f"{joint_like}_joint"
            target = None
            for key in (actuator_name, joint_like, joint_name):
                if key in self.joint_targets:
                    target = self.joint_targets[key]
                    break
            if target is None:
                continue
            if use_pd:
                # Find this joint's qpos/qvel indices for PD torque computation
                j_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if j_id < 0:
                    j_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, joint_like)
                if j_id >= 0:
                    qadr = int(self.model.jnt_qposadr[j_id])
                    dadr = int(self.model.jnt_dofadr[j_id])
                    gain = kp_hip if "hip" in joint_name else kp
                    tau = gain * (target - float(self.data.qpos[qadr])) - kd * float(self.data.qvel[dadr])
                    self.data.ctrl[actuator_idx] = tau
            else:
                self.data.ctrl[actuator_idx] = target

        if self.apply_cmd_vel and self.model.nq >= 7:
            q = self.data.qpos[3:7].copy()
            yaw = _quat_to_yaw_wxyz(q)
            dt = self.model.opt.timestep
            vx = self.cmd.linear.x * math.cos(yaw) - self.cmd.linear.y * math.sin(yaw)
            vy = self.cmd.linear.x * math.sin(yaw) + self.cmd.linear.y * math.cos(yaw)
            self.data.qpos[0] += vx * dt
            self.data.qpos[1] += vy * dt
            self.data.qpos[3:7] = _yaw_to_quat_wxyz(yaw + self.cmd.angular.z * dt)

    def _stamp(self) -> Time:
        sec = int(self.data.time)
        nsec = int((self.data.time - sec) * 1e9)
        return Time(sec=sec, nanosec=nsec)

    def _publish_clock(self, stamp: Time) -> None:
        self.pub_clock.publish(Clock(clock=stamp))

    def _publish_joint_state(self, stamp: Time) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = []
        msg.position = []
        msg.velocity = []
        for i in range(self.model.njnt):
            joint = self.model.joint(i)
            if not joint.name or int(self.model.jnt_type[i]) == self.mujoco.mjtJoint.mjJNT_FREE:
                continue
            qadr = int(self.model.jnt_qposadr[i])
            dadr = int(self.model.jnt_dofadr[i])
            msg.name.append(self._to_champ_joint_name(joint.name) if self.publish_champ_names else joint.name)
            msg.position.append(float(self.data.qpos[qadr]))
            msg.velocity.append(float(self.data.qvel[dadr]))
        self.pub_joint.publish(msg)

    @staticmethod
    def _to_champ_joint_name(name: str) -> str:
        mapping = {
            "FL_hip_joint": "lf_hip_joint",
            "FL_thigh_joint": "lf_upper_leg_joint",
            "FL_calf_joint": "lf_lower_leg_joint",
            "FR_hip_joint": "rf_hip_joint",
            "FR_thigh_joint": "rf_upper_leg_joint",
            "FR_calf_joint": "rf_lower_leg_joint",
            "RL_hip_joint": "lh_hip_joint",
            "RL_thigh_joint": "lh_upper_leg_joint",
            "RL_calf_joint": "lh_lower_leg_joint",
            "RR_hip_joint": "rh_hip_joint",
            "RR_thigh_joint": "rh_upper_leg_joint",
            "RR_calf_joint": "rh_lower_leg_joint",
        }
        return mapping.get(name, name)

    def _make_imu(self, stamp: Time) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame
        pos, quat = self._base_pose()
        x, y, z, w = _quat_wxyz_to_xyzw(quat)
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
        if self.model.nv >= 6:
            msg.angular_velocity.x = float(self.data.qvel[3])
            msg.angular_velocity.y = float(self.data.qvel[4])
            msg.angular_velocity.z = float(self.data.qvel[5])
        msg.linear_acceleration.z = 9.81
        return msg

    def _publish_odom_tf(self, stamp: Time) -> None:
        if not bool(self.get_parameter("publish_odom_tf").value):
            return
        pos, quat = self._base_pose()
        x, y, z, w = _quat_wxyz_to_xyzw(quat)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        if self.model.nv >= 3:
            odom.twist.twist.linear.x = float(self.data.qvel[0])
            odom.twist.twist.linear.y = float(self.data.qvel[1])
            odom.twist.twist.linear.z = float(self.data.qvel[2])
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.pub_tf.publish(TFMessage(transforms=[tf]))

    def _base_pose(self):
        if self.base_body_id >= 0:
            return self.data.xpos[self.base_body_id], self.data.xquat[self.base_body_id]
        if self.model.nq >= 7:
            return self.data.qpos[0:3], self.data.qpos[3:7]
        return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])

    def _make_lidar_cloud(self, stamp: Time) -> PointCloud2:
        max_range = float(self.get_parameter("lidar_range_m").value)
        n_h = int(self.get_parameter("lidar_horizontal_samples").value)
        n_v = int(self.get_parameter("lidar_vertical_samples").value)
        vfov = math.radians(float(self.get_parameter("lidar_vertical_fov_deg").value))
        pos, quat = self._base_pose()
        origin = np.asarray(pos, dtype=np.float64) + np.array([0.18, 0.0, 0.18])
        points = []
        scan_period = 1.0 / float(self.get_parameter("lidar_rate_hz").value)
        for ring, v in enumerate(np.linspace(-vfov / 2.0, vfov / 2.0, n_v)):
            cv, sv = math.cos(v), math.sin(v)
            for h_idx, h in enumerate(np.linspace(-math.pi, math.pi, n_h, endpoint=False)):
                direction = np.array([cv * math.cos(h), cv * math.sin(h), sv], dtype=np.float64)
                geom_id = np.array([-1], dtype=np.int32)
                dist = self.mujoco.mj_ray(self.model, self.data, origin, direction, None, 1, -1, geom_id)
                if 0.05 < dist < max_range:
                    p = origin + direction * dist
                    intensity = max(0.0, 1.0 - float(dist) / max_range)
                    rel_time = scan_period * (ring * n_h + h_idx) / max(1, n_h * n_v - 1)
                    points.append((float(p[0]), float(p[1]), float(p[2]), intensity, int(ring), float(rel_time)))
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(Header(stamp=stamp, frame_id=self.lidar_frame), fields, points)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoGo2Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
