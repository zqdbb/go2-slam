#!/usr/bin/env python3
"""ROS 2 MuJoCo Go2 bridge — step-embedded lidar, docker exec backend."""
from __future__ import annotations
import json, math, os, queue, subprocess, threading
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

def _quat_wxyz_to_xyzw(q):
    return float(q[1]), float(q[2]), float(q[3]), float(q[0])


class MujocoGo2Bridge(Node):
    def __init__(self):
        super().__init__("mujoco_go2_bridge")
        self.declare_parameter("model_path",        "/tmp/go2_mujoco/scene_icra2024_flat.xml")
        self.declare_parameter("server_script",     "/workspace/mujoco_server.py")
        self.declare_parameter("policy_path",       "/workspace/model_40000.pt")
        self.declare_parameter("mujoco_container",  "mujoco-huanghb-pi05-teleop")
        self.declare_parameter("mujoco_python",     "/lerobot/.venv/bin/python3")
        self.declare_parameter("mujoco_viewer",     True)
        self.declare_parameter("mujoco_viewer_display", os.environ.get("DISPLAY", ""))
        self.declare_parameter("mujoco_viewer_xauthority", os.environ.get("XAUTHORITY", ""))
        self.declare_parameter("lidar_frame",       "lidar_link")
        self.declare_parameter("base_frame",        "base_link")
        self.declare_parameter("odom_frame",        "odom")
        self.declare_parameter("publish_rate_hz",           50.0)
        self.declare_parameter("cmd_vel_timeout_s",          0.3)
        self.declare_parameter("lidar_rate_hz",             10.0)
        self.declare_parameter("lidar_range_m",             10.0)
        self.declare_parameter("lidar_horizontal_samples",  180)
        self.declare_parameter("lidar_vertical_samples",    16)
        self.declare_parameter("lidar_vertical_fov_deg",    30.0)

        self.base_frame  = str(self.get_parameter("base_frame").value)
        self.odom_frame  = str(self.get_parameter("odom_frame").value)
        self.lidar_frame = str(self.get_parameter("lidar_frame").value)
        model_path    = str(self.get_parameter("model_path").value)
        server_script = str(self.get_parameter("server_script").value)
        policy_path   = str(self.get_parameter("policy_path").value)
        container     = str(self.get_parameter("mujoco_container").value)
        py_exec       = str(self.get_parameter("mujoco_python").value)
        self.cmd = Twist()
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_s").value)
        self._last_cmd_time = self.get_clock().now()
        self._consecutive_timeouts = 0

        os.system(f"docker cp {server_script} {container}:/tmp/mujoco_server.py 2>/dev/null")
        subprocess.run(
            ["docker", "cp", policy_path, f"{container}:/tmp/model_40000.pt"],
            check=True,
        )

        viewer_enabled = bool(self.get_parameter("mujoco_viewer").value)
        server_args = ["docker", "exec", "-i"]
        if viewer_enabled:
            viewer_display = str(self.get_parameter("mujoco_viewer_display").value)
            viewer_xauth = str(self.get_parameter("mujoco_viewer_xauthority").value)
            container_xauth = "/tmp/go2_mapping.Xauthority"
            if viewer_xauth:
                subprocess.run(
                    ["docker", "cp", viewer_xauth, f"{container}:{container_xauth}"],
                    check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
                subprocess.run(
                    ["docker", "exec", "-u", "0", container, "chmod", "644", container_xauth],
                    check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            if viewer_display:
                server_args.extend(["-e", f"DISPLAY={viewer_display}"])
            if viewer_xauth:
                server_args.extend(["-e", f"XAUTHORITY={container_xauth}"])
        server_args.extend([
            container, py_exec, "/tmp/mujoco_server.py", model_path,
            "/tmp/model_40000.pt",
        ])
        if viewer_enabled:
            server_args.append("--viewer")
        self._proc = subprocess.Popen(
            server_args,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )
        ready = self._proc.stdout.readline()
        self.get_logger().info(f"Mujoco server: {ready.strip()}")

        # send lidar config
        lidar_cfg = {
            "action": "config_lidar",
            "n_h":   int(self.get_parameter("lidar_horizontal_samples").value),
            "n_v":   int(self.get_parameter("lidar_vertical_samples").value),
            "vfov":  float(self.get_parameter("lidar_vertical_fov_deg").value),
            "max_range": float(self.get_parameter("lidar_range_m").value),
            "rate":  float(self.get_parameter("lidar_rate_hz").value),
        }
        self._proc.stdin.write(json.dumps(lidar_cfg) + "\n")
        self._proc.stdin.flush()
        self._proc.stdout.readline()  # consume config ack

        self._resp_q = queue.Queue()
        threading.Thread(target=self._read_loop, daemon=True).start()

        self.pub_clock = self.create_publisher(Clock,       "/clock",        10)
        self.pub_joint = self.create_publisher(JointState,  "/joint_states", 10)
        self.pub_imu   = self.create_publisher(Imu,         "/imu",          10)
        self.pub_odom  = self.create_publisher(Odometry,    "/Odometry",     10)
        self.pub_tf    = self.create_publisher(TFMessage,   "/tf",           10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/points_raw",    2)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._step)
        self.get_logger().info(f"MujocoGo2Bridge ready — model={model_path}")

    def _read_loop(self):
        for line in self._proc.stdout:
            line = line.strip()
            if line:
                try:
                    self._resp_q.put(json.loads(line))
                except Exception:
                    pass

    def _send_recv(self, req: dict, timeout=2.0):
        self._proc.stdin.write(json.dumps(req) + "\n")
        self._proc.stdin.flush()
        try:
            return self._resp_q.get(timeout=timeout)
        except queue.Empty:
            return None

    def _cmd_cb(self, msg: Twist):
        self.cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _step(self):
        cmd = self.cmd
        if (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9 > self._cmd_vel_timeout:
            cmd = Twist()
        resp = self._send_recv({
            "action": "step",
            "vx": cmd.linear.x,
            "vy": cmd.linear.y,
            "yaw": cmd.angular.z,
        })
        if not resp or "state" not in resp:
            self._consecutive_timeouts += 1
            if self._consecutive_timeouts == 1 or self._consecutive_timeouts % 5 == 0:
                self.get_logger().error(
                    f"MuJoCo step timeout ({self._consecutive_timeouts} consecutive); "
                    "simulation backend is overloaded or unresponsive"
                )
            return
        if self._consecutive_timeouts:
            self.get_logger().info(
                f"MuJoCo step responses recovered after {self._consecutive_timeouts} timeout(s)"
            )
            self._consecutive_timeouts = 0
        state = resp["state"]
        t = state["time"]
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

        self.pub_clock.publish(Clock(clock=stamp))
        self._publish_joints(stamp, state["joints"])
        self._publish_imu(stamp, state)
        self._publish_odom_tf(stamp, state)

        if "points" in resp:
            self._publish_cloud(stamp, resp["points"])

    def _publish_joints(self, stamp, joints):
        msg = JointState(header=Header(stamp=stamp))
        for j in joints:
            msg.name.append(j["name"])
            msg.position.append(j["pos"])
            msg.velocity.append(j["vel"])
        self.pub_joint.publish(msg)

    def _publish_imu(self, stamp, state):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        x, y, z, w = _quat_wxyz_to_xyzw(state["quat"])
        msg.orientation.x = x; msg.orientation.y = y
        msg.orientation.z = z; msg.orientation.w = w
        vel = state["vel"]
        if len(vel) >= 6:
            msg.angular_velocity.x = vel[3]
            msg.angular_velocity.y = vel[4]
            msg.angular_velocity.z = vel[5]
        msg.linear_acceleration.z = 9.81
        self.pub_imu.publish(msg)

    def _publish_odom_tf(self, stamp, state):
        pos = state["pos"]
        x, y, z, w = _quat_wxyz_to_xyzw(state["quat"])
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        vel = state["vel"]
        if len(vel) >= 3:
            odom.twist.twist.linear.x = vel[0]
            odom.twist.twist.linear.y = vel[1]
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = pos[0]
        tf.transform.translation.y = pos[1]
        tf.transform.translation.z = pos[2]
        tf.transform.rotation = odom.pose.pose.orientation
        self.pub_tf.publish(TFMessage(transforms=[tf]))

    def _publish_cloud(self, stamp, points):
        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring",      offset=16, datatype=PointField.UINT16,  count=1),
            PointField(name="time",      offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        self.pub_cloud.publish(
            point_cloud2.create_cloud(
                Header(stamp=stamp, frame_id=self.lidar_frame), fields, points
            )
        )


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
