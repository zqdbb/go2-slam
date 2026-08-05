"""
trg_path_follower.py — TRG-planner 연동 Path Follower + Fallback Direct Goal Follower

TRG-planner 아키텍처:
  [우리 odom/goal] → bridge → TRG-planner → nav_msgs/Path → 여기서 cmd_vel 생성

TRG-planner 토픽:
  Subscribe (우리가 publish):
    /trg/input/default_pose  (PoseStamped)  ← 로봇 현재 pose
    /fake_robot_pose         (Odometry)     ← 로봇 odom (TRG-planner용 이름)
    /fake_goal               (PoseStamped)  ← 목표 위치
  Publish (우리가 subscribe):
    /trg/output/default_path (nav_msgs/Path) ← 계획된 경로

실행 모드:
  Mode 1 (TRG-planner 연동): --mode trg
    TRG-planner가 주는 Path를 Pure Pursuit으로 추적 → /cmd_vel
    SLAM팀 TRG-planner와 같은 ROS_DOMAIN_ID 설정 필요

  Mode 2 (단독 Goal Follower): --mode direct (default)
    /goal_pose (PoseStamped) 직접 받아서 P-control → /cmd_vel
    TRG-planner 없이도 동작

실행:
  # TRG-planner 연동 모드
  source /opt/ros/jazzy/setup.bash
  python3 scripts/ros2/trg_path_follower.py --mode trg

  # 단독 모드 (TRG-planner 없이 테스트)
  python3 scripts/ros2/trg_path_follower.py --mode direct
"""

import math
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path


# ─────────────────────────────────────────────────────────────────────────────
# 유틸리티
# ─────────────────────────────────────────────────────────────────────────────
def quat_to_yaw(q) -> float:
    """quaternion → yaw (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ─────────────────────────────────────────────────────────────────────────────
# Mode 1: TRG-planner Path Follower (Pure Pursuit)
# ─────────────────────────────────────────────────────────────────────────────
class TRGPathFollower(Node):
    """TRG-planner nav_msgs/Path를 Pure Pursuit으로 추적 → /cmd_vel.

    TRG-planner 토픽 브릿지 포함:
      /odom            → /fake_robot_pose  (TRG-planner odom 입력)
      /odom            → /trg/input/default_pose  (TRG-planner pose 입력)
      /goal_pose       → /fake_goal        (TRG-planner goal 입력)
      /trg/output/default_path → pure pursuit → /cmd_vel
    """

    def __init__(self, odom_topic: str, max_vx: float, max_wz: float,
                 lookahead: float, stop_dist: float):
        super().__init__("trg_path_follower")

        self.max_vx     = max_vx
        self.max_wz     = max_wz
        self.lookahead  = lookahead   # pure pursuit lookahead distance (m)
        self.stop_dist  = stop_dist

        # 상태
        self.cur_x   = 0.0
        self.cur_y   = 0.0
        self.cur_yaw = 0.0
        self.path: list[tuple[float, float]] = []
        self.path_active = False
        self.goal_x: float | None = None
        self.goal_y: float | None = None

        # ── Subscribers ────────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._cb_odom, 10)
        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self._cb_goal_pose, 10)
        self.sub_path = self.create_subscription(
            Path, "/trg/output/default_path", self._cb_path, 10)

        # ── Publishers ─────────────────────────────────────────────────────
        self.pub_cmd       = self.create_publisher(Twist, "/cmd_vel", 10)
        # TRG-planner 브릿지 토픽
        self.pub_fake_odom = self.create_publisher(Odometry, "/fake_robot_pose", 10)
        self.pub_trg_pose  = self.create_publisher(PoseStamped, "/trg/input/default_pose", 10)
        self.pub_fake_goal = self.create_publisher(PoseStamped, "/fake_goal", 10)

        # 제어 루프 (10Hz)
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f"[TRG Path Follower] odom={odom_topic}  "
            f"lookahead={lookahead}m  max_vx={max_vx}  max_wz={max_wz}")
        self.get_logger().info(
            "RViz2 '2D Goal Pose' → /goal_pose → TRG-planner → path → cmd_vel")

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        """odom 수신 → 상태 업데이트 + TRG-planner 브릿지."""
        self.cur_x   = msg.pose.pose.position.x
        self.cur_y   = msg.pose.pose.position.y
        self.cur_yaw = quat_to_yaw(msg.pose.pose.orientation)

        # TRG-planner에 odom 전달 (/fake_robot_pose)
        self.pub_fake_odom.publish(msg)

        # TRG-planner pose 입력 (/trg/input/default_pose)
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "map"
        ps.pose = msg.pose.pose
        self.pub_trg_pose.publish(ps)

    def _cb_goal_pose(self, msg: PoseStamped):
        """RViz2 goal → TRG-planner /fake_goal 브릿지."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.path_active = False  # 새 goal → 이전 path 무효화
        self.path = []

        # TRG-planner에 goal 전달
        fake_goal = PoseStamped()
        fake_goal.header = msg.header
        fake_goal.header.frame_id = "map"
        fake_goal.pose = msg.pose
        self.pub_fake_goal.publish(fake_goal)

        self.get_logger().info(
            f"새 목표 → TRG-planner 전달: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def _cb_path(self, msg: Path):
        """TRG-planner path 수신 → waypoints 리스트."""
        if not msg.poses:
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_active = True
        self.get_logger().info(f"TRG path 수신: {len(self.path)} waypoints")

    # ── Pure Pursuit 제어 ──────────────────────────────────────────────────
    def _control_loop(self):
        cmd = Twist()

        # TRG path가 없으면 정지
        if not self.path_active or not self.path:
            self.pub_cmd.publish(cmd)
            return

        # 최종 goal 도달 확인
        if self.goal_x is not None:
            dx = self.goal_x - self.cur_x
            dy = self.goal_y - self.cur_y
            if math.sqrt(dx*dx + dy*dy) < self.stop_dist:
                self.path_active = False
                self.get_logger().info("목표 도달!")
                self.pub_cmd.publish(cmd)
                return

        # Pure Pursuit: lookahead point 탐색
        target = self._find_lookahead_point()
        if target is None:
            # path 끝까지 왔는데 못 찾으면 마지막 waypoint
            target = self.path[-1]

        tx, ty = target
        dx = tx - self.cur_x
        dy = ty - self.cur_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.stop_dist:
            self.pub_cmd.publish(cmd)
            return

        # 목표 방향 → 회전 제어
        target_yaw = math.atan2(dy, dx)
        yaw_error  = normalize_angle(target_yaw - self.cur_yaw)

        wz = 1.8 * yaw_error
        wz = max(-self.max_wz, min(self.max_wz, wz))

        # 방향 정렬 시 전진 (Pure Pursuit: 속도도 곡률 반영)
        if abs(yaw_error) < 0.4:
            # 곡률 기반 속도 감소: yaw_error 클수록 느리게
            vx = self.max_vx * (1.0 - abs(yaw_error) / 0.4 * 0.5)
            vx = min(vx, self.max_vx)
        else:
            vx = 0.0  # 제자리 회전

        cmd.linear.x  = vx
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)

    def _find_lookahead_point(self) -> tuple[float, float] | None:
        """현재 위치에서 lookahead 거리 앞의 path point 반환."""
        # 가장 가까운 waypoint 인덱스 찾기
        min_dist = float('inf')
        closest_idx = 0
        for i, (wx, wy) in enumerate(self.path):
            d = math.sqrt((wx - self.cur_x)**2 + (wy - self.cur_y)**2)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # closest_idx 이후에서 lookahead 거리 이상인 첫 waypoint
        for i in range(closest_idx, len(self.path)):
            wx, wy = self.path[i]
            d = math.sqrt((wx - self.cur_x)**2 + (wy - self.cur_y)**2)
            if d >= self.lookahead:
                return (wx, wy)

        # lookahead 이상 없으면 마지막 point
        return self.path[-1] if self.path else None


# ─────────────────────────────────────────────────────────────────────────────
# Mode 2: Direct Goal Follower (TRG-planner 없이 단독 동작)
# ─────────────────────────────────────────────────────────────────────────────
class DirectGoalFollower(Node):
    """TRG-planner 없이 /goal_pose → P-control → /cmd_vel.

    SLAM팀 연동 전 단독 테스트용.
    """

    def __init__(self, odom_topic: str, max_vx: float, max_wz: float, stop_dist: float):
        super().__init__("direct_goal_follower")

        self.max_vx    = max_vx
        self.max_wz    = max_wz
        self.stop_dist = stop_dist
        self.yaw_align = 0.30

        self.kp_wz = 1.8
        self.kp_vx = 0.8

        self.goal_x: float | None = None
        self.goal_y: float | None = None
        self.cur_x   = 0.0
        self.cur_y   = 0.0
        self.cur_yaw = 0.0
        self.active  = False

        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self._cb_goal, 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self._cb_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f"[Direct Goal Follower] odom={odom_topic}  "
            f"max_vx={max_vx}  stop_dist={stop_dist}m")
        self.get_logger().info("RViz2 '2D Goal Pose' 버튼으로 목표 설정")

    def _cb_goal(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.active = True
        self.get_logger().info(f"새 목표: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def _cb_odom(self, msg: Odometry):
        self.cur_x   = msg.pose.pose.position.x
        self.cur_y   = msg.pose.pose.position.y
        self.cur_yaw = quat_to_yaw(msg.pose.pose.orientation)

    def _control_loop(self):
        cmd = Twist()
        if not self.active or self.goal_x is None:
            self.pub_cmd.publish(cmd)
            return

        dx   = self.goal_x - self.cur_x
        dy   = self.goal_y - self.cur_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.stop_dist:
            self.active = False
            self.get_logger().info(f"목표 도달! (dist={dist:.3f}m)")
            self.pub_cmd.publish(cmd)
            return

        target_yaw = math.atan2(dy, dx)
        yaw_error  = normalize_angle(target_yaw - self.cur_yaw)

        wz = self.kp_wz * yaw_error
        wz = max(-self.max_wz, min(self.max_wz, wz))

        if abs(yaw_error) < self.yaw_align:
            vx = min(self.kp_vx * dist, self.max_vx)
        else:
            vx = 0.0

        cmd.linear.x  = vx
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)

        if not hasattr(self, "_log_cnt"): self._log_cnt = 0
        self._log_cnt += 1
        if self._log_cnt % 20 == 0:
            self.get_logger().info(
                f"dist={dist:.2f}m  yaw_err={math.degrees(yaw_error):.1f}°  "
                f"vx={vx:.2f}  wz={wz:.2f}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Goal Follower / TRG-planner Path Follower")
    parser.add_argument("--mode", choices=["trg", "direct"], default="direct",
                        help="trg: TRG-planner path 추적 | direct: 단독 goal P-control")
    parser.add_argument("--odom-topic", default="/odom",
                        help="오도메트리 토픽 (sim: /odom, FAST-LIO2: /Odometry)")
    parser.add_argument("--max-vx",    type=float, default=0.8,
                        help="최대 전진 속도 m/s (default=0.8)")
    parser.add_argument("--max-wz",    type=float, default=1.2,
                        help="최대 회전 속도 rad/s (default=1.2)")
    parser.add_argument("--lookahead", type=float, default=1.5,
                        help="Pure Pursuit lookahead 거리 m (TRG 모드, default=1.5)")
    parser.add_argument("--stop-dist", type=float, default=0.3,
                        help="목표 도달 거리 기준 m (default=0.3)")
    args = parser.parse_args()

    rclpy.init()

    if args.mode == "trg":
        node = TRGPathFollower(
            odom_topic=args.odom_topic,
            max_vx=args.max_vx,
            max_wz=args.max_wz,
            lookahead=args.lookahead,
            stop_dist=args.stop_dist,
        )
        print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  TRG-planner 연동 모드
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  TRG-planner 먼저 실행:
    ros2 launch trg_planner_ros trg_planner_wMap.py map:=indoor

  이 노드가 자동으로 브릿지:
    /odom        → /fake_robot_pose    (TRG odom)
    /odom        → /trg/input/default_pose (TRG pose)
    /goal_pose   → /fake_goal          (TRG goal)
    /trg/output/default_path → Pure Pursuit → /cmd_vel

  RViz2에서 '2D Goal Pose' 클릭 → 자동 경로 생성 → Go2 이동
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")
    else:
        node = DirectGoalFollower(
            odom_topic=args.odom_topic,
            max_vx=args.max_vx,
            max_wz=args.max_wz,
            stop_dist=args.stop_dist,
        )
        print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  단독 Goal Follower 모드 (TRG-planner 없이)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  RViz2에서 '2D Goal Pose' 클릭 → P-control → /cmd_vel
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
