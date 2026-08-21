import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class Nav2PathBridge(Node):
    def __init__(self):
        super().__init__('nav2_path_bridge')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(Path, '/initial_path', qos)
        self.goal_subs = [
            self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_cb, qos),
            self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, qos),
        ]
        self.cli = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.busy = False
        self.busy_since = 0.0
        self.request_id = 0
        self.pending_goal = None
        self.create_timer(0.5, self.check_timeout)
        self.get_logger().info('Nav2 path bridge ready; waiting for /move_base_simple/goal or /goal_pose')

    def goal_cb(self, goal):
        if self.busy:
            self.pending_goal = goal
            self.get_logger().warn('Planner busy; queued latest goal')
            return
        if not goal.header.frame_id:
            goal.header.frame_id = 'world'
        if goal.header.frame_id != 'world':
            self.get_logger().warn(f'Goal frame is {goal.header.frame_id}; expected world')
        if not self.cli.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 /compute_path_to_pose action unavailable')
            return
        req = ComputePathToPose.Goal()
        req.goal = goal
        req.start = PoseStamped()
        req.start.header.frame_id = 'world'
        req.use_start = False
        req.planner_id = 'GridBased'
        self.busy = True
        self.busy_since = time.monotonic()
        self.request_id += 1
        request_id = self.request_id
        future = self.cli.send_goal_async(req)
        future.add_done_callback(lambda result: self.goal_response(result, request_id))
        self.get_logger().info(f'Planning global path to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')

    def finish_request(self, request_id):
        if request_id != self.request_id:
            return
        self.busy = False
        self.request_id += 1
        if self.pending_goal is not None:
            goal = self.pending_goal
            self.pending_goal = None
            self.goal_cb(goal)

    def check_timeout(self):
        if self.busy and time.monotonic() - self.busy_since > 5.0:
            expired_id = self.request_id
            self.get_logger().error('ComputePathToPose timed out after 5 s; planner bridge recovered')
            self.finish_request(expired_id)

    def goal_response(self, future, request_id):
        if request_id != self.request_id:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.finish_request(request_id)
            self.get_logger().error(f'ComputePathToPose goal failed: {exc}')
            return
        if not handle.accepted:
            self.finish_request(request_id)
            self.get_logger().error('ComputePathToPose goal rejected')
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda result: self.plan_done(result, request_id))

    def plan_done(self, future, request_id):
        if request_id != self.request_id:
            return
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.finish_request(request_id)
            self.get_logger().error(f'ComputePathToPose failed: {exc}')
            return
        path = result.path
        if len(path.poses) < 2:
            self.finish_request(request_id)
            self.get_logger().error(f'Nav2 returned too few path poses: {len(path.poses)}')
            return
        if not path.header.frame_id:
            path.header.frame_id = 'world'
        self.pub.publish(path)
        self.get_logger().info(f'Published global path with {len(path.poses)} poses to /initial_path')
        self.finish_request(request_id)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
