import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class Nav2LifecycleActivator(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_activator')
        self.get_state_client = self.create_client(GetState, '/planner_server/get_state')
        self.change_state_client = self.create_client(ChangeState, '/planner_server/change_state')
        self.pending = False
        self.complete = False
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        if self.complete or self.pending:
            return
        if not self.get_state_client.service_is_ready():
            self.get_logger().info('Waiting for planner_server lifecycle services')
            return
        self.pending = True
        future = self.get_state_client.call_async(GetState.Request())
        future.add_done_callback(self.state_received)

    def state_received(self, future):
        self.pending = False
        try:
            state = future.result().current_state.id
        except Exception as exc:
            self.get_logger().warn(f'Unable to query planner_server state: {exc}')
            return
        if state == State.PRIMARY_STATE_ACTIVE:
            self.complete = True
            self.timer.cancel()
            self.get_logger().info('planner_server lifecycle state is active')
        elif state == State.PRIMARY_STATE_UNCONFIGURED:
            self.request_transition(Transition.TRANSITION_CONFIGURE, 'configure')
        elif state == State.PRIMARY_STATE_INACTIVE:
            self.request_transition(Transition.TRANSITION_ACTIVATE, 'activate')
        else:
            self.get_logger().info(f'planner_server is transitioning (state={state}); waiting')

    def request_transition(self, transition_id, label):
        if not self.change_state_client.service_is_ready():
            return
        request = ChangeState.Request()
        request.transition.id = transition_id
        self.pending = True
        future = self.change_state_client.call_async(request)
        future.add_done_callback(lambda result: self.transition_done(result, label))

    def transition_done(self, future, label):
        self.pending = False
        try:
            success = future.result().success
        except Exception as exc:
            self.get_logger().warn(f'planner_server {label} request failed: {exc}')
            return
        if success:
            self.get_logger().info(f'planner_server {label} transition succeeded')
        else:
            self.get_logger().warn(f'planner_server {label} transition was rejected; retrying')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LifecycleActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
