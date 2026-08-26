#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from action_msgs.msg import GoalStatus
import cmd
import threading
import time
from typing import Optional, Callable

# Action 인터페이스 임포트
from rl_sar.action import RobotActivation, SetLocomotionMode, SetNavigationMode


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def status_name(code: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(code, f"CODE({code})")


class MultiActionClientNode(Node):
    """
    세 가지 액션(Activation, Locomotion, Navigation)을 위한 클라이언트를 관리하는 노드.
    """
    def __init__(self) -> None:
        super().__init__('multi_action_client_node')
        self._activation_client = ActionClient(self, RobotActivation, 'robot_activation')
        self._locomotion_client = ActionClient(self, SetLocomotionMode, 'set_locomotion_mode')
        self._navigation_client = ActionClient(self, SetNavigationMode, 'set_navigation_mode')

        # 최근 전송한 goal 핸들(취소 지원용)
        self._last_goal_handle = {
            'activation': None,
            'locomotion': None,
            'navigation': None,
        }

        # 결과/피드백을 CLI로 넘길 콜백 (ControlSuiteShell에서 등록)
        self._result_callback: Optional[Callable[[str], None]] = None
        self._feedback_callback_cli: Optional[Callable[[str], None]] = None

        # 피드백 로그 스로틀
        self._last_feedback_time = 0.0
        self._feedback_period = 0.2  # seconds

        # 프롬프트와 충돌 막기 위해 INFO 이하 숨김
        self.get_logger().set_level(LoggingSeverity.WARN)

    # 외부(Shell)가 콜백 등록
    def register_result_callback(self, cb: Callable[[str], None]) -> None:
        self._result_callback = cb

    def register_feedback_callback(self, cb: Callable[[str], None]) -> None:
        self._feedback_callback_cli = cb

    def send_goal(self, name: str, client: ActionClient, goal_msg) -> None:
        """
        주어진 클라이언트로 목표를 전송하는 범용 함수.
        """
        if not client.wait_for_server(timeout_sec=3.0):
            msg = f'[{name}] Action server not available!'
            self.get_logger().error(msg)
            if self._result_callback:
                self._result_callback(msg)
            return

        future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self.feedback_callback(name, fb)
        )
        future.add_done_callback(lambda f: self.goal_response_callback(name, f))

    def goal_response_callback(self, name: str, future) -> None:
        exc = future.exception()
        if exc:
            msg = f'[{name}] Goal request failed: {exc!r}'
            self.get_logger().error(msg)
            if self._result_callback:
                self._result_callback(msg)
            return

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            msg = f'[{name}] Goal rejected.'
            self.get_logger().warn(msg)
            if self._result_callback:
                self._result_callback(msg)
            return

        self._last_goal_handle[name] = goal_handle
        # 결과 비동기 수신
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(name, f))

    def get_result_callback(self, name: str, future) -> None:
        exc = future.exception()
        if exc:
            msg = f'[{name}] Result retrieval failed: {exc!r}'
            self.get_logger().error(msg)
            if self._result_callback:
                self._result_callback(msg)
            return

        result = future.result()
        if result is None:
            msg = f'[{name}] Empty result.'
            self.get_logger().warn(msg)
            if self._result_callback:
                self._result_callback(msg)
            return

        status = status_name(result.status)
        msg = f'[{name}] Finished with status: {status}\n[{name}] Result: {result.result}'
        if self._result_callback:
            self._result_callback(msg)

    def feedback_callback(self, name: str, feedback_msg) -> None:
        now = time.time()
        if now - self._last_feedback_time < self._feedback_period:
            return
        self._last_feedback_time = now
        if self._feedback_callback_cli:
            self._feedback_callback_cli(f'[{name}] Feedback: {feedback_msg.feedback}')

    # ---- 고수준 API ----
    def send_activation_goal(self, activate: bool) -> None:
        goal = RobotActivation.Goal()
        # 액션 정의에 맞게 필드명 확인 필요. 보통 'activation' 또는 'activate'
        # 여기서는 'activation'으로 가정.
        goal.activate = activate
        self.send_goal('activation', self._activation_client, goal)

    def send_locomotion_goal(self, mode: int) -> None:
        goal = SetLocomotionMode.Goal()
        goal.mode = mode
        self.send_goal('locomotion', self._locomotion_client, goal)

    def send_navigation_goal(self, mode: int) -> None:
        # SetNavigationMode가 정수 모드라고 가정 (필드명이 다르면 맞게 바꿔주세요)
        goal = SetNavigationMode.Goal()
        # 예: goal.mode = mode  (혹은 goal.navigation_mode = mode)
        goal.enable = mode
        self.send_goal('navigation', self._navigation_client, goal)

    def cancel_last_goal(self, name: str) -> None:
        gh = self._last_goal_handle.get(name)
        if gh is None:
            if self._result_callback:
                self._result_callback(f'[{name}] No goal handle to cancel.')
            return
        cancel_future = gh.cancel_goal_async()
        cancel_future.add_done_callback(lambda f: self._cancel_done(name, f))

    def _cancel_done(self, name: str, future) -> None:
        exc = future.exception()
        if exc:
            msg = f'[{name}] Cancel failed: {exc!r}'
            self.get_logger().error(msg)
            if self._result_callback:
                self._result_callback(msg)
            return
        res = future.result()
        if self._result_callback:
            self._result_callback(f'[{name}] Cancel response: {res.return_code}')
        self._last_goal_handle[name] = None


class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the ROS 2 control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self, node: MultiActionClientNode):
        super().__init__()
        self.node = node

        # 결과/피드백이 오면 프롬프트 복원 포함해서 출력
        self.node.register_result_callback(self._log_and_refresh_prompt)
        # 필요시 주석 해제해서 피드백도 표시
        # self.node.register_feedback_callback(self._log_and_refresh_prompt)

    def _log_and_refresh_prompt(self, msg: str):
        print(msg)
        self.stdout.write(self.prompt)
        self.stdout.flush()

    # ---- activation ----
    def do_activation(self, arg: str):
        "Activate robot: activation [true|false]"
        s = (arg or '').strip().lower()
        if s not in ('true', 'false'):
            self._log_and_refresh_prompt("Invalid argument. Please use 'true' or 'false'.")
            return
        self.node.send_activation_goal(s == 'true')
        # 결과가 올 때만 출력하도록 즉시 메시지는 생략

    # ---- navigation (integer mode) ----
    def do_nav_mode(self, arg: str):
        "Navigation robot: activation [true|false]"
        s = (arg or '').strip().lower()
        if s not in ('true', 'false'):
            self._log_and_refresh_prompt("Invalid argument. Please use 'true' or 'false'.")
            return
        self.node.send_navigation_goal(s == 'true')
        # 결과 출력은 콜백에서

    # ---- locomotion ----
    def do_loco_mode(self, arg: str):
        "Set locomotion mode (int). Usage: loco_mode [integer_mode]"
        s = (arg or '').strip()
        try:
            mode = int(s)
        except ValueError:
            self._log_and_refresh_prompt("Invalid argument. Provide an integer for the mode.")
            return
        self.node.send_locomotion_goal(mode)

    # ---- cancel helpers ----
    def do_cancel_activation(self, arg: str):
        "Cancel last activation goal"
        self.node.cancel_last_goal('activation')

    def do_cancel_navigation(self, arg: str):
        "Cancel last navigation goal"
        self.node.cancel_last_goal('navigation')

    def do_cancel_locomotion(self, arg: str):
        "Cancel last locomotion goal"
        self.node.cancel_last_goal('locomotion')

    # ---- quit/EOF ----
    def do_quit(self, arg: str):
        "Exit the shell."
        print("Shutting down...")
        return True

    def do_EOF(self, arg: str):
        "Exit on Ctrl-D"
        print()
        return self.do_quit(arg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)

    node = MultiActionClientNode()

    # 멀티스레드 실행기 + 별도 스레드에서 spin
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        shell = ControlSuiteShell(node)
        shell.cmdloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
