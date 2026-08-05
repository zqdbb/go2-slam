#!/usr/bin/env python3
"""Tiny keyboard teleop for publishing geometry_msgs/Twist to /cmd_vel."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


HELP = """
Keyboard teleop
----------------
move:     u i o
turn:     j k l
strafe:   a   d
reverse:  m , .

i/, : add forward/back speed
j/l : add pure yaw speed
a/d : add left/right strafe speed
u/o : add pure yaw speed, same as j/l
m/. : add left/right strafe speed, same as a/d
k : reset command to zero
space : reset robot state in MuJoCo
q : quit
"""


def make_twist(vx: float, vy: float, wz: float) -> Twist:
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    return msg


def clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return value
    return max(-limit, min(limit, value))


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish keyboard teleop Twist commands.")
    parser.add_argument("--domain-id", type=int, default=42)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--reset-topic", default="/go2/reset")
    parser.add_argument("--lin", type=float, default=0.12)
    parser.add_argument("--strafe", type=float, default=0.08)
    parser.add_argument("--yaw", type=float, default=0.25)
    parser.add_argument("--max-lin", type=float, default=0.0, help="0 disables the teleop cap.")
    parser.add_argument("--max-strafe", type=float, default=0.0, help="0 disables the teleop cap.")
    parser.add_argument("--max-yaw", type=float, default=0.0, help="0 disables the teleop cap.")
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = rclpy.create_node("go2_keyboard_teleop")
    pub = node.create_publisher(Twist, args.topic, 10)
    reset_pub = node.create_publisher(Empty, args.reset_topic, 10)

    old_attrs = termios.tcgetattr(sys.stdin)
    vx = vy = wz = 0.0
    period = 1.0 / max(args.rate, 1.0)

    print(HELP)
    print(f"Publishing {args.topic} on ROS_DOMAIN_ID={args.domain_id}")
    print(f"Robot reset topic: {args.reset_topic}")
    print("Commands accumulate until k resets them. Space resets the MuJoCo robot state.")
    if args.max_lin <= 0 and args.max_strafe <= 0 and args.max_yaw <= 0:
        print("Teleop caps: disabled")
    else:
        print(
            "Teleop caps: "
            f"|vx|<={args.max_lin:.2f}, |vy|<={args.max_strafe:.2f}, |wz|<={args.max_yaw:.2f}"
        )

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], period)
            if readable:
                key = sys.stdin.read(1)
                if key == "q":
                    break
                if key == "k":
                    vx = vy = wz = 0.0
                    print("cmd reset")
                elif key == " ":
                    vx = vy = wz = 0.0
                    pub.publish(make_twist(0.0, 0.0, 0.0))
                    reset_pub.publish(Empty())
                    print("robot reset")
                elif key == "i":
                    vx = clamp(vx + args.lin, args.max_lin)
                    vy = wz = 0.0
                    print(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
                elif key == ",":
                    vx = clamp(vx - args.lin, args.max_lin)
                    vy = wz = 0.0
                    print(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
                elif key in {"j", "u"}:
                    vx = vy = 0.0
                    wz = clamp(wz + args.yaw, args.max_yaw)
                    print(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
                elif key in {"l", "o"}:
                    vx = vy = 0.0
                    wz = clamp(wz - args.yaw, args.max_yaw)
                    print(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
                elif key in {"a", "m"}:
                    vx = wz = 0.0
                    vy = clamp(vy + args.strafe, args.max_strafe)
                    print(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
                elif key in {"d", "."}:
                    vx = wz = 0.0
                    vy = clamp(vy - args.strafe, args.max_strafe)
                    print(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
            pub.publish(make_twist(vx, vy, wz))
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        pub.publish(make_twist(0.0, 0.0, 0.0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
