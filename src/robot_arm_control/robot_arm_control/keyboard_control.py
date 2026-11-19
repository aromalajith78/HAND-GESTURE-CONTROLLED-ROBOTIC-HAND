#!/usr/bin/env python3
"""Keyboard teleoperation node for the 4-DOF interactive robotic arm."""

import json
import select
import sys
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BANNER = """
====================================================================
     ROS 2 HUMBLE - 4-DOF ROBOTIC ARM KEYBOARD TELEOPERATION
====================================================================
  Cartesian End-Effector Controls:
    W : Move UP (+Z)         S : Move DOWN (-Z)
    A : Move LEFT (+Y)       D : Move RIGHT (-Y)
    Q : Move FORWARD (+X)    E : Move BACKWARD (-X)

  Gripper & System Controls:
    O : Open Gripper         C : Close Gripper
    H : Home Position        X : EMERGENCY STOP
    + / - : Increase / Decrease Speed
    ESC or Ctrl+C : Quit
====================================================================
"""

MOVE_KEYS = {
    'w': (0.0, 0.0, 1.0, 'UP'),
    's': (0.0, 0.0, -1.0, 'DOWN'),
    'a': (0.0, 1.0, 0.0, 'LEFT'),
    'd': (0.0, -1.0, 0.0, 'RIGHT'),
    'q': (1.0, 0.0, 0.0, 'FORWARD'),
    'e': (-1.0, 0.0, 0.0, 'BACKWARD'),
}

CMD_KEYS = {
    'o': ('gripper_open', 'GRIPPER OPEN'),
    'c': ('gripper_close', 'GRIPPER CLOSE'),
    'h': ('home', 'HOME POSITION'),
    'x': ('stop', 'EMERGENCY STOP'),
}


class KeyboardControlNode(Node):
    """Reads non-blocking keyboard presses and publishes Cartesian/system commands."""

    def __init__(self) -> None:
        super().__init__('keyboard_control')
        self.vel_pub = self.create_publisher(Twist, '/arm_control/cartesian_vel', 10)
        self.cmd_pub = self.create_publisher(String, '/arm_control/command', 10)
        self.status_sub = self.create_subscription(
            String, '/arm_control/status', self._status_cb, 10
        )

        self.speed = 0.12  # m/s
        self.min_speed = 0.04
        self.max_speed = 0.24
        self.latest_status = {}
        self.last_action = 'Ready'

    def _status_cb(self, msg: String) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def publish_vel(self, vx: float, vy: float, vz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def publish_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def format_status_line(self) -> str:
        xyz = self.latest_status.get('ee_xyz', [0.0, 0.0, 0.0])
        mode = self.latest_status.get('mode', 'WAIT')
        warn = self.latest_status.get('warning', '')
        warn_str = f' | {warn}' if warn else ''
        return (
            f'\r[Speed: {self.speed:.2f} m/s] [Mode: {mode:<11}] '
            f'[EE X:{xyz[0]:+.3f} Y:{xyz[1]:+.3f} Z:{xyz[2]:+.3f}] '
            f'[Last: {self.last_action:<15}]{warn_str}   '
        )


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print('Error: keyboard_control must be run inside an interactive terminal (TTY).')
        sys.exit(1)

    rclpy.init(args=args)
    node = KeyboardControlNode()

    stop_event = threading.Event()

    def spin_loop() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()

    old_settings = termios.tcgetattr(sys.stdin)
    print(BANNER)

    active_vel = (0.0, 0.0, 0.0)
    vel_expiry = 0.0
    last_print = 0.0

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            now = time.monotonic()

            if select.select([sys.stdin], [], [], 0.03)[0]:
                ch = sys.stdin.read(1)
                if ch == '\x1b':  # ESC
                    break
                key = ch.lower()

                if key in MOVE_KEYS:
                    dx, dy, dz, label = MOVE_KEYS[key]
                    active_vel = (dx * node.speed, dy * node.speed, dz * node.speed)
                    vel_expiry = now + 0.16
                    node.publish_vel(*active_vel)
                    node.last_action = label
                elif key in CMD_KEYS:
                    cmd_str, label = CMD_KEYS[key]
                    active_vel = (0.0, 0.0, 0.0)
                    vel_expiry = 0.0
                    node.publish_vel(0.0, 0.0, 0.0)
                    node.publish_cmd(cmd_str)
                    node.last_action = label
                elif ch in ('+', '='):
                    node.speed = min(node.max_speed, round(node.speed + 0.02, 2))
                    node.last_action = f'Speed {node.speed:.2f}'
                elif ch in ('-', '_'):
                    node.speed = max(node.min_speed, round(node.speed - 0.02, 2))
                    node.last_action = f'Speed {node.speed:.2f}'
            else:
                if vel_expiry > 0.0:
                    if now < vel_expiry:
                        node.publish_vel(*active_vel)
                    else:
                        active_vel = (0.0, 0.0, 0.0)
                        vel_expiry = 0.0
                        node.publish_vel(0.0, 0.0, 0.0)

            if now - last_print >= 0.10:
                sys.stdout.write(node.format_status_line())
                sys.stdout.flush()
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        node.publish_vel(0.0, 0.0, 0.0)
        stop_event.set()
        spin_thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('\nKeyboard control exited cleanly.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
