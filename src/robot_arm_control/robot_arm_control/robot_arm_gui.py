#!/usr/bin/env python3
"""Tkinter graphical user interface for controlling the 4-DOF robotic arm."""

import json
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArmGuiNode(Node):
    """ROS 2 pub/sub bridge for the Tkinter robotic arm control panel."""

    def __init__(self) -> None:
        super().__init__('robot_arm_gui')
        self.vel_pub = self.create_publisher(Twist, '/arm_control/cartesian_vel', 10)
        self.cmd_pub = self.create_publisher(String, '/arm_control/command', 10)
        self.status_sub = self.create_subscription(
            String, '/arm_control/status', self._status_cb, 10
        )
        self.lock = threading.Lock()
        self.status_data = {
            'connected': False,
            'mode': 'CONNECTING',
            'warning': '',
            'ee_xyz': [0.0, 0.0, 0.0],
            'joints': [0.0, 0.0, 0.0, 0.0],
            'gripper': 0.020,
        }

    def _status_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            with self.lock:
                self.status_data = parsed
        except json.JSONDecodeError:
            pass

    def get_status(self) -> dict:
        with self.lock:
            return dict(self.status_data)

    def send_vel(self, vx: float, vy: float, vz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def send_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)


class RobotArmGuiApp:
    """Interactive dark-themed Tkinter control GUI with press-and-hold + click support."""

    BG = '#161b22'
    PANEL_BG = '#21262d'
    TEXT_FG = '#f0f6fc'
    MUTED_FG = '#8b949e'
    BTN_BG = '#30363d'
    BTN_ACTIVE = '#388bfd'
    GRIP_BG = '#1f6feb'
    HOME_BG = '#238636'
    STOP_BG = '#da3633'

    def __init__(self, root: tk.Tk, node: ArmGuiNode) -> None:
        self.root = root
        self.node = node
        self.root.title('ROS 2 Humble - 4-DOF Robot Arm Controller')
        self.root.configure(bg=self.BG)
        self.root.resizable(False, False)

        self.speed_var = tk.DoubleVar(value=0.12)
        self.active_dir = None  # (dx, dy, dz)
        self.press_time = 0.0
        self.min_pulse_sec = 0.14
        self.pulse_job = None

        self._build_ui()
        self._schedule_vel_loop()
        self._schedule_telemetry_refresh()

    def _build_ui(self) -> None:
        main = tk.Frame(self.root, bg=self.BG, padx=18, pady=16)
        main.pack(fill=tk.BOTH, expand=True)

        # Header
        header = tk.Frame(main, bg=self.BG)
        header.pack(fill=tk.X, pady=(0, 12))
        tk.Label(
            header,
            text='4-DOF Robotic Arm Controller',
            font=('Segoe UI', 15, 'bold'),
            fg=self.TEXT_FG,
            bg=self.BG,
        ).pack(side=tk.LEFT)

        self.status_badge = tk.Label(
            header,
            text='CONNECTING',
            font=('Segoe UI', 9, 'bold'),
            fg='#ffffff',
            bg='#6e7681',
            padx=10,
            pady=3,
        )
        self.status_badge.pack(side=tk.RIGHT)

        # Movement Pads Container
        pads = tk.Frame(main, bg=self.PANEL_BG, padx=14, pady=14, bd=1, relief=tk.FLAT)
        pads.pack(fill=tk.X, pady=(0, 12))

        # Horizontal / Depth Pad (FORWARD / BACKWARD / LEFT / RIGHT)
        xy_frame = tk.Frame(pads, bg=self.PANEL_BG)
        xy_frame.pack(side=tk.LEFT, padx=(0, 22))
        tk.Label(
            xy_frame,
            text='XY Planar Motion',
            font=('Segoe UI', 10, 'bold'),
            fg=self.MUTED_FG,
            bg=self.PANEL_BG,
        ).grid(row=0, column=0, columnspan=3, pady=(0, 8))

        self._make_hold_btn(xy_frame, 'FORWARD\n(+X)', (1, 0, 0)).grid(
            row=1, column=1, padx=4, pady=4
        )
        self._make_hold_btn(xy_frame, 'LEFT\n(+Y)', (0, 1, 0)).grid(
            row=2, column=0, padx=4, pady=4
        )
        self._make_hold_btn(xy_frame, 'RIGHT\n(-Y)', (0, -1, 0)).grid(
            row=2, column=2, padx=4, pady=4
        )
        self._make_hold_btn(xy_frame, 'BACKWARD\n(-X)', (-1, 0, 0)).grid(
            row=3, column=1, padx=4, pady=4
        )

        # Vertical Pad (UP / DOWN)
        z_frame = tk.Frame(pads, bg=self.PANEL_BG)
        z_frame.pack(side=tk.RIGHT, fill=tk.Y)
        tk.Label(
            z_frame,
            text='Vertical Motion',
            font=('Segoe UI', 10, 'bold'),
            fg=self.MUTED_FG,
            bg=self.PANEL_BG,
        ).pack(pady=(0, 8))

        self._make_hold_btn(z_frame, 'UP\n(+Z)', (0, 0, 1), width=12).pack(pady=6)
        self._make_hold_btn(z_frame, 'DOWN\n(-Z)', (0, 0, -1), width=12).pack(pady=6)

        # Gripper & System Actions
        actions = tk.Frame(main, bg=self.PANEL_BG, padx=14, pady=12)
        actions.pack(fill=tk.X, pady=(0, 12))

        grip_row = tk.Frame(actions, bg=self.PANEL_BG)
        grip_row.pack(fill=tk.X, pady=(0, 8))
        self._make_action_btn(
            grip_row, 'GRIPPER OPEN', self.GRIP_BG, lambda: self.node.send_cmd('gripper_open')
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 6))
        self._make_action_btn(
            grip_row, 'GRIPPER CLOSE', self.GRIP_BG, lambda: self.node.send_cmd('gripper_close')
        ).pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=(6, 0))

        sys_row = tk.Frame(actions, bg=self.PANEL_BG)
        sys_row.pack(fill=tk.X)
        self._make_action_btn(
            sys_row, 'HOME', self.HOME_BG, self._on_home
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 6))
        self._make_action_btn(
            sys_row, 'STOP', self.STOP_BG, self._on_stop
        ).pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=(6, 0))

        # Speed Control
        speed_panel = tk.Frame(main, bg=self.PANEL_BG, padx=14, pady=10)
        speed_panel.pack(fill=tk.X, pady=(0, 12))
        self.speed_label = tk.Label(
            speed_panel,
            text='Cartesian Speed: 0.12 m/s',
            font=('Segoe UI', 9, 'bold'),
            fg=self.TEXT_FG,
            bg=self.PANEL_BG,
        )
        self.speed_label.pack(anchor=tk.W)
        ttk.Scale(
            speed_panel,
            from_=0.04,
            to=0.24,
            orient=tk.HORIZONTAL,
            variable=self.speed_var,
            command=lambda _v: self.speed_label.config(
                text=f'Cartesian Speed: {self.speed_var.get():.2f} m/s'
            ),
        ).pack(fill=tk.X, pady=(4, 0))

        # Telemetry Panel
        telem = tk.Frame(main, bg=self.PANEL_BG, padx=14, pady=10)
        telem.pack(fill=tk.X)
        tk.Label(
            telem,
            text='Live Robot Telemetry',
            font=('Segoe UI', 10, 'bold'),
            fg=self.MUTED_FG,
            bg=self.PANEL_BG,
        ).pack(anchor=tk.W, pady=(0, 4))

        self.ee_label = tk.Label(
            telem,
            text='End-Effector:  X: +0.000 m   Y: +0.000 m   Z: +0.000 m',
            font=('Consolas', 10),
            fg=self.TEXT_FG,
            bg=self.PANEL_BG,
        )
        self.ee_label.pack(anchor=tk.W)

        self.joints_label = tk.Label(
            telem,
            text='Joints (deg):  Base: +0.0  Shld: +0.0  Elbw: +0.0  Wrst: +0.0',
            font=('Consolas', 9),
            fg=self.MUTED_FG,
            bg=self.PANEL_BG,
        )
        self.joints_label.pack(anchor=tk.W, pady=(2, 0))

        self.gripper_label = tk.Label(
            telem,
            text='Gripper: 20.0 mm',
            font=('Consolas', 9),
            fg=self.MUTED_FG,
            bg=self.PANEL_BG,
        )
        self.gripper_label.pack(anchor=tk.W, pady=(2, 0))

        self.warn_label = tk.Label(
            telem,
            text='',
            font=('Segoe UI', 9, 'bold'),
            fg='#f85149',
            bg=self.PANEL_BG,
        )
        self.warn_label.pack(anchor=tk.W, pady=(4, 0))

    def _make_hold_btn(
        self, parent: tk.Widget, label: str, direction: tuple, width: int = 10
    ) -> tk.Button:
        btn = tk.Button(
            parent,
            text=label,
            width=width,
            height=2,
            font=('Segoe UI', 9, 'bold'),
            fg=self.TEXT_FG,
            bg=self.BTN_BG,
            activebackground=self.BTN_ACTIVE,
            activeforeground='#ffffff',
            relief=tk.RAISED,
            bd=1,
            cursor='hand2',
        )
        btn.bind('<ButtonPress-1>', lambda _e, d=direction: self._start_move(d))
        btn.bind('<ButtonRelease-1>', lambda _e: self._stop_move())
        return btn

    def _make_action_btn(
        self, parent: tk.Widget, label: str, bg: str, cmd_fn
    ) -> tk.Button:
        return tk.Button(
            parent,
            text=label,
            height=2,
            font=('Segoe UI', 10, 'bold'),
            fg='#ffffff',
            bg=bg,
            activebackground=self.BTN_ACTIVE,
            activeforeground='#ffffff',
            relief=tk.RAISED,
            bd=1,
            cursor='hand2',
            command=cmd_fn,
        )

    def _start_move(self, direction: tuple) -> None:
        if self.pulse_job is not None:
            self.root.after_cancel(self.pulse_job)
            self.pulse_job = None
        self.active_dir = direction
        self.press_time = time.monotonic()
        self._publish_active_vel()

    def _stop_move(self) -> None:
        if self.active_dir is None:
            return
        elapsed = time.monotonic() - self.press_time
        remaining = self.min_pulse_sec - elapsed
        if remaining > 0.0:
            self.pulse_job = self.root.after(int(remaining * 1000), self._clear_move)
        else:
            self._clear_move()

    def _clear_move(self) -> None:
        self.pulse_job = None
        self.active_dir = None
        self.node.send_vel(0.0, 0.0, 0.0)

    def _on_home(self) -> None:
        self._clear_move()
        self.node.send_cmd('home')

    def _on_stop(self) -> None:
        if self.pulse_job is not None:
            self.root.after_cancel(self.pulse_job)
            self.pulse_job = None
        self.active_dir = None
        self.node.send_vel(0.0, 0.0, 0.0)
        self.node.send_cmd('stop')

    def _publish_active_vel(self) -> None:
        if self.active_dir is not None:
            spd = float(self.speed_var.get())
            self.node.send_vel(
                self.active_dir[0] * spd,
                self.active_dir[1] * spd,
                self.active_dir[2] * spd,
            )

    def _schedule_vel_loop(self) -> None:
        self._publish_active_vel()
        self.root.after(50, self._schedule_vel_loop)

    def _schedule_telemetry_refresh(self) -> None:
        st = self.node.get_status()
        connected = st.get('connected', False)
        mode = st.get('mode', 'WAIT') if connected else 'WAIT SIM'
        warning = st.get('warning', '')
        xyz = st.get('ee_xyz', [0.0, 0.0, 0.0])
        joints = st.get('joints', [0.0, 0.0, 0.0, 0.0])
        grip_mm = float(st.get('gripper', 0.0)) * 2000.0  # total jaw opening (2 fingers)

        badge_colors = {
            'IDLE': '#238636',
            'CARTESIAN': '#1f6feb',
            'HOMING': '#8957e5',
            'INTERACTIVE': '#1f6feb',
            'ESTOP': '#da3633',
            'WAIT SIM': '#6e7681',
        }
        self.status_badge.config(
            text=mode,
            bg=badge_colors.get(mode, '#6e7681'),
        )
        self.ee_label.config(
            text=f'End-Effector:  X: {xyz[0]:+.3f} m   Y: {xyz[1]:+.3f} m   Z: {xyz[2]:+.3f} m'
        )
        j_deg = [math.degrees(j) for j in joints]
        self.joints_label.config(
            text=(
                f'Joints (deg):  Base: {j_deg[0]:+5.1f}  Shld: {j_deg[1]:+5.1f}  '
                f'Elbw: {j_deg[2]:+5.1f}  Wrst: {j_deg[3]:+5.1f}'
            )
        )
        grip_state = 'OPEN' if grip_mm > 35.0 else ('CLOSED' if grip_mm < 5.0 else 'PARTIAL')
        self.gripper_label.config(text=f'Gripper Jaw Opening: {grip_mm:.1f} mm ({grip_state})')
        self.warn_label.config(text=warning)

        self.root.after(100, self._schedule_telemetry_refresh)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGuiNode()

    stop_event = threading.Event()

    def spin_loop() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    app = RobotArmGuiApp(root, node)

    def on_close() -> None:
        app._clear_move()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_vel(0.0, 0.0, 0.0)
        stop_event.set()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
