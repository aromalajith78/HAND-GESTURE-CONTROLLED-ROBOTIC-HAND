#!/usr/bin/env python3
"""Webcam hand-gesture teleoperation node using OpenCV and MediaPipe Hands."""

import json
import math
import time
from typing import Optional, Tuple

import cv2
from geometry_msgs.msg import Point, Twist
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class CameraHandControlNode(Node):
    """Maps webcam hand position and pinch gestures to smooth 4-DOF arm and gripper control."""

    EMA_ALPHA = 0.25
    PINCH_CLOSE_RATIO = 0.36
    PINCH_OPEN_RATIO = 0.52

    def __init__(self) -> None:
        super().__init__('camera_hand_control')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('mirror_image', True)
        self.declare_parameter('show_window', True)

        self.camera_index = int(self.get_parameter('camera_index').value)
        self.mirror_image = bool(self.get_parameter('mirror_image').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        self.target_pub = self.create_publisher(Point, '/arm_control/cartesian_target', 10)
        self.vel_pub = self.create_publisher(Twist, '/arm_control/cartesian_vel', 10)
        self.cmd_pub = self.create_publisher(String, '/arm_control/command', 10)
        self.status_sub = self.create_subscription(
            String, '/arm_control/status', self._status_cb, 10
        )

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.70,
            min_tracking_confidence=0.70,
        )

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Unable to open camera index {self.camera_index}. Check /dev/video* permissions.'
            )
            raise RuntimeError(f'Failed to open webcam index {self.camera_index}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.tracking_mode = 'MIRROR'  # MIRROR or JOYSTICK
        self.paused = False
        self.neutral_scale = 0.16
        self.smooth_cx: Optional[float] = None
        self.smooth_cy: Optional[float] = None
        self.smooth_scale: Optional[float] = None
        self.gripper_closed: Optional[bool] = None
        self.latest_status = {}
        self.last_goal_xyz = (0.275, 0.000, 0.403)
        self.should_exit = False

        self.timer = self.create_timer(0.033, self._process_frame)  # ~30 Hz
        self.get_logger().info(
            'Camera Hand Control started. Controls: SPACE=Pause/Stop, H=Home, C=Calibrate Depth, M=Mode, Q=Quit'
        )

    def _status_cb(self, msg: String) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _publish_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def _publish_vel(self, vx: float, vy: float, vz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def _publish_target(self, x: float, y: float, z: float) -> None:
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.target_pub.publish(msg)

    def map_hand_to_cartesian(
        self, cx: float, cy: float, scale: float
    ) -> Tuple[float, float, float]:
        """Map smoothed normalized hand coords (0..1) and scale to robot base_link XYZ."""
        x = 0.275 + float(np.clip((scale - self.neutral_scale) * 1.45, -0.14, 0.20))
        y = float(np.clip((0.50 - cx) * 0.56, -0.26, 0.26))
        z = float(np.clip(0.04 + (0.82 - cy) * 0.65, 0.03, 0.54))
        return x, y, z

    def map_hand_to_velocity(
        self, cx: float, cy: float, scale: float
    ) -> Tuple[float, float, float]:
        """Map hand offset from frame center deadzone to Cartesian velocity (m/s)."""
        def deadzone(val: float, half_dz: float, max_span: float, max_spd: float) -> float:
            if abs(val) <= half_dz:
                return 0.0
            norm = min(1.0, (abs(val) - half_dz) / max(1e-3, max_span - half_dz))
            return math.copysign(norm * max_spd, val)

        vx = deadzone(scale - self.neutral_scale, 0.03, 0.11, 0.14)
        vy = deadzone(0.50 - cx, 0.09, 0.42, 0.16)
        vz = deadzone(0.50 - cy, 0.09, 0.42, 0.16)
        return vx, vy, vz

    def _process_frame(self) -> None:
        if self.should_exit:
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Camera frame read failed.', throttle_duration_sec=2.0)
            return

        if self.mirror_image:
            frame = cv2.flip(frame, 1)

        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        hand_detected = False
        pinch_ratio = 1.0

        if results.multi_hand_landmarks:
            hand_detected = True
            lm = results.multi_hand_landmarks[0]
            self.mp_draw.draw_landmarks(frame, lm, self.mp_hands.HAND_CONNECTIONS)

            wrist = lm.landmark[self.mp_hands.HandLandmark.WRIST]
            mcp = lm.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
            thumb_tip = lm.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            index_tip = lm.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

            raw_cx = 0.5 * (wrist.x + mcp.x)
            raw_cy = 0.5 * (wrist.y + mcp.y)
            raw_scale = max(0.04, math.hypot(mcp.x - wrist.x, mcp.y - wrist.y))

            if self.smooth_cx is None:
                self.smooth_cx, self.smooth_cy, self.smooth_scale = raw_cx, raw_cy, raw_scale
            else:
                a = self.EMA_ALPHA
                self.smooth_cx = a * raw_cx + (1.0 - a) * self.smooth_cx
                self.smooth_cy = a * raw_cy + (1.0 - a) * self.smooth_cy
                self.smooth_scale = a * raw_scale + (1.0 - a) * self.smooth_scale

            pinch_dist = math.hypot(index_tip.x - thumb_tip.x, index_tip.y - thumb_tip.y)
            pinch_ratio = pinch_dist / max(self.smooth_scale, 0.05)

            # Draw palm center & pinch line
            px, py = int(self.smooth_cx * w), int(self.smooth_cy * h)
            cv2.circle(frame, (px, py), 8, (0, 220, 120), -1)
            tx, ty = int(thumb_tip.x * w), int(thumb_tip.y * h)
            ix, iy = int(index_tip.x * w), int(index_tip.y * h)
            pinch_color = (0, 70, 255) if pinch_ratio < self.PINCH_CLOSE_RATIO else (0, 255, 160)
            cv2.line(frame, (tx, ty), (ix, iy), pinch_color, 3)

            if not self.paused:
                if self.tracking_mode == 'MIRROR':
                    gx, gy, gz = self.map_hand_to_cartesian(
                        self.smooth_cx, self.smooth_cy, self.smooth_scale
                    )
                    self.last_goal_xyz = (gx, gy, gz)
                    self._publish_target(gx, gy, gz)
                else:
                    vx, vy, vz = self.map_hand_to_velocity(
                        self.smooth_cx, self.smooth_cy, self.smooth_scale
                    )
                    self._publish_vel(vx, vy, vz)

                # Gripper pinch hysteresis
                if pinch_ratio < self.PINCH_CLOSE_RATIO and self.gripper_closed is not True:
                    self.gripper_closed = True
                    self._publish_cmd('gripper_close')
                elif pinch_ratio > self.PINCH_OPEN_RATIO and self.gripper_closed is not False:
                    self.gripper_closed = False
                    self._publish_cmd('gripper_open')
        else:
            if not self.paused and self.tracking_mode == 'JOYSTICK':
                self._publish_vel(0.0, 0.0, 0.0)

        if self.show_window:
            self._draw_hud(frame, hand_detected, pinch_ratio)
            cv2.imshow('Camera Hand Control - 4-DOF Robot Arm', frame)
            key = cv2.waitKey(1) & 0xFF
            self._handle_key(key)

    def _draw_hud(self, frame: np.ndarray, hand_detected: bool, pinch_ratio: float) -> None:
        h, w, _ = frame.shape
        if self.tracking_mode == 'JOYSTICK':
            cx0, cy0 = w // 2, h // 2
            dx, dy = int(0.09 * w), int(0.09 * h)
            cv2.rectangle(frame, (cx0 - dx, cy0 - dy), (cx0 + dx, cy0 + dy), (180, 180, 180), 1)

        cv2.rectangle(frame, (0, 0), (w, 88), (22, 27, 34), -1)
        state_txt = 'PAUSED (STOP)' if self.paused else (
            'TRACKING' if hand_detected else 'NO HAND - HOLDING'
        )
        state_col = (0, 80, 255) if self.paused else (
            (80, 230, 110) if hand_detected else (0, 190, 255)
        )
        grip_txt = 'CLOSED' if self.gripper_closed else 'OPEN'
        ee = self.latest_status.get('ee_xyz', list(self.last_goal_xyz))

        cv2.putText(
            frame,
            f'Mode: {self.tracking_mode} | {state_txt} | Gripper: {grip_txt}',
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            state_col,
            2,
        )
        cv2.putText(
            frame,
            f'EE XYZ: ({ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f}) m | Pinch: {pinch_ratio:.2f}',
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (235, 240, 245),
            1,
        )
        cv2.putText(
            frame,
            'SPACE: Pause/Stop | H: Home | C: Calibrate Depth | M: Mode | Q: Quit',
            (12, 76),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (160, 175, 190),
            1,
        )

    def _handle_key(self, key: int) -> None:
        if key in (ord('q'), ord('Q'), 27):  # q or ESC
            self.should_exit = True
            return
        if key == ord(' '):
            self.paused = not self.paused
            if self.paused:
                self._publish_vel(0.0, 0.0, 0.0)
                self._publish_cmd('stop')
        elif key in (ord('h'), ord('H')):
            self.paused = False
            self._publish_cmd('home')
        elif key in (ord('c'), ord('C')):
            if self.smooth_scale is not None:
                self.neutral_scale = float(self.smooth_scale)
                self.get_logger().info(f'Calibrated neutral depth scale: {self.neutral_scale:.3f}')
        elif key in (ord('m'), ord('M')):
            self.tracking_mode = 'JOYSTICK' if self.tracking_mode == 'MIRROR' else 'MIRROR'
            self._publish_vel(0.0, 0.0, 0.0)

    def cleanup(self) -> None:
        try:
            self._publish_vel(0.0, 0.0, 0.0)
        except Exception:
            pass
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        if hasattr(self, 'hands') and self.hands is not None:
            self.hands.close()
        if self.show_window:
            cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = CameraHandControlNode()
    except RuntimeError:
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
