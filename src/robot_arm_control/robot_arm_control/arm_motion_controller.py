#!/usr/bin/env python3
"""Central 4-DOF arm & gripper motion controller with analytical IK and RViz InteractiveMarker."""

import json
import math
import time
from typing import Optional

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
from interactive_markers import InteractiveMarkerServer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from robot_arm_control.kinematics import ArmKinematics


class ArmMotionController(Node):
    """Coordinates Cartesian IK, smooth trajectory generation, gripper, and interactive markers."""

    CONTROL_DT = 0.02          # 50 Hz control loop
    TRAJ_LOOKAHEAD_NS = 35_000_000  # 35 ms trajectory segment lookahead
    VEL_WATCHDOG_SEC = 0.25    # Auto-stop if velocity stream stops
    HOME_DURATION_SEC = 1.50
    STEP_SPEED = 0.14          # m/s for discrete step commands

    def __init__(self) -> None:
        super().__init__('arm_motion_controller')

        self.joint_state_received = False
        self.initial_homing_done = False
        self.actual_joints = ArmKinematics.HOME_JOINTS.copy()
        self.target_joints = ArmKinematics.HOME_JOINTS.copy()
        x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
        self.target_xyz = np.array([x, y, z], dtype=float)
        self.gripper_pos = 0.020

        self.mode = 'IDLE'  # IDLE, CARTESIAN, TARGET, HOMING, INTERACTIVE, ESTOP
        self.limit_warning = ''

        self.cmd_vel = np.zeros(3, dtype=float)
        self.last_vel_time = 0.0
        self.step_end_time = 0.0

        self.cartesian_goal_xyz: Optional[np.ndarray] = None
        self.last_target_time = 0.0

        self.home_start_joints = ArmKinematics.HOME_JOINTS.copy()
        self.home_start_time = 0.0

        self.dragging_marker = False
        self.marker_goal_xyz: Optional[np.ndarray] = None
        self.last_marker_xyz = self.target_xyz.copy()

        # Publishers
        self.arm_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.gripper_traj_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )
        self.ee_pose_pub = self.create_publisher(
            PoseStamped, '/arm_control/ee_pose', 10
        )
        self.status_pub = self.create_publisher(
            String, '/arm_control/status', 10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )
        self.vel_sub = self.create_subscription(
            Twist, '/arm_control/cartesian_vel', self._cartesian_vel_cb, 10
        )
        self.target_sub = self.create_subscription(
            Point, '/arm_control/cartesian_target', self._cartesian_target_cb, 10
        )
        self.cmd_sub = self.create_subscription(
            String, '/arm_control/command', self._command_cb, 10
        )

        # RViz2 3-axis Interactive Marker around end-effector
        self.marker_server = InteractiveMarkerServer(self, 'arm_interactive_marker')
        self._init_interactive_marker()

        # Timers (wall-clock for responsive teleop even if sim pauses briefly)
        self.control_timer = self.create_timer(self.CONTROL_DT, self._control_loop)
        self.status_timer = self.create_timer(0.10, self._publish_status)

        self.get_logger().info('ArmMotionController initialized and waiting for /joint_states...')

    def _init_interactive_marker(self) -> None:
        """Create a 3-DOF XYZ interactive gizmo at the end-effector pose."""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_link'
        int_marker.name = 'ee_cartesian_target'
        int_marker.description = 'Drag End-Effector (X / Y / Z)'
        int_marker.scale = 0.14
        int_marker.pose.position.x = float(self.target_xyz[0])
        int_marker.pose.position.y = float(self.target_xyz[1])
        int_marker.pose.position.z = float(self.target_xyz[2])
        int_marker.pose.orientation.w = 1.0

        # Small central sphere visual
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.028
        sphere_marker.scale.y = 0.028
        sphere_marker.scale.z = 0.028
        sphere_marker.color.r = 0.15
        sphere_marker.color.g = 0.85
        sphere_marker.color.b = 0.35
        sphere_marker.color.a = 0.75

        center_ctrl = InteractiveMarkerControl()
        center_ctrl.always_visible = True
        center_ctrl.markers.append(sphere_marker)
        int_marker.controls.append(center_ctrl)

        # X, Y, Z move axes (MOVE_AXIS translates along local +X of control orientation)
        s = math.sqrt(0.5)
        axes = [
            ('move_x', 0.0, 0.0, 0.0, 1.0),
            ('move_y', 0.0, 0.0, s, s),
            ('move_z', 0.0, -s, 0.0, s),
        ]
        for name, qx, qy, qz, qw in axes:
            ctrl = InteractiveMarkerControl()
            ctrl.name = name
            ctrl.orientation.x = qx
            ctrl.orientation.y = qy
            ctrl.orientation.z = qz
            ctrl.orientation.w = qw
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(ctrl)

        self.marker_server.insert(int_marker, feedback_callback=self._marker_feedback_cb)
        self.marker_server.applyChanges()

    def _sync_interactive_marker(self, force: bool = False) -> None:
        """Keep the RViz2 interactive marker aligned with target_xyz."""
        if self.dragging_marker and not force:
            return
        if not force and np.linalg.norm(self.target_xyz - self.last_marker_xyz) < 1e-4:
            return
        pose = Pose()
        pose.position.x = float(self.target_xyz[0])
        pose.position.y = float(self.target_xyz[1])
        pose.position.z = float(self.target_xyz[2])
        pose.orientation.w = 1.0
        self.marker_server.setPose('ee_cartesian_target', pose)
        self.marker_server.applyChanges()
        self.last_marker_xyz = self.target_xyz.copy()

    def _marker_feedback_cb(self, feedback: InteractiveMarkerFeedback) -> None:
        """Handle interactive 3D dragging in RViz2."""
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging_marker = True
            self.cmd_vel[:] = 0.0
            self.mode = 'INTERACTIVE'
            self.limit_warning = ''
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.dragging_marker = True
            self.mode = 'INTERACTIVE'
            self.marker_goal_xyz = np.array([
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z,
            ], dtype=float)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.dragging_marker = False
            self.marker_goal_xyz = None
            if self.mode == 'INTERACTIVE':
                self.mode = 'IDLE'
            self._sync_interactive_marker(force=True)

    def _joint_state_cb(self, msg: JointState) -> None:
        """Update actual joint positions from joint_state_broadcaster."""
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        if not all(j in name_to_idx for j in ArmKinematics.ARM_JOINT_NAMES):
            return

        joints = np.array(
            [msg.position[name_to_idx[j]] for j in ArmKinematics.ARM_JOINT_NAMES],
            dtype=float,
        )
        self.actual_joints = joints

        if 'gripper_left_joint' in name_to_idx:
            self.gripper_pos = float(msg.position[name_to_idx['gripper_left_joint']])

        if not self.joint_state_received:
            self.joint_state_received = True
            self.target_joints = ArmKinematics.clamp_joints(joints)
            x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
            self.target_xyz = np.array([x, y, z], dtype=float)
            self._sync_interactive_marker(force=True)
            self.get_logger().info(
                f'Synced with /joint_states. Initial EE pose: ({x:.3f}, {y:.3f}, {z:.3f})'
            )

    def _cartesian_vel_cb(self, msg: Twist) -> None:
        """Receive continuous Cartesian velocity commands (m/s) in base_link frame."""
        vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z], dtype=float)
        now = time.monotonic()

        if np.linalg.norm(vel) < 1e-6:
            self.cmd_vel[:] = 0.0
            self.step_end_time = 0.0
            if self.mode == 'CARTESIAN':
                self.mode = 'IDLE'
            return

        # New intentional movement clears ESTOP/HOMING smoothly
        if self.mode in ('IDLE', 'ESTOP', 'HOMING', 'INTERACTIVE', 'TARGET'):
            if self.mode == 'ESTOP' and self.joint_state_received:
                self.target_joints = ArmKinematics.clamp_joints(self.actual_joints)
                x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
                self.target_xyz = np.array([x, y, z], dtype=float)
            self.mode = 'CARTESIAN'

        self.cmd_vel = vel
        self.last_vel_time = now
        self.step_end_time = 0.0

    def _cartesian_target_cb(self, msg: Point) -> None:
        """Receive absolute Cartesian goal coordinates (e.g., from camera hand tracking)."""
        now = time.monotonic()
        if self.mode == 'ESTOP' and self.joint_state_received:
            self.target_joints = ArmKinematics.clamp_joints(self.actual_joints)
            x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
            self.target_xyz = np.array([x, y, z], dtype=float)
        self.cmd_vel[:] = 0.0
        self.step_end_time = 0.0
        self.cartesian_goal_xyz = np.array(
            [msg.x, msg.y, max(ArmKinematics.MIN_EE_Z, float(msg.z))], dtype=float
        )
        self.last_target_time = now
        self.mode = 'TARGET'

    def _command_cb(self, msg: String) -> None:
        """Handle discrete commands: home, stop, gripper_open, gripper_close, step_*."""
        cmd = msg.data.strip().lower()
        now = time.monotonic()

        if cmd == 'stop':
            self._trigger_estop()
            return

        if cmd == 'home':
            self.cmd_vel[:] = 0.0
            self.dragging_marker = False
            self.marker_goal_xyz = None
            self.limit_warning = ''
            self.home_start_joints = (
                self.actual_joints.copy() if self.joint_state_received else self.target_joints.copy()
            )
            self.home_start_time = now
            self.home_duration_active = self.HOME_DURATION_SEC
            self.mode = 'HOMING'
            return

        if cmd == 'gripper_open':
            if self.mode == 'ESTOP':
                self.mode = 'IDLE'
            self.limit_warning = ''
            self._send_gripper_trajectory(ArmKinematics.GRIPPER_OPEN, duration_sec=0.50)
            return

        if cmd == 'gripper_close':
            if self.mode == 'ESTOP':
                self.mode = 'IDLE'
            self.limit_warning = ''
            self._send_gripper_trajectory(ArmKinematics.GRIPPER_CLOSE, duration_sec=0.50)
            return

        step_map = {
            'step_up': np.array([0.0, 0.0, self.STEP_SPEED]),
            'step_down': np.array([0.0, 0.0, -self.STEP_SPEED]),
            'step_left': np.array([0.0, self.STEP_SPEED, 0.0]),
            'step_right': np.array([0.0, -self.STEP_SPEED, 0.0]),
            'step_forward': np.array([self.STEP_SPEED, 0.0, 0.0]),
            'step_backward': np.array([-self.STEP_SPEED, 0.0, 0.0]),
        }
        if cmd in step_map:
            self.cmd_vel = step_map[cmd]
            self.last_vel_time = now
            self.step_end_time = now + 0.18
            self.mode = 'CARTESIAN'

    def _trigger_estop(self) -> None:
        """Immediately halt all arm and gripper trajectories at current joint states."""
        self.cmd_vel[:] = 0.0
        self.step_end_time = 0.0
        self.cartesian_goal_xyz = None
        self.dragging_marker = False
        self.marker_goal_xyz = None
        self.mode = 'ESTOP'
        self.limit_warning = 'EMERGENCY STOP ACTIVE'

        if self.joint_state_received:
            self.target_joints = ArmKinematics.clamp_joints(self.actual_joints)
            x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
            self.target_xyz = np.array([x, y, z], dtype=float)
            self._send_arm_trajectory(self.target_joints, duration_ns=50_000_000)
            self._send_gripper_trajectory(self.gripper_pos, duration_sec=0.05)
            self._sync_interactive_marker(force=True)

    def _control_loop(self) -> None:
        """50 Hz trajectory interpolation and safety enforcement loop."""
        if not self.joint_state_received:
            return

        now = time.monotonic()

        # Once arm_controller subscribes after spawn, smoothly recover any startup droop to HOME
        if not self.initial_homing_done:
            if (
                self.arm_traj_pub.get_subscription_count() > 0
                and self.gripper_traj_pub.get_subscription_count() > 0
            ):
                self.initial_homing_done = True
                self.home_start_joints = self.actual_joints.copy()
                self.home_start_time = now
                self.home_duration_active = 0.60
                self.mode = 'HOMING'
                self._send_gripper_trajectory(0.020, duration_sec=0.40)
            return

        if self.mode == 'ESTOP' or self.mode == 'IDLE':
            return

        if self.mode == 'HOMING':
            duration = getattr(self, 'home_duration_active', self.HOME_DURATION_SEC)
            elapsed = now - self.home_start_time
            progress = min(1.0, max(0.0, elapsed / duration))
            smooth_alpha = 0.5 * (1.0 - math.cos(math.pi * progress))
            next_joints = (
                (1.0 - smooth_alpha) * self.home_start_joints
                + smooth_alpha * ArmKinematics.HOME_JOINTS
            )
            self.target_joints = ArmKinematics.clamp_joints(next_joints)
            x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
            self.target_xyz = np.array([x, y, z], dtype=float)
            self._send_arm_trajectory(self.target_joints, duration_ns=self.TRAJ_LOOKAHEAD_NS)
            self._sync_interactive_marker()

            if progress >= 1.0:
                self.home_duration_active = self.HOME_DURATION_SEC
                self.mode = 'IDLE'
                self.limit_warning = ''
            return

        if self.mode == 'CARTESIAN':
            if self.step_end_time > 0.0 and now > self.step_end_time:
                self.cmd_vel[:] = 0.0
                self.step_end_time = 0.0
                self.mode = 'IDLE'
                return
            if self.step_end_time == 0.0 and (now - self.last_vel_time) > self.VEL_WATCHDOG_SEC:
                self.cmd_vel[:] = 0.0
                self.mode = 'IDLE'
                return

            candidate_xyz = self.target_xyz + self.cmd_vel * self.CONTROL_DT
            self._move_toward_xyz(candidate_xyz)
            self._sync_interactive_marker()
            return

        if self.mode == 'TARGET' and self.cartesian_goal_xyz is not None:
            if (now - self.last_target_time) > 0.40:
                self.cartesian_goal_xyz = None
                self.mode = 'IDLE'
                return

            delta = self.cartesian_goal_xyz - self.target_xyz
            dist = float(np.linalg.norm(delta))
            if dist > 1e-4:
                max_step_dist = 0.25 * self.CONTROL_DT
                step = delta if dist <= max_step_dist else (delta / dist) * max_step_dist
                candidate_xyz = self.target_xyz + step
                if not self._move_toward_xyz(candidate_xyz):
                    # Slide cleanly along reachable individual axes when touching boundary
                    for axis in (1, 2, 0):
                        if abs(step[axis]) > 1e-5:
                            proj = self.target_xyz.copy()
                            proj[axis] += step[axis]
                            if self._move_toward_xyz(proj):
                                break
                self._sync_interactive_marker()
            return

        if self.mode == 'INTERACTIVE' and self.marker_goal_xyz is not None:
            delta = self.marker_goal_xyz - self.target_xyz
            dist = float(np.linalg.norm(delta))
            max_step_dist = 0.22 * self.CONTROL_DT
            if dist > max_step_dist:
                candidate_xyz = self.target_xyz + (delta / dist) * max_step_dist
            else:
                candidate_xyz = self.marker_goal_xyz.copy()
            if dist > 1e-4:
                self._move_toward_xyz(candidate_xyz)

    def _move_toward_xyz(self, candidate_xyz: np.ndarray) -> bool:
        """Solve IK for candidate_xyz, enforce joint velocity & limits, and publish trajectory."""
        ik_joints = ArmKinematics.inverse_kinematics(
            float(candidate_xyz[0]),
            float(candidate_xyz[1]),
            float(candidate_xyz[2]),
            q_current=self.target_joints,
        )
        if ik_joints is None:
            self.limit_warning = 'Workspace / Joint Limit Reached'
            return False

        self.limit_warning = ''
        max_dq = ArmKinematics.MAX_JOINT_VELOCITIES * self.CONTROL_DT
        dq = np.clip(ik_joints - self.target_joints, -max_dq, max_dq)
        clipped = not np.allclose(dq, ik_joints - self.target_joints, atol=1e-5)

        self.target_joints = ArmKinematics.clamp_joints(self.target_joints + dq)
        if clipped:
            x, y, z, _ = ArmKinematics.forward_kinematics(self.target_joints)
            self.target_xyz = np.array([x, y, z], dtype=float)
        else:
            self.target_xyz = candidate_xyz.copy()

        self._send_arm_trajectory(self.target_joints, duration_ns=self.TRAJ_LOOKAHEAD_NS)
        return True

    def _send_arm_trajectory(self, joints: np.ndarray, duration_ns: int) -> None:
        """Publish a JointTrajectory setpoint to arm_controller."""
        msg = JointTrajectory()
        msg.joint_names = list(ArmKinematics.ARM_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in ArmKinematics.clamp_joints(joints)]
        pt.time_from_start = Duration(
            sec=duration_ns // 1_000_000_000,
            nanosec=duration_ns % 1_000_000_000,
        )
        msg.points.append(pt)
        self.arm_traj_pub.publish(msg)

    def _send_gripper_trajectory(self, finger_pos: float, duration_sec: float) -> None:
        """Publish symmetric opening/closing trajectory to gripper_controller."""
        clamped = float(np.clip(finger_pos, ArmKinematics.GRIPPER_CLOSE, ArmKinematics.GRIPPER_OPEN))
        msg = JointTrajectory()
        msg.joint_names = list(ArmKinematics.GRIPPER_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [clamped, clamped]
        ns_total = int(max(0.02, duration_sec) * 1_000_000_000)
        pt.time_from_start = Duration(
            sec=ns_total // 1_000_000_000,
            nanosec=ns_total % 1_000_000_000,
        )
        msg.points.append(pt)
        self.gripper_traj_pub.publish(msg)

    def _publish_status(self) -> None:
        """Publish live end-effector pose and JSON telemetry status."""
        if self.joint_state_received:
            x, y, z, pitch = ArmKinematics.forward_kinematics(self.actual_joints)
        else:
            x, y, z, pitch = ArmKinematics.forward_kinematics(self.target_joints)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        # Orientation pitched forward by pitch from +Z around +Y
        half = 0.5 * pitch
        pose_msg.pose.orientation.y = math.sin(half)
        pose_msg.pose.orientation.w = math.cos(half)
        self.ee_pose_pub.publish(pose_msg)

        status_payload = {
            'connected': self.joint_state_received,
            'mode': self.mode,
            'warning': self.limit_warning,
            'ee_xyz': [round(float(x), 4), round(float(y), 4), round(float(z), 4)],
            'joints': [round(float(j), 4) for j in self.actual_joints],
            'gripper': round(float(self.gripper_pos), 4),
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload)
        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
