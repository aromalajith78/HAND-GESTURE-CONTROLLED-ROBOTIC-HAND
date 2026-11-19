"""Analytical forward and inverse kinematics for the 4-DOF robotic arm."""

import math
from typing import Optional, Tuple
import numpy as np


class ArmKinematics:
    """Closed-form 4-DOF kinematics solver matching robot_arm_core.xacro."""

    Z_BASE = 0.140          # base_height (0.060) + shoulder_height (0.080)
    L1 = 0.240              # upper_arm_length
    L2 = 0.220              # forearm_length
    L3 = 0.140              # wrist_to_ee_length

    TH2_OFFSET = -0.350
    TH3_OFFSET = 1.750
    TH4_OFFSET = 0.17079632679

    ARM_JOINT_NAMES = [
        'base_joint',
        'shoulder_joint',
        'elbow_joint',
        'wrist_joint',
    ]
    GRIPPER_JOINT_NAMES = [
        'gripper_left_joint',
        'gripper_right_joint',
    ]

    JOINT_LIMITS = [
        (-2.9670, 2.9670),  # base_joint
        (-1.2200, 1.9200),  # shoulder_joint
        (-1.5500, 0.9000),  # elbow_joint
        (-2.4000, 2.4000),  # wrist_joint
    ]

    MAX_JOINT_VELOCITIES = np.array([2.0, 2.0, 2.2, 2.8], dtype=float)
    HOME_JOINTS = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

    GRIPPER_OPEN = 0.034
    GRIPPER_CLOSE = 0.000

    MIN_RADIUS = 0.095
    MIN_EE_Z = 0.026

    @classmethod
    def clamp_joints(cls, joints: np.ndarray) -> np.ndarray:
        """Clamp joint vector strictly within physical URDF limits."""
        out = np.array(joints, dtype=float)
        for i, (low, high) in enumerate(cls.JOINT_LIMITS):
            out[i] = float(np.clip(out[i], low, high))
        return out

    @classmethod
    def within_limits(cls, joints: np.ndarray, tol: float = 1e-6) -> bool:
        """Return True if all 4 arm joints lie within URDF limits."""
        for i, (low, high) in enumerate(cls.JOINT_LIMITS):
            if joints[i] < low - tol or joints[i] > high + tol:
                return False
        return True

    @classmethod
    def forward_kinematics(cls, joints: np.ndarray) -> Tuple[float, float, float, float]:
        """Compute end-effector (x, y, z, pitch) in base_link frame."""
        q1, q2, q3, q4 = [float(v) for v in joints]
        th2 = q2 + cls.TH2_OFFSET
        th23 = th2 + q3 + cls.TH3_OFFSET
        th234 = th23 + q4 + cls.TH4_OFFSET

        r = cls.L1 * math.sin(th2) + cls.L2 * math.sin(th23) + cls.L3 * math.sin(th234)
        z = cls.Z_BASE + cls.L1 * math.cos(th2) + cls.L2 * math.cos(th23) + cls.L3 * math.cos(th234)
        x = r * math.cos(q1)
        y = r * math.sin(q1)
        return x, y, z, th234

    @classmethod
    def _solve_planar_ik(
        cls, r: float, z: float, q1: float, pitch: float
    ) -> Optional[np.ndarray]:
        """Solve 3-DOF planar pitch-arm IK for fixed base yaw q1 and wrist pitch."""
        rw = r - cls.L3 * math.sin(pitch)
        zw = z - cls.Z_BASE - cls.L3 * math.cos(pitch)

        d_sq = rw * rw + zw * zw
        cos_th3 = (d_sq - cls.L1 * cls.L1 - cls.L2 * cls.L2) / (2.0 * cls.L1 * cls.L2)
        if cos_th3 < -0.996 or cos_th3 > 0.996:
            return None

        # Elbow-up branch keeps th3 > 0 so forearm bends cleanly over workspace
        th3 = math.acos(cos_th3)
        alpha = math.atan2(rw, zw)
        beta = math.atan2(cls.L2 * math.sin(th3), cls.L1 + cls.L2 * cos_th3)
        th2 = alpha - beta
        th4 = pitch - th2 - th3

        q2 = th2 - cls.TH2_OFFSET
        q3 = th3 - cls.TH3_OFFSET
        q4 = th4 - cls.TH4_OFFSET

        candidate = np.array([q1, q2, q3, q4], dtype=float)
        if not cls.within_limits(candidate):
            return None
        return cls.clamp_joints(candidate)

    @classmethod
    def inverse_kinematics(
        cls,
        x: float,
        y: float,
        z: float,
        q_current: Optional[np.ndarray] = None,
        preferred_pitch: float = math.pi / 2.0,
        max_step_diff: float = 0.50,
    ) -> Optional[np.ndarray]:
        """Solve analytical IK with soft wrist-pitch fallback and continuity check."""
        if z < cls.MIN_EE_Z:
            return None

        r = math.hypot(x, y)
        if r < cls.MIN_RADIUS:
            return None

        q1 = math.atan2(y, x)
        if q_current is not None:
            # Handle continuous yaw without +/-pi branch jump if within limits
            diff = q1 - float(q_current[0])
            while diff > math.pi:
                q1 -= 2.0 * math.pi
                diff = q1 - float(q_current[0])
            while diff < -math.pi:
                q1 += 2.0 * math.pi
                diff = q1 - float(q_current[0])

        q1_min, q1_max = cls.JOINT_LIMITS[0]
        if q1 < q1_min or q1 > q1_max:
            return None

        # Try preferred pitch first, then small symmetric deviations if near boundary
        pitch_offsets = [0.0]
        for step in range(1, 16):
            delta = step * 0.05
            pitch_offsets.append(delta)
            pitch_offsets.append(-delta)

        for delta in pitch_offsets:
            sol = cls._solve_planar_ik(r, z, q1, preferred_pitch + delta)
            if sol is None:
                continue
            if q_current is not None and np.max(np.abs(sol - q_current)) > max_step_diff:
                continue
            return sol

        return None
