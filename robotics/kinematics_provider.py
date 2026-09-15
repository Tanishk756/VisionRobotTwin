"""KinematicsProvider abstract interface and robot kinematic modeling services.

Decouples forward kinematics, spatial Jacobians, and raw inverse kinematics
from specific simulation physics engines (PyBullet, Pinocchio, KDL, etc.).
"""

from abc import ABC, abstractmethod
from typing import Optional, Sequence, Tuple
import numpy as np


class KinematicsProvider(ABC):
    """Abstract interface for robot kinematic modeling and spatial computations."""

    @abstractmethod
    def compute_fk(
        self, joint_positions: Sequence[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Calculates end-effector (position [x,y,z], quaternion [x,y,z,w]) for candidate joint configuration q.

        Must be strictly state-preserving for active physics simulation instances.

        Args:
            joint_positions: Joint angles for controllable arm joints (rad).

        Returns:
            Tuple of (position np.ndarray shape (3,), quaternion_xyzw np.ndarray shape (4,)).
        """
        pass

    @abstractmethod
    def compute_jacobian(
        self, joint_positions: Sequence[float]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Calculates spatial geometric Jacobian for candidate joint configuration q.

        Args:
            joint_positions: Joint angles for controllable arm joints (rad).

        Returns:
            Tuple of (J_linear (3,n), J_angular (3,n), J_full (6,n)).
        """
        pass

    @abstractmethod
    def solve_ik_raw(
        self,
        target_position: Sequence[float],
        target_orientation: Optional[Sequence[float]] = None,
        lower_limits: Optional[Sequence[float]] = None,
        upper_limits: Optional[Sequence[float]] = None,
        joint_ranges: Optional[Sequence[float]] = None,
        rest_poses: Optional[Sequence[float]] = None,
        joint_damping: Optional[Sequence[float]] = None,
        max_iterations: int = 100,
        residual_threshold: float = 1e-4,
    ) -> Tuple[float, ...]:
        """Invokes model engine numerical IK solver to calculate candidate joint angles.

        Args:
            target_position: Desired [x, y, z] target in base frame.
            target_orientation: Optional unit quaternion [x, y, z, w].
            lower_limits: Optional per-joint lower limits.
            upper_limits: Optional per-joint upper limits.
            joint_ranges: Optional per-joint ranges.
            rest_poses: Optional per-joint rest poses.
            joint_damping: Optional per-joint damping factors.
            max_iterations: Maximum solver iterations.
            residual_threshold: Convergence threshold.

        Returns:
            Tuple of candidate joint positions (rad) matching controllable arm joints.
        """
        pass
