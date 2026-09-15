"""KinematicsProvider abstract interface and PyBullet kinematic modeling services.

Decouples forward kinematics, spatial Jacobians, and raw inverse kinematics
from specific simulation physics engines.
"""

from abc import ABC, abstractmethod
import threading
from typing import List, Optional, Sequence, Tuple
import numpy as np
import pybullet as p

from utils.logger import get_logger

logger = get_logger("Robotics.KinematicsProvider")


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


class PyBulletKinematicsProvider(KinematicsProvider):
    """PyBullet implementation of KinematicsProvider.
    
    Provides state-preserving Forward Kinematics, geometric spatial Jacobians,
    and raw numerical Inverse Kinematics using an active PyBullet simulation instance.
    """

    def __init__(
        self,
        physics_client_id: int,
        robot_body_id: int,
        arm_joint_indices: Sequence[int],
        end_effector_link_index: int,
    ):
        self.client_id = int(physics_client_id)
        self.robot_id = int(robot_body_id)
        self.arm_joint_indices = tuple(int(j) for j in arm_joint_indices)
        self.ee_link_index = int(end_effector_link_index)
        self.num_arm_joints = len(self.arm_joint_indices)
        self._lock = threading.RLock()
        self._movable_joint_indices = self._discover_movable_joints()

    def _discover_movable_joints(self) -> List[int]:
        """Discovers all movable (revolute + prismatic) joints for PyBullet calculateJacobian."""
        movable = []
        try:
            num_total_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
            for i in range(num_total_joints):
                info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
                if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                    movable.append(i)
        except Exception as e:
            logger.debug(f"Could not inspect joints during init: {e}")
            movable = list(self.arm_joint_indices)
        return movable

    def compute_fk(
        self, joint_positions: Sequence[float]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Calculates end-effector (position [x,y,z], quaternion [x,y,z,w]) for candidate joint configuration q.

        Temporarily applies candidate joint angles in PyBullet, queries forward kinematics via
        getLinkState(computeForwardKinematics=True), and unconditionally restores original joint states (q, dq).
        Never calls stepSimulation. Thread-safe via provider RLock.

        Args:
            joint_positions: Joint angles for controllable arm joints (rad).

        Returns:
            (position, quaternion_xyzw) as (np.ndarray shape (3,), np.ndarray shape (4,)).

        Raises:
            ValueError: If input length does not match num_arm_joints or contains non-finite numbers.
        """
        if len(joint_positions) != self.num_arm_joints:
            raise ValueError(
                f"Expected {self.num_arm_joints} joint positions, got {len(joint_positions)}"
            )

        q_arr = np.asarray(joint_positions, dtype=np.float64)
        if not np.all(np.isfinite(q_arr)):
            raise ValueError("Joint positions contain non-finite values (NaN or Inf)")

        with self._lock:
            saved_states = p.getJointStates(
                self.robot_id, list(self.arm_joint_indices), physicsClientId=self.client_id
            )
            try:
                for j_idx, angle in zip(self.arm_joint_indices, q_arr):
                    p.resetJointState(
                        self.robot_id,
                        int(j_idx),
                        targetValue=float(angle),
                        targetVelocity=0.0,
                        physicsClientId=self.client_id,
                    )
                link_state = p.getLinkState(
                    self.robot_id,
                    int(self.ee_link_index),
                    computeForwardKinematics=True,
                    physicsClientId=self.client_id,
                )
                fk_pos = np.array(link_state[0], dtype=np.float64)
                fk_orn = np.array(link_state[1], dtype=np.float64)
                return fk_pos, fk_orn
            finally:
                for j_idx, state in zip(self.arm_joint_indices, saved_states):
                    p.resetJointState(
                        self.robot_id,
                        int(j_idx),
                        targetValue=float(state[0]),
                        targetVelocity=float(state[1]),
                        physicsClientId=self.client_id,
                    )

    def compute_jacobian(
        self, joint_positions: Sequence[float]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Calculates spatial geometric Jacobian for candidate joint configuration q.

        Args:
            joint_positions: Current joint angles for arm joints (rad).

        Returns:
            (J_linear, J_angular, J_full) where J_full has shape (6, n).

        Raises:
            ValueError: If input length does not match num_arm_joints or contains non-finite numbers.
        """
        if len(joint_positions) != self.num_arm_joints:
            raise ValueError(
                f"Expected {self.num_arm_joints} joint positions, got {len(joint_positions)}"
            )

        q_arr = np.asarray(joint_positions, dtype=np.float64)
        if not np.all(np.isfinite(q_arr)):
            raise ValueError("Joint positions contain non-finite values (NaN or Inf)")

        with self._lock:
            movable = self._movable_joint_indices or self._discover_movable_joints()
            num_movable = len(movable)

            if len(joint_positions) < num_movable:
                pad_len = num_movable - len(joint_positions)
                q_full = list(joint_positions) + [0.0] * pad_len
                zeros_full = [0.0] * num_movable
            else:
                q_full = list(joint_positions[:num_movable])
                zeros_full = [0.0] * num_movable

            j_trans, j_rot = p.calculateJacobian(
                bodyUniqueId=self.robot_id,
                linkIndex=self.ee_link_index,
                localPosition=[0.0, 0.0, 0.0],
                objPositions=q_full,
                objVelocities=zeros_full,
                objAccelerations=zeros_full,
                physicsClientId=self.client_id,
            )

            j_linear = np.array(j_trans, dtype=np.float64)[:, : self.num_arm_joints]
            j_angular = np.array(j_rot, dtype=np.float64)[:, : self.num_arm_joints]
            j_full = np.vstack([j_linear, j_angular])

            return j_linear, j_angular, j_full

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
        """Invokes model engine numerical IK solver to calculate candidate joint angles."""
        raise NotImplementedError("solve_ik_raw will be implemented in Task 6")
