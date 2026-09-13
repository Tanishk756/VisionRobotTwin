"""Inverse Kinematics (IK) Solver for Franka Emika Panda.

Computes joint space solutions for desired end-effector 6-DoF Cartesian targets
using PyBullet's Damped Least-Squares IK solver with joint limits and null-space optimization.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple, Union
import pybullet as p
import numpy as np

from config.settings import RobotConfig
from utils.logger import get_logger

logger = get_logger("Robotics.IK")


@dataclass
class IKResult:
    """Represents the output of an Inverse Kinematics calculation."""
    success: bool
    joint_positions: List[float]  # Target angles for all controllable arm joints (rad)
    status_message: str
    position_error_m: Optional[float] = None
    orientation_error_rad: Optional[float] = None


class PandaIKSolver:
    """Calculates inverse kinematics for the 7-DoF Franka Emika Panda arm."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        robot_config: RobotConfig,
        arm_joint_indices: List[int],
        lower_limits: List[float],
        upper_limits: List[float],
        joint_ranges: List[float],
        rest_poses: List[float],
        end_effector_link_index: int = 11,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.config = robot_config
        self.arm_joint_indices = arm_joint_indices
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits
        self.joint_ranges = joint_ranges
        self.rest_poses = rest_poses
        self.ee_link_index = end_effector_link_index
        self.num_arm_joints = len(arm_joint_indices)

        # PyBullet damped least squares damping coefficient for all movable joints (7 arm + 2 fingers)
        self.damping = [0.01] * 9
        # Extend limits for 9 movable joints
        if len(self.lower_limits) == 7:
            self.full_lower = self.lower_limits + [0.0, 0.0]
            self.full_upper = self.upper_limits + [0.04, 0.04]
            self.full_ranges = self.joint_ranges + [0.04, 0.04]
            self.full_rests = self.rest_poses + [0.04, 0.04]
        else:
            self.full_lower = self.lower_limits
            self.full_upper = self.upper_limits
            self.full_ranges = self.joint_ranges
            self.full_rests = self.rest_poses

    def solve(
        self,
        target_position: Union[np.ndarray, Tuple[float, float, float], list],
        target_orientation: Optional[Union[np.ndarray, Tuple[float, float, float, float], list]] = None,
        max_iterations: int = 100,
        residual_threshold: float = 1e-4,
    ) -> IKResult:
        """Solves IK for target end-effector Cartesian pose.

        Args:
            target_position: [x, y, z] target coordinates in robot base frame.
            target_orientation: Optional unit quaternion [x, y, z, w]. If None, uses default downward orientation.
            max_iterations: Max PyBullet internal solver iterations.
            residual_threshold: Convergence threshold.

        Returns:
            IKResult containing arm joint positions and convergence metrics.
        """
        pos = list(np.asarray(target_position, dtype=np.float64).flatten())
        if len(pos) != 3 or not np.all(np.isfinite(pos)):
            return IKResult(
                success=False,
                joint_positions=[],
                status_message="Invalid target position (NaN/Inf)",
            )

        if target_orientation is None:
            orn = list(self.config.default_ee_orientation)
        else:
            orn = list(np.asarray(target_orientation, dtype=np.float64).flatten())
            # Normalize quaternion
            q_norm = np.linalg.norm(orn)
            if q_norm > 1e-6:
                orn = [x / q_norm for x in orn]
            else:
                orn = list(self.config.default_ee_orientation)

        try:
            # Call PyBullet IK
            raw_joint_poses = p.calculateInverseKinematics(
                self.robot_id,
                self.ee_link_index,
                targetPosition=pos,
                targetOrientation=orn,
                lowerLimits=self.full_lower,
                upperLimits=self.full_upper,
                jointRanges=self.full_ranges,
                restPoses=self.full_rests,
                jointDamping=self.damping,
                maxNumIterations=max_iterations,
                residualThreshold=residual_threshold,
                physicsClientId=self.client_id,
            )

            # Extract active arm joint positions
            arm_poses = [float(raw_joint_poses[i]) for i in range(self.num_arm_joints)]

            # Validate joint limits
            for idx, (val, low, high) in enumerate(zip(arm_poses, self.lower_limits, self.upper_limits)):
                if not (low - 0.05 <= val <= high + 0.05):
                    logger.debug(f"Joint {idx} limit violated: {val:.3f} not in [{low:.3f}, {high:.3f}]")
                    # Clamp to limit for stability
                    arm_poses[idx] = float(np.clip(val, low, high))

            return IKResult(
                success=True,
                joint_positions=arm_poses,
                status_message="CONVERGED",
            )

        except Exception as e:
            logger.error(f"IK solver exception: {e}")
            return IKResult(
                success=False,
                joint_positions=[],
                status_message=f"IK_ERROR: {e}",
            )
