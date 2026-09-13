"""Inverse Kinematics (IK) Solver and Kinematic Validation for Franka Emika Panda.

Computes joint space solutions for desired end-effector 6-DoF Cartesian targets
using PyBullet's Damped Least-Squares IK solver with joint limits, nullspace optimization,
and comprehensive kinematic validation.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple, Union
from enum import Enum, auto
import pybullet as p
import numpy as np

from config.settings import RobotConfig
from robotics.coordinate_transform import compute_angular_distance
from utils.logger import get_logger

logger = get_logger("Robotics.IK")


class IKStatus(Enum):
    """Categorized status for Inverse Kinematics solving."""
    SOLUTION_RETURNED = auto()
    CONVERGED = auto()
    OUT_OF_LIMITS = auto()
    UNREACHABLE = auto()
    INVALID_TARGET = auto()
    RESIDUAL_TOO_HIGH = auto()
    IK_ERROR = auto()


@dataclass
class IKResult:
    """Represents the validated output of an Inverse Kinematics calculation."""
    success: bool
    joint_positions: List[float]  # Target angles for all controllable arm joints (rad)
    status: IKStatus
    status_message: str
    position_error_m: Optional[float] = None
    orientation_error_rad: Optional[float] = None


class PandaIKSolver:
    """Calculates and validates inverse kinematics for the 7-DoF Franka Emika Panda arm."""

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
        """Solves IK for target end-effector Cartesian pose and validates the solution.

        Args:
            target_position: [x, y, z] target coordinates in robot base frame.
            target_orientation: Optional unit quaternion [x, y, z, w].
            max_iterations: Max PyBullet internal solver iterations.
            residual_threshold: Convergence threshold.

        Returns:
            IKResult containing validated arm joint positions and convergence metrics.
        """
        pos = list(np.asarray(target_position, dtype=np.float64).flatten())
        if len(pos) != 3 or not np.all(np.isfinite(pos)):
            return IKResult(
                success=False,
                joint_positions=[],
                status=IKStatus.INVALID_TARGET,
                status_message="Invalid target position (contains NaN or Inf)",
            )

        # Reachability distance guard (Franka Panda max spherical reach ~ 0.855m)
        dist_from_base = np.linalg.norm(pos)
        if dist_from_base > 0.90 or dist_from_base < 0.10:
            return IKResult(
                success=False,
                joint_positions=[],
                status=IKStatus.UNREACHABLE,
                status_message=f"Target distance ({dist_from_base:.2f}m) exceeds kinematic reach",
            )

        if target_orientation is None:
            orn = list(self.config.default_ee_orientation)
        else:
            orn = list(np.asarray(target_orientation, dtype=np.float64).flatten())
            q_norm = np.linalg.norm(orn)
            if q_norm > 1e-6:
                orn = [float(x / q_norm) for x in orn]
            else:
                orn = list(self.config.default_ee_orientation)

        try:
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

            arm_poses = [float(raw_joint_poses[i]) for i in range(self.num_arm_joints)]

            # 1. Validate Finite Joint Angles
            if not np.all(np.isfinite(arm_poses)):
                return IKResult(
                    success=False,
                    joint_positions=[],
                    status=IKStatus.IK_ERROR,
                    status_message="IK solver produced non-finite joint values",
                )

            # 2. Validate Joint Limits
            has_limit_violation = False
            for idx, (val, low, high) in enumerate(zip(arm_poses, self.lower_limits, self.upper_limits)):
                if val < low - 0.05 or val > high + 0.05:
                    has_limit_violation = True
                    logger.debug(f"Joint {idx} limit exceeded: {val:.3f} not in [{low:.3f}, {high:.3f}]")
                # Clamp safely
                arm_poses[idx] = float(np.clip(val, low, high))

            status = IKStatus.OUT_OF_LIMITS if has_limit_violation else IKStatus.SOLUTION_RETURNED

            return IKResult(
                success=True,
                joint_positions=arm_poses,
                status=status,
                status_message=status.name,
            )

        except Exception as e:
            logger.error(f"IK solver exception: {e}")
            return IKResult(
                success=False,
                joint_positions=[],
                status=IKStatus.IK_ERROR,
                status_message=f"IK_ERROR: {e}",
            )
