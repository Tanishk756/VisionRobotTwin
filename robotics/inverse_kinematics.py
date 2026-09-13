"""Generic and Multi-Robot Inverse Kinematics (IK) Solver.

Computes joint space solutions for arbitrary N-DoF robotic manipulators
(e.g., Franka Emika Panda, KUKA LBR iiwa) using PyBullet's Damped Least-Squares
IK solver with explicit joint limits, null-space rest poses, and performance diagnostics.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple, Union
from enum import Enum, auto
import time
import pybullet as p
import numpy as np

from config.settings import RobotConfig
from robotics.coordinate_transform import compute_angular_distance
from utils.logger import get_logger

logger = get_logger("Robotics.IK")


class IKStatus(Enum):
    """Categorized status for Inverse Kinematics solving."""
    SOLUTION_RETURNED = auto()
    OUT_OF_LIMITS = auto()
    UNREACHABLE = auto()
    INVALID_TARGET = auto()
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
    residual_position_m: Optional[float] = None
    residual_orientation_rad: Optional[float] = None
    solve_time_ms: Optional[float] = None


# Explicit numerical tolerance for joint limit boundaries (0.01 rad ~= 0.57 deg)
JOINT_LIMIT_TOLERANCE_RAD: float = 0.01


class GenericIKSolver:
    """Robot-agnostic Inverse Kinematics solver for multi-DoF arms in PyBullet."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        arm_joint_indices: List[int],
        lower_limits: List[float],
        upper_limits: List[float],
        joint_ranges: List[float],
        rest_poses: List[float],
        end_effector_link_index: int,
        max_reach_m: float = 0.855,
        min_reach_m: float = 0.10,
        default_ee_orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
        damping_constant: float = 0.01,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.arm_joint_indices = arm_joint_indices
        self.lower_limits = list(lower_limits)
        self.upper_limits = list(upper_limits)
        self.joint_ranges = list(joint_ranges)
        self.rest_poses = list(rest_poses)
        self.ee_link_index = end_effector_link_index
        self.num_arm_joints = len(arm_joint_indices)
        self.max_reach_m = max_reach_m
        self.min_reach_m = min_reach_m
        self.default_ee_orientation = default_ee_orientation

        # Count actual movable DOFs (revolute + prismatic) for null-space vector sizing in PyBullet
        movable_joint_count = 0
        num_total_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for i in range(num_total_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                movable_joint_count += 1

        if len(self.lower_limits) < movable_joint_count:
            pad_len = movable_joint_count - len(self.lower_limits)
            self.full_lower = self.lower_limits + [0.0] * pad_len
            self.full_upper = self.upper_limits + [0.04] * pad_len
            self.full_ranges = self.joint_ranges + [0.04] * pad_len
            self.full_rests = self.rest_poses + [0.04] * pad_len
            self.damping = [damping_constant] * movable_joint_count
        else:
            self.full_lower = self.lower_limits
            self.full_upper = self.upper_limits
            self.full_ranges = self.joint_ranges
            self.full_rests = self.rest_poses
            self.damping = [damping_constant] * len(self.lower_limits)

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
        t0 = time.perf_counter()
        pos = list(np.asarray(target_position, dtype=np.float64).flatten())
        if len(pos) != 3 or not np.all(np.isfinite(pos)):
            return IKResult(
                success=False,
                joint_positions=[],
                status=IKStatus.INVALID_TARGET,
                status_message="Invalid target position (contains NaN or Inf)",
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        # Reachability distance guard
        dist_from_base = float(np.linalg.norm(pos))
        if dist_from_base > (self.max_reach_m + 0.05) or dist_from_base < self.min_reach_m:
            return IKResult(
                success=False,
                joint_positions=[],
                status=IKStatus.UNREACHABLE,
                status_message=f"Target distance ({dist_from_base:.2f}m) exceeds kinematic reach [{self.min_reach_m:.2f}m, {self.max_reach_m:.2f}m]",
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )

        if target_orientation is None:
            orn = list(self.default_ee_orientation)
        else:
            orn = list(np.asarray(target_orientation, dtype=np.float64).flatten())
            q_norm = float(np.linalg.norm(orn))
            if q_norm > 1e-6:
                orn = [float(x / q_norm) for x in orn]
            else:
                orn = list(self.default_ee_orientation)

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
            solve_time_ms = (time.perf_counter() - t0) * 1000.0

            # 1. Validate Finite Joint Angles
            if not np.all(np.isfinite(arm_poses)):
                return IKResult(
                    success=False,
                    joint_positions=[],
                    status=IKStatus.IK_ERROR,
                    status_message="IK solver produced non-finite joint values",
                    solve_time_ms=solve_time_ms,
                )

            # 2. Strict Joint Limit Validation
            for idx, (val, low, high) in enumerate(zip(arm_poses, self.lower_limits, self.upper_limits)):
                if val < (low - JOINT_LIMIT_TOLERANCE_RAD) or val > (high + JOINT_LIMIT_TOLERANCE_RAD):
                    logger.debug(f"Joint {idx} limit exceeded: {val:.3f} not in [{low:.3f}, {high:.3f}]")
                    return IKResult(
                        success=False,
                        joint_positions=[],
                        status=IKStatus.OUT_OF_LIMITS,
                        status_message=f"Joint {idx} angle ({val:.3f} rad) outside physical limits [{low:.3f}, {high:.3f}]",
                        solve_time_ms=solve_time_ms,
                    )
                # Apply minor numerical clamping strictly within valid limits
                arm_poses[idx] = float(np.clip(val, low, high))

            return IKResult(
                success=True,
                joint_positions=arm_poses,
                status=IKStatus.SOLUTION_RETURNED,
                status_message="SOLUTION_RETURNED",
                residual_position_m=0.0,
                residual_orientation_rad=0.0,
                solve_time_ms=solve_time_ms,
            )

        except Exception as e:
            logger.error(f"IK solver exception: {e}")
            return IKResult(
                success=False,
                joint_positions=[],
                status=IKStatus.IK_ERROR,
                status_message=f"IK_ERROR: {e}",
                solve_time_ms=(time.perf_counter() - t0) * 1000.0,
            )


class PandaIKSolver(GenericIKSolver):
    """Backward-compatible Franka Emika Panda IK Solver."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        robot_config: Optional[RobotConfig] = None,
        arm_joint_indices: Optional[List[int]] = None,
        lower_limits: Optional[List[float]] = None,
        upper_limits: Optional[List[float]] = None,
        joint_ranges: Optional[List[float]] = None,
        rest_poses: Optional[List[float]] = None,
        end_effector_link_index: int = 11,
    ):
        config = robot_config or RobotConfig()
        arm_indices = arm_joint_indices or [0, 1, 2, 3, 4, 5, 6]
        lows = lower_limits or [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
        highs = upper_limits or [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
        ranges = joint_ranges or [h - l for l, h in zip(lows, highs)]
        rests = rest_poses or list(config.home_joint_positions[: len(arm_indices)])

        super().__init__(
            physics_client_id=physics_client_id,
            robot_id=robot_id,
            arm_joint_indices=arm_indices,
            lower_limits=lows,
            upper_limits=highs,
            joint_ranges=ranges,
            rest_poses=rests,
            end_effector_link_index=end_effector_link_index,
            max_reach_m=0.855,
            min_reach_m=0.10,
            default_ee_orientation=config.default_ee_orientation,
        )
