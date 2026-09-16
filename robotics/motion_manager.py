"""Motion Manager: High-Level Trajectory Execution and Collision-Aware Planning Orchestration.

Coordinates Inverse Kinematics, direct-path validation, RRT-Connect obstacle avoidance,
path shortcutting, quintic trajectory generation, and time-stepped trajectory execution.
"""

from enum import Enum, auto
import time
from typing import List, Optional, Sequence, Tuple, Union
import numpy as np

from robotics.robot_controller import GenericRobotController
from robotics.inverse_kinematics import GenericIKSolver, IKStatus
from robotics.collision_provider import CollisionProvider
from robotics.planning import RRTConnectPlanner, shortcut_path, is_joint_path_collision_free, PlanningResult
from robotics.trajectory import JointQuinticTrajectory, TrajectoryExecutor
from utils.logger import get_logger

logger = get_logger("Robotics.MotionManager")


class MotionState(Enum):
    """Execution states for MotionManager."""
    IDLE = auto()
    DIRECT = auto()
    PLANNING = auto()
    EXECUTING = auto()
    COMPLETE = auto()
    FAILED = auto()
    HOLD = auto()


class MotionManager:
    """Manages collision-aware motion planning, trajectory synthesis, and execution lifecycle."""

    def __init__(
        self,
        robot_controller: GenericRobotController,
        ik_solver: GenericIKSolver,
        collision_checker: Optional[CollisionProvider] = None,
        trajectory_mode: str = "quintic",
        scene_type: str = "default",
        default_trajectory_duration: float = 2.0,
        collision_provider: Optional[CollisionProvider] = None,
    ):
        if collision_checker is not None and collision_provider is not None:
            if collision_checker is not collision_provider:
                raise ValueError("Conflicting collision_checker and collision_provider arguments provided.")
        effective_provider = collision_provider if collision_provider is not None else collision_checker

        self.controller = robot_controller
        self.ik_solver = ik_solver
        self.collision_provider = effective_provider
        self.checker = effective_provider
        self.trajectory_mode = trajectory_mode
        self.scene_type = scene_type
        self.default_duration = default_trajectory_duration

        lows, highs, _, _ = self.controller.get_joint_limits()
        if self.collision_provider is not None:
            self.planner = RRTConnectPlanner(
                lower_limits=lows,
                upper_limits=highs,
                collision_checker=self.collision_provider,
                arm_joint_indices=self.controller.arm_joint_indices,
                step_size_rad=0.10,
                max_iterations=400,
            )
        else:
            self.planner = None

        self.state = MotionState.IDLE
        self.active_trajectory: Optional[JointQuinticTrajectory] = None
        self.active_executor: Optional[TrajectoryExecutor] = None
        self.last_planning_result: Optional[PlanningResult] = None
        self._target_joint_positions: Optional[List[float]] = None

    @property
    def state_name(self) -> str:
        """Returns string representation of current motion state."""
        return self.state.name

    @property
    def progress_pct(self) -> float:
        """Returns execution progress percentage in [0.0, 100.0]."""
        if self.active_executor is not None:
            return self.active_executor.progress_pct
        if self.state == MotionState.COMPLETE:
            return 100.0
        return 0.0

    def plan_motion_to_pose(
        self,
        target_position: Sequence[float],
        target_orientation: Optional[Sequence[float]] = None,
        duration: Optional[float] = None,
    ) -> bool:
        """Plans a trajectory to target Cartesian pose.

        Args:
            target_position: Target [x, y, z] in robot base coordinates.
            target_orientation: Optional unit quaternion [x, y, z, w].
            duration: Trajectory execution duration (s).

        Returns:
            True if planning succeeded and trajectory was queued, False otherwise.
        """
        # 1. Solve IK
        ik_res = self.ik_solver.solve(target_position, target_orientation)
        if not ik_res.success:
            logger.warning(f"MotionManager IK solve failed: {ik_res.status_message}")
            self.state = MotionState.FAILED
            return False

        goal_q = np.array(ik_res.joint_positions, dtype=np.float64)
        current_q = np.array(self.controller.get_current_joint_positions(), dtype=np.float64)
        traj_duration = duration if duration is not None else self.default_duration

        # 2. Check direct linear joint path
        if self.collision_provider is not None:
            is_free, _ = is_joint_path_collision_free(
                current_q,
                goal_q,
                self.collision_provider,
                arm_joint_indices=self.controller.arm_joint_indices,
            )
        else:
            is_free = True

        if is_free:
            # Direct path is free
            self.state = MotionState.DIRECT
            if self.trajectory_mode == "quintic":
                self.active_trajectory = JointQuinticTrajectory(current_q, goal_q, duration=traj_duration)
                self.active_executor = TrajectoryExecutor(self.active_trajectory)
                self.state = MotionState.EXECUTING
            else:
                self._target_joint_positions = list(goal_q)
            return True

        # 3. Direct path is blocked -> Invoke RRT-Connect
        logger.info("Direct joint path blocked by obstacle; invoking RRT-Connect motion planner...")
        self.state = MotionState.PLANNING
        if self.planner is None:
            logger.error("No planner configured to handle blocked path.")
            self.state = MotionState.FAILED
            return False

        plan_res = self.planner.plan(current_q, goal_q)
        self.last_planning_result = plan_res

        if not plan_res.success or len(plan_res.path) < 2:
            logger.warning(f"RRT motion planning failed: {plan_res.reason}")
            self.state = MotionState.FAILED
            return False

        # 4. Shortcut path
        smoothed = shortcut_path(
            plan_res.path,
            self.collision_provider,
            arm_joint_indices=self.controller.arm_joint_indices,
            max_attempts=30,
        )

        # 5. Build trajectory
        from robotics.trajectory import PiecewiseJointTrajectory
        if len(smoothed) > 2:
            self.active_trajectory = PiecewiseJointTrajectory(smoothed, duration=traj_duration)
        else:
            self.active_trajectory = JointQuinticTrajectory(current_q, smoothed[-1], duration=traj_duration)

        self.active_executor = TrajectoryExecutor(self.active_trajectory)
        self.state = MotionState.EXECUTING
        return True

    def step(self, dt: float) -> Tuple[bool, float]:
        """Advances active trajectory execution by dt.

        Args:
            dt: Time step in seconds.

        Returns:
            (is_complete, progress_pct)
        """
        if self.state == MotionState.EXECUTING and self.active_executor is not None:
            sample, is_done, pct = self.active_executor.step(dt)
            self.controller.set_arm_joint_positions(
                list(sample.position), dt=dt, enforce_velocity_limits=True
            )
            if is_done:
                self.state = MotionState.COMPLETE
            return is_done, pct

        elif self.state == MotionState.DIRECT and self._target_joint_positions is not None:
            self.controller.set_arm_joint_positions(
                self._target_joint_positions, dt=dt, enforce_velocity_limits=True
            )
            self.state = MotionState.COMPLETE
            return True, 100.0

        elif self.state in (MotionState.HOLD, MotionState.FAILED):
            # Maintain current position
            curr_q = self.controller.get_current_joint_positions()
            self.controller.set_arm_joint_positions(curr_q, dt=dt, enforce_velocity_limits=True)
            return True, 0.0

        return True, 0.0

    def hold(self) -> None:
        """Places motion manager into safe HOLD state."""
        self.state = MotionState.HOLD
        if self.active_executor is not None:
            self.active_executor.abort()

    def reset(self) -> None:
        """Resets motion manager to IDLE."""
        self.state = MotionState.IDLE
        self.active_trajectory = None
        self.active_executor = None
        self._target_joint_positions = None
