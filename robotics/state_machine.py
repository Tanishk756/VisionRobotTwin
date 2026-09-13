"""Robotic Control Finite State Machine (FSM).

Coordinates high-level behavior across perception, manual teleoperation,
tracking-loss recovery, and autonomous pick-and-place sequence execution.
"""

from enum import Enum, auto
import time
from typing import Optional, Tuple
import numpy as np

from config.settings import StateMachineConfig, WorkspaceConfig
from utils.logger import get_logger

logger = get_logger("Robotics.StateMachine")


class RobotState(Enum):
    """FSM Operational States."""
    HOME = auto()
    SEARCH = auto()
    TRACK = auto()
    HOLD = auto()
    APPROACH = auto()
    PICK = auto()
    LIFT = auto()
    MOVE_TO_PLACE = auto()
    PLACE = auto()
    RETURN_HOME = auto()
    ERROR = auto()


class RoboticStateMachine:
    """Manages state transitions, timeouts, and waypoint sequencing."""

    def __init__(self, config: StateMachineConfig, ws_config: WorkspaceConfig):
        self.config = config
        self.ws_config = ws_config

        self.state: RobotState = RobotState.HOME
        self.mode: str = "MANUAL"  # 'MANUAL' or 'AUTO'

        # Timers
        self._state_enter_time: float = time.time()
        self._lost_marker_time: Optional[float] = None
        self._action_timer: Optional[float] = None

        # Autonomous Waypoints (Robot base frame coordinates)
        self.pick_target_pos: np.ndarray = np.array([0.45, -0.20, 0.035], dtype=np.float64)
        self.place_target_pos: np.ndarray = np.array([0.45, 0.20, 0.035], dtype=np.float64)
        self.current_waypoint: np.ndarray = np.array([0.40, 0.0, 0.40], dtype=np.float64)

        self._step_counter: int = 0

    @property
    def state_name(self) -> str:
        return self.state.name

    @property
    def lost_tracking_duration_s(self) -> float:
        if self._lost_marker_time is None:
            return 0.0
        return time.time() - self._lost_marker_time

    def transition_to(self, new_state: RobotState, reason: str = "") -> None:
        """Executes state transition and resets phase timers."""
        if new_state != self.state:
            logger.info(f"FSM State Transition: {self.state.name} -> {new_state.name} ({reason})")
            self.state = new_state
            self._state_enter_time = time.time()
            self._step_counter = 0

    def set_mode(self, mode: str) -> None:
        """Switches mode between MANUAL and AUTO."""
        mode_upper = mode.upper()
        if mode_upper in ("MANUAL", "AUTO"):
            self.mode = mode_upper
            logger.info(f"Mode set to {self.mode}")
            if self.mode == "MANUAL":
                self.transition_to(RobotState.SEARCH, "Switched to Manual mode")
            else:
                self.transition_to(RobotState.SEARCH, "Switched to Autonomous mode")

    def update_manual_mode(
        self,
        marker_detected: bool,
        target_pos_robot: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, bool]:
        """Updates FSM logic for Manual Tracking Mode.

        Returns:
            (commanded_cartesian_target, is_tracking_active)
        """
        now = time.time()

        if marker_detected and target_pos_robot is not None:
            self._lost_marker_time = None
            if self.state in (RobotState.HOME, RobotState.SEARCH, RobotState.HOLD):
                self.transition_to(RobotState.TRACK, "Marker Acquired")

            self.current_waypoint = target_pos_robot.copy()
            return self.current_waypoint, True

        # Marker NOT detected
        if self._lost_marker_time is None:
            self._lost_marker_time = now

        duration_lost = now - self._lost_marker_time

        if self.state == RobotState.TRACK:
            self.transition_to(RobotState.HOLD, "Marker temporarily occluded")

        if self.state == RobotState.HOLD:
            if duration_lost > self.config.lost_tracking_search_timeout_s:
                self.transition_to(RobotState.SEARCH, "Lost tracking timeout exceeded")

        # In HOLD or SEARCH, hold last known valid target or safe center
        return self.current_waypoint, False

    def update_auto_mode(
        self,
        current_ee_pos: np.ndarray,
        marker_1_pos: Optional[np.ndarray] = None,
        marker_2_pos: Optional[np.ndarray] = None,
        gripper_attach_fn=None,
        gripper_detach_fn=None,
    ) -> Tuple[np.ndarray, str]:
        """Executes autonomous pick-and-place waypoint state sequence.

        Sequence:
          SEARCH -> APPROACH -> PICK (attach) -> LIFT -> MOVE_TO_PLACE -> PLACE (detach) -> RETURN_HOME -> SEARCH
        """
        self._step_counter += 1
        now = time.time()

        # Update detected pick/place positions if available from vision
        if marker_1_pos is not None:
            self.pick_target_pos = marker_1_pos.copy()
            self.pick_target_pos[2] = self.config.pick_descent_height_m

        if marker_2_pos is not None:
            self.place_target_pos = marker_2_pos.copy()
            self.place_target_pos[2] = self.config.pick_descent_height_m

        # Compute distance to current waypoint
        dist_to_waypoint = float(np.linalg.norm(current_ee_pos - self.current_waypoint))
        arrived = dist_to_waypoint < self.config.waypoint_tolerance_m or self._step_counter > self.config.max_step_count_per_phase

        action_status = ""

        if self.state in (RobotState.HOME, RobotState.SEARCH):
            # Pre-pick approach waypoint (above pick target)
            self.current_waypoint = np.array([
                self.pick_target_pos[0],
                self.pick_target_pos[1],
                self.pick_target_pos[2] + self.config.approach_height_offset_m,
            ])
            self.transition_to(RobotState.APPROACH, "Beginning Pick Approach")

        elif self.state == RobotState.APPROACH:
            if arrived:
                # Descend directly onto pick object
                self.current_waypoint = self.pick_target_pos.copy()
                self.transition_to(RobotState.PICK, "Descended to Pick Target")

        elif self.state == RobotState.PICK:
            if arrived:
                if self._action_timer is None:
                    self._action_timer = now
                elif now - self._action_timer >= self.config.grasp_action_delay_s:
                    if gripper_attach_fn:
                        gripper_attach_fn()
                    action_status = "GRASP_ATTACHED"
                    self._action_timer = None
                    # Ascend with object
                    self.current_waypoint = np.array([
                        self.pick_target_pos[0],
                        self.pick_target_pos[1],
                        self.pick_target_pos[2] + self.config.approach_height_offset_m,
                    ])
                    self.transition_to(RobotState.LIFT, "Lifting Object")

        elif self.state == RobotState.LIFT:
            if arrived:
                # Move to position above place target
                self.current_waypoint = np.array([
                    self.place_target_pos[0],
                    self.place_target_pos[1],
                    self.place_target_pos[2] + self.config.approach_height_offset_m,
                ])
                self.transition_to(RobotState.MOVE_TO_PLACE, "Translating to Place Waypoint")

        elif self.state == RobotState.MOVE_TO_PLACE:
            if arrived:
                # Descend onto place pad
                self.current_waypoint = self.place_target_pos.copy()
                self.transition_to(RobotState.PLACE, "Descended to Place Pad")

        elif self.state == RobotState.PLACE:
            if arrived:
                if self._action_timer is None:
                    self._action_timer = now
                elif now - self._action_timer >= self.config.grasp_action_delay_s:
                    if gripper_detach_fn:
                        gripper_detach_fn()
                    action_status = "GRASP_RELEASED"
                    self._action_timer = None
                    # Return to safe Home / Standby
                    self.current_waypoint = np.array([
                        self.ws_config.robot_center_x,
                        self.ws_config.robot_center_y,
                        self.ws_config.robot_center_z,
                    ])
                    self.transition_to(RobotState.RETURN_HOME, "Returning Home")

        elif self.state == RobotState.RETURN_HOME:
            if arrived:
                self.transition_to(RobotState.SEARCH, "Pick-and-Place Cycle Completed")

        return self.current_waypoint, action_status
