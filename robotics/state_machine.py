"""Robotic Control Finite State Machine (FSM).

Coordinates deterministic behavior across perception gating, manual teleoperation,
tracking-loss recovery, and autonomous pick-and-place sequence execution.
"""

from collections import deque
from enum import Enum, auto
import time
from typing import Callable, Optional, Tuple
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
    """Manages state transitions, perception gating, and waypoint sequencing."""

    def __init__(self, config: StateMachineConfig, ws_config: WorkspaceConfig):
        self.config = config
        self.ws_config = ws_config

        self.state: RobotState = RobotState.HOME
        self.mode: str = "MANUAL"  # 'MANUAL' or 'AUTO'

        # Timers
        self._state_enter_time: float = time.time()
        self._lost_marker_time: Optional[float] = None
        self._action_timer: Optional[float] = None
        self._waypoint_start_time: float = time.time()

        # Perception Gating
        self._consecutive_pick_detections: int = 0
        self._consecutive_place_detections: int = 0
        self._pick_poses_buffer: deque[np.ndarray] = deque(maxlen=self.config.consecutive_detection_threshold)
        self._place_poses_buffer: deque[np.ndarray] = deque(maxlen=self.config.consecutive_detection_threshold)
        self._targets_frozen: bool = False

        # Autonomous Waypoints (Robot base frame coordinates)
        self.pick_target_pos: Optional[np.ndarray] = None
        self.place_target_pos: Optional[np.ndarray] = None
        self.current_waypoint: np.ndarray = np.array([
            self.ws_config.robot_center_x,
            self.ws_config.robot_center_y,
            self.ws_config.robot_center_z,
        ], dtype=np.float64)

        self._step_counter: int = 0
        self._waypoint_elapsed_s: float = 0.0

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
            self._waypoint_start_time = time.time()
            self._waypoint_elapsed_s = 0.0
            self._step_counter = 0
            self._action_timer = None

    def set_mode(self, mode: str) -> None:
        """Switches operational mode between MANUAL and AUTO."""
        mode_upper = mode.upper()
        if mode_upper in ("MANUAL", "AUTO"):
            self.mode = mode_upper
            logger.info(f"Operational mode switched to {self.mode}")
            self._targets_frozen = False
            self._consecutive_pick_detections = 0
            self._consecutive_place_detections = 0
            self.transition_to(RobotState.SEARCH, f"Switched mode to {self.mode}")

    def update_manual_mode(
        self,
        marker_detected: bool,
        target_pos_robot: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, bool]:
        """Updates FSM logic for Manual Tracking Mode with lost-tracking hold and recovery."""
        now = time.time()

        if marker_detected and target_pos_robot is not None:
            self._lost_marker_time = None
            if self.state in (RobotState.HOME, RobotState.SEARCH, RobotState.HOLD):
                self.transition_to(RobotState.TRACK, "Marker Acquired")

            self.current_waypoint = target_pos_robot.copy()
            return self.current_waypoint, True

        # Marker is NOT detected
        if self._lost_marker_time is None:
            self._lost_marker_time = now

        duration_lost = now - self._lost_marker_time

        if self.state == RobotState.TRACK:
            self.transition_to(RobotState.HOLD, "Marker temporarily occluded")

        if self.state == RobotState.HOLD:
            if duration_lost >= self.config.lost_tracking_search_timeout_s:
                self.transition_to(RobotState.SEARCH, "Lost tracking timeout exceeded")

        return self.current_waypoint, False

    def update_auto_mode(
        self,
        current_ee_pos: np.ndarray,
        marker_1_pos: Optional[np.ndarray] = None,
        marker_2_pos: Optional[np.ndarray] = None,
        gripper_attach_fn: Optional[Callable] = None,
        gripper_detach_fn: Optional[Callable] = None,
        on_targets_stabilized_fn: Optional[Callable[[np.ndarray, np.ndarray], None]] = None,
        dt: Optional[float] = None,
    ) -> Tuple[np.ndarray, str]:
        """Executes perception-gated autonomous pick-and-place sequence."""
        self._step_counter += 1
        now = time.time()
        if dt is not None and dt > 0.0:
            self._waypoint_elapsed_s += dt
            elapsed_waypoint_time = self._waypoint_elapsed_s
        else:
            elapsed_waypoint_time = now - self._waypoint_start_time

        # 1. Perception Gating Phase in SEARCH state
        if self.state in (RobotState.HOME, RobotState.SEARCH):
            if self.config.auto_demo:
                # Fallback coordinates for explicit offline demo mode
                self.pick_target_pos = np.array([0.45, -0.20, self.config.pick_descent_height_m], dtype=np.float64)
                self.place_target_pos = np.array([0.45, 0.20, self.config.pick_descent_height_m], dtype=np.float64)
                self._targets_frozen = True
            else:
                thresh = self.config.consecutive_detection_threshold
                if self._pick_poses_buffer.maxlen != thresh:
                    self._pick_poses_buffer = deque(self._pick_poses_buffer, maxlen=thresh)
                if self._place_poses_buffer.maxlen != thresh:
                    self._place_poses_buffer = deque(self._place_poses_buffer, maxlen=thresh)

                # Accumulate consecutive detections with strict reset on missed frame
                if marker_1_pos is not None:
                    self._consecutive_pick_detections += 1
                    self._pick_poses_buffer.append(np.asarray(marker_1_pos, dtype=np.float64))
                else:
                    self._consecutive_pick_detections = 0
                    self._pick_poses_buffer.clear()

                if marker_2_pos is not None:
                    self._consecutive_place_detections += 1
                    self._place_poses_buffer.append(np.asarray(marker_2_pos, dtype=np.float64))
                else:
                    self._consecutive_place_detections = 0
                    self._place_poses_buffer.clear()

                if (
                    self._consecutive_pick_detections >= self.config.consecutive_detection_threshold
                    and self._consecutive_place_detections >= self.config.consecutive_detection_threshold
                ):
                    # Compute robust median position across the consecutive sample window
                    pick_window = np.array(list(self._pick_poses_buffer))
                    place_window = np.array(list(self._place_poses_buffer))
                    pick_median = np.median(pick_window, axis=0)
                    place_median = np.median(place_window, axis=0)

                    pick_spread_m = float(np.max(np.std(pick_window, axis=0)))
                    place_spread_m = float(np.max(np.std(place_window, axis=0)))
                    logger.info(
                        f"Perception Gating Passed: Pick spread {pick_spread_m*1000.0:.2f} mm, "
                        f"Place spread {place_spread_m*1000.0:.2f} mm across "
                        f"{self.config.consecutive_detection_threshold} consecutive samples."
                    )

                    self.pick_target_pos = pick_median.copy()
                    self.pick_target_pos[2] = self.config.pick_descent_height_m
                    self.place_target_pos = place_median.copy()
                    self.place_target_pos[2] = self.config.pick_descent_height_m
                    self._targets_frozen = True
                    self._pick_poses_buffer.clear()
                    self._place_poses_buffer.clear()

                    if on_targets_stabilized_fn and self.pick_target_pos is not None and self.place_target_pos is not None:
                        on_targets_stabilized_fn(self.pick_target_pos, self.place_target_pos)

            if self._targets_frozen and self.pick_target_pos is not None:
                self.current_waypoint = np.array([
                    self.pick_target_pos[0],
                    self.pick_target_pos[1],
                    self.pick_target_pos[2] + self.config.approach_height_offset_m,
                ])
                self.transition_to(RobotState.APPROACH, "Beginning Pick Approach")
                return self.current_waypoint, "TARGETS_STABILIZED"

            return self.current_waypoint, "SEARCHING_FOR_MARKERS"

        # 2. Waypoint Arrival & Timeout Check
        dist_to_waypoint = float(np.linalg.norm(current_ee_pos - self.current_waypoint))
        arrived = dist_to_waypoint < self.config.waypoint_tolerance_m

        # If timeout exceeded before arrival, transition to ERROR
        if not arrived and (
            elapsed_waypoint_time > self.config.waypoint_timeout_s
            or self._step_counter > self.config.max_step_count_per_phase
        ):
            self.transition_to(
                RobotState.ERROR,
                f"Waypoint timeout ({elapsed_waypoint_time:.1f}s > {self.config.waypoint_timeout_s:.1f}s, dist: {dist_to_waypoint*100:.1f}cm)",
            )
            return self.current_waypoint, "TIMEOUT_ERROR"

        action_status = ""

        # 3. State Sequence Transitions
        if self.state == RobotState.APPROACH:
            if arrived:
                self.current_waypoint = self.pick_target_pos.copy()
                self.transition_to(RobotState.PICK, "Descended to Pick Target")

        elif self.state == RobotState.PICK:
            if arrived:
                if self._action_timer is None:
                    self._action_timer = now
                elif now - self._action_timer >= self.config.grasp_action_delay_s:
                    grasp_ok = True
                    if gripper_attach_fn:
                        grasp_res = gripper_attach_fn()
                        # If gripper returns structured GraspResult, validate success
                        if hasattr(grasp_res, "success"):
                            grasp_ok = grasp_res.success
                        elif grasp_res is False:
                            grasp_ok = False

                    if grasp_ok:
                        action_status = "GRASP_ATTACHED"
                        self._action_timer = None
                        self.current_waypoint = np.array([
                            self.pick_target_pos[0],
                            self.pick_target_pos[1],
                            self.pick_target_pos[2] + self.config.approach_height_offset_m,
                        ])
                        self.transition_to(RobotState.LIFT, "Lifting Object")
                    else:
                        self.transition_to(RobotState.ERROR, "Physical grasp attachment rejected")
                        action_status = "GRASP_FAILED"

        elif self.state == RobotState.LIFT:
            if arrived:
                self.current_waypoint = np.array([
                    self.place_target_pos[0],
                    self.place_target_pos[1],
                    self.place_target_pos[2] + self.config.approach_height_offset_m,
                ])
                self.transition_to(RobotState.MOVE_TO_PLACE, "Translating to Place Waypoint")

        elif self.state == RobotState.MOVE_TO_PLACE:
            if arrived:
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
                    self.current_waypoint = np.array([
                        self.place_target_pos[0],
                        self.place_target_pos[1],
                        self.place_target_pos[2] + self.config.approach_height_offset_m,
                    ])
                    self.transition_to(RobotState.RETURN_HOME, "Returning Home")

        elif self.state == RobotState.RETURN_HOME:
            if arrived:
                home_wp = np.array([
                    self.ws_config.robot_center_x,
                    self.ws_config.robot_center_y,
                    self.ws_config.robot_center_z,
                ])
                if np.linalg.norm(self.current_waypoint - home_wp) > 0.01:
                    self.current_waypoint = home_wp
                else:
                    self._targets_frozen = False
                    self._consecutive_pick_detections = 0
                    self._consecutive_place_detections = 0
                    self.transition_to(RobotState.SEARCH, "Pick-and-Place Cycle Completed")

        elif self.state == RobotState.ERROR:
            action_status = "ERROR_STATE"

        return self.current_waypoint, action_status

    def reset(self) -> None:
        """Resets state machine to initial SEARCH state."""
        self._targets_frozen = False
        self._consecutive_pick_detections = 0
        self._consecutive_place_detections = 0
        self._pick_poses_buffer.clear()
        self._place_poses_buffer.clear()
        self._lost_marker_time = None
        self._action_timer = None
        self.transition_to(RobotState.SEARCH, "FSM Reset")
