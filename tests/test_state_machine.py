"""Unit tests for Robotic State Machine transitions, perception gating, and sequence control."""

import time
import numpy as np
import pytest

from config.settings import StateMachineConfig, WorkspaceConfig
from robotics.state_machine import RoboticStateMachine, RobotState
from robotics.gripper import GraspResult


def test_manual_mode_acquisition_and_tracking():
    """Verify state transitions from SEARCH to TRACK upon marker detection."""
    sm_cfg = StateMachineConfig()
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)

    fsm.set_mode("MANUAL")
    assert fsm.state == RobotState.SEARCH

    target_pos = np.array([0.5, 0.0, 0.3])
    cmd_pos, is_tracking = fsm.update_manual_mode(marker_detected=True, target_pos_robot=target_pos)

    assert fsm.state == RobotState.TRACK
    assert is_tracking is True
    assert np.allclose(cmd_pos, target_pos)


def test_tracking_loss_hold_and_search_timeout():
    """Verify transition from TRACK -> HOLD -> SEARCH when marker is occluded."""
    sm_cfg = StateMachineConfig(
        lost_tracking_hold_timeout_s=0.05,
        lost_tracking_search_timeout_s=0.15,
    )
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)

    fsm.set_mode("MANUAL")
    target_pos = np.array([0.5, 0.0, 0.3])
    fsm.update_manual_mode(marker_detected=True, target_pos_robot=target_pos)
    assert fsm.state == RobotState.TRACK

    # 1. Immediately after occlusion: should enter HOLD
    fsm.update_manual_mode(marker_detected=False, target_pos_robot=None)
    assert fsm.state == RobotState.HOLD

    # 2. After search timeout: transitions to SEARCH
    time.sleep(0.18)
    fsm.update_manual_mode(marker_detected=False, target_pos_robot=None)
    assert fsm.state == RobotState.SEARCH


def test_auto_mode_perception_gating():
    """Verify auto mode remains in SEARCH unless both markers are detected consecutively."""
    sm_cfg = StateMachineConfig(consecutive_detection_threshold=3, auto_demo=False)
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    assert fsm.state == RobotState.SEARCH

    p_pick = np.array([0.45, -0.20, 0.035])
    p_place = np.array([0.45, 0.20, 0.035])

    # 1. No markers seen -> remains in SEARCH
    wp, status = fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]))
    assert fsm.state == RobotState.SEARCH
    assert status == "SEARCHING_FOR_MARKERS"

    # 2. Only 1 marker seen -> remains in SEARCH
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick)
    assert fsm.state == RobotState.SEARCH

    # 3. Both markers seen for 3 consecutive updates -> transitions to APPROACH
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)

    assert fsm.state == RobotState.APPROACH


def test_waypoint_timeout_transitions_to_error():
    """Verify that failing to reach a waypoint within timeout transitions to ERROR (not arrival)."""
    sm_cfg = StateMachineConfig(
        waypoint_timeout_s=0.05,
        max_step_count_per_phase=10,
        auto_demo=True,
    )
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    # Start in APPROACH
    wp, _ = fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]))
    assert fsm.state == RobotState.APPROACH

    # Stalled position far from waypoint
    stalled_pos = np.array([0.0, 0.0, 0.0])
    time.sleep(0.08)
    wp, status = fsm.update_auto_mode(current_ee_pos=stalled_pos)

    assert fsm.state == RobotState.ERROR
    assert "TIMEOUT_ERROR" in status


def test_distance_gated_grasp_rejection_and_acceptance():
    """Verify grasp rejection when end-effector is too far from object."""
    sm_cfg = StateMachineConfig(
        waypoint_tolerance_m=0.05,
        grasp_action_delay_s=0.01,
        auto_demo=True,
    )
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    # Transition from APPROACH to PICK
    wp, _ = fsm.update_auto_mode(current_ee_pos=np.array([0.45, -0.20, 0.175]))
    wp, _ = fsm.update_auto_mode(current_ee_pos=wp)
    assert fsm.state == RobotState.PICK

    # Grasp fails because distance was exceeded
    def failing_attach_fn():
        return GraspResult(success=False, distance_m=0.15, reason="DISTANCE_EXCEEDED")

    fsm.update_auto_mode(current_ee_pos=wp)
    time.sleep(0.02)
    wp, status = fsm.update_auto_mode(
        current_ee_pos=wp,
        gripper_attach_fn=failing_attach_fn,
    )

    assert fsm.state == RobotState.ERROR
    assert status == "GRASP_FAILED"
