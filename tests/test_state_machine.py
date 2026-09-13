"""Unit tests for Robotic State Machine transitions and sequence control."""

import time
import numpy as np
import pytest

from config.settings import StateMachineConfig, WorkspaceConfig
from robotics.state_machine import RoboticStateMachine, RobotState


def test_manual_mode_acquisition_and_tracking():
    """Verify state transitions from SEARCH to TRACK upon marker detection."""
    sm_cfg = StateMachineConfig()
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)

    fsm.set_mode("MANUAL")
    assert fsm.state == RobotState.SEARCH

    # Marker detected
    target_pos = np.array([0.5, 0.0, 0.3])
    cmd_pos, is_tracking = fsm.update_manual_mode(marker_detected=True, target_pos_robot=target_pos)

    assert fsm.state == RobotState.TRACK
    assert is_tracking is True
    assert np.allclose(cmd_pos, target_pos)


def test_tracking_loss_hold_and_search_timeout():
    """Verify transition from TRACK -> HOLD -> SEARCH when marker is occluded."""
    sm_cfg = StateMachineConfig(
        lost_tracking_hold_timeout_s=0.1,
        lost_tracking_search_timeout_s=0.2,
    )
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)

    fsm.set_mode("MANUAL")
    target_pos = np.array([0.5, 0.0, 0.3])
    fsm.update_manual_mode(marker_detected=True, target_pos_robot=target_pos)
    assert fsm.state == RobotState.TRACK

    # Marker disappears
    fsm.update_manual_mode(marker_detected=False, target_pos_robot=None)
    assert fsm.state == RobotState.HOLD

    # Wait past timeout
    time.sleep(0.25)
    fsm.update_manual_mode(marker_detected=False, target_pos_robot=None)
    assert fsm.state == RobotState.SEARCH


def test_autonomous_state_sequence():
    """Verify autonomous pick-and-place waypoint progression through full cycle."""
    sm_cfg = StateMachineConfig(
        waypoint_tolerance_m=0.05,
        grasp_action_delay_s=0.01,
    )
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    grasped = [False]

    def attach_fn():
        grasped[0] = True

    def detach_fn():
        grasped[0] = False

    # Start in SEARCH -> updates to APPROACH
    wp, _ = fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]))
    assert fsm.state == RobotState.APPROACH

    # Simulate arriving at pick object (transitions from APPROACH to PICK)
    wp, _ = fsm.update_auto_mode(current_ee_pos=wp)
    assert fsm.state == RobotState.PICK

    # Start grasp timer
    wp, _ = fsm.update_auto_mode(current_ee_pos=wp)
    time.sleep(0.02)
    # Grasp timer expires and transitions to LIFT
    wp, action = fsm.update_auto_mode(
        current_ee_pos=wp,
        gripper_attach_fn=attach_fn,
        gripper_detach_fn=detach_fn,
    )
    assert fsm.state == RobotState.LIFT
    assert grasped[0] is True

    # Simulate arriving at lift waypoint
    wp, _ = fsm.update_auto_mode(current_ee_pos=wp)
    assert fsm.state == RobotState.MOVE_TO_PLACE

    # Simulate arriving above place target
    wp, _ = fsm.update_auto_mode(current_ee_pos=wp)
    assert fsm.state == RobotState.PLACE

    # Start release timer
    wp, _ = fsm.update_auto_mode(current_ee_pos=wp)
    time.sleep(0.02)
    # Release timer expires and transitions to RETURN_HOME
    wp, action = fsm.update_auto_mode(
        current_ee_pos=wp,
        gripper_attach_fn=attach_fn,
        gripper_detach_fn=detach_fn,
    )
    assert fsm.state == RobotState.RETURN_HOME
    assert grasped[0] is False
