"""Unit tests for Robotic State Machine transitions, perception gating, and sequence control."""

import time
import numpy as np
import pytest

from config.settings import AppConfig, StateMachineConfig, WorkspaceConfig
from main import VisionRobotTwinApp
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


def test_interrupted_detections_reset_consecutive_counter():
    """Verify that a missed frame strictly resets consecutive counter to 0."""
    sm_cfg = StateMachineConfig(consecutive_detection_threshold=3, auto_demo=False)
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    p_pick = np.array([0.45, -0.20, 0.035])
    p_place = np.array([0.45, 0.20, 0.035])

    # 2 consecutive hits
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    assert fsm._consecutive_pick_detections == 2

    # Missed frame on marker 1
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=None, marker_2_pos=p_place)
    assert fsm._consecutive_pick_detections == 0
    assert len(fsm._pick_poses_buffer) == 0

    # Needs 3 fresh consecutive hits to transition
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    assert fsm.state == RobotState.SEARCH
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    assert fsm.state == RobotState.SEARCH
    fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_pick, marker_2_pos=p_place)
    assert fsm.state == RobotState.APPROACH


def test_noisy_acquisition_produces_stable_median_aggregate_target():
    """Verify that noisy samples and outliers are filtered out by median aggregation."""
    sm_cfg = StateMachineConfig(consecutive_detection_threshold=5, auto_demo=False)
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    # True central position = [0.45, -0.20, 0.035]
    # Feed 4 samples around central, and 1 large outlier at the end: [0.95, -0.90, 0.035]
    noisy_pick_samples = [
        np.array([0.451, -0.199, 0.035]),
        np.array([0.449, -0.201, 0.035]),
        np.array([0.450, -0.200, 0.035]),
        np.array([0.452, -0.198, 0.035]),
        np.array([0.950, -0.900, 0.035]),  # Outlier sample on final frame
    ]
    place_sample = np.array([0.45, 0.20, 0.035])

    for p_samp in noisy_pick_samples:
        fsm.update_auto_mode(current_ee_pos=np.array([0.4, 0.0, 0.4]), marker_1_pos=p_samp, marker_2_pos=place_sample)

    assert fsm._targets_frozen is True
    # Frozen target must approximate central median [0.45, -0.20] rather than the last noisy sample [0.95, -0.90]
    assert np.isclose(fsm.pick_target_pos[0], 0.451, atol=0.005)
    assert np.isclose(fsm.pick_target_pos[1], -0.199, atol=0.005)


def test_6dof_hold_preserves_commanded_orientation():
    """Verify that entering HOLD preserves the last valid 6-DoF commanded orientation."""
    config = AppConfig()
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.control_mode = "6dof"
    config.mode = "manual"

    app = VisionRobotTwinApp(config, headless_sim=True)
    try:
        # 1. Step while tracking with a non-default custom orientation
        custom_orn = np.array([0.7071, 0.0, 0.7071, 0.0])
        app._last_commanded_target_orn = custom_orn.copy()
        app.state_machine.transition_to(RobotState.TRACK, "Test tracking")

        # 2. Simulate losing marker -> Transition to HOLD
        cmd_target, is_tracking = app.state_machine.update_manual_mode(
            marker_detected=False,
            target_pos_robot=None,
        )
        assert app.state_machine.state == RobotState.HOLD

        # Process frame without marker
        blank_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        res = app.process_frame(frame=blank_frame, wall_dt=0.033)

        assert res.state_name == "HOLD"
        # Commanded orientation must NOT snap back to default_ee_orientation; it must retain custom_orn
        assert np.allclose(res.commanded_orientation, custom_orn, atol=1e-3)
    finally:
        app.cleanup()


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
