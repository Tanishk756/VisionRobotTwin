"""Unit tests for pause/HOLD joint freeze, operator context resets, and acquisition buffer bounds."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from config.settings import AppConfig, StateMachineConfig, WorkspaceConfig, TransformConfig
from main import VisionRobotTwinApp
from robotics.state_machine import RoboticStateMachine, RobotState
from robotics.workspace_mapper import WorkspaceMapper


def test_pause_true_joint_freeze():
    """Verify that pausing freezes commanded joint positions and stops drift toward previous target."""
    config = AppConfig()
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.mode = "manual"
    config.control_mode = "6dof"

    app = VisionRobotTwinApp(config, headless_sim=True)

    try:
        # Step once to establish tracking toward synthetic target
        res1 = app.process_frame(wall_dt=1.0 / 30.0)
        assert res1.success

        # Emulate SPACE keypress to enter PAUSE / HOLD
        app._is_paused = True
        app._paused_joint_positions = app.simulator.controller.get_current_joint_positions()
        frozen_joints = np.array(app._paused_joint_positions, dtype=np.float64)

        # Step 60 physics frames while paused
        for _ in range(60):
            res = app.process_frame(wall_dt=1.0 / 30.0)
            assert res.success

        # Current joints must remain at frozen pose
        current_joints = np.array(app.simulator.controller.get_current_joint_positions(), dtype=np.float64)
        joint_diff = np.max(np.abs(current_joints - frozen_joints))
        assert joint_diff < 0.02, f"Robot drifted while paused! Max joint deviation: {joint_diff:.4f} rad"

        # Resume tracking
        app._is_paused = False
        app._paused_joint_positions = None

        # Step 30 frames after resuming
        for _ in range(30):
            res = app.process_frame(wall_dt=1.0 / 30.0)
            assert res.success

    finally:
        app.cleanup()


def test_operator_context_reset_relative_orientation():
    """Verify that resetting workspace mapper (on Home/Mode switch/Reset) clears orientation reference."""
    tf_cfg = TransformConfig(
        transform_mode="relative",
        tool_orientation_offset=(1.0, 0.0, 0.0, 0.0),
    )
    ws_cfg = WorkspaceConfig()
    mapper = WorkspaceMapper(ws_cfg, tf_cfg)

    # Orientation 1 (Reference)
    q0 = np.array([0.0, 0.0, 0.0, 1.0])
    cam_pos = np.array([0.0, 0.0, 0.6])
    t0 = mapper.map_camera_to_robot(cam_pos, q0, enforce_slew_rate=False)
    dot0 = np.abs(np.dot(t0.orientation, [1.0, 0.0, 0.0, 0.0]))
    assert np.isclose(dot0, 1.0, atol=1e-3)

    # Orientation 2 (Rotated 90 degrees around Cam Z -> Robot X)
    q1 = (Rotation.from_euler("z", 90, degrees=True) * Rotation.from_quat(q0)).as_quat()
    t1 = mapper.map_camera_to_robot(cam_pos, q1, enforce_slew_rate=False)
    dot1 = np.abs(np.dot(t1.orientation, [1.0, 0.0, 0.0, 0.0]))
    # Orientation has rotated away from reference default
    assert dot1 < 0.9

    # Reset mapper (simulating Mode Switch / Home / Reset keypress)
    mapper.reset()

    # Present q1 again as the first observation after context reset
    # It must now establish a fresh reference and produce default tool orientation
    t_fresh = mapper.map_camera_to_robot(cam_pos, q1, enforce_slew_rate=False)
    dot_fresh = np.abs(np.dot(t_fresh.orientation, [1.0, 0.0, 0.0, 0.0]))
    assert np.isclose(dot_fresh, 1.0, atol=1e-3)


def test_acquisition_buffer_bounded_under_continuous_single_marker():
    """Verify perception buffer memory remains strictly bounded when single marker is observed."""
    threshold = 5
    sm_cfg = StateMachineConfig(consecutive_detection_threshold=threshold, auto_demo=False)
    ws_cfg = WorkspaceConfig()
    fsm = RoboticStateMachine(sm_cfg, ws_cfg)
    fsm.set_mode("AUTO")

    p_pick = np.array([0.45, -0.20, 0.035])

    # Feed 200 iterations with marker 1 present and marker 2 absent
    for _ in range(200):
        fsm.update_auto_mode(
            current_ee_pos=np.array([0.4, 0.0, 0.4]),
            marker_1_pos=p_pick,
            marker_2_pos=None,
        )

    # Buffer must be strictly bounded to threshold
    assert len(fsm._pick_poses_buffer) <= threshold
    assert fsm._consecutive_pick_detections == 200
    assert fsm._consecutive_place_detections == 0
    assert len(fsm._place_poses_buffer) == 0
    assert fsm.state == RobotState.SEARCH
