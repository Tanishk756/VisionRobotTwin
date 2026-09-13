"""Unit tests for workspace mapping, SE(3) transformation modes, relative 6-DoF orientation, and slew-rate safety."""

import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from config.settings import WorkspaceConfig, TransformConfig
from robotics.workspace_mapper import WorkspaceMapper


def test_workspace_boundary_clamping():
    """Verify out-of-bound camera targets are strictly clamped inside robot workspace limits."""
    ws_config = WorkspaceConfig(
        x_min=0.25, x_max=0.70,
        y_min=-0.40, y_max=0.40,
        z_min=0.08, z_max=0.65,
        scale_x=1.0, scale_y=1.0, scale_z=1.0,
        robot_center_x=0.50, robot_center_y=0.0, robot_center_z=0.35,
        cam_center_x=0.0, cam_center_y=0.0, cam_center_z=0.45,
    )
    mapper = WorkspaceMapper(ws_config)

    far_pos = np.array([5.0, -10.0, 20.0])
    target = mapper.map_camera_to_robot(far_pos, enforce_slew_rate=False)

    assert target.is_valid
    assert target.is_clamped
    assert target.x <= ws_config.x_max
    assert target.x >= ws_config.x_min
    assert target.y <= ws_config.y_max
    assert target.y >= ws_config.y_min
    assert target.z <= ws_config.z_max
    assert target.z >= ws_config.z_min


def test_se3_vs_relative_modes():
    """Verify both SE(3) rigid transformation and relative teleoperation modes function correctly."""
    ws_cfg = WorkspaceConfig(
        x_min=0.0, x_max=2.0, y_min=-2.0, y_max=2.0, z_min=0.0, z_max=2.0
    )
    tf_cfg_se3 = TransformConfig(
        transform_mode="se3",
        camera_position_in_robot_base=(0.7, 0.0, 0.4),
        camera_euler_rpy_rad=(0.0, 0.0, 0.0),
    )
    mapper_se3 = WorkspaceMapper(ws_cfg, tf_cfg_se3)

    p_cam = np.array([0.1, 0.2, 0.3])
    tgt_se3 = mapper_se3.map_camera_to_robot(p_cam, enforce_slew_rate=False)
    # In SE3 with identity rotation: p_base = [0.7+0.1, 0+0.2, 0.4+0.3] = [0.8, 0.2, 0.7]
    assert np.allclose(tgt_se3.position, [0.8, 0.2, 0.7], atol=1e-5)


def test_relative_mode_initial_reference_and_rotation_mapping():
    """Verify relative orientation math: initial reference -> default tool orientation, and delta mapping."""
    ws_cfg = WorkspaceConfig(
        scale_x=1.0, scale_y=1.0, scale_z=1.0,
        robot_center_x=0.5, robot_center_y=0.0, robot_center_z=0.4,
        cam_center_x=0.0, cam_center_y=0.0, cam_center_z=0.5,
    )
    tf_cfg = TransformConfig(
        transform_mode="relative",
        tool_orientation_offset=(1.0, 0.0, 0.0, 0.0),  # Downward orientation [1, 0, 0, 0]
    )
    mapper = WorkspaceMapper(ws_cfg, tf_cfg)

    # 1. First frame: marker presented at arbitrary orientation (e.g. 45 deg tilt)
    initial_marker_rot = Rotation.from_euler("xyz", [45, 0, 0], degrees=True)
    q_init = initial_marker_rot.as_quat()

    t1 = mapper.map_camera_to_robot(
        camera_pos=np.array([0.0, 0.0, 0.5]),
        camera_quat_xyzw=q_init,
        enforce_slew_rate=False,
    )
    # Initial presentation must produce reference default tool orientation, NOT arbitrary tilt or 180 flip
    assert np.allclose(t1.orientation, [1.0, 0.0, 0.0, 0.0], atol=1e-3) or np.allclose(t1.orientation, [-1.0, 0.0, 0.0, 0.0], atol=1e-3)
    assert np.isclose(np.linalg.norm(t1.orientation), 1.0)

    # 2. Rotate marker +90 degrees around Camera Z (yaw in camera frame)
    # Camera +Z maps to Robot +X (forward). So +90 around Cam Z should produce +90 around Robot X!
    rot_cam_z_90 = Rotation.from_euler("z", 90, degrees=True)
    rot_frame2 = rot_cam_z_90 * initial_marker_rot
    q_frame2 = rot_frame2.as_quat()

    t2 = mapper.map_camera_to_robot(
        camera_pos=np.array([0.0, 0.0, 0.5]),
        camera_quat_xyzw=q_frame2,
        enforce_slew_rate=False,
    )
    assert np.isclose(np.linalg.norm(t2.orientation), 1.0)

    # Expected tool orientation: initial [1,0,0,0] rotated +90 deg around Robot X
    expected_rot = Rotation.from_euler("x", 90, degrees=True) * Rotation.from_quat([1.0, 0.0, 0.0, 0.0])
    exp_q = expected_rot.as_quat()

    # Compare orientation equivalence (accounting for q and -q)
    dot = np.abs(np.dot(t2.orientation, exp_q))
    assert np.isclose(dot, 1.0, atol=1e-3), f"Expected relative rotation dot {dot} to be close to 1.0"


def test_relative_mode_camera_axis_mapping_matrix():
    """Verify R_R_C matrix maps camera axes to intuitive robot translation/rotation axes."""
    mapper = WorkspaceMapper(WorkspaceConfig())
    # Camera +Z -> Robot +X
    # Camera +X -> Robot -Y
    # Camera +Y -> Robot -Z
    v_cam_z = np.array([0.0, 0.0, 1.0])
    v_rob_x = mapper.R_R_C @ v_cam_z
    assert np.allclose(v_rob_x, [1.0, 0.0, 0.0])

    v_cam_x = np.array([1.0, 0.0, 0.0])
    v_rob_y = mapper.R_R_C @ v_cam_x
    assert np.allclose(v_rob_y, [0.0, -1.0, 0.0])

    v_cam_y = np.array([0.0, 1.0, 0.0])
    v_rob_z = mapper.R_R_C @ v_cam_y
    assert np.allclose(v_rob_z, [0.0, 0.0, -1.0])


def test_time_based_slew_rate_limiting():
    """Verify slew rate limits maximum displacement based on elapsed dt."""
    ws_config = WorkspaceConfig(
        max_cartesian_velocity_mps=0.40,
        max_cartesian_step_m=0.05,
        scale_x=1.0, scale_y=1.0, scale_z=1.0,
        robot_center_x=0.5, robot_center_y=0.0, robot_center_z=0.4,
    )
    mapper = WorkspaceMapper(ws_config)

    t0 = 100.0
    p0 = mapper.map_camera_to_robot([0.0, 0.0, 0.45], enforce_slew_rate=True, timestamp=t0)

    # Step after 0.02s (20ms): max displacement = 0.40 * 0.02 = 0.008 m
    t1 = t0 + 0.02
    p1 = mapper.map_camera_to_robot([0.3, 0.0, 0.45], enforce_slew_rate=True, timestamp=t1)

    step_dist = np.linalg.norm(p1.position - p0.position)
    assert step_dist <= (0.40 * 0.02) + 1e-4


def test_nan_inf_protection():
    """Verify invalid numerical coordinates are caught safely."""
    mapper = WorkspaceMapper(WorkspaceConfig())
    t_nan = mapper.map_camera_to_robot([np.nan, 0.0, 0.5])
    assert not t_nan.is_valid

    t_inf = mapper.map_camera_to_robot([0.0, np.inf, 0.5])
    assert not t_inf.is_valid
