"""Unit tests for workspace mapping, SE(3) transformation modes, boundaries, and slew-rate safety."""

import math
import numpy as np
import pytest

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
