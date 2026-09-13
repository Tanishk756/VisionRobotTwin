"""Unit tests for workspace mapping, axis transforms, boundaries, and slew-rate safety."""

import numpy as np
import pytest

from config.settings import WorkspaceConfig
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

    # Extremely large positive and negative camera translations
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


def test_axis_mapping_direction():
    """Verify camera movement maps intuitively into robot Cartesian space."""
    ws_config = WorkspaceConfig(
        x_min=0.0, x_max=1.0,
        y_min=-1.0, y_max=1.0,
        z_min=0.0, z_max=1.0,
        scale_x=1.0, scale_y=1.0, scale_z=1.0,
        robot_center_x=0.5, robot_center_y=0.0, robot_center_z=0.4,
        cam_center_x=0.0, cam_center_y=0.0, cam_center_z=0.5,
    )
    mapper = WorkspaceMapper(ws_config)

    # Center position
    center_cam = np.array([0.0, 0.0, 0.5])
    t_center = mapper.map_camera_to_robot(center_cam, enforce_slew_rate=False)
    assert np.allclose(t_center.position, [0.5, 0.0, 0.4])

    # Move marker upward (-Y in camera frame) -> should increase robot Z
    up_cam = np.array([0.0, -0.1, 0.5])
    t_up = mapper.map_camera_to_robot(up_cam, enforce_slew_rate=False)
    assert t_up.z > t_center.z

    # Move marker farther away (+Z in camera frame) -> should increase robot X (reach forward)
    away_cam = np.array([0.0, 0.0, 0.6])
    t_away = mapper.map_camera_to_robot(away_cam, enforce_slew_rate=False)
    assert t_away.x > t_center.x


def test_slew_rate_limiting():
    """Verify slew rate limits maximum displacement per cycle to eliminate sudden jumps."""
    ws_config = WorkspaceConfig(
        max_cartesian_step_m=0.02,
        scale_x=1.0, scale_y=1.0, scale_z=1.0,
        robot_center_x=0.5, robot_center_y=0.0, robot_center_z=0.4,
    )
    mapper = WorkspaceMapper(ws_config)

    # Initialize at center
    p0 = mapper.map_camera_to_robot([0.0, 0.0, 0.45], enforce_slew_rate=True)
    # Sudden large step
    p1 = mapper.map_camera_to_robot([0.3, 0.0, 0.45], enforce_slew_rate=True)

    step_dist = np.linalg.norm(p1.position - p0.position)
    assert step_dist <= ws_config.max_cartesian_step_m + 1e-6


def test_nan_inf_protection():
    """Verify invalid numerical coordinates are caught safely."""
    mapper = WorkspaceMapper(WorkspaceConfig())
    t_nan = mapper.map_camera_to_robot([np.nan, 0.0, 0.5])
    assert not t_nan.is_valid

    t_inf = mapper.map_camera_to_robot([0.0, np.inf, 0.5])
    assert not t_inf.is_valid
