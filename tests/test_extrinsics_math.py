"""Unit tests for World-Anchor Extrinsic Calibration mathematics and SE(3) solvers."""

import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from vision.extrinsics import (
    compute_robot_to_camera_transform,
    validate_extrinsic_transform,
)
from robotics.coordinate_transform import (
    create_homogeneous_matrix,
    invert_homogeneous_matrix,
    rotation_matrix_to_quaternion,
    is_valid_se3,
    compute_angular_distance,
)


def test_extrinsic_math_identity_case():
    """Validates T_robot_camera when anchor is at identity in both frames."""
    T_robot_anchor = np.eye(4, dtype=np.float64)
    T_camera_anchor = np.eye(4, dtype=np.float64)

    T_robot_camera = compute_robot_to_camera_transform(T_robot_anchor, T_camera_anchor)

    assert is_valid_se3(T_robot_camera)
    assert np.allclose(T_robot_camera, np.eye(4), atol=1e-6)


def test_extrinsic_math_pure_translation():
    """Validates analytical solution with pure Cartesian translations."""
    # Robot to Anchor: X=0.6m, Y=0.1m, Z=0.0m
    T_robot_anchor = create_homogeneous_matrix((0.0, 0.0, 0.0), (0.6, 0.1, 0.0))
    # Camera to Anchor: X=0.0m, Y=0.0m, Z=0.5m
    T_camera_anchor = create_homogeneous_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.5))

    T_robot_camera = compute_robot_to_camera_transform(T_robot_anchor, T_camera_anchor)

    assert is_valid_se3(T_robot_camera)
    # Expected camera in robot frame: X = 0.6, Y = 0.1, Z = -0.5
    expected_cam_pos = np.array([0.6, 0.1, -0.5])
    assert np.allclose(T_robot_camera[:3, 3], expected_cam_pos, atol=1e-5)


def test_extrinsic_math_90deg_rotation():
    """Validates analytical solution with 90-degree yaw rotation."""
    # Anchor in robot frame rotated by 90 deg (pi/2) yaw
    T_robot_anchor = create_homogeneous_matrix((0.0, 0.0, np.pi / 2.0), (0.5, 0.0, 0.2))
    # Anchor in camera frame without rotation
    T_camera_anchor = create_homogeneous_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.4))

    T_robot_camera = compute_robot_to_camera_transform(T_robot_anchor, T_camera_anchor)

    assert is_valid_se3(T_robot_camera)
    # Validate point transformation consistency
    # A point on the anchor P_anchor = [0, 0, 0]
    # P_camera = T_camera_anchor @ [0,0,0,1]^T = [0, 0, 0.4, 1]
    # P_robot = T_robot_camera @ P_camera
    p_anchor = np.array([0.0, 0.0, 0.0, 1.0])
    p_cam = T_camera_anchor @ p_anchor
    p_robot_via_cam = T_robot_camera @ p_cam
    p_robot_direct = T_robot_anchor @ p_anchor

    assert np.allclose(p_robot_via_cam, p_robot_direct, atol=1e-5)


def test_extrinsic_math_combined_rotation_and_translation():
    """Validates full 6-DoF composite transformation with arbitrary RPY and translation."""
    T_robot_anchor = create_homogeneous_matrix((0.2, -0.4, 0.8), (0.45, -0.15, 0.35))
    T_camera_anchor = create_homogeneous_matrix((-0.1, 0.3, -0.5), (0.05, -0.02, 0.60))

    T_robot_camera = compute_robot_to_camera_transform(T_robot_anchor, T_camera_anchor)

    assert is_valid_se3(T_robot_camera)

    # Test random 3D points
    np.random.seed(42)
    for _ in range(10):
        p_anchor = np.array([*np.random.uniform(-0.1, 0.1, size=3), 1.0])
        p_cam = T_camera_anchor @ p_anchor
        p_robot_expected = T_robot_anchor @ p_anchor
        p_robot_calc = T_robot_camera @ p_cam
        assert np.allclose(p_robot_calc, p_robot_expected, atol=1e-5)


def test_extrinsic_validation_zero_residual():
    """Validates validate_extrinsic_transform produces zero error for perfect observations."""
    T_robot_anchor = create_homogeneous_matrix((np.pi, 0.0, 0.0), (0.50, 0.0, 0.0))
    T_camera_anchor = create_homogeneous_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.45))

    T_robot_camera = compute_robot_to_camera_transform(T_robot_anchor, T_camera_anchor)

    trans_err_mm, rot_err_deg = validate_extrinsic_transform(
        T_robot_camera=T_robot_camera,
        T_camera_anchor_measured=T_camera_anchor,
        T_robot_anchor_expected=T_robot_anchor,
    )

    assert trans_err_mm < 1e-3
    assert rot_err_deg < 1e-3


def test_extrinsic_math_invalid_inputs():
    """Asserts that invalid shapes, NaN, or Inf inputs raise ValueError."""
    T_valid = np.eye(4)
    T_invalid_shape = np.eye(3)
    T_nan = np.eye(4)
    T_nan[0, 0] = np.nan

    with pytest.raises(ValueError):
        compute_robot_to_camera_transform(T_invalid_shape, T_valid)

    with pytest.raises(ValueError):
        compute_robot_to_camera_transform(T_valid, T_nan)
