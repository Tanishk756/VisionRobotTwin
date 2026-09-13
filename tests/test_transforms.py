"""Unit tests for SE(3) kinematics, rigid body transformations, and coordinate frames."""

import math
import numpy as np
import pytest

from robotics.coordinate_transform import (
    SE3Transform,
    create_homogeneous_matrix,
    invert_homogeneous_matrix,
    compose_transforms,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    multiply_quaternions,
    compute_angular_distance,
    is_valid_se3,
)


def test_rotation_matrix_orthogonality():
    """Verify rotation matrices satisfy R @ R.T = I and det(R) = +1."""
    angles = [
        (0.0, 0.0, 0.0),
        (math.pi / 4, -math.pi / 6, math.pi / 3),
        (math.pi, 0.26, -0.5),
        (-math.pi / 2, math.pi / 2, 0.0),
    ]
    for roll, pitch, yaw in angles:
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        assert R.shape == (3, 3)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)


def test_euler_and_quaternion_roundtrip():
    """Verify seamless bidirectional conversions between Euler, Matrix, and Quaternions."""
    roll, pitch, yaw = 0.2, -0.4, 0.6
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    r_est, p_est, y_est = rotation_matrix_to_euler(R)

    assert np.isclose(roll, r_est, atol=1e-5)
    assert np.isclose(pitch, p_est, atol=1e-5)
    assert np.isclose(yaw, y_est, atol=1e-5)

    quat = rotation_matrix_to_quaternion(R)
    assert np.isclose(np.linalg.norm(quat), 1.0, atol=1e-6)

    R_from_q = quaternion_to_rotation_matrix(quat)
    assert np.allclose(R, R_from_q, atol=1e-6)


def test_quaternion_multiplication_and_angular_distance():
    """Verify Hamilton product and geodesic distance calculation."""
    q_ident = np.array([0.0, 0.0, 0.0, 1.0])
    q_rot_z90 = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])

    q_mult = multiply_quaternions(q_ident, q_rot_z90)
    assert np.allclose(q_mult, q_rot_z90, atol=1e-6)

    dist = compute_angular_distance(q_ident, q_rot_z90)
    assert np.isclose(dist, math.pi / 2, atol=1e-5)


def test_se3_inverse():
    """Verify analytical homogeneous transform inversion: T @ T^-1 == I."""
    t = [0.5, -0.3, 0.7]
    rpy = (0.3, -0.2, 0.5)
    T = create_homogeneous_matrix(rpy, t)
    T_inv = invert_homogeneous_matrix(T)

    assert is_valid_se3(T)
    assert is_valid_se3(T_inv)

    identity = np.eye(4)
    assert np.allclose(T @ T_inv, identity, atol=1e-6)
    assert np.allclose(T_inv @ T, identity, atol=1e-6)


def test_se3_composition_and_associativity():
    """Verify transform composition chain T_1 @ T_2 @ T_3."""
    T1 = SE3Transform.from_rpy(0.1, 0.2, 0.3, 0.1, 0.0, 0.0)
    T2 = SE3Transform.from_rpy(0.4, -0.1, 0.2, 0.0, 0.2, 0.0)
    T3 = SE3Transform.from_rpy(-0.2, 0.3, 0.1, 0.0, 0.0, 0.3)

    T_chain1 = (T1 @ T2) @ T3
    T_chain2 = T1 @ (T2 @ T3)

    assert np.allclose(T_chain1.matrix, T_chain2.matrix, atol=1e-6)
    assert T_chain1.is_valid()


def test_point_transformation():
    """Verify transforming a 3D point from marker frame to robot base frame."""
    T_base_cam = SE3Transform.from_rpy(0.7, 0.0, 0.4, math.pi, 0.0, 0.0)
    T_cam_marker = SE3Transform.from_rpy(0.0, 0.0, 0.4, 0.0, 0.0, 0.0)

    T_base_marker = T_base_cam @ T_cam_marker
    marker_origin_in_base = T_base_marker.transform_point([0, 0, 0])

    assert np.allclose(marker_origin_in_base, T_base_marker.translation, atol=1e-6)
