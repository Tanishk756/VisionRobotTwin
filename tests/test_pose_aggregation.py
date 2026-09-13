"""Unit tests for robust pose aggregation and outlier rejection."""

import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from vision.extrinsics import aggregate_camera_poses
from robotics.coordinate_transform import create_homogeneous_matrix


def test_aggregate_camera_poses_clean_samples():
    """Validates aggregation on clean Gaussian-distributed pose samples."""
    np.random.seed(42)
    base_t = np.array([0.02, -0.01, 0.48])
    base_rpy = (0.05, -0.02, 0.01)

    transforms = []
    for _ in range(20):
        noisy_t = base_t + np.random.normal(0, 0.001, size=3)  # 1 mm noise
        noisy_rpy = [r + np.random.normal(0, 0.002) for r in base_rpy]
        transforms.append(create_homogeneous_matrix(tuple(noisy_rpy), noisy_t))

    robust_T, rejected, t_std_mm, r_std_deg = aggregate_camera_poses(
        transforms,
        max_pos_deviation_m=0.03,
        max_rot_deviation_deg=10.0,
    )

    assert rejected == 0
    assert np.allclose(robust_T[:3, 3], base_t, atol=2e-3)
    assert t_std_mm < 3.0  # < 3mm std
    assert r_std_deg < 1.0  # < 1 deg std


def test_aggregate_camera_poses_with_outliers():
    """Asserts that severe positional or rotational outliers are rejected."""
    np.random.seed(123)
    base_t = np.array([0.00, 0.00, 0.50])
    base_rpy = (0.0, 0.0, 0.0)

    transforms = []
    # 15 nominal samples
    for _ in range(15):
        t = base_t + np.random.normal(0, 0.0005, size=3)
        transforms.append(create_homogeneous_matrix(base_rpy, t))

    # Add 2 large positional outliers (10 cm off)
    transforms.append(create_homogeneous_matrix(base_rpy, base_t + np.array([0.10, 0.0, 0.0])))
    transforms.append(create_homogeneous_matrix(base_rpy, base_t + np.array([0.0, -0.10, 0.0])))

    # Add 1 large rotational outlier (30 deg off)
    transforms.append(create_homogeneous_matrix((0.52, 0.0, 0.0), base_t))

    robust_T, rejected, t_std_mm, r_std_deg = aggregate_camera_poses(
        transforms,
        max_pos_deviation_m=0.03,
        max_rot_deviation_deg=10.0,
    )

    assert rejected >= 3
    assert np.allclose(robust_T[:3, 3], base_t, atol=1e-3)
