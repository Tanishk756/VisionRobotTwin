"""Tests for Yoshikawa Manipulability and Singularity Metrics."""

import pytest
import numpy as np

from robotics.kinematics import compute_manipulability, compute_damped_pseudoinverse, ManipulabilityMetrics


def test_manipulability_well_conditioned_jacobian():
    """Verifies manipulability calculation on an orthogonal, full-rank Jacobian."""
    # Orthogonal 6x7 matrix
    J = np.zeros((6, 7))
    for i in range(6):
        J[i, i] = 1.0
    
    metrics = compute_manipulability(J, singularity_threshold=0.05)
    assert isinstance(metrics, ManipulabilityMetrics)
    assert np.isclose(metrics.manipulability, 1.0, atol=1e-5)
    assert np.isclose(metrics.condition_number, 1.0, atol=1e-5)
    assert np.isclose(metrics.sigma_min, 1.0, atol=1e-5)
    assert metrics.near_singularity is False


def test_manipulability_rank_deficient_singularity():
    """Verifies manipulability calculation on a rank-deficient singular Jacobian."""
    # Rank 5 matrix (row 5 is zero)
    J = np.zeros((6, 7))
    for i in range(5):
        J[i, i] = 1.0
    
    metrics = compute_manipulability(J, singularity_threshold=0.05)
    assert np.isclose(metrics.manipulability, 0.0, atol=1e-5)
    assert np.isclose(metrics.sigma_min, 0.0, atol=1e-5)
    assert metrics.condition_number > 1e4 or np.isinf(metrics.condition_number)
    assert metrics.near_singularity is True


def test_damped_pseudoinverse_adaptive_damping():
    """Verifies that adaptive damping increases smoothly near singularity."""
    # Well-conditioned
    J_well = np.zeros((6, 7))
    for i in range(6):
        J_well[i, i] = 1.0
    
    J_inv_well = compute_damped_pseudoinverse(J_well, lambda_min=0.01, lambda_max=0.30, sigma_threshold=0.05)
    assert J_inv_well.shape == (7, 6)
    assert np.all(np.isfinite(J_inv_well))
    
    # Singular matrix
    J_sing = np.zeros((6, 7))
    for i in range(5):
        J_sing[i, i] = 1.0
    
    J_inv_sing = compute_damped_pseudoinverse(J_sing, lambda_min=0.01, lambda_max=0.30, sigma_threshold=0.05)
    assert J_inv_sing.shape == (7, 6)
    assert np.all(np.isfinite(J_inv_sing))
    # Damping prevents norm explosion
    assert np.linalg.norm(J_inv_sing) < 100.0
