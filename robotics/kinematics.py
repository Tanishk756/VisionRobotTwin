"""Differential Kinematics, Geometric Jacobian, and Manipulability Metrics.

Provides spatial Jacobian computation, Yoshikawa manipulability index,
singular value decomposition, condition number analysis, and adaptive damped
least-squares (DLS) pseudoinverse calculation.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

from utils.logger import get_logger

logger = get_logger("Robotics.Kinematics")


@dataclass(frozen=True)
class ManipulabilityMetrics:
    """Quantitative singularity and manipulability diagnostics for a manipulator."""
    manipulability: float
    condition_number: float
    sigma_min: float
    sigma_max: float
    near_singularity: bool


def compute_fk_at_configuration(
    physics_client_id: int,
    robot_id: int,
    arm_joint_indices: List[int],
    joint_positions: List[float],
    ee_link_index: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """Computes forward kinematics (EE position and quaternion) for a candidate joint configuration.

    Legacy backward-compatibility wrapper delegating to PyBulletKinematicsProvider.

    Args:
        physics_client_id: PyBullet client ID.
        robot_id: PyBullet robot body ID.
        arm_joint_indices: Controllable arm joint indices.
        joint_positions: Candidate joint positions (rad).
        ee_link_index: Link index for the end effector.

    Returns:
        (position, quaternion_xyzw) as (np.ndarray shape (3,), np.ndarray shape (4,)).
    """
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    provider = PyBulletKinematicsProvider(
        physics_client_id=physics_client_id,
        robot_body_id=robot_id,
        arm_joint_indices=arm_joint_indices,
        end_effector_link_index=ee_link_index,
    )
    return provider.compute_fk(joint_positions)


def compute_jacobian(
    physics_client_id: int,
    robot_id: int,
    ee_link_index: int,
    arm_joint_indices: List[int],
    joint_positions: List[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Computes spatial geometric Jacobian at the current joint configuration.

    Legacy backward-compatibility wrapper delegating to PyBulletKinematicsProvider.

    Args:
        physics_client_id: PyBullet client ID.
        robot_id: PyBullet robot body ID.
        ee_link_index: End-effector link index.
        arm_joint_indices: Controllable arm joint indices.
        joint_positions: Current joint angles for arm joints (rad).

    Returns:
        (J_linear, J_angular, J_full) where J_full has shape (6, n).
    """
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    provider = PyBulletKinematicsProvider(
        physics_client_id=physics_client_id,
        robot_body_id=robot_id,
        arm_joint_indices=arm_joint_indices,
        end_effector_link_index=ee_link_index,
    )
    return provider.compute_jacobian(joint_positions)


def compute_manipulability(
    J: np.ndarray,
    singularity_threshold: float = 0.05,
) -> ManipulabilityMetrics:
    """Calculates Yoshikawa manipulability index and SVD metrics for Jacobian J (6 x n).

    w = sqrt(det(J * J^T)) = prod(sigma_i)

    Args:
        J: (6, n) geometric Jacobian matrix.
        singularity_threshold: Threshold below which sigma_min triggers near_singularity.

    Returns:
        ManipulabilityMetrics instance.
    """
    try:
        U, S, Vt = np.linalg.svd(J)
        # S has 6 singular values in descending order
        sigma_max = float(S[0]) if len(S) > 0 else 0.0
        sigma_min = float(S[-1]) if len(S) > 0 else 0.0

        # Yoshikawa index: product of singular values
        manipulability = float(np.prod(S)) if len(S) >= 6 else 0.0

        # Condition number: sigma_max / sigma_min (clipped for numerical stability)
        if sigma_min > 1e-9:
            condition_number = float(sigma_max / sigma_min)
        else:
            condition_number = float("inf")

        near_singularity = bool(sigma_min < singularity_threshold)

        return ManipulabilityMetrics(
            manipulability=max(0.0, manipulability),
            condition_number=condition_number,
            sigma_min=max(0.0, sigma_min),
            sigma_max=max(0.0, sigma_max),
            near_singularity=near_singularity,
        )
    except Exception as e:
        logger.warning(f"Failed to compute manipulability: {e}")
        return ManipulabilityMetrics(
            manipulability=0.0,
            condition_number=float("inf"),
            sigma_min=0.0,
            sigma_max=0.0,
            near_singularity=True,
        )


def compute_damped_pseudoinverse(
    J: np.ndarray,
    lambda_min: float = 0.01,
    lambda_max: float = 0.25,
    sigma_threshold: float = 0.05,
) -> np.ndarray:
    """Computes adaptive Damped Least-Squares (DLS) pseudoinverse for Jacobian J (6 x n).

    J_dls = J^T @ inv(J @ J^T + lambda^2 * I)

    Damping lambda scales smoothly from lambda_min (well-conditioned) to lambda_max (near singularity).

    Args:
        J: (6, n) geometric Jacobian matrix.
        lambda_min: Minimum damping factor in dexterous regions.
        lambda_max: Maximum damping factor near singular configurations.
        sigma_threshold: Singular value threshold below which damping increases.

    Returns:
        (n, 6) damped pseudoinverse matrix.
    """
    m, n = J.shape
    try:
        # Calculate smallest singular value to determine adaptive damping
        _, S, _ = np.linalg.svd(J)
        sigma_min = float(S[-1]) if len(S) > 0 else 0.0

        if sigma_min >= sigma_threshold:
            damping = lambda_min
        else:
            ratio = max(0.0, sigma_min / max(sigma_threshold, 1e-6))
            damping = np.sqrt(lambda_min**2 + (1.0 - ratio**2) * (lambda_max**2 - lambda_min**2))

        # Damped Least Squares formula
        JJT = J @ J.T  # (6, 6)
        damped_matrix = JJT + (damping**2) * np.eye(m)
        inv_damped = np.linalg.inv(damped_matrix)
        J_dls = J.T @ inv_damped  # (n, 6)
        return J_dls
    except Exception as e:
        logger.error(f"Error computing damped pseudoinverse: {e}")
        return np.linalg.pinv(J, rcond=1e-3)
