"""Unit and integration tests for the KinematicsProvider interface and providers."""

import pytest
from typing import Sequence, Tuple, Optional
import numpy as np


def test_kinematics_provider_cannot_be_instantiated_directly():
    """KinematicsProvider is an abstract base class and should reject direct instantiation."""
    from robotics.kinematics_provider import KinematicsProvider

    with pytest.raises(TypeError):
        KinematicsProvider()


def test_kinematics_provider_subclass_contract():
    """A concrete implementation of KinematicsProvider must implement compute_fk, compute_jacobian, and solve_ik_raw."""
    from robotics.kinematics_provider import KinematicsProvider

    class DummyProvider(KinematicsProvider):
        def compute_fk(self, joint_positions: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
            return np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])

        def compute_jacobian(
            self, joint_positions: Sequence[float]
        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
            return np.zeros((3, 7)), np.zeros((3, 7)), np.zeros((6, 7))

        def solve_ik_raw(
            self,
            target_position: Sequence[float],
            target_orientation: Optional[Sequence[float]] = None,
            lower_limits: Optional[Sequence[float]] = None,
            upper_limits: Optional[Sequence[float]] = None,
            joint_ranges: Optional[Sequence[float]] = None,
            rest_poses: Optional[Sequence[float]] = None,
            joint_damping: Optional[Sequence[float]] = None,
            max_iterations: int = 100,
            residual_threshold: float = 1e-4,
        ) -> Tuple[float, ...]:
            return tuple([0.0] * 7)

    dummy = DummyProvider()
    pos, orn = dummy.compute_fk([0.0] * 7)
    assert pos.shape == (3,)
    assert orn.shape == (4,)

    j_lin, j_ang, j_full = dummy.compute_jacobian([0.0] * 7)
    assert j_lin.shape == (3, 7)
    assert j_ang.shape == (3, 7)
    assert j_full.shape == (6, 7)

    ik_q = dummy.solve_ik_raw([0.3, 0.0, 0.5])
    assert len(ik_q) == 7
