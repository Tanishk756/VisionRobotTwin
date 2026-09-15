"""Robotics execution backend interfaces and implementations."""

from typing import Optional, Sequence
from robotics.backends.base import (
    BackendHealthStatus,
    RobotBackend,
    TimestampedJointState,
)
from robotics.backends.mock_backend import MockRobotBackend
from robotics.backends.pybullet_backend import PyBulletRobotBackend


def create_pybullet_backend(
    physics_client_id: int,
    robot_body_id: int,
    arm_joint_indices: Sequence[int],
    joint_names: Sequence[str],
    default_joint_force: float = 200.0,
) -> PyBulletRobotBackend:
    """Factory helper to construct and connect a PyBulletRobotBackend."""
    backend = PyBulletRobotBackend(
        physics_client_id=physics_client_id,
        robot_body_id=robot_body_id,
        arm_joint_indices=arm_joint_indices,
        joint_names=joint_names,
        default_joint_force=default_joint_force,
    )
    backend.connect()
    return backend


def create_mock_backend(
    joint_names: Sequence[str],
    initial_positions: Optional[Sequence[float]] = None,
) -> MockRobotBackend:
    """Factory helper to construct and connect a MockRobotBackend (test infrastructure)."""
    backend = MockRobotBackend(
        joint_names=joint_names,
        initial_positions=initial_positions,
    )
    backend.connect()
    return backend


__all__ = [
    "BackendHealthStatus",
    "MockRobotBackend",
    "PyBulletRobotBackend",
    "RobotBackend",
    "TimestampedJointState",
    "create_mock_backend",
    "create_pybullet_backend",
]
