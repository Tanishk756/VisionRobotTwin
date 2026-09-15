"""Tests for RobotBackend contract, BackendHealthStatus, and TimestampedJointState."""

import dataclasses
import numpy as np
import pytest

from robotics.backends.base import (
    BackendHealthStatus,
    RobotBackend,
    TimestampedJointState,
)


def test_timestamped_joint_state_fields_and_immutability():
    """Verify that TimestampedJointState stores immutable tuples and rejects mutation."""
    state = TimestampedJointState(
        source_timestamp_s=100.0,
        receive_timestamp_s=100.002,
        joint_names=("joint1", "joint2"),
        positions=(0.1, 0.2),
        velocities=(0.01, -0.02),
        efforts=(5.0, 10.0),
        sequence_id=42,
    )

    assert state.source_timestamp_s == 100.0
    assert state.receive_timestamp_s == 100.002
    assert state.joint_names == ("joint1", "joint2")
    assert state.positions == (0.1, 0.2)
    assert state.velocities == (0.01, -0.02)
    assert state.efforts == (5.0, 10.0)
    assert state.sequence_id == 42

    with pytest.raises(dataclasses.FrozenInstanceError):
        state.sequence_id = 43  # type: ignore[misc]


def test_timestamped_joint_state_validation():
    """Verify validation of lengths and numeric finiteness."""
    # Mismatched positions length
    with pytest.raises(ValueError, match="Length mismatch"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1", "j2"),
            positions=(0.1,),
            velocities=(0.0, 0.0),
        )

    # Mismatched velocities length
    with pytest.raises(ValueError, match="Length mismatch"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1", "j2"),
            positions=(0.1, 0.2),
            velocities=(0.0,),
        )

    # Mismatched efforts length
    with pytest.raises(ValueError, match="Length mismatch"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1", "j2"),
            positions=(0.1, 0.2),
            velocities=(0.0, 0.0),
            efforts=(1.0,),
        )

    # Non-finite position
    with pytest.raises(ValueError, match="Non-finite"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1",),
            positions=(float("nan"),),
            velocities=(0.0,),
        )

    # Non-finite velocity
    with pytest.raises(ValueError, match="Non-finite"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1",),
            positions=(0.0,),
            velocities=(float("inf"),),
        )

    # Non-finite effort
    with pytest.raises(ValueError, match="Non-finite"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(float("nan"),),
        )


def test_timestamped_joint_state_array_copies():
    """Verify that array getter methods return new, copy-safe NumPy arrays."""
    state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=("j1", "j2"),
        positions=(0.5, -0.5),
        velocities=(0.1, -0.1),
        efforts=(2.0, 4.0),
    )

    pos_arr = state.get_positions_array()
    assert isinstance(pos_arr, np.ndarray)
    np.testing.assert_array_equal(pos_arr, [0.5, -0.5])
    pos_arr[0] = 999.0
    assert state.positions[0] == 0.5  # Original remains unaffected

    vel_arr = state.get_velocities_array()
    assert isinstance(vel_arr, np.ndarray)
    np.testing.assert_array_equal(vel_arr, [0.1, -0.1])
    vel_arr[0] = 999.0
    assert state.velocities[0] == 0.1

    eff_arr = state.get_efforts_array()
    assert isinstance(eff_arr, np.ndarray)
    np.testing.assert_array_equal(eff_arr, [2.0, 4.0])
    eff_arr[0] = 999.0
    assert state.efforts[0] == 2.0

    # Test state without efforts
    state_no_efforts = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=("j1",),
        positions=(0.0,),
        velocities=(0.0,),
    )
    assert state_no_efforts.get_efforts_array() is None


def test_timestamped_joint_state_age_and_freshness():
    """Verify age calculation and timeout freshness evaluations."""
    state = TimestampedJointState(
        source_timestamp_s=10.0,
        receive_timestamp_s=10.0,
        joint_names=("j1",),
        positions=(0.0,),
        velocities=(0.0,),
    )

    assert state.age_s(10.0) == 0.0
    assert state.age_s(10.05) == pytest.approx(0.05)
    assert state.age_s(9.0) == 0.0  # Monotonic clamp at 0.0

    assert state.is_fresh(current_time_s=10.02, timeout_s=0.05) is True
    assert state.is_fresh(current_time_s=10.10, timeout_s=0.05) is False


def test_backend_health_status_enum():
    """Verify enum members exist and are distinct."""
    assert BackendHealthStatus.HEALTHY is not None
    assert BackendHealthStatus.DEGRADED is not None
    assert BackendHealthStatus.DISCONNECTED is not None
    assert BackendHealthStatus.ERROR is not None


def test_robot_backend_abstract_methods_cannot_be_instantiated():
    """Verify RobotBackend is an abstract base class that cannot be instantiated directly."""
    with pytest.raises(TypeError):
        RobotBackend()  # type: ignore[abstract]
