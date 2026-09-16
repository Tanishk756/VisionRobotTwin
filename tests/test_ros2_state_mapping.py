"""Tests for pure ROS-independent JointState message mapping and validation."""

import math
import pytest
from robotics.backends.base import TimestampedJointState
from robotics.backends.ros2_state_mapping import (
    map_joint_state_payload,
    extract_ros_timestamp,
    JointStateMappingError,
)


def test_canonical_and_permuted_joint_ordering():
    """Verify that incoming joint positions are mapped into exact expected_joint_names order."""
    expected = ("panda_joint1", "panda_joint2", "panda_joint3")

    # Canonical order
    state_can = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["panda_joint1", "panda_joint2", "panda_joint3"],
        positions=[0.1, 0.2, 0.3],
        velocities=[0.01, 0.02, 0.03],
        efforts=[1.0, 2.0, 3.0],
        source_timestamp_s=10.0,
        receive_timestamp_s=10.002,
        sequence_id=1,
    )
    assert isinstance(state_can, TimestampedJointState)
    assert state_can.joint_names == expected
    assert state_can.positions == (0.1, 0.2, 0.3)
    assert state_can.velocities == (0.01, 0.02, 0.03)
    assert state_can.efforts == (1.0, 2.0, 3.0)
    assert state_can.source_timestamp_s == 10.0
    assert state_can.receive_timestamp_s == 10.002
    assert state_can.sequence_id == 1

    # Reversed order
    state_rev = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["panda_joint3", "panda_joint2", "panda_joint1"],
        positions=[0.3, 0.2, 0.1],
        velocities=[0.03, 0.02, 0.01],
        efforts=[3.0, 2.0, 1.0],
        source_timestamp_s=11.0,
        receive_timestamp_s=11.002,
        sequence_id=2,
    )
    assert state_rev.joint_names == expected
    assert state_rev.positions == (0.1, 0.2, 0.3)
    assert state_rev.velocities == (0.01, 0.02, 0.03)
    assert state_rev.efforts == (1.0, 2.0, 3.0)

    # Arbitrary permutation
    state_perm = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["panda_joint2", "panda_joint1", "panda_joint3"],
        positions=[0.2, 0.1, 0.3],
        velocities=[0.02, 0.01, 0.03],
        efforts=[2.0, 1.0, 3.0],
        source_timestamp_s=12.0,
        receive_timestamp_s=12.002,
        sequence_id=3,
    )
    assert state_perm.joint_names == expected
    assert state_perm.positions == (0.1, 0.2, 0.3)


def test_extra_joints_pruning():
    """Verify that extraneous joints (gripper fingers, mobile base, etc.) are safely ignored."""
    expected = ("j1", "j2")
    state = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["finger1", "j2", "wheel_left", "j1", "finger2"],
        positions=[0.04, -0.5, 1.5, 0.5, 0.04],
        velocities=[0.0, -0.05, 0.1, 0.05, 0.0],
        efforts=[0.0, 10.0, 2.0, -10.0, 0.0],
        source_timestamp_s=5.0,
        receive_timestamp_s=5.001,
        sequence_id=10,
    )
    assert state.joint_names == expected
    assert state.positions == (0.5, -0.5)
    assert state.velocities == (0.05, -0.05)
    assert state.efforts == (-10.0, 10.0)


def test_missing_expected_joint_rejection():
    """Verify that missing expected joints raise JointStateMappingError."""
    expected = ("j1", "j2", "j3")
    with pytest.raises(JointStateMappingError, match="Missing expected joint"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1, 0.2],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_duplicate_incoming_names_rejection():
    """Verify that duplicate incoming joint names raise JointStateMappingError due to ambiguity."""
    expected = ("j1", "j2")
    with pytest.raises(JointStateMappingError, match="Duplicate"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j1", "j2"],
            positions=[0.1, 0.2, 0.3],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_empty_names_and_positions_rejection():
    """Verify that empty names or positions are rejected."""
    expected = ("j1",)
    with pytest.raises(JointStateMappingError, match="empty"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=[],
            positions=[],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    with pytest.raises(JointStateMappingError, match="empty"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_length_mismatch_rejection():
    """Verify that array length mismatches raise JointStateMappingError."""
    expected = ("j1", "j2")

    # Position length != name length
    with pytest.raises(JointStateMappingError, match="Position length"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Velocity length != 0 and != name length
    with pytest.raises(JointStateMappingError, match="Velocity length"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1, 0.2],
            velocities=[0.01],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Effort length != 0 and != name length
    with pytest.raises(JointStateMappingError, match="Effort length"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1, 0.2],
            velocities=[],
            efforts=[1.0, 2.0, 3.0],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_optional_velocity_and_effort_handling():
    """Verify that empty velocity and effort arrays result in None."""
    expected = ("j1", "j2")
    state = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["j1", "j2"],
        positions=[0.5, -0.5],
        velocities=[],
        efforts=None,
        source_timestamp_s=1.0,
        receive_timestamp_s=1.002,
        sequence_id=5,
    )
    assert state.positions == (0.5, -0.5)
    assert state.velocities is None
    assert state.efforts is None


def test_nan_and_inf_rejection():
    """Verify that NaN or +/-Inf in positions, velocities, or efforts raises JointStateMappingError."""
    expected = ("j1",)

    # NaN in position
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[float("nan")],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Inf in position
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[float("inf")],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # NaN in velocity
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[0.0],
            velocities=[float("nan")],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Inf in effort
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[0.0],
            velocities=[0.0],
            efforts=[float("-inf")],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_extract_ros_timestamp():
    """Verify conversion of ROS sec/nanosec header stamps into float seconds."""
    assert extract_ros_timestamp(1694700000, 500000000) == 1694700000.5
    assert extract_ros_timestamp(0, 0) == 0.0
    assert extract_ros_timestamp(10, 123456789) == 10.123456789

    # Non-finite or negative nanosec
    with pytest.raises(JointStateMappingError):
        extract_ros_timestamp(10, -1)
    with pytest.raises(JointStateMappingError):
        extract_ros_timestamp(-1, 0)
