"""Tests for GuardedRobotBackend decorator, state machine, and fail-closed authorization."""

import time
from unittest.mock import MagicMock
import pytest

from robotics.backends.base import (
    BackendCommandDisabledError,
    BackendError,
    BackendHealthStatus,
    RobotBackend,
    RobotBackendCapabilities,
    TimestampedJointState,
)
from robotics.robot_model import (
    JointMotionType,
    JointRole,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)
from robotics.safety import (
    CommandSafetyConfig,
    CommandSafetyFaultCode,
    CommandSafetyViolationError,
    GuardedRobotBackend,
    SafetyGuardState,
)


def _make_model_and_backend(dof=2):
    joints = tuple(
        ResolvedJointMetadata(
            model_index=i,
            canonical_index=i,
            name=f"j{i+1}",
            role=JointRole.ARM,
            motion_type=JointMotionType.REVOLUTE,
            lower_limit=-2.0,
            upper_limit=2.0,
            max_force=50.0,
            max_velocity=1.5,
            link_name=f"l{i+1}",
        )
        for i in range(dof)
    )
    model = ResolvedRobotModel(
        robot_id="test_bot",
        display_name="TestBot",
        all_joints=joints,
        arm_joints=joints,
        gripper_joints=(),
        ee_link_name=f"l{dof}",
        home_joint_positions=tuple(0.0 for _ in range(dof)),
    )

    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True
    mock_backend.health_status.return_value = BackendHealthStatus.HEALTHY
    mock_backend.transport_capabilities = RobotBackendCapabilities(
        read_only=False,
        position_commands=True,
        velocity_commands=True,
        effort_limit_override=False,
        halt_motion=True,
    )

    state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=time.monotonic(),
        joint_names=tuple(f"j{i+1}" for i in range(dof)),
        positions=tuple(0.0 for _ in range(dof)),
        velocities=tuple(0.0 for _ in range(dof)),
    )
    mock_backend.get_joint_state.return_value = state
    return model, mock_backend


def test_initial_state_is_disarmed():
    """Verify newly constructed GuardedRobotBackend is DISARMED and commands raise BackendCommandDisabledError."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    assert guard.guard_state == SafetyGuardState.DISARMED
    assert guard.is_armed is False

    with pytest.raises(BackendCommandDisabledError, match="(?i)disarmed"):
        guard.command_joint_positions([0.1, 0.2])

    with pytest.raises(BackendCommandDisabledError, match="(?i)disarmed"):
        guard.command_joint_velocities([0.05, 0.05])


def test_delegation_of_backend_contracts():
    """Verify GuardedRobotBackend delegates is_connected, get_joint_state, and transport_capabilities."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    assert guard.is_connected() is True
    assert guard.transport_capabilities.position_commands is True
    assert guard.health_status() == BackendHealthStatus.HEALTHY

    state = guard.get_joint_state()
    assert state.joint_names == ("j1", "j2")


def test_evaluate_readiness_and_arm_success():
    """Verify preflight check passes on healthy backend and arm() transitions to ARMED."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    report = guard.evaluate_readiness()
    assert report.ready_to_arm is True
    assert report.backend_connected is True
    assert report.backend_health_ok is True
    assert report.state_fresh is True
    assert report.model_match is True

    guard.arm()
    assert guard.is_armed is True
    assert guard.guard_state == SafetyGuardState.ARMED


def test_arm_fails_when_backend_disconnected():
    """Verify arm() fails and remains DISARMED when underlying backend is disconnected."""
    model, mock_backend = _make_model_and_backend()
    mock_backend.is_connected.return_value = False
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    with pytest.raises(BackendError, match="(?i)disconnected"):
        guard.arm()

    assert guard.guard_state == SafetyGuardState.DISARMED
    assert guard.is_armed is False


def test_arm_fails_when_joint_names_mismatch():
    """Verify arm() fails when telemetry joint names do not match ResolvedRobotModel."""
    model, mock_backend = _make_model_and_backend()
    bad_state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=time.monotonic(),
        joint_names=("wrong_1", "wrong_2"),
        positions=(0.0, 0.0),
    )
    mock_backend.get_joint_state.return_value = bad_state
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    with pytest.raises(BackendError, match="(?i)do not match"):
        guard.arm()

    assert guard.guard_state == SafetyGuardState.DISARMED


def test_arm_fails_when_telemetry_stale():
    """Verify arm() fails when telemetry state receive timestamp is older than state_timeout_s."""
    model, mock_backend = _make_model_and_backend()
    stale_state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=time.monotonic() - 5.0,  # 5 seconds old vs 0.5s timeout
        joint_names=("j1", "j2"),
        positions=(0.0, 0.0),
    )
    mock_backend.get_joint_state.return_value = stale_state
    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(state_timeout_s=0.5),
    )

    with pytest.raises(BackendError, match="(?i)stale"):
        guard.arm()

    assert guard.guard_state == SafetyGuardState.DISARMED
