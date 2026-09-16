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

    with pytest.raises(BackendError, match="(?i)not connected"):
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


def test_position_limit_violation_rejection_and_fault_latching():
    """Verify out-of-bounds position command is rejected, latches fault, and doesn't dispatch."""
    model, mock_backend = _make_model_and_backend()  # Limits: [-2.0, 2.0]
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard.arm()

    # Target exceeding upper limit (2.5 > 2.0)
    with pytest.raises(CommandSafetyViolationError, match="(?i)position limit"):
        guard.command_joint_positions([2.5, 0.0])

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault is not None
    assert guard.active_fault.code == CommandSafetyFaultCode.POSITION_LIMIT_VIOLATION
    assert mock_backend.command_joint_positions.call_count == 0


def test_position_step_jump_violation_rejection():
    """Verify large position jump from current state is rejected and latches POSITION_STEP_VIOLATION."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(max_position_step_by_joint=(0.2, 0.2)),
    )
    guard.arm()

    # Current position is (0.0, 0.0). Jump to (0.5, 0.0) exceeds 0.2 step limit
    with pytest.raises(CommandSafetyViolationError, match="(?i)position step"):
        guard.command_joint_positions([0.5, 0.0])

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault.code == CommandSafetyFaultCode.POSITION_STEP_VIOLATION
    assert mock_backend.command_joint_positions.call_count == 0


def test_velocity_limit_violation_rejection():
    """Verify velocity command exceeding velocity_limit_scale * max_velocity is rejected."""
    model, mock_backend = _make_model_and_backend()  # Max vel: 1.5
    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(velocity_limit_scale=0.8),  # Allowed: 1.2
    )
    guard.arm()

    with pytest.raises(CommandSafetyViolationError, match="(?i)velocity limit"):
        guard.command_joint_velocities([1.4, 0.0])

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault.code == CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION
    assert mock_backend.command_joint_velocities.call_count == 0


def test_require_velocity_feedback_enforcement():
    """Verify velocity command requires velocities in state when configured."""
    model, mock_backend = _make_model_and_backend()
    # State with None velocities
    no_vel_state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=time.monotonic(),
        joint_names=("j1", "j2"),
        positions=(0.0, 0.0),
        velocities=None,
    )
    mock_backend.get_joint_state.return_value = no_vel_state

    # When False: allowed
    guard_ok = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(require_velocity_feedback_for_velocity_commands=False),
    )
    guard_ok.arm()
    assert guard_ok.command_joint_velocities([0.5, 0.0]) is True

    # When True: rejected
    guard_req = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(require_velocity_feedback_for_velocity_commands=True),
    )
    guard_req.arm()
    with pytest.raises(CommandSafetyViolationError, match="(?i)velocity feedback"):
        guard_req.command_joint_velocities([0.5, 0.0])

    assert guard_req.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard_req.active_fault.code == CommandSafetyFaultCode.STATE_INVALID


def test_fault_does_not_auto_clear_and_blocks_further_commands():
    """Verify that once a fault is latched, subsequent valid commands are blocked until reset."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard.arm()

    # Trigger limit violation fault
    with pytest.raises(CommandSafetyViolationError):
        guard.command_joint_positions([3.0, 0.0])

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED

    # Try sending a completely valid command while in FAULT_LATCHED
    with pytest.raises(CommandSafetyViolationError, match="(?i)FAULT_LATCHED"):
        guard.command_joint_positions([0.0, 0.0])

    assert mock_backend.command_joint_positions.call_count == 0


def test_reset_fault_lifecycle_and_transition_to_disarmed():
    """Verify reset_fault clears fault to DISARMED (never ARMED) requiring explicit re-arm."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard.arm()

    # Trigger fault
    with pytest.raises(CommandSafetyViolationError):
        guard.command_joint_positions([3.0, 0.0])

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault is not None

    # Reset fault
    guard.reset_fault()
    assert guard.guard_state == SafetyGuardState.DISARMED
    assert guard.is_armed is False
    assert guard.active_fault is None

    # Command while DISARMED fails
    with pytest.raises(BackendCommandDisabledError):
        guard.command_joint_positions([0.1, 0.1])

    # Re-arm succeeds and enables commanding
    guard.arm()
    assert guard.is_armed is True
    assert guard.command_joint_positions([0.1, 0.1]) is True


def test_reset_fault_rejected_when_readiness_fails():
    """Verify reset_fault fails and remains FAULT_LATCHED if backend prerequisites fail."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard.arm()

    # Trigger fault
    with pytest.raises(CommandSafetyViolationError):
        guard.command_joint_positions([3.0, 0.0])

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED

    # Backend disconnects before reset attempt
    mock_backend.is_connected.return_value = False
    with pytest.raises(BackendError, match="(?i)readiness checks failed"):
        guard.reset_fault()

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED


def test_reset_fault_only_allowed_from_fault_latched():
    """Verify calling reset_fault when DISARMED or ARMED raises BackendError."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    with pytest.raises(BackendError, match="(?i)only allowed from FAULT_LATCHED"):
        guard.reset_fault()

    guard.arm()
    with pytest.raises(BackendError, match="(?i)only allowed from FAULT_LATCHED"):
        guard.reset_fault()


def test_privileged_software_stop_semantics():
    """Verify request_software_stop and halt_motion succeed from ARMED, DISARMED, and FAULT_LATCHED."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    # 1. From DISARMED
    assert guard.request_software_stop() is True
    assert mock_backend.halt_motion.call_count == 1
    assert guard.guard_state == SafetyGuardState.DISARMED

    # 2. From ARMED -> transitions to DISARMED
    guard.arm()
    assert guard.guard_state == SafetyGuardState.ARMED
    guard.halt_motion()
    assert mock_backend.halt_motion.call_count == 2
    assert guard.guard_state == SafetyGuardState.DISARMED

    # 3. From FAULT_LATCHED -> stays FAULT_LATCHED
    guard.arm()
    with pytest.raises(CommandSafetyViolationError):
        guard.command_joint_positions([3.0, 0.0])
    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    guard.halt_motion()
    assert mock_backend.halt_motion.call_count == 3  # 1 (DISARMED) + 1 (ARMED) + 1 (FAULT_LATCHED)
    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED


def test_privileged_software_stop_failure_latches_fault():
    """Verify failure in underlying halt_motion latches SOFTWARE_STOP_FAILED."""
    model, mock_backend = _make_model_and_backend()
    mock_backend.halt_motion.side_effect = RuntimeError("Transport communication dropped")
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)

    with pytest.raises(BackendError, match="(?i)software stop request failed"):
        guard.request_software_stop()

    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault.code == CommandSafetyFaultCode.SOFTWARE_STOP_FAILED


def test_disarm_software_stop_lifecycle():
    """Verify disarm skips halt if no motion occurred, but executes halt if motion occurred."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard.arm()

    # Case A: Armed but no motion commands -> disarm doesn't halt
    guard.disarm()
    assert guard.guard_state == SafetyGuardState.DISARMED
    assert mock_backend.halt_motion.call_count == 0

    # Case B: Armed and commanded motion -> disarm executes halt
    guard.arm()
    guard.command_joint_positions([0.1, 0.1])
    guard.disarm()
    assert guard.guard_state == SafetyGuardState.DISARMED
    assert mock_backend.halt_motion.call_count == 1


def test_custom_readiness_probe_integration():
    """Verify custom CommandReadinessProbe is queried during evaluate_readiness and arm."""
    model, mock_backend = _make_model_and_backend()
    mock_probe = MagicMock()
    mock_probe.check.return_value = (False, ("External safety loop open",))

    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        readiness_probe=mock_probe,
    )

    report = guard.evaluate_readiness()
    assert report.ready_to_arm is False
    assert report.external_readiness_ok is False
    assert "External safety loop open" in report.reasons

    with pytest.raises(BackendError, match="(?i)External safety loop open"):
        guard.arm()

    assert guard.guard_state == SafetyGuardState.DISARMED


def test_watchdog_inactive_when_armed_with_no_motion():
    """Verify watchdog does not trigger when ARMED without any dispatched motion."""
    model, mock_backend = _make_model_and_backend()
    fake_time = [100.0]

    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(command_watchdog_timeout_s=0.2),
        time_source=lambda: fake_time[0],
    )
    guard.arm()

    # Advance fake time significantly
    fake_time[0] = 200.0
    assert guard.check_watchdog() is False
    assert guard.guard_state == SafetyGuardState.ARMED
    assert mock_backend.halt_motion.call_count == 0


def test_watchdog_deadline_reset_and_timeout_trigger():
    """Verify watchdog deadline resets on valid command and halts on starvation."""
    model, mock_backend = _make_model_and_backend()
    fake_time = [100.0]

    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(command_watchdog_timeout_s=0.2),
        time_source=lambda: fake_time[0],
    )
    guard.arm()

    # 1. First command activates watchdog at t=100.0
    assert guard.command_joint_positions([0.1, 0.1]) is True
    assert guard.accepted_command_sequence == 1

    # 2. Check at t=100.1 (< 0.2s elapsed) -> no timeout
    fake_time[0] = 100.1
    assert guard.check_watchdog() is False
    assert guard.guard_state == SafetyGuardState.ARMED

    # 3. New command at t=100.15 resets watchdog deadline to 100.35
    fake_time[0] = 100.15
    assert guard.command_joint_positions([0.2, 0.2]) is True
    assert guard.accepted_command_sequence == 2

    # 4. Check at t=100.30 (0.15s since last command < 0.2s) -> no timeout
    fake_time[0] = 100.30
    assert guard.check_watchdog() is False
    assert guard.guard_state == SafetyGuardState.ARMED

    # 5. Check at t=100.36 (0.21s since last command > 0.2s) -> timeout triggers!
    fake_time[0] = 100.36
    assert guard.check_watchdog() is True
    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault is not None
    assert guard.active_fault.code == CommandSafetyFaultCode.COMMAND_WATCHDOG_TIMEOUT
    assert mock_backend.halt_motion.call_count == 1

    # 6. Subsequent check does not re-trigger or repeat halt
    assert guard.check_watchdog() is False
    assert mock_backend.halt_motion.call_count == 1


def test_watchdog_halt_failure_latches_halt_failed():
    """Verify watchdog timeout when underlying halt fails latches COMMAND_WATCHDOG_HALT_FAILED."""
    model, mock_backend = _make_model_and_backend()
    mock_backend.halt_motion.side_effect = RuntimeError("Halt communication timeout")
    fake_time = [100.0]

    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(command_watchdog_timeout_s=0.2),
        time_source=lambda: fake_time[0],
    )
    guard.arm()
    guard.command_joint_positions([0.1, 0.1])

    fake_time[0] = 100.5  # Timeout exceeded
    assert guard.check_watchdog() is True
    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard.active_fault.code == CommandSafetyFaultCode.COMMAND_WATCHDOG_HALT_FAILED


def test_watchdog_concurrency_overlap_safety():
    """Verify command dispatch and watchdog evaluation cannot execute concurrently."""
    import threading
    import time

    model, mock_backend = _make_model_and_backend()
    active_calls = 0
    max_active_calls = 0
    tracker_lock = threading.Lock()

    def slow_command_positions(*args, **kwargs):
        nonlocal active_calls, max_active_calls
        with tracker_lock:
            active_calls += 1
            if active_calls > max_active_calls:
                max_active_calls = active_calls
        time.sleep(0.005)
        with tracker_lock:
            active_calls -= 1
        return True

    def slow_halt_motion():
        nonlocal active_calls, max_active_calls
        with tracker_lock:
            active_calls += 1
            if active_calls > max_active_calls:
                max_active_calls = active_calls
        time.sleep(0.005)
        with tracker_lock:
            active_calls -= 1

    mock_backend.command_joint_positions.side_effect = slow_command_positions
    mock_backend.halt_motion.side_effect = slow_halt_motion

    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(command_watchdog_timeout_s=0.01),
    )
    guard.arm()

    def command_worker():
        for _ in range(20):
            try:
                guard.command_joint_positions([0.05, 0.05])
            except Exception:
                pass
            time.sleep(0.001)

    def watchdog_worker():
        for _ in range(20):
            try:
                guard.check_watchdog()
            except Exception:
                pass
            time.sleep(0.001)

    threads = [
        threading.Thread(target=command_worker),
        threading.Thread(target=watchdog_worker),
        threading.Thread(target=command_worker),
    ]

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    # Verify maximum concurrent executions inside backend was exactly 1 (0 overlap)
    assert max_active_calls <= 1


def test_disconnect_lifecycle_variations():
    """Verify disconnect behavior for DISARMED, ARMED without motion, active motion, and stop failure."""
    model, mock_backend = _make_model_and_backend()

    # 1. DISARMED: disconnects directly without calling halt
    guard1 = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard1.disconnect()
    assert guard1.guard_state == SafetyGuardState.DISARMED
    assert mock_backend.halt_motion.call_count == 0
    assert mock_backend.disconnect.call_count == 1

    mock_backend.reset_mock()

    # 2. ARMED but no motion: disarms + disconnects directly without calling halt
    guard2 = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard2.arm()
    guard2.disconnect()
    assert guard2.guard_state == SafetyGuardState.DISARMED
    assert mock_backend.halt_motion.call_count == 0
    assert mock_backend.disconnect.call_count == 1

    mock_backend.reset_mock()

    # 3. ARMED with active motion: attempts software halt first, then disconnects
    guard3 = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard3.arm()
    guard3.command_joint_positions([0.1, 0.1])
    guard3.disconnect()
    assert guard3.guard_state == SafetyGuardState.DISARMED
    assert mock_backend.halt_motion.call_count == 1
    assert mock_backend.disconnect.call_count == 1

    mock_backend.reset_mock()

    # 4. ARMED with active motion and halt failure: latches fault, raises BackendError, does NOT disconnect backend
    mock_backend.halt_motion.side_effect = RuntimeError("Halt network error")
    guard4 = GuardedRobotBackend(underlying_backend=mock_backend, resolved_model=model)
    guard4.arm()
    guard4.command_joint_positions([0.1, 0.1])
    with pytest.raises(BackendError, match="(?i)Disconnect halted"):
        guard4.disconnect()

    assert guard4.guard_state == SafetyGuardState.FAULT_LATCHED
    assert guard4.active_fault.code == CommandSafetyFaultCode.SOFTWARE_STOP_FAILED
    assert mock_backend.halt_motion.call_count == 1
    assert mock_backend.disconnect.call_count == 0


def test_command_audit_log_boundedness_and_records():
    """Verify CommandAuditRecord generation, boundedness, and diagnostic details."""
    model, mock_backend = _make_model_and_backend()
    guard = GuardedRobotBackend(
        underlying_backend=mock_backend,
        resolved_model=model,
        safety_config=CommandSafetyConfig(audit_history_limit=3),
    )

    # 1. Rejected command while DISARMED
    with pytest.raises(BackendCommandDisabledError):
        guard.command_joint_positions([0.1, 0.1])

    # 2. Arm guard
    guard.arm()

    # 3. Successful command 1
    guard.command_joint_positions([0.1, 0.1])

    # 4. Rejected limit command
    with pytest.raises(CommandSafetyViolationError):
        guard.command_joint_positions([5.0, 0.0])

    # 5. Successful velocity command after fault reset & re-arm
    guard.reset_fault()
    guard.arm()
    guard.command_joint_velocities([0.2, 0.2])

    logs = guard.audit_log
    assert len(logs) == 3  # bounded by audit_history_limit=3

    # Check last record
    last_rec = logs[-1]
    assert last_rec.operation == "command_joint_velocities"
    assert last_rec.accepted is True
    assert last_rec.command_mode == "velocity"
    assert last_rec.max_abs_value == pytest.approx(0.2)
    assert last_rec.command_sequence == 2



