"""Hardware-Readiness Software Command Safety Layer.

This module provides defensive software interlocks, preflight verification, command safety envelope
enforcement, fault latching, and software watchdog monitoring for robot backends.

CRITICAL NOTICE:
This module is NON-SAFETY-RATED application-level software protection. It does not provide certified
hardware safety, STO, or physical emergency stop functionality.
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Callable, Optional, Protocol, Sequence, Tuple, runtime_checkable

from robotics.backends.base import (
    BackendCommandDisabledError,
    BackendError,
    BackendHealthStatus,
    RobotBackend,
    RobotBackendCapabilities,
    TimestampedJointState,
)
from robotics.robot_model import ResolvedRobotModel


class SafetyGuardState(Enum):
    """Execution authorization state for GuardedRobotBackend."""
    DISARMED = "disarmed"
    ARMED = "armed"
    FAULT_LATCHED = "fault_latched"


class CommandSafetyFaultCode(Enum):
    """Specific cause identifiers for command safety faults."""
    NONE = "none"
    STATE_UNAVAILABLE = "state_unavailable"
    STATE_STALE = "state_stale"
    STATE_INVALID = "state_invalid"
    MODEL_MISMATCH = "model_mismatch"
    BACKEND_UNHEALTHY = "backend_unhealthy"
    COMMAND_TRANSPORT_UNAVAILABLE = "command_transport_unavailable"
    POSITION_LIMIT_VIOLATION = "position_limit_violation"
    POSITION_STEP_VIOLATION = "position_step_violation"
    VELOCITY_LIMIT_VIOLATION = "velocity_limit_violation"
    COMMAND_DISPATCH_FAILED = "command_dispatch_failed"
    COMMAND_WATCHDOG_TIMEOUT = "command_watchdog_timeout"
    COMMAND_WATCHDOG_HALT_FAILED = "command_watchdog_halt_failed"
    SOFTWARE_STOP_FAILED = "software_stop_failed"
    READINESS_FAILED = "readiness_failed"


class CommandSafetyViolationError(BackendError):
    """Raised when a commanded target violates safety envelopes or guard authorization."""
    pass


class CommandWatchdogTimeoutError(BackendError):
    """Raised when the software command watchdog expires due to command starvation."""
    pass


@dataclass(frozen=True)
class CommandSafetyFault:
    """Immutable record of an active or historical safety fault."""
    code: CommandSafetyFaultCode
    monotonic_timestamp_s: float
    detail: str


@dataclass(frozen=True)
class CommandAuditRecord:
    """Bounded diagnostic log entry for dispatched and rejected software commands."""
    monotonic_timestamp_s: float
    operation: str
    accepted: bool
    reason: Optional[str] = None
    command_sequence: int = 0
    max_abs_value: Optional[float] = None
    command_mode: Optional[str] = None


@dataclass(frozen=True)
class CommandSafetyConfig:
    """Configuration parameters for the hardware-readiness command safety layer."""
    state_timeout_s: float = 0.5
    command_watchdog_timeout_s: float = 0.2
    position_limit_margin_rad: float = 0.0
    velocity_limit_scale: float = 1.0
    max_position_step_by_joint: Optional[Tuple[float, ...]] = None
    require_velocity_feedback_for_velocity_commands: bool = False
    watchdog_enabled: bool = True
    audit_history_limit: int = 100

    def __post_init__(self) -> None:
        if self.state_timeout_s <= 0.0:
            raise ValueError(f"state_timeout_s must be positive, got {self.state_timeout_s}")
        if self.command_watchdog_timeout_s <= 0.0:
            raise ValueError(f"command_watchdog_timeout_s must be positive, got {self.command_watchdog_timeout_s}")
        if not (0.0 < self.velocity_limit_scale <= 1.0):
            raise ValueError(f"velocity_limit_scale must be in (0, 1.0], got {self.velocity_limit_scale}")
        if self.max_position_step_by_joint is not None:
            if any(step <= 0.0 for step in self.max_position_step_by_joint):
                raise ValueError(f"All values in max_position_step_by_joint must be positive: {self.max_position_step_by_joint}")
        if self.audit_history_limit <= 0:
            raise ValueError(f"audit_history_limit must be positive, got {self.audit_history_limit}")


@dataclass(frozen=True)
class SoftwareCommandReadinessReport:
    """Immutable diagnostic report detailing software preflight readiness checks."""
    backend_connected: bool
    backend_health_ok: bool
    state_available: bool
    state_fresh: bool
    model_match: bool
    command_mode_supported: bool
    halt_supported: bool
    transport_ready: bool
    external_readiness_ok: bool
    fault_clear: bool
    ready_to_arm: bool
    reasons: Tuple[str, ...] = ()


@runtime_checkable
class CommandReadinessProbe(Protocol):
    """Protocol for optional external readiness checks (e.g. ROS 2 controller manager)."""

    def check(self, backend: RobotBackend, model: ResolvedRobotModel) -> Tuple[bool, Tuple[str, ...]]:
        """Inspects subsystem configuration/lifecycle and returns (is_ready, reasons)."""
        ...


class CommandWatchdog:
    """Pure logic evaluator for application-level command watchdog timeouts."""

    @staticmethod
    def evaluate_timeout(
        now_s: float,
        guard_state: SafetyGuardState,
        motion_session_active: bool,
        last_accepted_command_s: Optional[float],
        timeout_s: float,
    ) -> bool:
        """Determines whether a command watchdog timeout has elapsed.

        Watchdog triggers ONLY when the system is ARMED, has active motion sessions,
        and the elapsed time since the last accepted motion command strictly exceeds timeout_s.
        """
        if guard_state != SafetyGuardState.ARMED:
            return False
        if not motion_session_active:
            return False
        if last_accepted_command_s is None:
            return False
        return (now_s - last_accepted_command_s) > timeout_s


class GuardedRobotBackend(RobotBackend):
    """Defensive software safety decorator for command-capable robot backends.

    This decorator enforces fail-closed command authorization, static joint and velocity safety
    envelopes, maximum position step guards, non-auto-clearing fault latching, and software watchdog monitoring.
    """

    def __init__(
        self,
        underlying_backend: RobotBackend,
        resolved_model: ResolvedRobotModel,
        safety_config: Optional[CommandSafetyConfig] = None,
        readiness_probe: Optional[CommandReadinessProbe] = None,
        time_source: Optional[Callable[[], float]] = None,
    ) -> None:
        self._backend = underlying_backend
        self._model = resolved_model
        self._config = safety_config or CommandSafetyConfig()
        self._readiness_probe = readiness_probe
        self._time_source = time_source or time.monotonic

        self._command_lock = threading.RLock()
        self._guard_state = SafetyGuardState.DISARMED
        self._active_fault: Optional[CommandSafetyFault] = None
        self._accepted_command_sequence = 0
        self._rejected_command_count = 0
        self._motion_session_active = False
        self._last_accepted_command_s: Optional[float] = None

        self._watchdog_stop_event = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None
        self._audit_log: deque[CommandAuditRecord] = deque(maxlen=self._config.audit_history_limit)

    @property
    def guard_state(self) -> SafetyGuardState:
        """Returns the current SafetyGuardState."""
        with self._command_lock:
            return self._guard_state

    @property
    def is_armed(self) -> bool:
        """Returns True if the safety guard is currently ARMED."""
        with self._command_lock:
            return self._guard_state == SafetyGuardState.ARMED

    @property
    def active_fault(self) -> Optional[CommandSafetyFault]:
        """Returns the active CommandSafetyFault if in FAULT_LATCHED state, else None."""
        with self._command_lock:
            return self._active_fault

    @property
    def accepted_command_sequence(self) -> int:
        """Returns the count of successfully validated and dispatched motion commands."""
        with self._command_lock:
            return self._accepted_command_sequence

    @property
    def rejected_command_count(self) -> int:
        """Returns the total count of rejected command attempts."""
        with self._command_lock:
            return self._rejected_command_count

    @property
    def audit_log(self) -> Tuple[CommandAuditRecord, ...]:
        """Returns a snapshot tuple of bounded command audit records."""
        with self._command_lock:
            return tuple(self._audit_log)

    def _record_audit(
        self,
        operation: str,
        accepted: bool,
        reason: Optional[str] = None,
        max_abs_value: Optional[float] = None,
        command_mode: Optional[str] = None,
    ) -> None:
        rec = CommandAuditRecord(
            monotonic_timestamp_s=self._time_source(),
            operation=operation,
            accepted=accepted,
            reason=reason,
            command_sequence=self._accepted_command_sequence,
            max_abs_value=max_abs_value,
            command_mode=command_mode,
        )
        self._audit_log.append(rec)

    @property
    def transport_capabilities(self) -> RobotBackendCapabilities:
        """Returns capabilities of the underlying execution backend."""
        return self._backend.transport_capabilities

    def evaluate_readiness(self, ignore_active_fault: bool = False) -> SoftwareCommandReadinessReport:
        """Evaluates readiness conditions for arming or fault clearing.

        If ignore_active_fault is True, fault_clear evaluates whether the underlying prerequisites
        are healthy regardless of the current FAULT_LATCHED state.
        """
        reasons = []

        is_conn = self._backend.is_connected()
        if not is_conn:
            reasons.append("Underlying backend is not connected.")

        health = self._backend.health_status()
        health_ok = health in (BackendHealthStatus.HEALTHY, BackendHealthStatus.DEGRADED)
        if not health_ok:
            reasons.append(f"Underlying backend health is {health.name}.")

        state_avail = False
        state_fresh = False
        model_match = False

        try:
            state = self._backend.get_joint_state()
            state_avail = True
            now_mono = self._time_source()

            age = now_mono - state.receive_timestamp_s
            if age <= self._config.state_timeout_s:
                state_fresh = True
            else:
                reasons.append(f"Joint state telemetry is stale ({age:.4f}s > {self._config.state_timeout_s:.4f}s).")

            if state.joint_names != self._model.arm_joint_names:
                model_match = False
                reasons.append(f"Telemetry joint names {state.joint_names} do not match model arm joint names {self._model.arm_joint_names}.")
            elif len(state.positions) != len(self._model.arm_joints):
                model_match = False
                reasons.append(f"Telemetry positions length {len(state.positions)} != model DoF {len(self._model.arm_joints)}.")
            else:
                model_match = True

        except Exception as err:
            state_avail = False
            state_fresh = False
            model_match = False
            reasons.append(f"Failed to acquire joint state from underlying backend: {err}")

        caps = self._backend.transport_capabilities
        cmd_mode_supp = caps.position_commands or caps.velocity_commands
        if not cmd_mode_supp:
            reasons.append("Underlying backend does not support position or velocity commanding.")

        halt_supp = caps.halt_motion
        if not halt_supp:
            reasons.append("Underlying backend does not declare halt_motion capability.")

        transport_ready = is_conn and health_ok

        external_readiness_ok = True
        if self._readiness_probe is not None:
            try:
                probe_ok, probe_reasons = self._readiness_probe.check(self._backend, self._model)
                if not probe_ok:
                    external_readiness_ok = False
                    reasons.extend(probe_reasons)
            except Exception as probe_err:
                external_readiness_ok = False
                reasons.append(f"Readiness probe check failed with error: {probe_err}")

        fault_clear = (self._active_fault is None) if not ignore_active_fault else True

        ready_to_arm = (
            is_conn
            and health_ok
            and state_avail
            and state_fresh
            and model_match
            and cmd_mode_supp
            and halt_supp
            and transport_ready
            and external_readiness_ok
            and fault_clear
        )

        return SoftwareCommandReadinessReport(
            backend_connected=is_conn,
            backend_health_ok=health_ok,
            state_available=state_avail,
            state_fresh=state_fresh,
            model_match=model_match,
            command_mode_supported=cmd_mode_supp,
            halt_supported=halt_supp,
            transport_ready=transport_ready,
            external_readiness_ok=external_readiness_ok,
            fault_clear=fault_clear,
            ready_to_arm=ready_to_arm,
            reasons=tuple(reasons),
        )

    def arm(self) -> None:
        """Executes preflight verification and transitions guard to ARMED upon success."""
        with self._command_lock:
            if self._guard_state == SafetyGuardState.ARMED:
                return
            if self._guard_state == SafetyGuardState.FAULT_LATCHED:
                raise CommandSafetyViolationError("Cannot arm while FAULT_LATCHED. Call reset_fault() first.")

            report = self.evaluate_readiness()
            if not report.ready_to_arm:
                reasons_str = "; ".join(report.reasons)
                raise BackendError(f"Command preflight verification failed: {reasons_str}")

            self._guard_state = SafetyGuardState.ARMED

    def disarm(self) -> None:
        """Revokes command authorization, performing software halt first if motion occurred."""
        with self._command_lock:
            if self._motion_session_active:
                try:
                    self.request_software_stop()
                except Exception as err:
                    raise BackendError(f"Disarm software stop failed: {err}") from err

            self._motion_session_active = False
            self._guard_state = SafetyGuardState.DISARMED

    def reset_fault(self) -> None:
        """Clears FAULT_LATCHED state after verifying readiness prerequisites are healthy.

        Transitions to DISARMED upon success. Requires explicit arm() to resume commanding.
        """
        with self._command_lock:
            if self._guard_state != SafetyGuardState.FAULT_LATCHED:
                raise BackendError(
                    f"reset_fault() is only allowed from FAULT_LATCHED state, current state is {self._guard_state.name}."
                )

            report = self.evaluate_readiness(ignore_active_fault=True)
            if not report.ready_to_arm:
                reasons_str = "; ".join(report.reasons)
                raise BackendError(f"Fault reset rejected because readiness checks failed: {reasons_str}")

            self._active_fault = None
            self._guard_state = SafetyGuardState.DISARMED

    def request_software_stop(self) -> bool:
        """Privileged software stop path. Bypasses ARMED gating, position/velocity envelopes,

        and existing FAULT_LATCHED state to dispatch halt_motion() to the underlying backend.
        """
        with self._command_lock:
            try:
                self._backend.halt_motion()
                self._motion_session_active = False
                if self._guard_state == SafetyGuardState.ARMED:
                    self._guard_state = SafetyGuardState.DISARMED
                return True
            except Exception as err:
                self._motion_session_active = False
                self._latch_fault(
                    CommandSafetyFaultCode.SOFTWARE_STOP_FAILED,
                    f"Underlying software stop failed: {err}",
                    trigger_halt=False,
                )
                raise BackendError(f"Software stop request failed: {err}") from err

    def halt_motion(self) -> None:
        """Privileged software stop mapping to underlying halt_motion()."""
        self.request_software_stop()

    def is_connected(self) -> bool:
        """Delegates connection status query to the underlying execution backend."""
        return self._backend.is_connected()

    def health_status(self) -> BackendHealthStatus:
        """Delegates communication health query to the underlying execution backend."""
        return self._backend.health_status()

    def get_joint_state(self) -> TimestampedJointState:
        """Delegates joint state acquisition to the underlying execution backend."""
        return self._backend.get_joint_state()

    def connect(self) -> bool:
        """Connects underlying backend while leaving safety guard in DISARMED state."""
        with self._command_lock:
            return self._backend.connect()

    def disconnect(self) -> None:
        """Cleans up safety guard and disconnects underlying execution backend."""
        with self._command_lock:
            if self._motion_session_active:
                try:
                    self.request_software_stop()
                except Exception as err:
                    raise BackendError(f"Disconnect halted: active motion stop failed: {err}") from err

            self.stop_watchdog_monitor()
            self._guard_state = SafetyGuardState.DISARMED
            self._motion_session_active = False
            self._backend.disconnect()

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Dispatches target joint positions following safety envelope and authorization validation."""
        with self._command_lock:
            if self._guard_state == SafetyGuardState.DISARMED:
                self._rejected_command_count += 1
                self._record_audit("command_joint_positions", accepted=False, reason="DISARMED", command_mode="position")
                raise BackendCommandDisabledError("Safety guard is DISARMED. Call arm() first.")
            if self._guard_state == SafetyGuardState.FAULT_LATCHED:
                self._rejected_command_count += 1
                fault_detail = self._active_fault.detail if self._active_fault else "Unknown fault"
                self._record_audit("command_joint_positions", accepted=False, reason=f"FAULT_LATCHED: {fault_detail}", command_mode="position")
                raise CommandSafetyViolationError(f"Safety guard is FAULT_LATCHED ({fault_detail}). Call reset_fault() then arm().")

            # 1. Finite and DoF validation
            expected_dof = len(self._model.arm_joints)
            if len(target_positions) != expected_dof:
                msg = f"Target positions length {len(target_positions)} != model DoF {expected_dof}"
                self._record_audit("command_joint_positions", accepted=False, reason=msg, command_mode="position")
                self._latch_fault(
                    CommandSafetyFaultCode.POSITION_LIMIT_VIOLATION,
                    msg,
                )
                raise CommandSafetyViolationError(msg)

            for i, val in enumerate(target_positions):
                if not math.isfinite(val):
                    msg = f"Non-finite target position at index {i}: {val}"
                    self._record_audit("command_joint_positions", accepted=False, reason=msg, command_mode="position")
                    self._latch_fault(
                        CommandSafetyFaultCode.POSITION_LIMIT_VIOLATION,
                        msg,
                    )
                    raise CommandSafetyViolationError(msg)

            # 2. Joint limits validation
            tol = 1e-9
            for i, (val, joint) in enumerate(zip(target_positions, self._model.arm_joints)):
                if val < joint.lower_limit - tol or val > joint.upper_limit + tol:
                    msg = f"Position limit violation: Target position {val:.4f} at joint {joint.name} violates limits [{joint.lower_limit:.4f}, {joint.upper_limit:.4f}]"
                    self._record_audit(
                        "command_joint_positions",
                        accepted=False,
                        reason=msg,
                        max_abs_value=max(abs(v) for v in target_positions if math.isfinite(v)),
                        command_mode="position",
                    )
                    self._latch_fault(
                        CommandSafetyFaultCode.POSITION_LIMIT_VIOLATION,
                        msg,
                    )
                    raise CommandSafetyViolationError(msg)

            # 3. Position step jump validation
            state = self._backend.get_joint_state()
            if self._config.max_position_step_by_joint is not None:
                step_limits = self._config.max_position_step_by_joint
                for i, (target_q, meas_q, max_step) in enumerate(zip(target_positions, state.positions, step_limits)):
                    diff = abs(target_q - meas_q)
                    if diff > max_step:
                        msg = f"Position step jump violation: Position step jump {diff:.4f} at joint {self._model.arm_joints[i].name} exceeds limit {max_step:.4f}"
                        self._record_audit(
                            "command_joint_positions",
                            accepted=False,
                            reason=msg,
                            max_abs_value=max(abs(v) for v in target_positions if math.isfinite(v)),
                            command_mode="position",
                        )
                        self._latch_fault(
                            CommandSafetyFaultCode.POSITION_STEP_VIOLATION,
                            msg,
                        )
                        raise CommandSafetyViolationError(msg)

            # 4. Dispatch to underlying backend
            res = self._backend.command_joint_positions(target_positions)
            if res:
                self._accepted_command_sequence += 1
                self._last_accepted_command_s = self._time_source()
                self._motion_session_active = True
                self._record_audit(
                    "command_joint_positions",
                    accepted=True,
                    max_abs_value=max(abs(v) for v in target_positions),
                    command_mode="position",
                )
                return True
            else:
                self._record_audit(
                    "command_joint_positions",
                    accepted=False,
                    reason="Underlying backend dispatch returned False",
                    command_mode="position",
                )
                return False

    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Dispatches target joint velocities following safety envelope and authorization validation."""
        with self._command_lock:
            if self._guard_state == SafetyGuardState.DISARMED:
                self._rejected_command_count += 1
                self._record_audit("command_joint_velocities", accepted=False, reason="DISARMED", command_mode="velocity")
                raise BackendCommandDisabledError("Safety guard is DISARMED. Call arm() first.")
            if self._guard_state == SafetyGuardState.FAULT_LATCHED:
                self._rejected_command_count += 1
                fault_detail = self._active_fault.detail if self._active_fault else "Unknown fault"
                self._record_audit("command_joint_velocities", accepted=False, reason=f"FAULT_LATCHED: {fault_detail}", command_mode="velocity")
                raise CommandSafetyViolationError(f"Safety guard is FAULT_LATCHED ({fault_detail}). Call reset_fault() then arm().")

            expected_dof = len(self._model.arm_joints)
            if len(target_velocities) != expected_dof:
                msg = f"Target velocities length {len(target_velocities)} != model DoF {expected_dof}"
                self._record_audit("command_joint_velocities", accepted=False, reason=msg, command_mode="velocity")
                self._latch_fault(
                    CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION,
                    msg,
                )
                raise CommandSafetyViolationError(msg)

            for i, val in enumerate(target_velocities):
                if not math.isfinite(val):
                    msg = f"Non-finite target velocity at index {i}: {val}"
                    self._record_audit("command_joint_velocities", accepted=False, reason=msg, command_mode="velocity")
                    self._latch_fault(
                        CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION,
                        msg,
                    )
                    raise CommandSafetyViolationError(msg)

            # Velocity limit validation
            scale = self._config.velocity_limit_scale
            tol = 1e-9
            for i, (val, joint) in enumerate(zip(target_velocities, self._model.arm_joints)):
                max_allowed = scale * joint.max_velocity
                if abs(val) > max_allowed + tol:
                    msg = f"Velocity limit violation: Target velocity {val:.4f} at joint {joint.name} exceeds scaled envelope {max_allowed:.4f}"
                    self._record_audit(
                        "command_joint_velocities",
                        accepted=False,
                        reason=msg,
                        max_abs_value=max(abs(v) for v in target_velocities if math.isfinite(v)),
                        command_mode="velocity",
                    )
                    self._latch_fault(
                        CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION,
                        msg,
                    )
                    raise CommandSafetyViolationError(msg)

            # State check
            state = self._backend.get_joint_state()
            if self._config.require_velocity_feedback_for_velocity_commands:
                if state.velocities is None or len(state.velocities) != expected_dof or not all(math.isfinite(v) for v in state.velocities):
                    msg = "State telemetry is missing valid velocity feedback required by configuration."
                    self._record_audit("command_joint_velocities", accepted=False, reason=msg, command_mode="velocity")
                    self._latch_fault(
                        CommandSafetyFaultCode.STATE_INVALID,
                        msg,
                    )
                    raise CommandSafetyViolationError(msg)

            res = self._backend.command_joint_velocities(target_velocities, effort_limit=effort_limit)
            if res:
                self._accepted_command_sequence += 1
                self._last_accepted_command_s = self._time_source()
                self._motion_session_active = True
                self._record_audit(
                    "command_joint_velocities",
                    accepted=True,
                    max_abs_value=max(abs(v) for v in target_velocities),
                    command_mode="velocity",
                )
                return True
            else:
                self._record_audit(
                    "command_joint_velocities",
                    accepted=False,
                    reason="Underlying backend dispatch returned False",
                    command_mode="velocity",
                )
                return False

    def check_watchdog(self, now_s: Optional[float] = None) -> bool:
        """Evaluates watchdog status and triggers software stop if deadline exceeded.

        Returns True if a timeout was detected and fault latched, False otherwise.
        """
        current_time = now_s if now_s is not None else self._time_source()
        if not self._config.watchdog_enabled:
            return False

        with self._command_lock:
            # Recheck conditions inside command lock to prevent race with newly accepted commands
            is_timeout = CommandWatchdog.evaluate_timeout(
                now_s=current_time,
                guard_state=self._guard_state,
                motion_session_active=self._motion_session_active,
                last_accepted_command_s=self._last_accepted_command_s,
                timeout_s=self._config.command_watchdog_timeout_s,
            )
            if not is_timeout:
                return False

            # Timeout confirmed -> dispatch halt and latch fault
            try:
                self._backend.halt_motion()
                self._guard_state = SafetyGuardState.FAULT_LATCHED
                self._active_fault = CommandSafetyFault(
                    code=CommandSafetyFaultCode.COMMAND_WATCHDOG_TIMEOUT,
                    monotonic_timestamp_s=current_time,
                    detail=f"Command watchdog timeout exceeded ({self._config.command_watchdog_timeout_s:.3f}s)",
                )
            except Exception as err:
                self._guard_state = SafetyGuardState.FAULT_LATCHED
                self._active_fault = CommandSafetyFault(
                    code=CommandSafetyFaultCode.COMMAND_WATCHDOG_HALT_FAILED,
                    monotonic_timestamp_s=current_time,
                    detail=f"Command watchdog timeout software halt failed: {err}",
                )
            finally:
                self._motion_session_active = False

            return True

    def start_watchdog_monitor(self, poll_interval_s: float = 0.02) -> None:
        """Starts a background monitor thread that periodically checks the watchdog."""
        with self._command_lock:
            if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
                return
            self._watchdog_stop_event.clear()
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_monitor_loop,
                args=(poll_interval_s,),
                daemon=True,
                name="GuardedBackendWatchdog",
            )
            self._watchdog_thread.start()

    def stop_watchdog_monitor(self) -> None:
        """Stops the background watchdog monitor thread."""
        self._watchdog_stop_event.set()
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)
            self._watchdog_thread = None

    def _watchdog_monitor_loop(self, poll_interval_s: float) -> None:
        while not self._watchdog_stop_event.is_set():
            if self.check_watchdog():
                break
            self._watchdog_stop_event.wait(timeout=poll_interval_s)

    def _latch_fault(self, code: CommandSafetyFaultCode, detail: str, trigger_halt: bool = True) -> None:
        """Internal helper transitioning state to FAULT_LATCHED and recording fault detail."""
        self._guard_state = SafetyGuardState.FAULT_LATCHED
        self._active_fault = CommandSafetyFault(
            code=code,
            monotonic_timestamp_s=self._time_source(),
            detail=detail,
        )
        self._rejected_command_count += 1
        if trigger_halt and self._motion_session_active:
            try:
                self._backend.halt_motion()
            except Exception:
                pass
        self._motion_session_active = False
