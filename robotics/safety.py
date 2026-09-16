"""Hardware-Readiness Software Command Safety Layer.

This module provides defensive software interlocks, preflight verification, command safety envelope
enforcement, fault latching, and software watchdog monitoring for robot backends.

CRITICAL NOTICE:
This module is NON-SAFETY-RATED application-level software protection. It does not provide certified
hardware safety, STO, or physical emergency stop functionality.
"""

from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Optional, Sequence, Tuple

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
    """Categorized root causes for safety violations and fault latching."""
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
    """Raised when a command violates the software safety envelope."""
    pass


class CommandWatchdogTimeoutError(BackendError):
    """Raised when the command watchdog timer expires during an active motion session."""
    pass


@dataclass(frozen=True)
class CommandSafetyFault:
    """Immutable record of an active or historical safety fault."""
    code: CommandSafetyFaultCode
    monotonic_timestamp_s: float
    detail: str


@dataclass(frozen=True)
class CommandSafetyConfig:
    """Configuration for GuardedRobotBackend safety parameters."""
    state_timeout_s: float = 0.5
    command_watchdog_timeout_s: float = 0.2
    position_limit_margin_rad: float = 0.0
    velocity_limit_scale: float = 1.0
    max_position_step_by_joint: Optional[Tuple[float, ...]] = None
    require_velocity_feedback_for_velocity_commands: bool = False
    watchdog_enabled: bool = True

    def __post_init__(self) -> None:
        if self.state_timeout_s <= 0.0:
            raise ValueError(f"state_timeout_s must be positive, got {self.state_timeout_s}")
        if self.command_watchdog_timeout_s <= 0.0:
            raise ValueError(f"command_watchdog_timeout_s must be positive, got {self.command_watchdog_timeout_s}")
        if not (0.0 < self.velocity_limit_scale <= 1.0):
            raise ValueError(f"velocity_limit_scale must be in (0.0, 1.0], got {self.velocity_limit_scale}")
        if self.position_limit_margin_rad < 0.0:
            raise ValueError(f"position_limit_margin_rad must be non-negative, got {self.position_limit_margin_rad}")


@dataclass(frozen=True)
class SoftwareCommandReadinessReport:
    """Diagnostic report detailing software preflight check evaluation."""
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
    ) -> None:
        if not isinstance(underlying_backend, RobotBackend):
            raise TypeError(f"underlying_backend must be an instance of RobotBackend, got {type(underlying_backend)}")
        if not isinstance(resolved_model, ResolvedRobotModel):
            raise TypeError(f"resolved_model must be an instance of ResolvedRobotModel, got {type(resolved_model)}")

        self._backend = underlying_backend
        self._model = resolved_model
        self._config = safety_config if safety_config is not None else CommandSafetyConfig()

        self._command_lock = threading.RLock()
        self._guard_state = SafetyGuardState.DISARMED
        self._active_fault: Optional[CommandSafetyFault] = None
        self._motion_session_active: bool = False
        self._last_accepted_command_s: Optional[float] = None
        self._accepted_command_sequence: int = 0
        self._rejected_command_count: int = 0

    @property
    def guard_state(self) -> SafetyGuardState:
        """Returns the current authorization state of the safety guard."""
        with self._command_lock:
            return self._guard_state

    @property
    def is_armed(self) -> bool:
        """Returns True if the safety guard is currently ARMED."""
        with self._command_lock:
            return self._guard_state == SafetyGuardState.ARMED

    @property
    def active_fault(self) -> Optional[CommandSafetyFault]:
        """Returns the active latched fault if currently in FAULT_LATCHED state."""
        with self._command_lock:
            return self._active_fault

    @property
    def transport_capabilities(self) -> RobotBackendCapabilities:
        """Delegates capability introspection to the underlying execution backend."""
        return self._backend.transport_capabilities

    def evaluate_readiness(self) -> SoftwareCommandReadinessReport:
        """Evaluates software command preflight readiness conditions without mutating guard state."""
        reasons = []
        is_conn = self._backend.is_connected()
        if not is_conn:
            reasons.append("Underlying execution backend is disconnected.")

        health = self._backend.health_status()
        health_ok = health in (BackendHealthStatus.HEALTHY, BackendHealthStatus.DEGRADED)
        if not health_ok:
            reasons.append(f"Underlying backend health is not acceptable: {health.value}")

        state_avail = True
        state_fresh = True
        model_match = True
        try:
            state = self._backend.get_joint_state()
            # Freshness check
            now_mono = time.monotonic()
            age_s = now_mono - state.receive_timestamp_s
            if age_s > self._config.state_timeout_s:
                state_fresh = False
                reasons.append(f"Joint telemetry is stale (age {age_s:.3f}s > timeout {self._config.state_timeout_s:.3f}s).")

            # Model alignment check
            if state.joint_names != self._model.arm_joint_names:
                model_match = False
                reasons.append(f"Telemetry joint names {state.joint_names} do not match model arm joint names {self._model.arm_joint_names}.")

            if len(state.positions) != len(self._model.arm_joints):
                model_match = False
                reasons.append(f"Telemetry positions length {len(state.positions)} != model DoF {len(self._model.arm_joints)}.")

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
        fault_clear = self._active_fault is None

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
                    self._backend.halt_motion()
                except Exception as err:
                    self._guard_state = SafetyGuardState.FAULT_LATCHED
                    self._active_fault = CommandSafetyFault(
                        code=CommandSafetyFaultCode.SOFTWARE_STOP_FAILED,
                        monotonic_timestamp_s=time.monotonic(),
                        detail=f"Software stop failed during disarm: {err}",
                    )
                    raise BackendError(f"Disarm software stop failed: {err}") from err

            self._motion_session_active = False
            self._guard_state = SafetyGuardState.DISARMED

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
            self._guard_state = SafetyGuardState.DISARMED
            self._motion_session_active = False
            self._backend.disconnect()

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Dispatches target joint positions following safety envelope and authorization validation."""
        with self._command_lock:
            if self._guard_state == SafetyGuardState.DISARMED:
                self._rejected_command_count += 1
                raise BackendCommandDisabledError("Safety guard is DISARMED. Call arm() first.")
            if self._guard_state == SafetyGuardState.FAULT_LATCHED:
                self._rejected_command_count += 1
                fault_detail = self._active_fault.detail if self._active_fault else "Unknown fault"
                raise CommandSafetyViolationError(f"Safety guard is FAULT_LATCHED ({fault_detail}). Call reset_fault() then arm().")

            # 1. Finite and DoF validation
            expected_dof = len(self._model.arm_joints)
            if len(target_positions) != expected_dof:
                self._latch_fault(
                    CommandSafetyFaultCode.POSITION_LIMIT_VIOLATION,
                    f"Target positions length {len(target_positions)} != model DoF {expected_dof}",
                )
                raise CommandSafetyViolationError(f"Target positions length {len(target_positions)} != model DoF {expected_dof}")

            for i, val in enumerate(target_positions):
                if not math.isfinite(val):
                    self._latch_fault(
                        CommandSafetyFaultCode.POSITION_LIMIT_VIOLATION,
                        f"Non-finite target position at index {i}: {val}",
                    )
                    raise CommandSafetyViolationError(f"Non-finite target position at index {i}: {val}")

            # 2. Joint limits validation
            tol = 1e-9
            for i, (val, joint) in enumerate(zip(target_positions, self._model.arm_joints)):
                if val < joint.lower_limit - tol or val > joint.upper_limit + tol:
                    msg = f"Position limit violation: Target position {val:.4f} at joint {joint.name} violates limits [{joint.lower_limit:.4f}, {joint.upper_limit:.4f}]"
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
                        self._latch_fault(
                            CommandSafetyFaultCode.POSITION_STEP_VIOLATION,
                            msg,
                        )
                        raise CommandSafetyViolationError(msg)

            # 4. Dispatch to underlying backend
            res = self._backend.command_joint_positions(target_positions)
            if res:
                self._accepted_command_sequence += 1
                self._last_accepted_command_s = time.monotonic()
                self._motion_session_active = True
                return True
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
                raise BackendCommandDisabledError("Safety guard is DISARMED. Call arm() first.")
            if self._guard_state == SafetyGuardState.FAULT_LATCHED:
                self._rejected_command_count += 1
                fault_detail = self._active_fault.detail if self._active_fault else "Unknown fault"
                raise CommandSafetyViolationError(f"Safety guard is FAULT_LATCHED ({fault_detail}). Call reset_fault() then arm().")

            expected_dof = len(self._model.arm_joints)
            if len(target_velocities) != expected_dof:
                self._latch_fault(
                    CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION,
                    f"Target velocities length {len(target_velocities)} != model DoF {expected_dof}",
                )
                raise CommandSafetyViolationError(f"Target velocities length {len(target_velocities)} != model DoF {expected_dof}")

            for i, val in enumerate(target_velocities):
                if not math.isfinite(val):
                    self._latch_fault(
                        CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION,
                        f"Non-finite target velocity at index {i}: {val}",
                    )
                    raise CommandSafetyViolationError(f"Non-finite target velocity at index {i}: {val}")

            # Velocity limit validation
            scale = self._config.velocity_limit_scale
            tol = 1e-9
            for i, (val, joint) in enumerate(zip(target_velocities, self._model.arm_joints)):
                max_allowed = scale * joint.max_velocity
                if abs(val) > max_allowed + tol:
                    msg = f"Velocity limit violation: Target velocity {val:.4f} at joint {joint.name} exceeds scaled envelope {max_allowed:.4f}"
                    self._latch_fault(
                        CommandSafetyFaultCode.VELOCITY_LIMIT_VIOLATION,
                        msg,
                    )
                    raise CommandSafetyViolationError(msg)

            # State check
            state = self._backend.get_joint_state()
            if self._config.require_velocity_feedback_for_velocity_commands:
                if state.velocities is None or len(state.velocities) != expected_dof or not all(math.isfinite(v) for v in state.velocities):
                    self._latch_fault(
                        CommandSafetyFaultCode.STATE_INVALID,
                        "State telemetry is missing valid velocity feedback required by configuration.",
                    )
                    raise CommandSafetyViolationError("State telemetry is missing valid velocity feedback required by configuration.")

            res = self._backend.command_joint_velocities(target_velocities, effort_limit=effort_limit)
            if res:
                self._accepted_command_sequence += 1
                self._last_accepted_command_s = time.monotonic()
                self._motion_session_active = True
                return True
            return False

    def halt_motion(self) -> None:
        """Privileged software stop mapping to underlying halt_motion()."""
        with self._command_lock:
            self._backend.halt_motion()
            self._motion_session_active = False
            if self._guard_state == SafetyGuardState.ARMED:
                self._guard_state = SafetyGuardState.DISARMED

    def _latch_fault(self, code: CommandSafetyFaultCode, detail: str) -> None:
        """Internal helper transitioning state to FAULT_LATCHED and recording fault detail."""
        self._guard_state = SafetyGuardState.FAULT_LATCHED
        self._active_fault = CommandSafetyFault(
            code=code,
            monotonic_timestamp_s=time.monotonic(),
            detail=detail,
        )
        self._rejected_command_count += 1
        if self._motion_session_active:
            try:
                self._backend.halt_motion()
            except Exception:
                pass
            self._motion_session_active = False
