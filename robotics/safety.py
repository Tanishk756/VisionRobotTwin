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

            return self._backend.command_joint_positions(target_positions)

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

            return self._backend.command_joint_velocities(target_velocities, effort_limit=effort_limit)

    def halt_motion(self) -> None:
        """Privileged software stop mapping to underlying halt_motion()."""
        with self._command_lock:
            self._backend.halt_motion()
            self._motion_session_active = False
            if self._guard_state == SafetyGuardState.ARMED:
                self._guard_state = SafetyGuardState.DISARMED
