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
from typing import Optional, Sequence, Tuple

from robotics.backends.base import BackendError


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
