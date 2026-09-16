"""Immutable schemas and three-state safety abstractions for physical robot hardware identity.

Phase B4.1 vendor-neutral specification for verifying real manipulator identity,
normalizing multi-vendor safety status signals, and detecting hardware mismatches.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Sequence, Tuple

from robotics.backends.base import BackendError


class SafetySignalState(Enum):
    """Three-state evaluation for safety-critical physical robot signals.

    Semantics:
    - SAFE: Condition is positively verified acceptable and healthy.
    - UNSAFE: Condition indicates an active fault, hazard, or stop condition.
    - UNKNOWN: Condition is unsupported, unexposed, stale, disconnected, or unverified.
    """

    SAFE = "SAFE"
    UNSAFE = "UNSAFE"
    UNKNOWN = "UNKNOWN"


class HardwareIdentityMismatchError(BackendError):
    """Raised when actual physical robot hardware identity fails to match expected commissioning profile."""


@dataclass(frozen=True)
class PhysicalRobotIdentity:
    """Immutable identity record reported directly by physical robot hardware or driver telemetry.

    Attributes:
        vendor: Manufacturer / vendor name (non-empty string).
        model: Robot manipulator model name (non-empty string).
        joint_names: Canonical tuple of joint names in physical transmission order.
        joint_count: Total number of arm joints (strictly equal to len(joint_names)).
        receive_timestamp_s: Local monotonic time of identity acquisition.
        serial_number: Optional hardware serial number string.
        robot_identifier: Optional user/cell robot identifier string.
        firmware_version: Optional controller firmware version string.
        hardware_revision: Optional physical hardware revision string.
        control_endpoint_identity: Optional IP address, URI, or interface descriptor.
        source_timestamp_s: Optional source/hardware clock timestamp in seconds.
    """

    vendor: str
    model: str
    joint_names: Tuple[str, ...]
    joint_count: int
    receive_timestamp_s: float
    serial_number: Optional[str] = None
    robot_identifier: Optional[str] = None
    firmware_version: Optional[str] = None
    hardware_revision: Optional[str] = None
    control_endpoint_identity: Optional[str] = None
    source_timestamp_s: Optional[float] = None

    def __post_init__(self) -> None:
        if not self.vendor or not self.vendor.strip():
            raise ValueError("vendor cannot be empty.")
        if not self.model or not self.model.strip():
            raise ValueError("model cannot be empty.")
        if not self.joint_names:
            raise ValueError("joint_names cannot be empty.")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError(f"duplicate joint name found in {self.joint_names}")
        if self.joint_count != len(self.joint_names):
            raise ValueError(
                f"joint_count ({self.joint_count}) != len(joint_names) ({len(self.joint_names)})"
            )
        if not math.isfinite(self.receive_timestamp_s):
            raise ValueError(f"receive_timestamp_s must be finite, got {self.receive_timestamp_s}")
        if self.source_timestamp_s is not None and not math.isfinite(self.source_timestamp_s):
            raise ValueError(f"source_timestamp_s must be finite, got {self.source_timestamp_s}")


@dataclass(frozen=True)
class PhysicalSafetyStatus:
    """Normalized immutable snapshot of physical hardware safety, fault, and drive signals.

    All safety signals are normalized into safe-oriented condition fields evaluated
    via SafetySignalState (SAFE, UNSAFE, UNKNOWN).
    """

    communication_healthy: SafetySignalState
    vendor_fault_clear: SafetySignalState
    protective_stop_clear: SafetySignalState
    emergency_stop_clear: SafetySignalState
    external_control_ready: SafetySignalState
    drives_state_safe_or_known: SafetySignalState
    operational_mode_safe: SafetySignalState
    brakes_state_safe_or_known: SafetySignalState
    receive_timestamp_s: float
    source_timestamp_s: Optional[float] = None
    vendor_status_summary: Optional[str] = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.receive_timestamp_s):
            raise ValueError(f"receive_timestamp_s must be finite, got {self.receive_timestamp_s}")
        if self.source_timestamp_s is not None and not math.isfinite(self.source_timestamp_s):
            raise ValueError(f"source_timestamp_s must be finite, got {self.source_timestamp_s}")

    def signals_equal(self, other: "PhysicalSafetyStatus") -> bool:
        """Returns True if all 8 safety signal states match regardless of timestamps."""
        return (
            self.communication_healthy == other.communication_healthy
            and self.vendor_fault_clear == other.vendor_fault_clear
            and self.protective_stop_clear == other.protective_stop_clear
            and self.emergency_stop_clear == other.emergency_stop_clear
            and self.external_control_ready == other.external_control_ready
            and self.drives_state_safe_or_known == other.drives_state_safe_or_known
            and self.operational_mode_safe == other.operational_mode_safe
            and self.brakes_state_safe_or_known == other.brakes_state_safe_or_known
        )

    def all_signals_safe(self) -> bool:
        """Returns True if all 8 normalized safety condition fields are verified SAFE."""
        signals = (
            self.communication_healthy,
            self.vendor_fault_clear,
            self.protective_stop_clear,
            self.emergency_stop_clear,
            self.external_control_ready,
            self.drives_state_safe_or_known,
            self.operational_mode_safe,
            self.brakes_state_safe_or_known,
        )
        return all(sig == SafetySignalState.SAFE for sig in signals)

    def is_signal_subset_safe(self, signal_names: Sequence[str]) -> bool:
        """Evaluates whether a specific subset of safety signal attributes are all SAFE."""
        for name in signal_names:
            val = getattr(self, name, SafetySignalState.UNKNOWN)
            if val != SafetySignalState.SAFE:
                return False
        return True


@dataclass(frozen=True)
class ExpectedPhysicalRobotIdentity:
    """Expected physical hardware configuration profile for identity verification."""

    vendor: str
    model: str
    joint_names: Tuple[str, ...]
    joint_count: int
    serial_number: Optional[str] = None
    robot_identifier: Optional[str] = None

    def matches(self, actual: PhysicalRobotIdentity) -> Tuple[bool, Tuple[str, ...]]:
        """Compares actual physical robot identity against commissioning expectations.

        Returns:
            Tuple of (matches: bool, reasons: Tuple[str, ...]).
        """
        reasons = []

        if actual.vendor != self.vendor:
            reasons.append(f"Vendor mismatch: expected '{self.vendor}', got '{actual.vendor}'")

        if actual.model != self.model:
            reasons.append(f"Model mismatch: expected '{self.model}', got '{actual.model}'")

        if actual.joint_count != self.joint_count:
            reasons.append(
                f"Joint count mismatch: expected {self.joint_count}, got {actual.joint_count}"
            )

        if actual.joint_names != self.joint_names:
            reasons.append(
                f"Joint names/order mismatch: expected {self.joint_names}, got {actual.joint_names}"
            )

        if self.serial_number is not None and actual.serial_number != self.serial_number:
            reasons.append(
                f"Serial number mismatch: expected '{self.serial_number}', got '{actual.serial_number}'"
            )

        if self.robot_identifier is not None and actual.robot_identifier != self.robot_identifier:
            reasons.append(
                f"Robot identifier mismatch: expected '{self.robot_identifier}', got '{actual.robot_identifier}'"
            )

        return len(reasons) == 0, tuple(reasons)
