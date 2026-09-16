"""Protocols, schemas, and evaluator for physical robot readiness and observation safety.

Phase B4.1 vendor-neutral framework for evaluating physical telemetry freshness,
identity matching, model compatibility, and multi-signal safety chain status.
"""

from dataclasses import dataclass
import time
from typing import Optional, Protocol, Sequence, Tuple

from robotics.backends.base import BackendHealthStatus, TimestampedJointState
from robotics.backends.physical_identity import (
    ExpectedPhysicalRobotIdentity,
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)
from robotics.robot_model import ResolvedRobotModel


class PhysicalTelemetryProvider(Protocol):
    """Pure protocol for observing physical robot hardware telemetry.

    Defines observation-only capabilities. Contains zero command, zero drive-enable,
    and zero fault-clearing methods.
    """

    def connect(self) -> bool:
        """Establishes read-only observation connection to physical driver/hardware."""
        ...

    def disconnect(self) -> None:
        """Terminates read-only observation connection."""
        ...

    def is_connected(self) -> bool:
        """Returns True if telemetry channel is connected."""
        ...

    def read_joint_state(self) -> TimestampedJointState:
        """Returns latest timestamped joint state in physical transmission ordering."""
        ...

    def read_identity(self) -> PhysicalRobotIdentity:
        """Returns actual immutable physical robot hardware identity."""
        ...

    def read_safety_status(self) -> PhysicalSafetyStatus:
        """Returns normalized three-state physical safety status."""
        ...

    def health_status(self) -> BackendHealthStatus:
        """Returns provider transport health status."""
        ...


@dataclass(frozen=True)
class PhysicalReadinessReport:
    """Comprehensive readiness evaluation report separating observation from command eligibility.

    Attributes:
        ready_for_observation: True if provider is connected, healthy, and streaming fresh state.
        eligible_for_future_command_commissioning: True if all identity, model, limit, and safety
            checks are verified SAFE and ready for future Phase B4.2 commissioning.
        identity_match: True if actual identity matches expected profile.
        joint_model_match: True if joint names, count, and ordering match ResolvedRobotModel.
        communication_healthy: True if communication signal is SAFE.
        state_fresh: True if joint state age is within configured physical_state_timeout_s.
        vendor_fault_clear: True if vendor fault signal is SAFE.
        protective_stop_clear: True if protective stop signal is SAFE.
        emergency_stop_clear: True if emergency stop signal is SAFE.
        drives_state_known: True if drives state signal is SAFE.
        external_control_ready: True if external control signal is SAFE.
        safety_status_complete: True if all required safety signals are SAFE.
        limits_resolved: True if joint limits provenance was successfully validated.
        reasons: Tuple of explanatory rejection / diagnostic strings.
        evaluated_at_monotonic_s: Monotonic timestamp when report was produced.
    """

    ready_for_observation: bool
    eligible_for_future_command_commissioning: bool
    identity_match: bool
    joint_model_match: bool
    communication_healthy: bool
    state_fresh: bool
    vendor_fault_clear: bool
    protective_stop_clear: bool
    emergency_stop_clear: bool
    drives_state_known: bool
    external_control_ready: bool
    safety_status_complete: bool
    limits_resolved: bool
    reasons: Tuple[str, ...]
    evaluated_at_monotonic_s: float


class PhysicalReadinessEvaluator:
    """Deterministic readiness evaluator for physical telemetry providers."""

    def __init__(
        self,
        physical_state_timeout_s: float = 0.5,
        required_safety_signals: Sequence[str] = (
            "communication_healthy",
            "vendor_fault_clear",
            "protective_stop_clear",
            "emergency_stop_clear",
            "external_control_ready",
        ),
    ) -> None:
        self._physical_state_timeout_s = physical_state_timeout_s
        self._required_safety_signals = tuple(required_safety_signals)

    def evaluate(
        self,
        provider: PhysicalTelemetryProvider,
        resolved_model: ResolvedRobotModel,
        expected_identity: Optional[ExpectedPhysicalRobotIdentity] = None,
        limits_resolved: bool = True,
        now_s: Optional[float] = None,
    ) -> PhysicalReadinessReport:
        """Evaluates physical readiness against telemetry, model, and safety requirements."""
        current_time = now_s if now_s is not None else time.monotonic()
        reasons = []

        # 1. Connection and transport health
        is_conn = provider.is_connected()
        health = provider.health_status()
        if not is_conn:
            reasons.append("Provider is not connected.")
        if health != BackendHealthStatus.HEALTHY:
            reasons.append(f"Provider health status is {health.name}.")

        # 2. Joint state acquisition and freshness
        state_avail = False
        state_fresh = False
        joint_model_match = False
        try:
            state = provider.read_joint_state()
            state_avail = True
            age = current_time - state.receive_timestamp_s
            if age <= self._physical_state_timeout_s:
                state_fresh = True
            else:
                reasons.append(
                    f"Joint state telemetry is stale ({age:.4f}s > {self._physical_state_timeout_s:.4f}s)."
                )

            # Check model joint matching
            if state.joint_names != resolved_model.arm_joint_names:
                reasons.append(
                    f"State joint names {state.joint_names} != model arm joints {resolved_model.arm_joint_names}."
                )
            elif len(state.positions) != len(resolved_model.arm_joints):
                reasons.append(
                    f"State DoF {len(state.positions)} != model arm DoF {len(resolved_model.arm_joints)}."
                )
            else:
                joint_model_match = True
        except Exception as err:
            reasons.append(f"Failed to read joint state from provider: {err}")

        # 3. Identity acquisition and matching
        identity_match = False
        try:
            ident = provider.read_identity()
            if expected_identity is not None:
                matches, ident_reasons = expected_identity.matches(ident)
                if matches:
                    identity_match = True
                else:
                    reasons.extend(ident_reasons)
            else:
                # No specific expectation configured, identity available
                identity_match = True
        except Exception as err:
            reasons.append(f"Failed to read identity from provider: {err}")

        # 4. Safety status evaluation
        comm_healthy = False
        vendor_fault_clear = False
        protective_stop_clear = False
        emergency_stop_clear = False
        drives_state_known = False
        external_control_ready = False
        safety_status_complete = False

        try:
            safety = provider.read_safety_status()
            comm_healthy = safety.communication_healthy == SafetySignalState.SAFE
            vendor_fault_clear = safety.vendor_fault_clear == SafetySignalState.SAFE
            protective_stop_clear = safety.protective_stop_clear == SafetySignalState.SAFE
            emergency_stop_clear = safety.emergency_stop_clear == SafetySignalState.SAFE
            drives_state_known = safety.drives_state_safe_or_known == SafetySignalState.SAFE
            external_control_ready = safety.external_control_ready == SafetySignalState.SAFE

            # Check required safety signals
            safety_status_complete = safety.is_signal_subset_safe(self._required_safety_signals)
            if not safety_status_complete:
                for sig_name in self._required_safety_signals:
                    val = getattr(safety, sig_name, SafetySignalState.UNKNOWN)
                    if val != SafetySignalState.SAFE:
                        reasons.append(f"Required safety signal '{sig_name}' is {val.value}.")
        except Exception as err:
            reasons.append(f"Failed to read safety status from provider: {err}")

        # 5. Limit resolution check
        if not limits_resolved:
            reasons.append("Physical joint limits are not resolved or provenance is inconsistent.")

        # Observation requires connected, healthy transport with fresh joint state
        ready_for_observation = (
            is_conn
            and health == BackendHealthStatus.HEALTHY
            and state_avail
            and state_fresh
            and joint_model_match
        )

        # Future commanding requires observation ready PLUS identity match, safety completeness, and limits resolved
        eligible_for_command = (
            ready_for_observation
            and identity_match
            and safety_status_complete
            and limits_resolved
        )

        return PhysicalReadinessReport(
            ready_for_observation=ready_for_observation,
            eligible_for_future_command_commissioning=eligible_for_command,
            identity_match=identity_match,
            joint_model_match=joint_model_match,
            communication_healthy=comm_healthy,
            state_fresh=state_fresh,
            vendor_fault_clear=vendor_fault_clear,
            protective_stop_clear=protective_stop_clear,
            emergency_stop_clear=emergency_stop_clear,
            drives_state_known=drives_state_known,
            external_control_ready=external_control_ready,
            safety_status_complete=safety_status_complete,
            limits_resolved=limits_resolved,
            reasons=tuple(reasons),
            evaluated_at_monotonic_s=current_time,
        )
