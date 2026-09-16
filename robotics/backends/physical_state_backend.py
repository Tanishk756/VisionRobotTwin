"""Strictly read-only physical robot execution backend wrapping PhysicalTelemetryProvider.

Phase B4.1 vendor-neutral backend ensuring complete observation capability while
strictly rejecting all command operations and guaranteeing zero physical side-effects.
"""

from dataclasses import dataclass
import math
import time
from typing import Any, Callable, Dict, Optional, Sequence

from robotics.backends.base import (
    BackendError,
    BackendHealthStatus,
    BackendStateStaleError,
    BackendStateUnavailableError,
    ReadOnlyBackendError,
    RobotBackend,
    RobotBackendCapabilities,
    TimestampedJointState,
)
from robotics.backends.physical_identity import (
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
)
from robotics.backends.physical_readiness import PhysicalTelemetryProvider
from robotics.robot_model import ResolvedRobotModel


@dataclass(frozen=True)
class PhysicalBackendDiagnostics:
    """Immutable diagnostic snapshot of physical backend operation."""

    is_connected: bool
    health_status: BackendHealthStatus
    messages_read: int
    last_receive_monotonic_s: Optional[float]
    last_source_timestamp_s: Optional[float]
    is_stale: bool
    last_error: Optional[str]


class PhysicalRobotStateBackend(RobotBackend):
    """Strictly read-only execution backend wrapping a physical telemetry provider.

    Enforces fail-closed safety by rejecting any command dispatch attempt with ReadOnlyBackendError.
    """

    def __init__(
        self,
        provider: PhysicalTelemetryProvider,
        resolved_model: ResolvedRobotModel,
        state_timeout_s: float = 0.5,
        time_source: Optional[Callable[[], float]] = None,
    ) -> None:
        self._provider = provider
        self._model = resolved_model
        self._state_timeout_s = state_timeout_s
        self._time_source = time_source or time.monotonic

        self._messages_read = 0
        self._last_receive_monotonic_s: Optional[float] = None
        self._last_source_timestamp_s: Optional[float] = None
        self._last_error: Optional[str] = None

    def connect(self) -> bool:
        """Establishes read-only observation connection with zero command side effects."""
        return self._provider.connect()

    def disconnect(self) -> None:
        """Terminates read-only observation connection."""
        self._provider.disconnect()

    def is_connected(self) -> bool:
        """Queries whether provider is connected."""
        return self._provider.is_connected()

    def health_status(self) -> BackendHealthStatus:
        """Returns transport health status."""
        return self._provider.health_status()

    def get_joint_state(self) -> TimestampedJointState:
        """Returns fresh timestamped joint state mapped against ResolvedRobotModel.

        Raises:
            BackendStateUnavailableError: If state cannot be read or provider disconnected.
            BackendStateStaleError: If telemetry age exceeds state_timeout_s.
            BackendError: If joint names, DoF, or numerical values are invalid.
        """
        now_mono = self._time_source()
        try:
            state = self._provider.read_joint_state()
        except Exception as err:
            self._last_error = str(err)
            raise BackendStateUnavailableError(f"Failed to read joint state from provider: {err}") from err

        # Check freshness
        age = now_mono - state.receive_timestamp_s
        if age > self._state_timeout_s:
            self._last_error = f"Stale state: {age:.4f}s > {self._state_timeout_s:.4f}s"
            raise BackendStateStaleError(
                f"Physical joint state telemetry is stale ({age:.4f}s > {self._state_timeout_s:.4f}s)."
            )

        # Validate joint names and order against canonical model
        if state.joint_names != self._model.arm_joint_names:
            msg = f"State joint names {state.joint_names} != model arm joints {self._model.arm_joint_names}."
            self._last_error = msg
            raise BackendError(msg)

        if len(state.positions) != len(self._model.arm_joints):
            msg = f"State positions length {len(state.positions)} != model DoF {len(self._model.arm_joints)}."
            self._last_error = msg
            raise BackendError(msg)

        for i, val in enumerate(state.positions):
            if not math.isfinite(val):
                msg = f"Non-finite position value at joint index {i}: {val}"
                self._last_error = msg
                raise BackendError(msg)

        self._messages_read += 1
        self._last_receive_monotonic_s = state.receive_timestamp_s
        self._last_source_timestamp_s = state.source_timestamp_s
        self._last_error = None
        return state

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Prohibited on physical read-only backend."""
        raise ReadOnlyBackendError(
            "PhysicalRobotStateBackend is strictly read-only. Commanding is prohibited in Phase B4.1."
        )

    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Prohibited on physical read-only backend."""
        raise ReadOnlyBackendError(
            "PhysicalRobotStateBackend is strictly read-only. Commanding is prohibited in Phase B4.1."
        )

    def halt_motion(self) -> None:
        """Prohibited on physical read-only backend."""
        raise ReadOnlyBackendError(
            "PhysicalRobotStateBackend is strictly read-only. halt_motion is prohibited in Phase B4.1."
        )

    @property
    def transport_capabilities(self) -> RobotBackendCapabilities:
        """Declares strictly read-only capabilities with zero commanding support."""
        return RobotBackendCapabilities(
            read_only=True,
            position_commands=False,
            velocity_commands=False,
            effort_limit_override=False,
            halt_motion=False,
        )

    def read_identity(self) -> PhysicalRobotIdentity:
        """Delegates identity query to provider."""
        return self._provider.read_identity()

    def read_safety_status(self) -> PhysicalSafetyStatus:
        """Delegates safety status query to provider."""
        return self._provider.read_safety_status()

    def diagnostics(self) -> PhysicalBackendDiagnostics:
        """Returns diagnostic operational metrics."""
        now_mono = self._time_source()
        is_stale = (
            (now_mono - self._last_receive_monotonic_s > self._state_timeout_s)
            if self._last_receive_monotonic_s is not None
            else True
        )
        return PhysicalBackendDiagnostics(
            is_connected=self.is_connected(),
            health_status=self.health_status(),
            messages_read=self._messages_read,
            last_receive_monotonic_s=self._last_receive_monotonic_s,
            last_source_timestamp_s=self._last_source_timestamp_s,
            is_stale=is_stale,
            last_error=self._last_error,
        )
