"""Execution backend base contracts, enums, and data models."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Optional, Sequence, Tuple
import numpy as np


class BackendError(RuntimeError):
    """Base exception for all execution backend operations."""
    pass


class BackendStateUnavailableError(BackendError):
    """Raised when joint state telemetry has not yet been received on the backend."""
    pass


class BackendStateStaleError(BackendError):
    """Raised when joint state telemetry is older than the configured staleness timeout."""
    pass


class BackendStateFieldUnavailableError(BackendError):
    """Raised when an optional joint state field (e.g. velocities or efforts) is requested but absent."""
    pass


class ReadOnlyBackendError(BackendError):
    """Raised when a commanding operation is attempted on a read-only execution backend."""
    pass


class OptionalDependencyError(BackendError):
    """Raised when an optional dependency required by a specific backend is missing."""
    pass


class BackendHealthStatus(Enum):
    """Health and communication status of an execution backend."""

    HEALTHY = auto()
    DEGRADED = auto()
    DISCONNECTED = auto()
    ERROR = auto()


@dataclass(frozen=True)
class TimestampedJointState:
    """Immutable, timestamped joint telemetry snapshot from an execution backend.

    Attributes:
        source_timestamp_s: Timestamp (s) recorded at origin/driver (e.g. ROS header stamp).
        receive_timestamp_s: Monotonic timestamp (s) recorded upon local receipt.
        joint_names: Ordered tuple of active joint names.
        positions: Ordered tuple of joint positions in radians (mandatory).
        velocities: Optional ordered tuple of joint velocities in rad/s.
        efforts: Optional ordered tuple of joint efforts/torques in N or Nm.
        sequence_id: Monotonically increasing frame identifier.
    """

    source_timestamp_s: float
    receive_timestamp_s: float
    joint_names: Tuple[str, ...]
    positions: Tuple[float, ...]
    velocities: Optional[Tuple[float, ...]] = None
    efforts: Optional[Tuple[float, ...]] = None
    sequence_id: int = 0

    def __post_init__(self) -> None:
        num_joints = len(self.joint_names)
        if len(self.positions) != num_joints:
            raise ValueError(
                f"Length mismatch: {num_joints} joint_names vs {len(self.positions)} positions"
            )
        if self.velocities is not None and len(self.velocities) != num_joints:
            raise ValueError(
                f"Length mismatch: {num_joints} joint_names vs {len(self.velocities)} velocities"
            )
        if self.efforts is not None and len(self.efforts) != num_joints:
            raise ValueError(
                f"Length mismatch: {num_joints} joint_names vs {len(self.efforts)} efforts"
            )

        if not math.isfinite(self.source_timestamp_s) or not math.isfinite(self.receive_timestamp_s):
            raise ValueError("Non-finite timestamp value")

        for i, val in enumerate(self.positions):
            if not math.isfinite(val):
                raise ValueError(f"Non-finite position value at index {i}: {val}")
        if self.velocities is not None:
            for i, val in enumerate(self.velocities):
                if not math.isfinite(val):
                    raise ValueError(f"Non-finite velocity value at index {i}: {val}")
        if self.efforts is not None:
            for i, val in enumerate(self.efforts):
                if not math.isfinite(val):
                    raise ValueError(f"Non-finite effort value at index {i}: {val}")

    def get_positions_array(self) -> np.ndarray:
        """Returns a new, copy-safe NumPy array of joint positions."""
        return np.array(self.positions, dtype=np.float64)

    def get_velocities_array(self) -> np.ndarray:
        """Returns a new, copy-safe NumPy array of joint velocities, or raises BackendStateFieldUnavailableError if None."""
        if self.velocities is None:
            raise BackendStateFieldUnavailableError("Joint velocities are not available in this joint state.")
        return np.array(self.velocities, dtype=np.float64)

    def get_efforts_array(self) -> Optional[np.ndarray]:
        """Returns a new, copy-safe NumPy array of joint efforts, or None if unavailable."""
        if self.efforts is None:
            return None
        return np.array(self.efforts, dtype=np.float64)

    def age_s(self, current_time_s: float) -> float:
        """Returns the data age in seconds relative to current_time_s using local receive timestamp."""
        return max(0.0, current_time_s - self.receive_timestamp_s)

    def is_fresh(self, current_time_s: float, timeout_s: float) -> bool:
        """Evaluates whether the telemetry is fresh within the configured timeout window."""
        return self.age_s(current_time_s) <= timeout_s


class RobotBackend(ABC):
    """Abstract execution backend interface for robot state acquisition and command transport."""

    @abstractmethod
    def connect(self) -> bool:
        """Establishes transport connection or attaches to existing simulator client."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Closes transport connection / detaches backend and halts joint motion."""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Returns True if the backend transport is logically attached and underlying connection is active."""
        pass

    @abstractmethod
    def get_joint_state(self) -> TimestampedJointState:
        """Returns the most recent timestamped joint state."""
        pass

    @abstractmethod
    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Dispatches target joint positions (rad) to the joint controller."""
        pass

    @abstractmethod
    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Dispatches target joint velocities (rad/s) with optional effort/force limit (N/Nm)."""
        pass

    @abstractmethod
    def halt_motion(self) -> None:
        """Sends an immediate software motion stop command (e.g. zero velocities)."""
        pass

    @abstractmethod
    def health_status(self) -> BackendHealthStatus:
        """Returns the transport health and communication state."""
        pass
