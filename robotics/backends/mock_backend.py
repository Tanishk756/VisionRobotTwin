"""Deterministic mock execution backend for contract and unit testing."""

import math
import time
from typing import List, Optional, Sequence, Tuple

from robotics.backends.base import (
    BackendHealthStatus,
    RobotBackend,
    TimestampedJointState,
)


class MockRobotBackend(RobotBackend):
    """Deterministic, standalone mock execution backend.

    This class is strictly test infrastructure. It performs no physics simulation,
    URDF parsing, or kinematics computations. It deterministically tracks commanded
    states and provides inspection hooks for unit tests.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        initial_positions: Optional[Sequence[float]] = None,
    ) -> None:
        if not joint_names:
            raise ValueError("joint_names sequence must not be empty")
        self._joint_names: Tuple[str, ...] = tuple(joint_names)
        self._num_joints: int = len(self._joint_names)

        if initial_positions is not None:
            if len(initial_positions) != self._num_joints:
                raise ValueError(
                    f"initial_positions length {len(initial_positions)} != num_joints {self._num_joints}"
                )
            for p in initial_positions:
                if not math.isfinite(p):
                    raise ValueError(f"Non-finite initial position: {p}")
            self._current_positions: Tuple[float, ...] = tuple(float(p) for p in initial_positions)
        else:
            self._current_positions = tuple(0.0 for _ in range(self._num_joints))

        self._current_velocities: Tuple[float, ...] = tuple(0.0 for _ in range(self._num_joints))
        self._current_efforts: Tuple[float, ...] = tuple(0.0 for _ in range(self._num_joints))

        self._connected: bool = False
        self._sequence_id: int = 0
        self._is_halted: bool = False

        self.last_commanded_positions: Optional[Tuple[float, ...]] = None
        self.last_commanded_velocities: Optional[Tuple[float, ...]] = None
        self.last_effort_limit: Optional[float] = None
        self.command_history: List[tuple] = []

    @property
    def is_halted(self) -> bool:
        """Indicates whether motion has been halted."""
        return self._is_halted

    def connect(self) -> bool:
        """Logically connects/activates the mock backend."""
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Logically disconnects the mock backend and halts motion."""
        self.halt_motion()
        self._connected = False

    def is_connected(self) -> bool:
        """Returns True if the mock backend is connected."""
        return self._connected

    def health_status(self) -> BackendHealthStatus:
        """Returns the current backend health status."""
        if self._connected:
            return BackendHealthStatus.HEALTHY
        return BackendHealthStatus.DISCONNECTED

    def get_joint_state(self) -> TimestampedJointState:
        """Returns the current deterministic joint state snapshot."""
        now = time.monotonic()
        self._sequence_id += 1
        return TimestampedJointState(
            source_timestamp_s=now,
            receive_timestamp_s=now,
            joint_names=self._joint_names,
            positions=self._current_positions,
            velocities=self._current_velocities,
            efforts=self._current_efforts,
            sequence_id=self._sequence_id,
        )

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Dispatches target joint positions."""
        if not self._connected:
            raise RuntimeError("Cannot command positions when backend is disconnected")

        if len(target_positions) != self._num_joints:
            raise ValueError(
                f"Length mismatch: expected {self._num_joints} positions, got {len(target_positions)}"
            )

        validated_positions = []
        for val in target_positions:
            if not math.isfinite(val):
                raise ValueError(f"Non-finite position command: {val}")
            validated_positions.append(float(val))

        pos_tuple = tuple(validated_positions)
        self.last_commanded_positions = pos_tuple
        self._current_positions = pos_tuple
        self._is_halted = False
        self.command_history.append(("position", pos_tuple))
        return True

    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Dispatches target joint velocities with optional effort limit."""
        if not self._connected:
            raise RuntimeError("Cannot command velocities when backend is disconnected")

        if len(target_velocities) != self._num_joints:
            raise ValueError(
                f"Length mismatch: expected {self._num_joints} velocities, got {len(target_velocities)}"
            )

        validated_velocities = []
        for val in target_velocities:
            if not math.isfinite(val):
                raise ValueError(f"Non-finite velocity command: {val}")
            validated_velocities.append(float(val))

        if effort_limit is not None:
            if not math.isfinite(effort_limit):
                raise ValueError(f"Non-finite effort_limit: {effort_limit}")
            if effort_limit < 0:
                raise ValueError(f"effort_limit must be non-negative, got {effort_limit}")
            effort_val: Optional[float] = float(effort_limit)
        else:
            effort_val = None

        vel_tuple = tuple(validated_velocities)
        self.last_commanded_velocities = vel_tuple
        self.last_effort_limit = effort_val
        self._current_velocities = vel_tuple
        self._is_halted = False
        self.command_history.append(("velocity", vel_tuple, effort_val))
        return True

    def halt_motion(self) -> None:
        """Halts motion by zeroing velocities."""
        zero_vels = tuple(0.0 for _ in range(self._num_joints))
        self._current_velocities = zero_vels
        self.last_commanded_velocities = zero_vels
        self._is_halted = True
        self.command_history.append(("halt", zero_vels))
