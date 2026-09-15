"""PyBullet execution backend for robot joint state acquisition and motor command transport."""

import math
import time
from typing import List, Optional, Sequence, Tuple
try:
    import pybullet as p
except ImportError:
    p = None  # type: ignore

from robotics.backends.base import (
    BackendHealthStatus,
    RobotBackend,
    TimestampedJointState,
)


class PyBulletRobotBackend(RobotBackend):
    """Execution backend interfacing with a PyBullet simulation instance.

    In accordance with Architectural Ruling B, this backend attaches to an existing
    physics client ID and robot body ID owned and stepped by PyBulletSimulator. It does
    NOT create a new physics client or destroy the shared simulation environment upon
    disconnection.
    """

    def __init__(
        self,
        physics_client_id: int,
        robot_body_id: int,
        arm_joint_indices: Sequence[int],
        joint_names: Sequence[str],
        default_joint_force: float = 200.0,
    ) -> None:
        if len(arm_joint_indices) != len(joint_names):
            raise ValueError(
                f"Length mismatch: {len(arm_joint_indices)} arm_joint_indices vs {len(joint_names)} joint_names"
            )
        if not math.isfinite(default_joint_force) or default_joint_force <= 0.0:
            raise ValueError(f"default_joint_force must be finite and positive, got {default_joint_force}")

        self.physics_client_id: int = int(physics_client_id)
        self.robot_body_id: int = int(robot_body_id)
        self.arm_joint_indices: List[int] = [int(idx) for idx in arm_joint_indices]
        self.joint_names: Tuple[str, ...] = tuple(joint_names)
        self.default_joint_force: float = float(default_joint_force)

        self._attached: bool = False
        self._sequence_id: int = 0

    def connect(self) -> bool:
        """Attaches to the active PyBullet client and marks backend active."""
        try:
            info = p.getConnectionInfo(physicsClientId=self.physics_client_id)
            if not info.get("isConnected", 0):
                self._attached = False
                return False
        except Exception:
            self._attached = False
            return False

        self._attached = True
        return True

    def disconnect(self) -> None:
        """Detaches backend and halts motion without destroying the simulator client."""
        if self._attached:
            try:
                self.halt_motion()
            except Exception:
                pass
            self._attached = False

    def is_connected(self) -> bool:
        """Returns True if backend is logically attached and underlying PyBullet client is live."""
        if not self._attached:
            return False
        try:
            info = p.getConnectionInfo(physicsClientId=self.physics_client_id)
            return bool(info.get("isConnected", 0))
        except Exception:
            return False

    def health_status(self) -> BackendHealthStatus:
        """Returns the health status based on connection liveness."""
        if self.is_connected():
            return BackendHealthStatus.HEALTHY
        return BackendHealthStatus.DISCONNECTED

    def get_joint_state(self) -> TimestampedJointState:
        """Queries PyBullet joint telemetry and returns an immutable TimestampedJointState.

        Note: PyBullet getJointStates does not provide an external hardware timestamp,
        so source_timestamp_s and receive_timestamp_s are sampled simultaneously from
        the local monotonic clock.
        """
        if not self.is_connected():
            raise RuntimeError("Cannot read joint state from disconnected PyBullet backend")

        now = time.monotonic()
        raw_states = p.getJointStates(
            self.robot_body_id,
            self.arm_joint_indices,
            physicsClientId=self.physics_client_id,
        )

        positions = tuple(float(s[0]) for s in raw_states)
        velocities = tuple(float(s[1]) for s in raw_states)
        efforts = tuple(float(s[3]) for s in raw_states)

        self._sequence_id += 1
        return TimestampedJointState(
            source_timestamp_s=now,
            receive_timestamp_s=now,
            joint_names=self.joint_names,
            positions=positions,
            velocities=velocities,
            efforts=efforts,
            sequence_id=self._sequence_id,
        )

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Dispatches target joint positions (rad) to PyBullet position control."""
        if not self.is_connected():
            raise RuntimeError("Cannot command positions to disconnected PyBullet backend")

        if len(target_positions) != len(self.arm_joint_indices):
            raise ValueError(
                f"Length mismatch: expected {len(self.arm_joint_indices)} positions, got {len(target_positions)}"
            )

        validated_positions = []
        for val in target_positions:
            if not math.isfinite(val):
                raise ValueError(f"Non-finite position command: {val}")
            validated_positions.append(float(val))

        p.setJointMotorControlArray(
            bodyIndex=self.robot_body_id,
            jointIndices=self.arm_joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=validated_positions,
            forces=[self.default_joint_force] * len(self.arm_joint_indices),
            physicsClientId=self.physics_client_id,
        )
        return True

    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Dispatches target joint velocities (rad/s) to PyBullet velocity control."""
        if not self.is_connected():
            raise RuntimeError("Cannot command velocities to disconnected PyBullet backend")

        if len(target_velocities) != len(self.arm_joint_indices):
            raise ValueError(
                f"Length mismatch: expected {len(self.arm_joint_indices)} velocities, got {len(target_velocities)}"
            )

        validated_velocities = []
        for val in target_velocities:
            if not math.isfinite(val):
                raise ValueError(f"Non-finite velocity command: {val}")
            validated_velocities.append(float(val))

        if effort_limit is not None:
            if not math.isfinite(effort_limit):
                raise ValueError(f"Non-finite effort_limit: {effort_limit}")
            if effort_limit < 0.0:
                raise ValueError(f"effort_limit must be non-negative, got {effort_limit}")
            forces = [float(effort_limit)] * len(self.arm_joint_indices)
        else:
            forces = [self.default_joint_force] * len(self.arm_joint_indices)

        p.setJointMotorControlArray(
            bodyIndex=self.robot_body_id,
            jointIndices=self.arm_joint_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=validated_velocities,
            forces=forces,
            physicsClientId=self.physics_client_id,
        )
        return True

    def halt_motion(self) -> None:
        """Halts motion by commanding zero velocities for all arm joints."""
        if self.is_connected():
            self.command_joint_velocities([0.0] * len(self.arm_joint_indices))
