"""Target-neutral mock physical hardware telemetry adapter for observation testing."""

import time
from typing import Optional, Sequence, Tuple

from robotics.backends.base import BackendHealthStatus, TimestampedJointState
from robotics.backends.physical_identity import (
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)
from robotics.backends.physical_readiness import PhysicalTelemetryProvider


class MockPhysicalTelemetryAdapter:
    """Mock implementation of PhysicalTelemetryProvider for unit and contract testing.

    Provides deterministic simulation of healthy, faulted, stale, or mismatched physical
    robot hardware without real network, ROS, or vendor driver dependencies.
    """

    def __init__(
        self,
        identity: PhysicalRobotIdentity,
        joint_state: Optional[TimestampedJointState] = None,
        safety_status: Optional[PhysicalSafetyStatus] = None,
        connected: bool = True,
        health: BackendHealthStatus = BackendHealthStatus.HEALTHY,
    ) -> None:
        self._identity = identity
        self._joint_state = joint_state
        self._safety_status = safety_status
        self._connected = connected
        self._health = health

        # Telemetry query counters
        self.connect_count = 0
        self.disconnect_count = 0
        self.read_identity_count = 0
        self.read_joint_state_count = 0
        self.read_safety_status_count = 0
        self.health_status_count = 0

        # Strict command side effect counter (must remain 0)
        self.command_side_effect_count = 0

    def connect(self) -> bool:
        self.connect_count += 1
        self._connected = True
        return True

    def disconnect(self) -> None:
        self.disconnect_count += 1
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def read_joint_state(self) -> TimestampedJointState:
        self.read_joint_state_count += 1
        if not self._connected:
            raise RuntimeError("Cannot read joint state: adapter is not connected.")
        if self._joint_state is None:
            raise RuntimeError("No joint state telemetry configured in mock adapter.")
        return self._joint_state

    def read_identity(self) -> PhysicalRobotIdentity:
        self.read_identity_count += 1
        if not self._connected:
            raise RuntimeError("Cannot read identity: adapter is not connected.")
        return self._identity

    def read_safety_status(self) -> PhysicalSafetyStatus:
        self.read_safety_status_count += 1
        if not self._connected:
            raise RuntimeError("Cannot read safety status: adapter is not connected.")
        if self._safety_status is None:
            raise RuntimeError("No safety status configured in mock adapter.")
        return self._safety_status

    def health_status(self) -> BackendHealthStatus:
        self.health_status_count += 1
        if not self._connected:
            return BackendHealthStatus.DISCONNECTED
        return self._health

    def set_joint_state(self, state: TimestampedJointState) -> None:
        """Helper to inject new joint state telemetry."""
        self._joint_state = state

    def set_safety_status(self, status: PhysicalSafetyStatus) -> None:
        """Helper to inject new safety status."""
        self._safety_status = status

    def set_health(self, health: BackendHealthStatus) -> None:
        """Helper to inject new health status."""
        self._health = health

    @classmethod
    def create_nominal(
        cls,
        vendor: str = "MockVendor",
        model: str = "MockArm",
        joint_names: Tuple[str, ...] = ("joint1", "joint2"),
        positions: Optional[Tuple[float, ...]] = None,
        now_s: Optional[float] = None,
    ) -> "MockPhysicalTelemetryAdapter":
        """Creates a nominal, healthy, connected mock telemetry adapter."""
        t = now_s if now_s is not None else time.monotonic()
        pos = positions if positions is not None else tuple(0.0 for _ in joint_names)
        vel = tuple(0.0 for _ in joint_names)

        ident = PhysicalRobotIdentity(
            vendor=vendor,
            model=model,
            serial_number="MOCK-SN-001",
            robot_identifier="mock_cell_arm_1",
            firmware_version="1.0.0",
            joint_names=joint_names,
            joint_count=len(joint_names),
            receive_timestamp_s=t,
        )

        state = TimestampedJointState(
            source_timestamp_s=t,
            receive_timestamp_s=t,
            joint_names=joint_names,
            positions=pos,
            velocities=vel,
            sequence_id=1,
        )

        safety = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.SAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.SAFE,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            receive_timestamp_s=t,
            vendor_status_summary="NOMINAL",
        )

        return cls(
            identity=ident,
            joint_state=state,
            safety_status=safety,
            connected=True,
            health=BackendHealthStatus.HEALTHY,
        )

    @classmethod
    def create_disconnected(cls) -> "MockPhysicalTelemetryAdapter":
        """Creates a disconnected mock adapter."""
        ident = PhysicalRobotIdentity(
            vendor="MockVendor",
            model="MockArm",
            joint_names=("joint1",),
            joint_count=1,
            receive_timestamp_s=time.monotonic(),
        )
        return cls(identity=ident, connected=False, health=BackendHealthStatus.DISCONNECTED)

    @classmethod
    def create_faulted(cls, fault_detail: str = "Hardware thermal trip") -> "MockPhysicalTelemetryAdapter":
        """Creates a mock adapter reporting an active vendor hardware fault."""
        adapter = cls.create_nominal()
        t = time.monotonic()
        faulted_safety = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.UNSAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.SAFE,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            receive_timestamp_s=t,
            vendor_status_summary=fault_detail,
        )
        adapter.set_safety_status(faulted_safety)
        adapter.set_health(BackendHealthStatus.DEGRADED)
        return adapter

    @classmethod
    def create_e_stop_active(cls) -> "MockPhysicalTelemetryAdapter":
        """Creates a mock adapter reporting an active Emergency Stop."""
        adapter = cls.create_nominal()
        t = time.monotonic()
        estop_safety = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.SAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.UNSAFE,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            receive_timestamp_s=t,
            vendor_status_summary="EMERGENCY_STOP_ASSERTED",
        )
        adapter.set_safety_status(estop_safety)
        return adapter
