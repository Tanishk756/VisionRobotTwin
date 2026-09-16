"""Unit tests for MockPhysicalTelemetryAdapter and observation isolation."""

import pytest

from robotics.backends.base import BackendHealthStatus, TimestampedJointState
from robotics.backends.physical_identity import (
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)
from tests.mocks.mock_physical_adapter import MockPhysicalTelemetryAdapter


class TestMockPhysicalTelemetryAdapter:
    """Tests for MockPhysicalTelemetryAdapter behavior and isolation."""

    def test_nominal_adapter_creation_and_telemetry(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal(
            vendor="MockVendor",
            model="MockArm",
            joint_names=("joint1", "joint2"),
        )
        assert adapter.is_connected() is True
        assert adapter.health_status() == BackendHealthStatus.HEALTHY

        ident = adapter.read_identity()
        assert ident.vendor == "MockVendor"
        assert ident.joint_count == 2

        state = adapter.read_joint_state()
        assert state.joint_names == ("joint1", "joint2")
        assert len(state.positions) == 2

        safety = adapter.read_safety_status()
        assert safety.all_signals_safe() is True
        assert adapter.command_side_effect_count == 0

    def test_disconnected_scenario(self):
        adapter = MockPhysicalTelemetryAdapter.create_disconnected()
        assert adapter.is_connected() is False
        assert adapter.health_status() == BackendHealthStatus.DISCONNECTED

        with pytest.raises(RuntimeError, match="not connected"):
            adapter.read_joint_state()

    def test_faulted_scenario(self):
        adapter = MockPhysicalTelemetryAdapter.create_faulted(fault_detail="Hardware thermal trip")
        safety = adapter.read_safety_status()
        assert safety.vendor_fault_clear == SafetySignalState.UNSAFE
        assert safety.vendor_status_summary == "Hardware thermal trip"

    def test_e_stop_active_scenario(self):
        adapter = MockPhysicalTelemetryAdapter.create_e_stop_active()
        safety = adapter.read_safety_status()
        assert safety.emergency_stop_clear == SafetySignalState.UNSAFE

    def test_call_counting_and_zero_commands(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal(
            vendor="MockVendor",
            model="MockArm",
            joint_names=("joint1",),
        )
        adapter.connect()
        adapter.read_identity()
        adapter.read_joint_state()
        adapter.read_safety_status()
        adapter.health_status()
        adapter.disconnect()

        assert adapter.connect_count == 1
        assert adapter.read_identity_count == 1
        assert adapter.read_joint_state_count == 1
        assert adapter.read_safety_status_count == 1
        assert adapter.health_status_count == 1
        assert adapter.disconnect_count == 1
        assert adapter.command_side_effect_count == 0
