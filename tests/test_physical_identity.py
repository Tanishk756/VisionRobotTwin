"""Unit tests for Phase B4.1 Physical Identity and Three-State Safety Schemas."""

import pytest

from robotics.backends.physical_identity import (
    ExpectedPhysicalRobotIdentity,
    HardwareIdentityMismatchError,
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)


class TestSafetySignalState:
    """Tests for SafetySignalState three-state evaluation enum."""

    def test_enum_members(self):
        assert SafetySignalState.SAFE.value == "SAFE"
        assert SafetySignalState.UNSAFE.value == "UNSAFE"
        assert SafetySignalState.UNKNOWN.value == "UNKNOWN"

    def test_unknown_is_not_safe(self):
        sig = SafetySignalState.UNKNOWN
        assert sig != SafetySignalState.SAFE


class TestPhysicalRobotIdentity:
    """Tests for immutable PhysicalRobotIdentity telemetry schema."""

    def test_valid_identity_creation(self):
        ident = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            serial_number="SN-12345",
            robot_identifier="franka_arm_1",
            firmware_version="4.2.1",
            hardware_revision="revB",
            joint_names=("panda_joint1", "panda_joint2"),
            joint_count=2,
            control_endpoint_identity="192.168.1.100",
            source_timestamp_s=100.0,
            receive_timestamp_s=100.05,
        )
        assert ident.vendor == "Franka"
        assert ident.model == "Panda"
        assert ident.serial_number == "SN-12345"
        assert ident.joint_count == 2
        assert ident.joint_names == ("panda_joint1", "panda_joint2")

    def test_optional_fields_default_to_none(self):
        ident = PhysicalRobotIdentity(
            vendor="UniversalRobots",
            model="UR5e",
            joint_names=("ur_joint1",),
            joint_count=1,
            receive_timestamp_s=50.0,
        )
        assert ident.serial_number is None
        assert ident.robot_identifier is None
        assert ident.firmware_version is None
        assert ident.hardware_revision is None
        assert ident.control_endpoint_identity is None
        assert ident.source_timestamp_s is None

    def test_empty_vendor_raises_value_error(self):
        with pytest.raises(ValueError, match="vendor cannot be empty"):
            PhysicalRobotIdentity(
                vendor="",
                model="Panda",
                joint_names=("j1",),
                joint_count=1,
                receive_timestamp_s=1.0,
            )

    def test_empty_model_raises_value_error(self):
        with pytest.raises(ValueError, match="model cannot be empty"):
            PhysicalRobotIdentity(
                vendor="KUKA",
                model="",
                joint_names=("j1",),
                joint_count=1,
                receive_timestamp_s=1.0,
            )

    def test_empty_joint_names_raises_value_error(self):
        with pytest.raises(ValueError, match="joint_names cannot be empty"):
            PhysicalRobotIdentity(
                vendor="KUKA",
                model="iiwa",
                joint_names=(),
                joint_count=0,
                receive_timestamp_s=1.0,
            )

    def test_duplicate_joint_names_raises_value_error(self):
        with pytest.raises(ValueError, match="duplicate joint name"):
            PhysicalRobotIdentity(
                vendor="KUKA",
                model="iiwa",
                joint_names=("joint1", "joint1"),
                joint_count=2,
                receive_timestamp_s=1.0,
            )

    def test_joint_count_mismatch_raises_value_error(self):
        with pytest.raises(ValueError, match="joint_count .* != len"):
            PhysicalRobotIdentity(
                vendor="KUKA",
                model="iiwa",
                joint_names=("joint1", "joint2"),
                joint_count=3,
                receive_timestamp_s=1.0,
            )

    def test_non_finite_timestamp_raises_value_error(self):
        with pytest.raises(ValueError, match="receive_timestamp_s must be finite"):
            PhysicalRobotIdentity(
                vendor="KUKA",
                model="iiwa",
                joint_names=("joint1",),
                joint_count=1,
                receive_timestamp_s=float("nan"),
            )

    def test_frozen_immutability(self):
        ident = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("j1",),
            joint_count=1,
            receive_timestamp_s=1.0,
        )
        with pytest.raises(Exception):
            ident.vendor = "NewVendor"


class TestPhysicalSafetyStatus:
    """Tests for normalized PhysicalSafetyStatus schema."""

    def test_valid_safety_status(self):
        status = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.SAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.SAFE,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            source_timestamp_s=10.0,
            receive_timestamp_s=10.01,
            vendor_status_summary="OPERATIONAL_NOMINAL",
        )
        assert status.communication_healthy == SafetySignalState.SAFE
        assert status.all_signals_safe() is True

    def test_any_unsafe_or_unknown_fails_all_signals_safe(self):
        status_unsafe = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.UNSAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.SAFE,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            receive_timestamp_s=10.0,
        )
        assert status_unsafe.all_signals_safe() is False

        status_unknown = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.SAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.UNKNOWN,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            receive_timestamp_s=10.0,
        )
        assert status_unknown.all_signals_safe() is False

    def test_check_required_signals(self):
        status = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.SAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.SAFE,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.UNKNOWN,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.UNKNOWN,
            receive_timestamp_s=10.0,
        )
        assert (
            status.is_signal_subset_safe(
                ("communication_healthy", "emergency_stop_clear", "protective_stop_clear")
            )
            is True
        )
        assert (
            status.is_signal_subset_safe(
                ("communication_healthy", "drives_state_safe_or_known")
            )
            is False
        )


class TestExpectedPhysicalRobotIdentity:
    """Tests for ExpectedPhysicalRobotIdentity matching engine."""

    def test_exact_identity_match(self):
        actual = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            serial_number="SN-999",
            robot_identifier="arm_0",
            joint_names=("panda_j1", "panda_j2"),
            joint_count=2,
            receive_timestamp_s=1.0,
        )
        expected = ExpectedPhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("panda_j1", "panda_j2"),
            joint_count=2,
            serial_number="SN-999",
            robot_identifier="arm_0",
        )
        matches, reasons = expected.matches(actual)
        assert matches is True
        assert len(reasons) == 0

    def test_optional_serial_passes_when_expected_is_none(self):
        actual = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            serial_number="SN-999",
            joint_names=("panda_j1", "panda_j2"),
            joint_count=2,
            receive_timestamp_s=1.0,
        )
        expected = ExpectedPhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("panda_j1", "panda_j2"),
            joint_count=2,
            serial_number=None,
        )
        matches, reasons = expected.matches(actual)
        assert matches is True

    def test_vendor_mismatch(self):
        actual = PhysicalRobotIdentity(
            vendor="KUKA",
            model="Panda",
            joint_names=("j1",),
            joint_count=1,
            receive_timestamp_s=1.0,
        )
        expected = ExpectedPhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("j1",),
            joint_count=1,
        )
        matches, reasons = expected.matches(actual)
        assert matches is False
        assert any("Vendor mismatch" in r for r in reasons)

    def test_model_mismatch(self):
        actual = PhysicalRobotIdentity(
            vendor="Franka",
            model="FR3",
            joint_names=("j1",),
            joint_count=1,
            receive_timestamp_s=1.0,
        )
        expected = ExpectedPhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("j1",),
            joint_count=1,
        )
        matches, reasons = expected.matches(actual)
        assert matches is False
        assert any("Model mismatch" in r for r in reasons)

    def test_joint_ordering_mismatch(self):
        actual = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("j2", "j1"),
            joint_count=2,
            receive_timestamp_s=1.0,
        )
        expected = ExpectedPhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("j1", "j2"),
            joint_count=2,
        )
        matches, reasons = expected.matches(actual)
        assert matches is False
        assert any("Joint names/order mismatch" in r for r in reasons)

    def test_serial_mismatch_when_expected_specified(self):
        actual = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            serial_number="SN-001",
            joint_names=("j1",),
            joint_count=1,
            receive_timestamp_s=1.0,
        )
        expected = ExpectedPhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("j1",),
            joint_count=1,
            serial_number="SN-002",
        )
        matches, reasons = expected.matches(actual)
        assert matches is False
        assert any("Serial number mismatch" in r for r in reasons)
