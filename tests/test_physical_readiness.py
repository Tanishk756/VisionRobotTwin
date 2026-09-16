"""Unit tests for PhysicalReadinessProvider protocol, report schema, and evaluator."""

import pytest

from robotics.backends.base import BackendHealthStatus, TimestampedJointState
from robotics.backends.physical_identity import (
    ExpectedPhysicalRobotIdentity,
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)
from robotics.backends.physical_readiness import (
    PhysicalReadinessEvaluator,
    PhysicalReadinessReport,
    PhysicalTelemetryProvider,
)
from robotics.robot_model import (
    JointMotionType,
    JointRole,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)


def _make_test_robot_model(dof: int = 2) -> ResolvedRobotModel:
    joints = tuple(
        ResolvedJointMetadata(
            model_index=i,
            canonical_index=i,
            name=f"joint{i+1}",
            role=JointRole.ARM,
            motion_type=JointMotionType.REVOLUTE,
            lower_limit=-3.14,
            upper_limit=3.14,
            max_force=50.0,
            max_velocity=2.0,
            link_name=f"link{i+1}",
        )
        for i in range(dof)
    )
    return ResolvedRobotModel(
        robot_id="test_robot",
        display_name="TestRobot",
        all_joints=joints,
        arm_joints=joints,
        gripper_joints=(),
        ee_link_name=f"link{dof}",
    )


class DummyTelemetryProvider:
    """Mock telemetry provider for testing readiness evaluation."""

    def __init__(
        self,
        connected: bool = True,
        identity: PhysicalRobotIdentity = None,
        state: TimestampedJointState = None,
        safety_status: PhysicalSafetyStatus = None,
        health: BackendHealthStatus = BackendHealthStatus.HEALTHY,
    ):
        self._connected = connected
        self._identity = identity
        self._state = state
        self._safety_status = safety_status
        self._health = health

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def read_joint_state(self) -> TimestampedJointState:
        if self._state is None:
            raise RuntimeError("No joint state")
        return self._state

    def read_identity(self) -> PhysicalRobotIdentity:
        if self._identity is None:
            raise RuntimeError("No identity")
        return self._identity

    def read_safety_status(self) -> PhysicalSafetyStatus:
        if self._safety_status is None:
            raise RuntimeError("No safety status")
        return self._safety_status

    def health_status(self) -> BackendHealthStatus:
        return self._health


class TestPhysicalReadinessEvaluation:
    """Tests for PhysicalReadinessEvaluator."""

    def test_nominal_observation_and_command_eligibility(self):
        model = _make_test_robot_model(dof=2)
        ident = PhysicalRobotIdentity(
            vendor="TestVendor",
            model="ModelX",
            joint_names=("joint1", "joint2"),
            joint_count=2,
            receive_timestamp_s=100.0,
        )
        expected_ident = ExpectedPhysicalRobotIdentity(
            vendor="TestVendor",
            model="ModelX",
            joint_names=("joint1", "joint2"),
            joint_count=2,
        )
        state = TimestampedJointState(
            source_timestamp_s=100.0,
            receive_timestamp_s=100.05,
            joint_names=("joint1", "joint2"),
            positions=(0.0, 0.0),
            velocities=(0.0, 0.0),
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
            receive_timestamp_s=100.05,
        )
        provider = DummyTelemetryProvider(
            connected=True,
            identity=ident,
            state=state,
            safety_status=safety,
        )

        evaluator = PhysicalReadinessEvaluator(physical_state_timeout_s=0.2)
        report = evaluator.evaluate(
            provider=provider,
            resolved_model=model,
            expected_identity=expected_ident,
            now_s=100.10,
        )

        assert report.ready_for_observation is True
        assert report.eligible_for_future_command_commissioning is True
        assert report.identity_match is True
        assert report.joint_model_match is True
        assert report.state_fresh is True
        assert len(report.reasons) == 0

    def test_disconnected_provider_blocks_all_readiness(self):
        model = _make_test_robot_model(dof=2)
        provider = DummyTelemetryProvider(connected=False)

        evaluator = PhysicalReadinessEvaluator()
        report = evaluator.evaluate(
            provider=provider,
            resolved_model=model,
            now_s=100.0,
        )

        assert report.ready_for_observation is False
        assert report.eligible_for_future_command_commissioning is False
        assert any("Provider is not connected" in r for r in report.reasons)

    def test_stale_state_blocks_observation_and_command(self):
        model = _make_test_robot_model(dof=2)
        ident = PhysicalRobotIdentity(
            vendor="TestVendor",
            model="ModelX",
            joint_names=("joint1", "joint2"),
            joint_count=2,
            receive_timestamp_s=100.0,
        )
        state = TimestampedJointState(
            source_timestamp_s=90.0,
            receive_timestamp_s=90.0,  # 10s old
            joint_names=("joint1", "joint2"),
            positions=(0.0, 0.0),
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
            receive_timestamp_s=90.0,
        )
        provider = DummyTelemetryProvider(
            connected=True,
            identity=ident,
            state=state,
            safety_status=safety,
        )

        evaluator = PhysicalReadinessEvaluator(physical_state_timeout_s=0.5)
        report = evaluator.evaluate(
            provider=provider,
            resolved_model=model,
            now_s=100.0,
        )

        assert report.state_fresh is False
        assert report.ready_for_observation is False
        assert report.eligible_for_future_command_commissioning is False
        assert any("stale" in r for r in report.reasons)

    def test_unknown_or_unsafe_safety_signal_blocks_command_eligibility(self):
        model = _make_test_robot_model(dof=2)
        ident = PhysicalRobotIdentity(
            vendor="TestVendor",
            model="ModelX",
            joint_names=("joint1", "joint2"),
            joint_count=2,
            receive_timestamp_s=100.0,
        )
        state = TimestampedJointState(
            source_timestamp_s=100.0,
            receive_timestamp_s=100.0,
            joint_names=("joint1", "joint2"),
            positions=(0.0, 0.0),
            sequence_id=1,
        )
        # Emergency stop is UNKNOWN
        safety = PhysicalSafetyStatus(
            communication_healthy=SafetySignalState.SAFE,
            vendor_fault_clear=SafetySignalState.SAFE,
            protective_stop_clear=SafetySignalState.SAFE,
            emergency_stop_clear=SafetySignalState.UNKNOWN,
            external_control_ready=SafetySignalState.SAFE,
            drives_state_safe_or_known=SafetySignalState.SAFE,
            operational_mode_safe=SafetySignalState.SAFE,
            brakes_state_safe_or_known=SafetySignalState.SAFE,
            receive_timestamp_s=100.0,
        )
        provider = DummyTelemetryProvider(
            connected=True,
            identity=ident,
            state=state,
            safety_status=safety,
        )

        evaluator = PhysicalReadinessEvaluator()
        report = evaluator.evaluate(
            provider=provider,
            resolved_model=model,
            now_s=100.05,
        )

        # Observation is allowed, but command eligibility is blocked
        assert report.ready_for_observation is True
        assert report.eligible_for_future_command_commissioning is False
        assert report.emergency_stop_clear is False
        assert any("emergency_stop_clear" in r for r in report.reasons)

    def test_unresolved_limits_blocks_command_eligibility(self):
        model = _make_test_robot_model(dof=2)
        ident = PhysicalRobotIdentity(
            vendor="TestVendor",
            model="ModelX",
            joint_names=("joint1", "joint2"),
            joint_count=2,
            receive_timestamp_s=100.0,
        )
        state = TimestampedJointState(
            source_timestamp_s=100.0,
            receive_timestamp_s=100.0,
            joint_names=("joint1", "joint2"),
            positions=(0.0, 0.0),
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
            receive_timestamp_s=100.0,
        )
        provider = DummyTelemetryProvider(
            connected=True,
            identity=ident,
            state=state,
            safety_status=safety,
        )

        evaluator = PhysicalReadinessEvaluator()
        report = evaluator.evaluate(
            provider=provider,
            resolved_model=model,
            limits_resolved=False,
            now_s=100.05,
        )

        assert report.ready_for_observation is True
        assert report.eligible_for_future_command_commissioning is False
        assert report.limits_resolved is False
        assert any("limits are not resolved" in r for r in report.reasons)
