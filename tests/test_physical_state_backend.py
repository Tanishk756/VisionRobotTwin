"""Unit tests for PhysicalRobotStateBackend read-only contracts and zero-command guarantees."""

import pytest

from robotics.backends.base import (
    BackendHealthStatus,
    BackendStateStaleError,
    ReadOnlyBackendError,
    TimestampedJointState,
)
from robotics.backends.physical_identity import (
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)
from robotics.backends.physical_state_backend import PhysicalRobotStateBackend
from robotics.robot_model import (
    JointMotionType,
    JointRole,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)
from tests.mocks.mock_physical_adapter import MockPhysicalTelemetryAdapter


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


class TestPhysicalRobotStateBackend:
    """Tests verifying read-only behavior, error semantics, and zero-command guarantees."""

    def test_construction_has_zero_side_effects(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal()
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(adapter, model)

        assert adapter.connect_count == 0
        assert adapter.read_joint_state_count == 0
        assert adapter.command_side_effect_count == 0

    def test_connect_and_disconnect_delegation(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal()
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(adapter, model)

        assert backend.connect() is True
        assert adapter.connect_count == 1
        assert backend.is_connected() is True

        backend.disconnect()
        assert adapter.disconnect_count == 1
        assert backend.is_connected() is False

    def test_get_joint_state_nominal(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal(
            joint_names=("joint1", "joint2"),
            positions=(0.1, -0.2),
        )
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(adapter, model, state_timeout_s=1.0)

        backend.connect()
        state = backend.get_joint_state()
        assert state.joint_names == ("joint1", "joint2")
        assert state.positions == (0.1, -0.2)
        assert backend.health_status() == BackendHealthStatus.HEALTHY

    def test_get_joint_state_stale_raises_backend_state_stale_error(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal(now_s=100.0)
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(
            adapter,
            model,
            state_timeout_s=0.5,
            time_source=lambda: 102.0,  # 2.0s elapsed > 0.5s
        )
        backend.connect()

        with pytest.raises(BackendStateStaleError, match="stale"):
            backend.get_joint_state()

    def test_command_methods_strictly_raise_read_only_backend_error(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal()
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(adapter, model)
        backend.connect()

        with pytest.raises(ReadOnlyBackendError, match="strictly read-only"):
            backend.command_joint_positions([0.1, 0.2])

        with pytest.raises(ReadOnlyBackendError, match="strictly read-only"):
            backend.command_joint_velocities([0.1, 0.2])

        with pytest.raises(ReadOnlyBackendError, match="strictly read-only"):
            backend.halt_motion()

        # Command side effect counter must strictly remain 0
        assert adapter.command_side_effect_count == 0

    def test_transport_capabilities_strictly_read_only(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal()
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(adapter, model)

        caps = backend.transport_capabilities
        assert caps.read_only is True
        assert caps.position_commands is False
        assert caps.velocity_commands is False
        assert caps.effort_limit_override is False
        assert caps.halt_motion is False

    def test_identity_and_safety_passthrough(self):
        adapter = MockPhysicalTelemetryAdapter.create_nominal(
            vendor="Franka",
            model="Panda",
            joint_names=("joint1", "joint2"),
        )
        model = _make_test_robot_model(dof=2)
        backend = PhysicalRobotStateBackend(adapter, model)

        ident = backend.read_identity()
        assert ident.vendor == "Franka"
        assert ident.model == "Panda"

        safety = backend.read_safety_status()
        assert safety.all_signals_safe() is True
