"""Tests for ROS2SimulationBackend optional dependency handling, gating, and command dispatch."""

import time
import pytest
from unittest.mock import MagicMock, patch

from robotics.backends.base import (
    BackendCommandDisabledError,
    BackendCommandUnavailableError,
    BackendHealthStatus,
    BackendStateStaleError,
    BackendStateUnavailableError,
    OptionalDependencyError,
    RobotBackendCapabilities,
    TimestampedJointState,
    UnsupportedBackendOperationError,
)
from robotics.backends.ros2_state_mapping import (
    ROS2JointStateBackendConfig,
    ROS2SimulationBackendConfig,
    ROS2SimulationDiagnostics,
)


def _make_configs(command_mode="position", require_subscriber_ready=True):
    state_cfg = ROS2JointStateBackendConfig(
        expected_joint_names=("j1", "j2"),
        state_timeout_s=0.5,
    )
    sim_cfg = ROS2SimulationBackendConfig(
        state_config=state_cfg,
        command_mode=command_mode,
        position_command_topic="/test_pos_cmd",
        velocity_command_topic="/test_vel_cmd",
        require_subscriber_ready=require_subscriber_ready,
    )
    return state_cfg, sim_cfg


class MockFloat64MultiArray:
    def __init__(self):
        self.data = []


@pytest.fixture(autouse=True)
def _patch_ros_modules():
    """Ensure tests can instantiate ROS2SimulationBackend with mock rclpy environment."""
    import robotics.backends.ros2_simulation_backend as sim_module
    import robotics.backends.ros2_joint_state_backend as state_module

    with patch.object(sim_module, "_HAS_RCLPY", True), \
         patch.object(state_module, "_HAS_RCLPY", True), \
         patch.object(sim_module, "Float64MultiArray", MockFloat64MultiArray):
        yield


def test_optional_dependency_error_when_ros_absent():
    """Verify instantiation raises OptionalDependencyError when rclpy or std_msgs is missing."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, sim_cfg = _make_configs()
    with patch.object(sim_module, "_HAS_RCLPY", False):
        with pytest.raises(OptionalDependencyError, match="ROS2 dependencies"):
            sim_module.ROS2SimulationBackend(config=sim_cfg)


def test_transport_capabilities_for_position_and_velocity_modes():
    """Verify transport_capabilities reflects active command_mode."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position")
    _, vel_cfg = _make_configs("velocity")

    b_pos = sim_module.ROS2SimulationBackend(config=pos_cfg)
    assert b_pos.transport_capabilities.read_only is False
    assert b_pos.transport_capabilities.position_commands is True
    assert b_pos.transport_capabilities.velocity_commands is False
    assert b_pos.transport_capabilities.effort_limit_override is False
    assert b_pos.transport_capabilities.halt_motion is True

    b_vel = sim_module.ROS2SimulationBackend(config=vel_cfg)
    assert b_vel.transport_capabilities.read_only is False
    assert b_vel.transport_capabilities.position_commands is False
    assert b_vel.transport_capabilities.velocity_commands is True
    assert b_vel.transport_capabilities.effort_limit_override is False
    assert b_vel.transport_capabilities.halt_motion is True


def test_commands_disabled_by_default():
    """Verify that command and halt calls raise BackendCommandDisabledError before explicit enable."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position", require_subscriber_ready=False)

    b = sim_module.ROS2SimulationBackend(config=pos_cfg)
    b._is_connected = True
    b._state_backend = MagicMock()
    b._state_backend.is_connected.return_value = True

    assert not b.commands_enabled

    with pytest.raises(BackendCommandDisabledError, match="(?i)commands are disabled"):
        b.command_joint_positions([0.1, 0.2])

    with pytest.raises(BackendCommandDisabledError, match="(?i)commands are disabled"):
        b.halt_motion()


def test_mode_exclusivity_rejection():
    """Verify position backend rejects velocity calls and velocity backend rejects position calls."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position", require_subscriber_ready=False)
    _, vel_cfg = _make_configs("velocity", require_subscriber_ready=False)

    b_pos = sim_module.ROS2SimulationBackend(config=pos_cfg)
    b_pos._is_connected = True
    b_pos._commands_enabled = True

    with pytest.raises(UnsupportedBackendOperationError, match="position"):
        b_pos.command_joint_velocities([0.01, 0.02])

    b_vel = sim_module.ROS2SimulationBackend(config=vel_cfg)
    b_vel._is_connected = True
    b_vel._commands_enabled = True

    with pytest.raises(UnsupportedBackendOperationError, match="velocity"):
        b_vel.command_joint_positions([0.1, 0.2])


def test_velocity_mode_effort_limit_rejection():
    """Verify velocity backend rejects non-None effort_limit."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, vel_cfg = _make_configs("velocity", require_subscriber_ready=False)

    b_vel = sim_module.ROS2SimulationBackend(config=vel_cfg)
    b_vel._is_connected = True
    b_vel._commands_enabled = True

    with pytest.raises(UnsupportedBackendOperationError, match="effort_limit"):
        b_vel.command_joint_velocities([0.1, 0.2], effort_limit=50.0)


def test_state_freshness_gate_for_ordinary_commands():
    """Verify position command is blocked when telemetry state is unavailable or stale."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position", require_subscriber_ready=False)

    b = sim_module.ROS2SimulationBackend(config=pos_cfg)
    b._is_connected = True
    b._commands_enabled = True
    b._cmd_publisher = MagicMock()
    b._state_backend = MagicMock()

    # No state
    b._state_backend.get_joint_state.side_effect = BackendStateUnavailableError("No telemetry")
    with pytest.raises(BackendStateUnavailableError):
        b.command_joint_positions([0.1, 0.2])
    assert b._cmd_publisher.publish.call_count == 0

    # Stale state
    b._state_backend.get_joint_state.side_effect = BackendStateStaleError("Telemetry is stale")
    with pytest.raises(BackendStateStaleError):
        b.command_joint_positions([0.1, 0.2])
    assert b._cmd_publisher.publish.call_count == 0


def test_position_halt_requires_fresh_state_and_holds_measured_q():
    """Verify position halt publishes measured q_current and fails if state is stale."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position", require_subscriber_ready=False)

    b = sim_module.ROS2SimulationBackend(config=pos_cfg)
    b._is_connected = True
    b._commands_enabled = True
    b._cmd_publisher = MagicMock()
    b._state_backend = MagicMock()

    # Fresh state
    fresh_state = TimestampedJointState(
        source_timestamp_s=10.0,
        receive_timestamp_s=time.monotonic(),
        joint_names=("j1", "j2"),
        positions=(0.55, -0.44),
    )
    b._state_backend.get_joint_state.return_value = fresh_state

    b.halt_motion()
    b._cmd_publisher.publish.assert_called_once()
    published_msg = b._cmd_publisher.publish.call_args[0][0]
    assert published_msg.data == [0.55, -0.44]

    # Stale state during halt
    b._state_backend.get_joint_state.side_effect = BackendStateStaleError("Stale")
    with pytest.raises(BackendStateStaleError):
        b.halt_motion()


def test_velocity_halt_sends_zero_vector_even_when_telemetry_stale():
    """Verify velocity halt sends [0.0]*dof without requiring fresh telemetry."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, vel_cfg = _make_configs("velocity", require_subscriber_ready=False)

    b = sim_module.ROS2SimulationBackend(config=vel_cfg)
    b._is_connected = True
    b._commands_enabled = True
    b._cmd_publisher = MagicMock()
    b._state_backend = MagicMock()
    # Telemetry is stale / unavailable
    b._state_backend.get_joint_state.side_effect = BackendStateStaleError("Stale")

    b.halt_motion()
    b._cmd_publisher.publish.assert_called_once()
    published_msg = b._cmd_publisher.publish.call_args[0][0]
    assert published_msg.data == [0.0, 0.0]


def test_subscriber_readiness_gating():
    """Verify enable_simulation_commands fails when 0 subscribers present and require_subscriber_ready=True."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position", require_subscriber_ready=True)

    b = sim_module.ROS2SimulationBackend(config=pos_cfg)
    b._is_connected = True
    b._cmd_publisher = MagicMock()
    b._cmd_publisher.get_subscription_count.return_value = 0
    b._state_backend = MagicMock()
    b._state_backend.is_connected.return_value = True
    b._state_backend.get_joint_state.return_value = TimestampedJointState(
        source_timestamp_s=10.0,
        receive_timestamp_s=time.monotonic(),
        joint_names=("j1", "j2"),
        positions=(0.0, 0.0),
    )

    # Enable fails because subscriber count == 0
    with pytest.raises(BackendCommandUnavailableError, match="0 active subscribers"):
        b.enable_simulation_commands()
    assert not b.commands_enabled

    # When subscriber count >= 1, enable succeeds
    b._cmd_publisher.get_subscription_count.return_value = 1
    b.enable_simulation_commands()
    assert b.commands_enabled

    # If subscriber disappears during command dispatch
    b._cmd_publisher.get_subscription_count.return_value = 0
    with pytest.raises(BackendCommandUnavailableError, match="disconnected"):
        b.command_joint_positions([0.1, 0.2])


def test_diagnostics_snapshot():
    """Verify diagnostics() returns ROS2SimulationDiagnostics with command counts and state diagnostics."""
    import robotics.backends.ros2_simulation_backend as sim_module

    _, pos_cfg = _make_configs("position", require_subscriber_ready=False)

    b = sim_module.ROS2SimulationBackend(config=pos_cfg)
    diag = b.diagnostics()
    assert isinstance(diag, ROS2SimulationDiagnostics)
    assert diag.commands_attempted == 0
    assert diag.commands_published == 0
    assert diag.commands_rejected == 0
    assert diag.commands_enabled is False
