"""Tests for pure ROS-independent JointState message mapping and validation."""

import math
import pytest
from robotics.backends.base import TimestampedJointState
from robotics.backends.ros2_state_mapping import (
    JointStateMappingError,
    ROS2JointStateBackendConfig,
    ROS2JointStateDiagnostics,
    extract_ros_timestamp,
    map_joint_state_payload,
)


def test_canonical_and_permuted_joint_ordering():
    """Verify that incoming joint positions are mapped into exact expected_joint_names order."""
    expected = ("panda_joint1", "panda_joint2", "panda_joint3")

    # Canonical order
    state_can = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["panda_joint1", "panda_joint2", "panda_joint3"],
        positions=[0.1, 0.2, 0.3],
        velocities=[0.01, 0.02, 0.03],
        efforts=[1.0, 2.0, 3.0],
        source_timestamp_s=10.0,
        receive_timestamp_s=10.002,
        sequence_id=1,
    )
    assert isinstance(state_can, TimestampedJointState)
    assert state_can.joint_names == expected
    assert state_can.positions == (0.1, 0.2, 0.3)
    assert state_can.velocities == (0.01, 0.02, 0.03)
    assert state_can.efforts == (1.0, 2.0, 3.0)
    assert state_can.source_timestamp_s == 10.0
    assert state_can.receive_timestamp_s == 10.002
    assert state_can.sequence_id == 1

    # Reversed order
    state_rev = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["panda_joint3", "panda_joint2", "panda_joint1"],
        positions=[0.3, 0.2, 0.1],
        velocities=[0.03, 0.02, 0.01],
        efforts=[3.0, 2.0, 1.0],
        source_timestamp_s=11.0,
        receive_timestamp_s=11.002,
        sequence_id=2,
    )
    assert state_rev.joint_names == expected
    assert state_rev.positions == (0.1, 0.2, 0.3)
    assert state_rev.velocities == (0.01, 0.02, 0.03)
    assert state_rev.efforts == (1.0, 2.0, 3.0)

    # Arbitrary permutation
    state_perm = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["panda_joint2", "panda_joint1", "panda_joint3"],
        positions=[0.2, 0.1, 0.3],
        velocities=[0.02, 0.01, 0.03],
        efforts=[2.0, 1.0, 3.0],
        source_timestamp_s=12.0,
        receive_timestamp_s=12.002,
        sequence_id=3,
    )
    assert state_perm.joint_names == expected
    assert state_perm.positions == (0.1, 0.2, 0.3)


def test_extra_joints_pruning():
    """Verify that extraneous joints (gripper fingers, mobile base, etc.) are safely ignored."""
    expected = ("j1", "j2")
    state = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["finger1", "j2", "wheel_left", "j1", "finger2"],
        positions=[0.04, -0.5, 1.5, 0.5, 0.04],
        velocities=[0.0, -0.05, 0.1, 0.05, 0.0],
        efforts=[0.0, 10.0, 2.0, -10.0, 0.0],
        source_timestamp_s=5.0,
        receive_timestamp_s=5.001,
        sequence_id=10,
    )
    assert state.joint_names == expected
    assert state.positions == (0.5, -0.5)
    assert state.velocities == (0.05, -0.05)
    assert state.efforts == (-10.0, 10.0)


def test_missing_expected_joint_rejection():
    """Verify that missing expected joints raise JointStateMappingError."""
    expected = ("j1", "j2", "j3")
    with pytest.raises(JointStateMappingError, match="Missing expected joint"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1, 0.2],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_duplicate_incoming_names_rejection():
    """Verify that duplicate incoming joint names raise JointStateMappingError due to ambiguity."""
    expected = ("j1", "j2")
    with pytest.raises(JointStateMappingError, match="Duplicate"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j1", "j2"],
            positions=[0.1, 0.2, 0.3],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_empty_names_and_positions_rejection():
    """Verify that empty names or positions are rejected."""
    expected = ("j1",)
    with pytest.raises(JointStateMappingError, match="empty"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=[],
            positions=[],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    with pytest.raises(JointStateMappingError, match="empty"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_length_mismatch_rejection():
    """Verify that array length mismatches raise JointStateMappingError."""
    expected = ("j1", "j2")

    # Position length != name length
    with pytest.raises(JointStateMappingError, match="Position length"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Velocity length != 0 and != name length
    with pytest.raises(JointStateMappingError, match="Velocity length"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1, 0.2],
            velocities=[0.01],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Effort length != 0 and != name length
    with pytest.raises(JointStateMappingError, match="Effort length"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1", "j2"],
            positions=[0.1, 0.2],
            velocities=[],
            efforts=[1.0, 2.0, 3.0],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_optional_velocity_and_effort_handling():
    """Verify that empty velocity and effort arrays result in None."""
    expected = ("j1", "j2")
    state = map_joint_state_payload(
        expected_joint_names=expected,
        incoming_names=["j1", "j2"],
        positions=[0.5, -0.5],
        velocities=[],
        efforts=None,
        source_timestamp_s=1.0,
        receive_timestamp_s=1.002,
        sequence_id=5,
    )
    assert state.positions == (0.5, -0.5)
    assert state.velocities is None
    assert state.efforts is None


def test_nan_and_inf_rejection():
    """Verify that NaN or +/-Inf in positions, velocities, or efforts raises JointStateMappingError."""
    expected = ("j1",)

    # NaN in position
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[float("nan")],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Inf in position
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[float("inf")],
            velocities=[],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # NaN in velocity
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[0.0],
            velocities=[float("nan")],
            efforts=[],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )

    # Inf in effort
    with pytest.raises(JointStateMappingError, match="Non-finite"):
        map_joint_state_payload(
            expected_joint_names=expected,
            incoming_names=["j1"],
            positions=[0.0],
            velocities=[0.0],
            efforts=[float("-inf")],
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            sequence_id=1,
        )


def test_extract_ros_timestamp():
    """Verify conversion of ROS sec/nanosec header stamps into float seconds."""
    assert extract_ros_timestamp(1694700000, 500000000) == 1694700000.5
    assert extract_ros_timestamp(0, 0) == 0.0
    assert extract_ros_timestamp(10, 123456789) == 10.123456789

    # Non-finite or negative nanosec
    with pytest.raises(JointStateMappingError):
        extract_ros_timestamp(10, -1)
    with pytest.raises(JointStateMappingError):
        extract_ros_timestamp(-1, 0)


def test_ros2_config_defaults_and_validation():
    """Verify ROS2JointStateBackendConfig defaults and field validation."""
    cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))
    assert cfg.expected_joint_names == ("j1", "j2")
    assert cfg.joint_state_topic == "/joint_states"
    assert cfg.node_name == "visionrobottwin_joint_state"
    assert cfg.node_namespace == ""
    assert cfg.state_timeout_s == 1.0
    assert cfg.qos_reliability == "best_effort"
    assert cfg.qos_depth == 5
    assert cfg.domain_id is None

    # Empty expected joints
    with pytest.raises(ValueError, match="expected_joint_names"):
        ROS2JointStateBackendConfig(expected_joint_names=())

    # Duplicate expected joints
    with pytest.raises(ValueError, match="Duplicate"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1", "j1"))

    # Empty topic
    with pytest.raises(ValueError, match="joint_state_topic"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1",), joint_state_topic="")

    # Empty node_name
    with pytest.raises(ValueError, match="node_name"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1",), node_name="")

    # Invalid timeout
    with pytest.raises(ValueError, match="state_timeout_s"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1",), state_timeout_s=0.0)

    # Invalid qos depth
    with pytest.raises(ValueError, match="qos_depth"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1",), qos_depth=0)

    # Invalid qos reliability
    with pytest.raises(ValueError, match="qos_reliability"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1",), qos_reliability="invalid_qos")

    # Negative domain_id
    with pytest.raises(ValueError, match="domain_id"):
        ROS2JointStateBackendConfig(expected_joint_names=("j1",), domain_id=-1)


def test_ros2_diagnostics_dataclass():
    """Verify ROS2JointStateDiagnostics immutability and fields."""
    diag = ROS2JointStateDiagnostics(
        messages_received=100,
        valid_messages=98,
        invalid_messages=2,
        last_receive_monotonic_s=100.5,
        last_source_timestamp_s=50.0,
        last_validation_error="Missing joint j2",
        topic="/joint_states",
        is_stale=False,
        executor_error=None,
    )
    assert diag.messages_received == 100
    assert diag.valid_messages == 98
    assert diag.invalid_messages == 2
    assert diag.last_receive_monotonic_s == 100.5
    assert diag.last_source_timestamp_s == 50.0
    assert diag.last_validation_error == "Missing joint j2"
    assert diag.topic == "/joint_states"
    assert diag.is_stale is False
    assert diag.executor_error is None


def test_ros2_simulation_command_mode_enum():
    """Verify ROS2SimulationCommandMode enum members."""
    from robotics.backends.ros2_state_mapping import ROS2SimulationCommandMode

    assert ROS2SimulationCommandMode.POSITION.name == "POSITION"
    assert ROS2SimulationCommandMode.VELOCITY.name == "VELOCITY"


def test_ros2_simulation_backend_config_valid():
    """Verify valid construction and defaults of ROS2SimulationBackendConfig."""
    from robotics.backends.ros2_state_mapping import (
        ROS2JointStateBackendConfig,
        ROS2SimulationBackendConfig,
    )

    state_cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))
    cfg = ROS2SimulationBackendConfig(state_config=state_cfg)

    assert cfg.state_config is state_cfg
    assert cfg.command_mode == "position"
    assert cfg.position_command_topic == "/forward_position_controller/commands"
    assert cfg.velocity_command_topic == "/forward_velocity_controller/commands"
    assert cfg.commands_enabled is False
    assert cfg.require_subscriber_ready is True
    assert cfg.command_qos == "system_default"
    assert cfg.environment == "simulation"
    assert cfg.command_node_name == "visionrobottwin_sim_command"


def test_ros2_simulation_backend_config_validation_rejections():
    """Verify validation rejections for ROS2SimulationBackendConfig."""
    from robotics.backends.ros2_state_mapping import (
        ROS2JointStateBackendConfig,
        ROS2SimulationBackendConfig,
    )

    state_cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))

    # Invalid state_config type
    with pytest.raises(TypeError, match="state_config"):
        ROS2SimulationBackendConfig(state_config="not_a_config")  # type: ignore[arg-type]

    # Non-simulation environment
    with pytest.raises(ValueError, match="environment must strictly be 'simulation'"):
        ROS2SimulationBackendConfig(state_config=state_cfg, environment="hardware")

    # Invalid command mode
    with pytest.raises(ValueError, match="command_mode"):
        ROS2SimulationBackendConfig(state_config=state_cfg, command_mode="torque")

    # Empty position topic in position mode
    with pytest.raises(ValueError, match="position_command_topic"):
        ROS2SimulationBackendConfig(
            state_config=state_cfg, command_mode="position", position_command_topic=""
        )

    # Empty velocity topic in velocity mode
    with pytest.raises(ValueError, match="velocity_command_topic"):
        ROS2SimulationBackendConfig(
            state_config=state_cfg, command_mode="velocity", velocity_command_topic=""
        )

    # Invalid command QoS
    with pytest.raises(ValueError, match="command_qos"):
        ROS2SimulationBackendConfig(state_config=state_cfg, command_qos="invalid_qos")

    # Empty node name
    with pytest.raises(ValueError, match="command_node_name"):
        ROS2SimulationBackendConfig(state_config=state_cfg, command_node_name="")


def test_ros2_simulation_diagnostics_dataclass():
    """Verify ROS2SimulationDiagnostics fields and immutability."""
    from robotics.backends.ros2_state_mapping import (
        ROS2JointStateDiagnostics,
        ROS2SimulationDiagnostics,
    )

    state_diag = ROS2JointStateDiagnostics(
        messages_received=10,
        valid_messages=10,
        invalid_messages=0,
        last_receive_monotonic_s=100.0,
        last_source_timestamp_s=50.0,
        last_validation_error=None,
        topic="/joint_states",
        is_stale=False,
        executor_error=None,
    )
    sim_diag = ROS2SimulationDiagnostics(
        commands_attempted=5,
        commands_published=4,
        commands_rejected=1,
        last_command_monotonic_s=100.1,
        last_command_mode="position",
        last_command_vector=(0.1, 0.2),
        last_command_error="Subscriber not ready",
        last_halt_monotonic_s=99.0,
        command_subscriber_count=1,
        commands_enabled=True,
        state_diagnostics=state_diag,
    )
    assert sim_diag.commands_attempted == 5
    assert sim_diag.commands_published == 4
    assert sim_diag.commands_rejected == 1
    assert sim_diag.last_command_monotonic_s == 100.1
    assert sim_diag.last_command_mode == "position"
    assert sim_diag.last_command_vector == (0.1, 0.2)
    assert sim_diag.last_command_error == "Subscriber not ready"
    assert sim_diag.last_halt_monotonic_s == 99.0
    assert sim_diag.command_subscriber_count == 1
    assert sim_diag.commands_enabled is True
    assert sim_diag.state_diagnostics is state_diag

