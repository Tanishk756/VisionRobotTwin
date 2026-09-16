"""Pure ROS-independent JointState telemetry mapping and validation.

This module provides deterministic name-based mapping, extraneous joint pruning,
and telemetry validation for ROS2 sensor_msgs/msg/JointState payloads without
requiring rclpy, sensor_msgs, or any ROS2 system packages.
"""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import List, Optional, Sequence, Tuple

from robotics.backends.base import BackendError, TimestampedJointState


class JointStateMappingError(BackendError):
    """Raised when incoming joint telemetry cannot be mapped into canonical format."""
    pass


@dataclass(frozen=True)
class ROS2JointStateBackendConfig:
    """Configuration for ROS2 read-only joint state subscriber backend.

    Attributes:
        expected_joint_names: Ordered tuple of active arm joint names to extract.
        joint_state_topic: ROS topic to subscribe to for sensor_msgs/msg/JointState.
        node_name: ROS node name for the subscriber.
        node_namespace: Optional ROS namespace for the node.
        state_timeout_s: Staleness timeout in seconds (default 1.0s).
        qos_reliability: QoS reliability setting ('best_effort' | 'reliable').
        qos_depth: QoS history depth (default 5 for standard SensorDataQoS).
        domain_id: Optional integer ROS domain ID (0-101, or None for env default).
    """

    expected_joint_names: Tuple[str, ...]
    joint_state_topic: str = "/joint_states"
    node_name: str = "visionrobottwin_joint_state"
    node_namespace: str = ""
    state_timeout_s: float = 1.0
    qos_reliability: str = "best_effort"
    qos_depth: int = 5
    domain_id: Optional[int] = None

    def __post_init__(self) -> None:
        if not self.expected_joint_names:
            raise ValueError("expected_joint_names must be a non-empty sequence of joint names.")
        if len(set(self.expected_joint_names)) != len(self.expected_joint_names):
            raise ValueError(f"Duplicate names in expected_joint_names: {self.expected_joint_names}")
        if not self.joint_state_topic or not self.joint_state_topic.strip():
            raise ValueError("joint_state_topic must not be empty.")
        if not self.node_name or not self.node_name.strip():
            raise ValueError("node_name must not be empty.")
        if self.state_timeout_s <= 0.0 or not math.isfinite(self.state_timeout_s):
            raise ValueError(f"state_timeout_s must be a positive finite float, got {self.state_timeout_s}")
        if self.qos_depth < 1:
            raise ValueError(f"qos_depth must be at least 1, got {self.qos_depth}")
        if self.qos_reliability not in ("best_effort", "reliable"):
            raise ValueError(
                f"qos_reliability must be 'best_effort' or 'reliable', got '{self.qos_reliability}'"
            )
        if self.domain_id is not None and self.domain_id < 0:
            raise ValueError(f"domain_id must be non-negative if specified, got {self.domain_id}")


@dataclass(frozen=True)
class ROS2JointStateDiagnostics:
    """Immutable snapshot of ROS2 joint state telemetry diagnostics."""

    messages_received: int
    valid_messages: int
    invalid_messages: int
    last_receive_monotonic_s: Optional[float]
    last_source_timestamp_s: Optional[float]
    last_validation_error: Optional[str]
    topic: str
    is_stale: bool
    executor_error: Optional[str] = None


def extract_ros_timestamp(sec: int, nanosec: int) -> float:
    """Converts ROS2 Time message components into a floating-point seconds timestamp.

    Args:
        sec: Non-negative integer seconds.
        nanosec: Non-negative integer nanoseconds [0, 999999999].

    Returns:
        Source timestamp in seconds.

    Raises:
        JointStateMappingError: If seconds or nanoseconds are invalid/negative.
    """
    if sec < 0 or nanosec < 0:
        raise JointStateMappingError(f"Invalid negative ROS timestamp: sec={sec}, nanosec={nanosec}")
    return float(sec) + float(nanosec) * 1e-9


def map_joint_state_payload(
    expected_joint_names: Sequence[str],
    incoming_names: Sequence[str],
    positions: Sequence[float],
    velocities: Optional[Sequence[float]] = None,
    efforts: Optional[Sequence[float]] = None,
    source_timestamp_s: float = 0.0,
    receive_timestamp_s: float = 0.0,
    sequence_id: int = 0,
) -> TimestampedJointState:
    """Maps incoming joint telemetry by name into the exact order of expected_joint_names.

    Rules:
    - incoming_names must be non-empty and have no duplicate entries.
    - Every joint in expected_joint_names must be present in incoming_names.
    - Extraneous joints in incoming_names not in expected_joint_names are ignored.
    - positions is mandatory and len(positions) must equal len(incoming_names).
    - velocities is optional (None or empty sequence); if non-empty, len(velocities) must equal len(incoming_names).
    - efforts is optional (None or empty sequence); if non-empty, len(efforts) must equal len(incoming_names).
    - All mapped values (positions, velocities, efforts, timestamps) must be finite floats.

    Args:
        expected_joint_names: Canonical ordered list of joint names to extract.
        incoming_names: Joint names as received from the ROS publisher.
        positions: Joint positions corresponding to incoming_names.
        velocities: Optional joint velocities corresponding to incoming_names.
        efforts: Optional joint efforts corresponding to incoming_names.
        source_timestamp_s: Source timestamp from header stamp.
        receive_timestamp_s: Local monotonic timestamp captured at reception.
        sequence_id: Monotonically increasing valid message counter.

    Returns:
        Immutable TimestampedJointState ordered exactly according to expected_joint_names.

    Raises:
        JointStateMappingError: If any validation or mapping invariant is violated.
    """
    if not expected_joint_names:
        raise JointStateMappingError("expected_joint_names must not be empty.")
    if not incoming_names:
        raise JointStateMappingError("incoming_names must not be empty.")
    if not positions:
        raise JointStateMappingError("positions must not be empty.")

    num_incoming = len(incoming_names)
    if len(positions) != num_incoming:
        raise JointStateMappingError(
            f"Position length ({len(positions)}) != incoming names length ({num_incoming})."
        )

    has_velocities = velocities is not None and len(velocities) > 0
    if has_velocities and len(velocities) != num_incoming:
        raise JointStateMappingError(
            f"Velocity length ({len(velocities)}) != incoming names length ({num_incoming})."
        )

    has_efforts = efforts is not None and len(efforts) > 0
    if has_efforts and len(efforts) != num_incoming:
        raise JointStateMappingError(
            f"Effort length ({len(efforts)}) != incoming names length ({num_incoming})."
        )

    # Check for duplicate incoming names
    name_lookup = {}
    for idx, name in enumerate(incoming_names):
        if name in name_lookup:
            raise JointStateMappingError(f"Duplicate incoming joint name detected: '{name}'.")
        name_lookup[name] = idx

    # Map by expected joint name
    mapped_positions: List[float] = []
    mapped_velocities: Optional[List[float]] = [] if has_velocities else None
    mapped_efforts: Optional[List[float]] = [] if has_efforts else None

    for req_name in expected_joint_names:
        if req_name not in name_lookup:
            raise JointStateMappingError(
                f"Missing expected joint '{req_name}' in incoming joint state names {list(incoming_names)}."
            )
        idx = name_lookup[req_name]

        pos_val = float(positions[idx])
        if not math.isfinite(pos_val):
            raise JointStateMappingError(f"Non-finite position value for joint '{req_name}': {pos_val}")
        mapped_positions.append(pos_val)

        if has_velocities and mapped_velocities is not None and velocities is not None:
            vel_val = float(velocities[idx])
            if not math.isfinite(vel_val):
                raise JointStateMappingError(f"Non-finite velocity value for joint '{req_name}': {vel_val}")
            mapped_velocities.append(vel_val)

        if has_efforts and mapped_efforts is not None and efforts is not None:
            eff_val = float(efforts[idx])
            if not math.isfinite(eff_val):
                raise JointStateMappingError(f"Non-finite effort value for joint '{req_name}': {eff_val}")
            mapped_efforts.append(eff_val)

    if not math.isfinite(source_timestamp_s) or not math.isfinite(receive_timestamp_s):
        raise JointStateMappingError(
            f"Non-finite timestamp values: source={source_timestamp_s}, receive={receive_timestamp_s}"
        )

    return TimestampedJointState(
        source_timestamp_s=float(source_timestamp_s),
        receive_timestamp_s=float(receive_timestamp_s),
        joint_names=tuple(expected_joint_names),
        positions=tuple(mapped_positions),
        velocities=tuple(mapped_velocities) if mapped_velocities is not None else None,
        efforts=tuple(mapped_efforts) if mapped_efforts is not None else None,
        sequence_id=int(sequence_id),
    )


class ROS2SimulationCommandMode(Enum):
    """Supported commanding modes for ROS2SimulationBackend."""

    POSITION = auto()
    VELOCITY = auto()


@dataclass(frozen=True)
class ROS2SimulationBackendConfig:
    """Immutable configuration for ROS2SimulationBackend.

    IMPORTANT SAFETY NOTICE:
    Setting environment="simulation" is a SOFTWARE POLICY DECLARATION only.
    The software cannot independently prove that a configured ROS command endpoint
    is not connected to physical hardware. Physical robot commanding is strictly
    prohibited in Phase B2.

    Attributes:
        state_config: Configuration for the composed read-only JointState backend.
        command_mode: Command streaming mode ('position' or 'velocity').
        position_command_topic: Command topic for JointGroupPositionController (used when command_mode='position').
        velocity_command_topic: Command topic for JointGroupVelocityController (used when command_mode='velocity').
        commands_enabled: If True, commands are enabled upon connection. Defaults to False.
        require_subscriber_ready: If True, requiring at least one active subscriber before commands succeed.
        command_qos: QoS profile for command publisher ('system_default', 'reliable', 'best_effort').
        environment: Software policy declaration. Must strictly equal 'simulation'.
        command_node_name: ROS2 node name for the command publisher node.
        command_node_namespace: ROS2 namespace for the command publisher node.
    """

    state_config: ROS2JointStateBackendConfig
    command_mode: str = "position"
    position_command_topic: Optional[str] = "/forward_position_controller/commands"
    velocity_command_topic: Optional[str] = "/forward_velocity_controller/commands"
    commands_enabled: bool = False
    require_subscriber_ready: bool = True
    command_qos: str = "system_default"
    environment: str = "simulation"
    command_node_name: str = "visionrobottwin_sim_command"
    command_node_namespace: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.state_config, ROS2JointStateBackendConfig):
            raise TypeError(
                f"state_config must be an instance of ROS2JointStateBackendConfig, got {type(self.state_config)}"
            )

        if self.environment != "simulation":
            raise ValueError(
                f"environment must strictly be 'simulation' for ROS2SimulationBackend, got '{self.environment}'. "
                "Physical hardware command backends are not supported."
            )

        if self.command_mode not in ("position", "velocity"):
            raise ValueError(f"command_mode must be 'position' or 'velocity', got '{self.command_mode}'")

        if self.command_qos not in ("system_default", "reliable", "best_effort"):
            raise ValueError(
                f"command_qos must be one of ('system_default', 'reliable', 'best_effort'), got '{self.command_qos}'"
            )

        if self.command_mode == "position":
            if not self.position_command_topic or not self.position_command_topic.strip():
                raise ValueError("position_command_topic must be a non-empty string when command_mode is 'position'")
        elif self.command_mode == "velocity":
            if not self.velocity_command_topic or not self.velocity_command_topic.strip():
                raise ValueError("velocity_command_topic must be a non-empty string when command_mode is 'velocity'")

        if not self.command_node_name or not self.command_node_name.strip():
            raise ValueError("command_node_name must be a non-empty string")


@dataclass(frozen=True)
class ROS2SimulationDiagnostics:
    """Immutable runtime diagnostics snapshot for ROS2SimulationBackend.

    Attributes:
        commands_attempted: Total command dispatch invocations.
        commands_published: Successfully published messages.
        commands_rejected: Rejected command dispatches.
        last_command_monotonic_s: Monotonic timestamp of last command attempt.
        last_command_mode: Active command mode ('position' or 'velocity').
        last_command_vector: Copy of last commanded vector.
        last_command_error: Error message from last rejected command.
        last_halt_monotonic_s: Monotonic timestamp of last halt_motion call.
        command_subscriber_count: Count of active subscribers to command topic.
        commands_enabled: Boolean flag indicating if command dispatch is authorized.
        state_diagnostics: Telemetry diagnostics snapshot from composed state backend.
    """

    commands_attempted: int = 0
    commands_published: int = 0
    commands_rejected: int = 0
    last_command_monotonic_s: Optional[float] = None
    last_command_mode: str = "position"
    last_command_vector: Optional[Tuple[float, ...]] = None
    last_command_error: Optional[str] = None
    last_halt_monotonic_s: Optional[float] = None
    command_subscriber_count: int = 0
    commands_enabled: bool = False
    state_diagnostics: Optional[ROS2JointStateDiagnostics] = None

