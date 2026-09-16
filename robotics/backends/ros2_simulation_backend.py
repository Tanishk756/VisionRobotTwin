"""Simulation-only ROS2 execution backend for joint position/velocity command transport.

IMPORTANT SAFETY NOTICE:
This backend is strictly designated for SIMULATED environments (ros2_control simulation,
Gazebo, or mock hardware). Setting environment="simulation" is a software policy declaration;
the software cannot independently prove that a configured ROS command endpoint is not
connected to physical hardware. Physical robot commanding is strictly prohibited in Phase B2.
"""

import math
import threading
import time
from typing import Optional, Sequence, Tuple

from robotics.backends.base import (
    BackendCommandDisabledError,
    BackendCommandUnavailableError,
    BackendHealthStatus,
    BackendStateStaleError,
    BackendStateUnavailableError,
    OptionalDependencyError,
    RobotBackend,
    RobotBackendCapabilities,
    TimestampedJointState,
    UnsupportedBackendOperationError,
)
from robotics.backends.ros2_joint_state_backend import ROS2JointStateBackend
from robotics.backends.ros2_state_mapping import (
    ROS2SimulationBackendConfig,
    ROS2SimulationDiagnostics,
)

# Guarded optional ROS2 imports
_HAS_RCLPY: bool = False
_RCLPY_IMPORT_ERROR: Optional[Exception] = None

rclpy = None
Context = None
Node = None
DurabilityPolicy = None
HistoryPolicy = None
QoSProfile = None
ReliabilityPolicy = None
qos_profile_system_default = None
Float64MultiArray = None

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_system_default,
    )
    from std_msgs.msg import Float64MultiArray

    _HAS_RCLPY = True
except (ImportError, Exception) as _err:
    _RCLPY_IMPORT_ERROR = _err


class ROS2SimulationBackend(RobotBackend):
    """Simulation command execution backend composing ROS2JointStateBackend.

    This backend:
    - Composes a read-only ROS2JointStateBackend instance for telemetry ingestion.
    - Manages an isolated command ROS2 Context, Node, and Publisher.
    - Specializes in exactly one immutable command mode ('position' or 'velocity') per instance.
    - Streams std_msgs/msg/Float64MultiArray messages to configured forward controllers.
    - Enforces fail-closed authorization: starts disabled by default until enable_simulation_commands().
    - Enforces telemetry state freshness before dispatching position and nonzero velocity commands.
    - Implements mode-specific software halt (velocity zero vs position measured hold).
    """

    def __init__(self, config: ROS2SimulationBackendConfig) -> None:
        if not _HAS_RCLPY:
            raise OptionalDependencyError(
                f"ROS2 dependencies (rclpy, std_msgs) are required to instantiate ROS2SimulationBackend: {_RCLPY_IMPORT_ERROR}. "
                "Ensure ROS2 Humble is installed and your environment is sourced."
            )
        if not isinstance(config, ROS2SimulationBackendConfig):
            raise TypeError(f"config must be an instance of ROS2SimulationBackendConfig, got {type(config)}")

        self._config = config
        self._lock = threading.RLock()

        # Compose read-only telemetry backend
        self._state_backend = ROS2JointStateBackend(config.state_config)

        self._cmd_context = None
        self._cmd_node = None
        self._cmd_publisher = None
        self._is_connected: bool = False
        self._commands_enabled: bool = bool(config.commands_enabled)

        self._commands_attempted: int = 0
        self._commands_published: int = 0
        self._commands_rejected: int = 0
        self._last_command_monotonic_s: Optional[float] = None
        self._last_command_vector: Optional[Tuple[float, ...]] = None
        self._last_command_error: Optional[str] = None
        self._last_halt_monotonic_s: Optional[float] = None

    @property
    def transport_capabilities(self) -> RobotBackendCapabilities:
        """Returns the declared command/telemetry capabilities for this backend instance."""
        is_pos = self._config.command_mode == "position"
        is_vel = self._config.command_mode == "velocity"
        return RobotBackendCapabilities(
            read_only=False,
            position_commands=is_pos,
            velocity_commands=is_vel,
            effort_limit_override=False,
            halt_motion=True,
        )

    @property
    def commands_enabled(self) -> bool:
        """Returns True if command dispatch is currently authorized."""
        with self._lock:
            return self._commands_enabled

    def command_endpoint_ready(self) -> bool:
        """Returns True if the command publisher is active and has at least one subscriber."""
        with self._lock:
            if not self._is_connected or self._cmd_publisher is None:
                return False
            try:
                return self._cmd_publisher.get_subscription_count() >= 1
            except Exception:
                return False

    def connect(self) -> bool:
        """Connects composed state backend and establishes isolated command publisher node."""
        with self._lock:
            if self._is_connected:
                return True

            # 1. Connect state backend
            if not self._state_backend.connect():
                return False

            # 2. Construct dedicated private context for command publishing
            cmd_ctx = Context() if Context is not None else (rclpy.context.Context() if rclpy is not None else None)
            if cmd_ctx is None:
                self._state_backend.disconnect()
                raise OptionalDependencyError("Cannot construct command ROS2 Context.")

            self._cmd_context = cmd_ctx
            if self._config.state_config.domain_id is not None:
                rclpy.init(context=self._cmd_context, domain_id=self._config.state_config.domain_id)
            else:
                rclpy.init(context=self._cmd_context)

            self._cmd_node = rclpy.create_node(
                self._config.command_node_name,
                namespace=self._config.command_node_namespace,
                context=self._cmd_context,
            )

            # 3. Resolve QoS profile
            if self._config.command_qos == "reliable":
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST if HistoryPolicy is not None else 1,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE if ReliabilityPolicy is not None else 1,
                    durability=DurabilityPolicy.VOLATILE if DurabilityPolicy is not None else 2,
                )
            elif self._config.command_qos == "best_effort":
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST if HistoryPolicy is not None else 1,
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT if ReliabilityPolicy is not None else 2,
                    durability=DurabilityPolicy.VOLATILE if DurabilityPolicy is not None else 2,
                )
            else:
                qos = qos_profile_system_default if qos_profile_system_default is not None else 10

            # 4. Create single selected command publisher
            if self._config.command_mode == "position":
                topic = self._config.position_command_topic
            else:
                topic = self._config.velocity_command_topic

            msg_type = Float64MultiArray if Float64MultiArray is not None else object
            self._cmd_publisher = self._cmd_node.create_publisher(msg_type, topic, qos)

            self._is_connected = True
            return True

    def disconnect(self) -> None:
        """Revokes command authorization, destroys command node/context, and disconnects state backend."""
        with self._lock:
            self._commands_enabled = False
            self._is_connected = False
            node = self._cmd_node
            pub = self._cmd_publisher
            ctx = self._cmd_context

        # Disconnect composed state backend
        self._state_backend.disconnect()

        with self._lock:
            if node is not None:
                try:
                    if pub is not None:
                        node.destroy_publisher(pub)
                    node.destroy_node()
                except Exception:
                    pass

            if ctx is not None:
                try:
                    if ctx.ok():
                        ctx.shutdown()
                except Exception:
                    pass

            self._cmd_publisher = None
            self._cmd_node = None
            self._cmd_context = None

    def is_connected(self) -> bool:
        """Returns True if both state and command transports are active."""
        with self._lock:
            return self._is_connected and self._state_backend.is_connected()

    def enable_simulation_commands(self) -> None:
        """Authorizes command publication after verifying state backend health and endpoint readiness."""
        with self._lock:
            if not self.is_connected():
                raise BackendCommandUnavailableError("Cannot enable simulation commands: backend is not connected.")

            # Validate telemetry freshness
            self._state_backend.get_joint_state()

            # Validate subscriber readiness if required
            if self._config.require_subscriber_ready and self._cmd_publisher is not None:
                subs = self._cmd_publisher.get_subscription_count()
                if subs < 1:
                    topic = (
                        self._config.position_command_topic
                        if self._config.command_mode == "position"
                        else self._config.velocity_command_topic
                    )
                    raise BackendCommandUnavailableError(
                        f"Cannot enable simulation commands: command publisher on topic '{topic}' has 0 active subscribers."
                    )

            self._commands_enabled = True

    def disable_commands(self) -> None:
        """Revokes command publication authorization immediately."""
        with self._lock:
            self._commands_enabled = False

    def health_status(self) -> BackendHealthStatus:
        """Returns the aggregate communication health of the backend."""
        with self._lock:
            if not self._is_connected:
                return BackendHealthStatus.DISCONNECTED
            state_health = self._state_backend.health_status()
            if state_health != BackendHealthStatus.HEALTHY:
                return state_health
            if self._config.require_subscriber_ready and self._cmd_publisher is not None:
                if self._cmd_publisher.get_subscription_count() < 1:
                    return BackendHealthStatus.DEGRADED
            return BackendHealthStatus.HEALTHY

    def get_joint_state(self) -> TimestampedJointState:
        """Delegates joint state acquisition to the composed read-only backend."""
        return self._state_backend.get_joint_state()

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Dispatches target joint positions to the configured position command topic."""
        with self._lock:
            self._commands_attempted += 1

            if not self._is_connected:
                err = "Backend is not connected."
                self._record_rejection(err)
                raise BackendCommandUnavailableError(err)

            if self._config.command_mode != "position":
                err = f"Backend is configured in '{self._config.command_mode}' mode; command_joint_positions is unsupported."
                self._record_rejection(err)
                raise UnsupportedBackendOperationError(err)

            if not self._commands_enabled:
                err = "Commands are disabled. Call enable_simulation_commands() first."
                self._record_rejection(err)
                raise BackendCommandDisabledError(err)

            if self._config.require_subscriber_ready and self._cmd_publisher is not None:
                if self._cmd_publisher.get_subscription_count() < 1:
                    err = f"Command subscriber on topic '{self._config.position_command_topic}' has disconnected."
                    self._record_rejection(err)
                    raise BackendCommandUnavailableError(err)

            # Freshness gate
            self._state_backend.get_joint_state()

            # Validate input dimensions and finiteness
            expected_dof = len(self._config.state_config.expected_joint_names)
            if len(target_positions) != expected_dof:
                err = f"Target positions length {len(target_positions)} != expected DoF {expected_dof}"
                self._record_rejection(err)
                raise ValueError(err)

            for i, val in enumerate(target_positions):
                if not math.isfinite(val):
                    err = f"Non-finite target position at index {i}: {val}"
                    self._record_rejection(err)
                    raise ValueError(err)

            # Publish Float64MultiArray
            msg = Float64MultiArray()
            msg.data = [float(val) for val in target_positions]
            if self._cmd_publisher is not None:
                self._cmd_publisher.publish(msg)

            now = time.monotonic()
            self._commands_published += 1
            self._last_command_monotonic_s = now
            self._last_command_vector = tuple(float(val) for val in target_positions)
            self._last_command_error = None
            return True

    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Dispatches target joint velocities to the configured velocity command topic."""
        with self._lock:
            self._commands_attempted += 1

            if effort_limit is not None:
                err = (
                    "effort_limit is not supported by ROS2SimulationBackend (Float64MultiArray carries no torque limit). "
                    "Fail closed rather than dropping constraint."
                )
                self._record_rejection(err)
                raise UnsupportedBackendOperationError(err)

            if not self._is_connected:
                err = "Backend is not connected."
                self._record_rejection(err)
                raise BackendCommandUnavailableError(err)

            if self._config.command_mode != "velocity":
                err = f"Backend is configured in '{self._config.command_mode}' mode; command_joint_velocities is unsupported."
                self._record_rejection(err)
                raise UnsupportedBackendOperationError(err)

            if not self._commands_enabled:
                err = "Commands are disabled. Call enable_simulation_commands() first."
                self._record_rejection(err)
                raise BackendCommandDisabledError(err)

            if self._config.require_subscriber_ready and self._cmd_publisher is not None:
                if self._cmd_publisher.get_subscription_count() < 1:
                    err = f"Command subscriber on topic '{self._config.velocity_command_topic}' has disconnected."
                    self._record_rejection(err)
                    raise BackendCommandUnavailableError(err)

            # Freshness gate for ordinary velocity commands
            self._state_backend.get_joint_state()

            expected_dof = len(self._config.state_config.expected_joint_names)
            if len(target_velocities) != expected_dof:
                err = f"Target velocities length {len(target_velocities)} != expected DoF {expected_dof}"
                self._record_rejection(err)
                raise ValueError(err)

            for i, val in enumerate(target_velocities):
                if not math.isfinite(val):
                    err = f"Non-finite target velocity at index {i}: {val}"
                    self._record_rejection(err)
                    raise ValueError(err)

            msg = Float64MultiArray()
            msg.data = [float(val) for val in target_velocities]
            if self._cmd_publisher is not None:
                self._cmd_publisher.publish(msg)

            now = time.monotonic()
            self._commands_published += 1
            self._last_command_monotonic_s = now
            self._last_command_vector = tuple(float(val) for val in target_velocities)
            self._last_command_error = None
            return True

    def halt_motion(self) -> None:
        """Sends an immediate application software motion stop."""
        with self._lock:
            if not self._is_connected:
                raise BackendCommandUnavailableError("Cannot halt: backend is not connected.")

            if not self._commands_enabled:
                raise BackendCommandDisabledError("Cannot halt: commands are disabled.")

            expected_dof = len(self._config.state_config.expected_joint_names)
            now = time.monotonic()

            if self._config.command_mode == "velocity":
                # Velocity halt: publishes zero vector without requiring fresh telemetry
                msg = Float64MultiArray()
                msg.data = [0.0] * expected_dof
                if self._cmd_publisher is not None:
                    self._cmd_publisher.publish(msg)
                self._last_halt_monotonic_s = now
            else:
                # Position halt: requires fresh measured q to publish hold setpoint
                state = self._state_backend.get_joint_state()
                msg = Float64MultiArray()
                msg.data = list(state.positions)
                if self._cmd_publisher is not None:
                    self._cmd_publisher.publish(msg)
                self._last_halt_monotonic_s = now

    def diagnostics(self) -> ROS2SimulationDiagnostics:
        """Returns an immutable snapshot of runtime command metrics."""
        with self._lock:
            sub_count = 0
            if self._cmd_publisher is not None:
                try:
                    sub_count = self._cmd_publisher.get_subscription_count()
                except Exception:
                    sub_count = 0

            state_diag = self._state_backend.diagnostics() if hasattr(self._state_backend, "diagnostics") else None

            return ROS2SimulationDiagnostics(
                commands_attempted=self._commands_attempted,
                commands_published=self._commands_published,
                commands_rejected=self._commands_rejected,
                last_command_monotonic_s=self._last_command_monotonic_s,
                last_command_mode=self._config.command_mode,
                last_command_vector=self._last_command_vector,
                last_command_error=self._last_command_error,
                last_halt_monotonic_s=self._last_halt_monotonic_s,
                command_subscriber_count=sub_count,
                commands_enabled=self._commands_enabled,
                state_diagnostics=state_diag,
            )

    def _record_rejection(self, error_message: str) -> None:
        """Internal helper recording command rejection metrics."""
        self._commands_rejected += 1
        self._last_command_error = error_message
