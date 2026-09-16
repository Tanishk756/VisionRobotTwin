"""Read-only ROS2 execution backend for sensor_msgs/msg/JointState telemetry ingestion.

This module provides a strictly read-only implementation of RobotBackend that subscribes
to ROS2 /joint_states, maps incoming joint telemetry into canonical VisionRobotTwin order,
isolates clock domains (source vs receive timestamps), and enforces fail-closed safety
by rejecting all commanding operations.
"""

import math
import threading
import time
from typing import Optional, Sequence

from robotics.backends.base import (
    BackendHealthStatus,
    BackendStateStaleError,
    BackendStateUnavailableError,
    OptionalDependencyError,
    ReadOnlyBackendError,
    RobotBackend,
    TimestampedJointState,
)
from robotics.backends.ros2_state_mapping import (
    ROS2JointStateBackendConfig,
    ROS2JointStateDiagnostics,
    extract_ros_timestamp,
    map_joint_state_payload,
)

# Guarded optional ROS2 imports
_HAS_RCLPY: bool = False
_RCLPY_IMPORT_ERROR: Optional[Exception] = None

rclpy = None
Context = None
SingleThreadedExecutor = None
Node = None
DurabilityPolicy = None
HistoryPolicy = None
QoSProfile = None
ReliabilityPolicy = None
ROSJointState = None

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import JointState as ROSJointState

    _HAS_RCLPY = True
except (ImportError, Exception) as _err:
    _RCLPY_IMPORT_ERROR = _err


class ROS2JointStateBackend(RobotBackend):
    """Read-only execution backend consuming ROS2 sensor_msgs/msg/JointState telemetry.

    This backend:
    - Owns an isolated private rclpy.context.Context per connection lifecycle.
    - Runs a SingleThreadedExecutor on a background daemon thread.
    - Re-orders incoming joint names by exact string matching against expected_joint_names.
    - Enforces fail-closed safety by raising ReadOnlyBackendError on all command endpoints.
    - Evaluates telemetry freshness using local monotonic receive timestamps.
    """

    def __init__(self, config: ROS2JointStateBackendConfig) -> None:
        if not _HAS_RCLPY:
            raise OptionalDependencyError(
                f"ROS2 dependencies (rclpy, sensor_msgs) are required to instantiate ROS2JointStateBackend: {_RCLPY_IMPORT_ERROR}. "
                "Ensure ROS2 Humble is installed and your environment is sourced."
            )
        if not isinstance(config, ROS2JointStateBackendConfig):
            raise TypeError(f"config must be an instance of ROS2JointStateBackendConfig, got {type(config)}")

        self._config = config
        self._lock = threading.RLock()

        self._context = None
        self._executor = None
        self._node = None
        self._subscription = None
        self._spin_thread: Optional[threading.Thread] = None

        self._latest_state: Optional[TimestampedJointState] = None
        self._sequence_id: int = 0
        self._messages_received: int = 0
        self._valid_messages: int = 0
        self._invalid_messages: int = 0
        self._last_receive_monotonic_s: Optional[float] = None
        self._last_source_timestamp_s: Optional[float] = None
        self._last_validation_error: Optional[str] = None
        self._executor_error: Optional[str] = None
        self._is_connected: bool = False

    @property
    def transport_capabilities(self):
        """Returns the declared command/telemetry capabilities of ROS2JointStateBackend."""
        from robotics.backends.base import RobotBackendCapabilities
        return RobotBackendCapabilities(
            read_only=True,
            position_commands=False,
            velocity_commands=False,
            effort_limit_override=False,
            halt_motion=False,
        )

    def connect(self) -> bool:
        """Establishes private ROS2 context, creates node and subscriber, and starts background executor."""
        with self._lock:
            if self._is_connected:
                return True

            # Construct brand new Context for each connect lifecycle
            self._context = Context() if Context is not None else rclpy.context.Context()
            if self._config.domain_id is not None:
                rclpy.init(context=self._context, domain_id=self._config.domain_id)
            else:
                rclpy.init(context=self._context)

            self._node = rclpy.create_node(
                self._config.node_name,
                namespace=self._config.node_namespace,
                context=self._context,
            )

            # Build QoS profile
            if QoSProfile is not None and ReliabilityPolicy is not None and HistoryPolicy is not None and DurabilityPolicy is not None:
                reliability = (
                    ReliabilityPolicy.BEST_EFFORT
                    if self._config.qos_reliability == "best_effort"
                    else ReliabilityPolicy.RELIABLE
                )
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=self._config.qos_depth,
                    reliability=reliability,
                    durability=DurabilityPolicy.VOLATILE,
                )
            else:
                qos = self._config.qos_depth

            self._subscription = self._node.create_subscription(
                ROSJointState,
                self._config.joint_state_topic,
                self._on_joint_state_message,
                qos,
            )

            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)

            self._spin_thread = threading.Thread(
                target=self._executor_spin_worker,
                name=f"ROS2JointStateBackend-{self._config.node_name}",
                daemon=True,
            )
            self._is_connected = True
            self._executor_error = None
            self._spin_thread.start()
            return True

    def disconnect(self) -> None:
        """Stops background executor, joins spin thread, destroys ROS node, and shuts down private context."""
        with self._lock:
            if not self._is_connected:
                return
            self._is_connected = False
            executor = self._executor
            spin_thread = self._spin_thread
            context = self._context
            node = self._node
            subscription = self._subscription

        # Shutdown executor from caller thread to wake spin loop
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass

        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=2.0)

        with self._lock:
            if node is not None:
                try:
                    if subscription is not None:
                        node.destroy_subscription(subscription)
                    node.destroy_node()
                except Exception:
                    pass

            if context is not None:
                try:
                    if context.ok():
                        context.shutdown()
                except Exception:
                    pass

            self._executor = None
            self._spin_thread = None
            self._node = None
            self._subscription = None
            self._context = None

    def is_connected(self) -> bool:
        """Returns True if the private ROS2 context and node are active."""
        with self._lock:
            return self._is_connected

    def health_status(self) -> BackendHealthStatus:
        """Returns the current communication health of the backend."""
        with self._lock:
            if not self._is_connected:
                return BackendHealthStatus.DISCONNECTED
            if self._executor_error is not None:
                return BackendHealthStatus.ERROR
            if self._latest_state is None:
                return BackendHealthStatus.DEGRADED
            now = time.monotonic()
            if not self._latest_state.is_fresh(now, self._config.state_timeout_s):
                return BackendHealthStatus.DEGRADED
            return BackendHealthStatus.HEALTHY

    def get_joint_state(self) -> TimestampedJointState:
        """Returns the most recent validated joint state telemetry snapshot."""
        with self._lock:
            if not self._is_connected:
                raise BackendStateUnavailableError("Backend is not connected.")
            if self._latest_state is None:
                raise BackendStateUnavailableError(
                    f"No joint state telemetry received yet on topic '{self._config.joint_state_topic}'."
                )
            now = time.monotonic()
            if not self._latest_state.is_fresh(now, self._config.state_timeout_s):
                age = self._latest_state.age_s(now)
                raise BackendStateStaleError(
                    f"Joint state telemetry is stale (age {age:.3f}s > timeout {self._config.state_timeout_s:.3f}s)."
                )
            return self._latest_state

    def command_joint_positions(self, target_positions: Sequence[float]) -> bool:
        """Fails closed: commanding is strictly unsupported on read-only telemetry backend."""
        raise ReadOnlyBackendError("ROS2JointStateBackend is read-only and cannot command robot motion.")

    def command_joint_velocities(
        self,
        target_velocities: Sequence[float],
        effort_limit: Optional[float] = None,
    ) -> bool:
        """Fails closed: commanding is strictly unsupported on read-only telemetry backend."""
        raise ReadOnlyBackendError("ROS2JointStateBackend is read-only and cannot command robot motion.")

    def halt_motion(self) -> None:
        """Fails closed: halt_motion cannot command physical robot hardware on read-only telemetry backend."""
        raise ReadOnlyBackendError("ROS2JointStateBackend is read-only; halt_motion() cannot halt physical hardware.")

    def diagnostics(self) -> ROS2JointStateDiagnostics:
        """Returns an immutable snapshot of runtime telemetry metrics."""
        with self._lock:
            now = time.monotonic()
            is_stale = (
                self._latest_state is None
                or not self._latest_state.is_fresh(now, self._config.state_timeout_s)
            )
            return ROS2JointStateDiagnostics(
                messages_received=self._messages_received,
                valid_messages=self._valid_messages,
                invalid_messages=self._invalid_messages,
                last_receive_monotonic_s=self._last_receive_monotonic_s,
                last_source_timestamp_s=self._last_source_timestamp_s,
                last_validation_error=self._last_validation_error,
                topic=self._config.joint_state_topic,
                is_stale=is_stale,
                executor_error=self._executor_error,
            )

    def _executor_spin_worker(self) -> None:
        """Internal worker function spinning the single-threaded ROS2 executor."""
        try:
            if self._executor is not None:
                self._executor.spin()
        except Exception as e:
            with self._lock:
                self._executor_error = str(e)
                self._is_connected = False

    def _on_joint_state_message(self, msg) -> None:
        """Subscription callback mapping incoming sensor_msgs/msg/JointState into canonical state."""
        receive_monotonic = time.monotonic()
        with self._lock:
            self._messages_received += 1

        try:
            sec = int(msg.header.stamp.sec)
            nanosec = int(msg.header.stamp.nanosec)
            source_stamp_s = extract_ros_timestamp(sec, nanosec)

            state = map_joint_state_payload(
                expected_joint_names=self._config.expected_joint_names,
                incoming_names=list(msg.name),
                positions=list(msg.position),
                velocities=list(msg.velocity) if len(msg.velocity) > 0 else None,
                efforts=list(msg.effort) if len(msg.effort) > 0 else None,
                source_timestamp_s=source_stamp_s,
                receive_timestamp_s=receive_monotonic,
                sequence_id=self._sequence_id + 1,
            )

            with self._lock:
                self._sequence_id += 1
                self._latest_state = state
                self._valid_messages += 1
                self._last_receive_monotonic_s = receive_monotonic
                self._last_source_timestamp_s = source_stamp_s
                self._last_validation_error = None
        except Exception as err:
            with self._lock:
                self._invalid_messages += 1
                self._last_validation_error = str(err)
