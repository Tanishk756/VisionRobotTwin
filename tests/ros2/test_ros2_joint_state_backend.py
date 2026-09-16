"""Real ROS2 integration tests for ROS2JointStateBackend.

These tests require an active ROS2 installation (rclpy, sensor_msgs).
They will be skipped automatically in environments where ROS2 is not present (e.g. Windows CI).
"""

import time
import pytest

# Skip the entire module if rclpy or sensor_msgs is missing
rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

sensor_msgs = pytest.importorskip("sensor_msgs")
from sensor_msgs.msg import JointState as ROSJointState
from builtin_interfaces.msg import Time as ROSTime

from robotics.backends.base import (
    BackendStateUnavailableError,
    BackendStateStaleError,
    BackendStateFieldUnavailableError,
    ReadOnlyBackendError,
)
from robotics.backends.ros2_state_mapping import ROS2JointStateBackendConfig
from robotics.backends.ros2_joint_state_backend import ROS2JointStateBackend


def _create_publisher_helper(topic_name: str, qos_profile: QoSProfile, context: Context):
    """Helper to create a dedicated publisher node in its own context."""
    node = rclpy.create_node("test_joint_state_publisher", context=context)
    pub = node.create_publisher(ROSJointState, topic_name, qos_profile)
    return node, pub


def test_ros2_joint_state_reordering_and_mapping():
    """Verify backend receives real ROS2 JointState message, reorders joints correctly, and maps all fields."""
    pub_ctx = Context()
    rclpy.init(context=pub_ctx)
    try:
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_node, pub = _create_publisher_helper("/test_reorder_joint_states", qos, pub_ctx)

        config = ROS2JointStateBackendConfig(
            expected_joint_names=("joint_a", "joint_b", "joint_c"),
            joint_state_topic="/test_reorder_joint_states",
            node_name="test_backend_sub_node",
            state_timeout_s=1.0,
            qos_reliability="best_effort",
            qos_depth=5,
        )
        backend = ROS2JointStateBackend(config)
        assert backend.connect() is True
        assert backend.is_connected() is True

        # Publish deliberately reordered joint state with extra joint
        msg = ROSJointState()
        msg.header.stamp = ROSTime(sec=123, nanosec=456000000)
        msg.name = ["joint_c", "extra_finger", "joint_a", "joint_b"]
        msg.position = [3.0, 99.0, 1.0, 2.0]
        msg.velocity = [0.3, 0.99, 0.1, 0.2]
        msg.effort = [30.0, 990.0, 10.0, 20.0]

        # Publish repeatedly until received or timeout
        start = time.monotonic()
        received_state = None
        while time.monotonic() - start < 3.0:
            pub.publish(msg)
            rclpy.spin_once(pub_node, timeout_sec=0.05)
            if backend.health_status().is_healthy:
                received_state = backend.get_joint_state()
                break
            time.sleep(0.05)

        assert received_state is not None
        assert received_state.joint_names == ("joint_a", "joint_b", "joint_c")
        assert received_state.positions == (1.0, 2.0, 3.0)
        assert received_state.velocities == (0.1, 0.2, 0.3)
        assert received_state.efforts == (10.0, 20.0, 30.0)
        assert received_state.source_timestamp_s == pytest.approx(123.456, rel=1e-5)
        assert received_state.sequence_id >= 1

        backend.disconnect()
        assert backend.is_connected() is False
    finally:
        pub_node.destroy_node()
        pub_ctx.try_shutdown()


def test_ros2_joint_state_position_only():
    """Verify backend accepts position-only JointState and reports unavailable velocities/efforts."""
    pub_ctx = Context()
    rclpy.init(context=pub_ctx)
    try:
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_node, pub = _create_publisher_helper("/test_pos_only_joint_states", qos, pub_ctx)

        config = ROS2JointStateBackendConfig(
            expected_joint_names=("j1", "j2"),
            joint_state_topic="/test_pos_only_joint_states",
            node_name="test_backend_pos_only",
            state_timeout_s=1.0,
            qos_reliability="best_effort",
            qos_depth=5,
        )
        backend = ROS2JointStateBackend(config)
        assert backend.connect() is True

        msg = ROSJointState()
        msg.name = ["j1", "j2"]
        msg.position = [1.5, -0.5]
        msg.velocity = []
        msg.effort = []

        start = time.monotonic()
        received_state = None
        while time.monotonic() - start < 3.0:
            pub.publish(msg)
            rclpy.spin_once(pub_node, timeout_sec=0.05)
            if backend.health_status().is_healthy:
                received_state = backend.get_joint_state()
                break
            time.sleep(0.05)

        assert received_state is not None
        assert received_state.positions == (1.5, -0.5)
        assert received_state.velocities is None
        assert received_state.efforts is None

        with pytest.raises(BackendStateFieldUnavailableError):
            received_state.get_velocities_array()

        backend.disconnect()
    finally:
        pub_node.destroy_node()
        pub_ctx.try_shutdown()


def test_ros2_stale_state_timeout():
    """Verify backend marks health DEGRADED and raises BackendStateStaleError when state expires."""
    pub_ctx = Context()
    rclpy.init(context=pub_ctx)
    try:
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_node, pub = _create_publisher_helper("/test_stale_joint_states", qos, pub_ctx)

        config = ROS2JointStateBackendConfig(
            expected_joint_names=("j1",),
            joint_state_topic="/test_stale_joint_states",
            node_name="test_backend_stale",
            state_timeout_s=0.2,
            qos_reliability="best_effort",
            qos_depth=5,
        )
        backend = ROS2JointStateBackend(config)
        assert backend.connect() is True

        msg = ROSJointState()
        msg.name = ["j1"]
        msg.position = [0.0]

        # Publish 1 message and confirm healthy
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            pub.publish(msg)
            rclpy.spin_once(pub_node, timeout_sec=0.05)
            if backend.health_status().is_healthy:
                break
            time.sleep(0.05)

        assert backend.health_status().is_healthy is True
        assert backend.get_joint_state().positions == (0.0,)

        # Stop publishing and wait for stale timeout (timeout is 0.2s)
        time.sleep(0.3)

        assert backend.health_status().is_healthy is False
        assert backend.health_status().status == "DEGRADED"
        with pytest.raises(BackendStateStaleError):
            backend.get_joint_state()

        backend.disconnect()
    finally:
        pub_node.destroy_node()
        pub_ctx.try_shutdown()


def test_ros2_lifecycle_and_reconnect():
    """Verify backend connects, disconnects cleanly, joins thread, and safely reconnects with new context."""
    config = ROS2JointStateBackendConfig(
        expected_joint_names=("j1",),
        joint_state_topic="/test_lifecycle_joint_states",
        node_name="test_backend_lifecycle",
    )
    backend = ROS2JointStateBackend(config)

    # First connect
    assert backend.connect() is True
    assert backend.is_connected() is True
    thread1 = backend._spin_thread
    assert thread1 is not None
    assert thread1.is_alive() is True

    # Disconnect
    backend.disconnect()
    assert backend.is_connected() is False
    assert thread1.is_alive() is False
    assert backend._spin_thread is None
    assert backend._node is None
    assert backend._context is None

    # Second connect
    assert backend.connect() is True
    assert backend.is_connected() is True
    thread2 = backend._spin_thread
    assert thread2 is not None
    assert thread2.is_alive() is True
    assert thread2 is not thread1

    backend.disconnect()
    assert thread2.is_alive() is False


def test_ros2_unrelated_context_isolation():
    """Verify disconnecting ROS2JointStateBackend does not disrupt an unrelated external ROS2 Context."""
    ext_ctx = Context()
    rclpy.init(context=ext_ctx)
    try:
        ext_node = rclpy.create_node("unrelated_external_node", context=ext_ctx)
        assert ext_ctx.ok() is True

        config = ROS2JointStateBackendConfig(
            expected_joint_names=("j1",),
            joint_state_topic="/test_isolation_joint_states",
            node_name="test_backend_isolation",
        )
        backend = ROS2JointStateBackend(config)
        assert backend.connect() is True
        assert backend.is_connected() is True

        # Disconnect backend
        backend.disconnect()
        assert backend.is_connected() is False

        # Verify external context & node are completely unharmed
        assert ext_ctx.ok() is True
        assert ext_node.handle is not None
    finally:
        ext_node.destroy_node()
        ext_ctx.try_shutdown()


def test_ros2_read_only_command_rejection_and_entity_audit():
    """Verify read-only backend rejects all command calls and creates zero robot command publishers/clients."""
    config = ROS2JointStateBackendConfig(
        expected_joint_names=("j1", "j2"),
        joint_state_topic="/test_readonly_joint_states",
        node_name="test_backend_readonly",
    )
    backend = ROS2JointStateBackend(config)
    assert backend.connect() is True

    with pytest.raises(ReadOnlyBackendError, match="read-only"):
        backend.command_joint_positions([0.0, 0.0])

    with pytest.raises(ReadOnlyBackendError, match="read-only"):
        backend.command_joint_velocities([0.0, 0.0])

    with pytest.raises(ReadOnlyBackendError, match="read-only"):
        backend.halt_motion()

    # Entity audit: check created publishers on the backend node
    node = backend._node
    assert node is not None
    # Node should have zero command publishers (only default ROS internal publishers like parameter events / rosout if any)
    for pub in node.publishers:
        topic_name = pub.topic_name
        assert "command" not in topic_name.lower()
        assert "trajectory" not in topic_name.lower()

    # Node should have zero action clients or service clients for robot command/control
    assert len(list(node.clients)) == 0

    backend.disconnect()
