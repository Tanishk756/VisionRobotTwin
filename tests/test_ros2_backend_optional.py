"""Tests for ROS2JointStateBackend optional dependency handling, lifecycle, and read-only safety."""

import time
import pytest
from unittest.mock import MagicMock, patch

from robotics.backends.base import (
    BackendHealthStatus,
    BackendStateFieldUnavailableError,
    BackendStateStaleError,
    BackendStateUnavailableError,
    OptionalDependencyError,
    ReadOnlyBackendError,
    TimestampedJointState,
)
from robotics.backends.ros2_state_mapping import (
    ROS2JointStateBackendConfig,
    ROS2JointStateDiagnostics,
)


def test_optional_dependency_error_when_ros_absent():
    """Verify that attempting to instantiate ROS2JointStateBackend raises OptionalDependencyError when rclpy is missing."""
    import robotics.backends.ros2_joint_state_backend as ros2_module

    cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))

    with patch.object(ros2_module, "_HAS_RCLPY", False):
        with pytest.raises(OptionalDependencyError, match="ROS2 dependencies"):
            ros2_module.ROS2JointStateBackend(config=cfg)


def test_read_only_command_fail_closed():
    """Verify that all command methods fail closed by raising ReadOnlyBackendError."""
    import robotics.backends.ros2_joint_state_backend as ros2_module

    cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))

    with patch.object(ros2_module, "_HAS_RCLPY", True):
        backend = ros2_module.ROS2JointStateBackend(config=cfg)

        with pytest.raises(ReadOnlyBackendError, match="read-only"):
            backend.command_joint_positions([0.1, 0.2])

        with pytest.raises(ReadOnlyBackendError, match="read-only"):
            backend.command_joint_velocities([0.01, 0.02])

        with pytest.raises(ReadOnlyBackendError, match="read-only"):
            backend.halt_motion()


def test_state_availability_and_staleness_fail_closed():
    """Verify get_joint_state raises BackendStateUnavailableError before first message and BackendStateStaleError when stale."""
    import robotics.backends.ros2_joint_state_backend as ros2_module

    cfg = ROS2JointStateBackendConfig(
        expected_joint_names=("j1", "j2"),
        state_timeout_s=0.2,
    )

    with patch.object(ros2_module, "_HAS_RCLPY", True):
        backend = ros2_module.ROS2JointStateBackend(config=cfg)

        # Before connect
        assert not backend.is_connected()
        assert backend.health_status() == BackendHealthStatus.DISCONNECTED
        with pytest.raises(BackendStateUnavailableError, match="not connected"):
            backend.get_joint_state()

        # Manually set connected state without receiving messages
        backend._is_connected = True
        assert backend.health_status() == BackendHealthStatus.DEGRADED
        with pytest.raises(BackendStateUnavailableError, match="No joint state telemetry received yet"):
            backend.get_joint_state()

        # Inject fresh mock state
        now = time.monotonic()
        fresh_state = TimestampedJointState(
            source_timestamp_s=100.0,
            receive_timestamp_s=now,
            joint_names=("j1", "j2"),
            positions=(0.5, -0.5),
            velocities=(0.0, 0.0),
        )
        backend._latest_state = fresh_state
        assert backend.health_status() == BackendHealthStatus.HEALTHY
        retrieved = backend.get_joint_state()
        assert retrieved == fresh_state

        # Make state stale
        stale_state = TimestampedJointState(
            source_timestamp_s=100.0,
            receive_timestamp_s=now - 1.0,  # 1s old > 0.2s timeout
            joint_names=("j1", "j2"),
            positions=(0.5, -0.5),
            velocities=(0.0, 0.0),
        )
        backend._latest_state = stale_state
        assert backend.health_status() == BackendHealthStatus.DEGRADED
        with pytest.raises(BackendStateStaleError, match="stale"):
            backend.get_joint_state()


def test_diagnostics_snapshot():
    """Verify diagnostics() returns accurate immutable telemetry metadata."""
    import robotics.backends.ros2_joint_state_backend as ros2_module

    cfg = ROS2JointStateBackendConfig(
        expected_joint_names=("j1", "j2"),
        joint_state_topic="/custom_joint_states",
        state_timeout_s=0.5,
    )

    with patch.object(ros2_module, "_HAS_RCLPY", True):
        backend = ros2_module.ROS2JointStateBackend(config=cfg)
        diag = backend.diagnostics()
        assert isinstance(diag, ROS2JointStateDiagnostics)
        assert diag.topic == "/custom_joint_states"
        assert diag.messages_received == 0
        assert diag.valid_messages == 0
        assert diag.invalid_messages == 0
        assert diag.is_stale is True


def test_callback_valid_and_invalid_message_handling():
    """Verify _on_joint_state_message increments counts, updates state on valid msg, and records error on invalid msg."""
    import robotics.backends.ros2_joint_state_backend as ros2_module

    cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))

    with patch.object(ros2_module, "_HAS_RCLPY", True):
        backend = ros2_module.ROS2JointStateBackend(config=cfg)

        class MockStamp:
            sec = 100
            nanosec = 500000000

        class MockHeader:
            stamp = MockStamp()

        class MockMsg:
            header = MockHeader()
            name = ["j2", "extra_finger", "j1"]
            position = [-0.5, 0.04, 0.5]
            velocity = [-0.05, 0.0, 0.05]
            effort = [1.0, 0.0, 2.0]

        # 1. Process valid message
        backend._on_joint_state_message(MockMsg())
        assert backend._messages_received == 1
        assert backend._valid_messages == 1
        assert backend._invalid_messages == 0
        assert backend._latest_state is not None
        assert backend._latest_state.joint_names == ("j1", "j2")
        assert backend._latest_state.positions == (0.5, -0.5)
        assert backend._latest_state.velocities == (0.05, -0.05)
        assert backend._latest_state.efforts == (2.0, 1.0)
        assert backend._latest_state.source_timestamp_s == 100.5

        # 2. Process malformed message (missing j2)
        class MalformedMsg:
            header = MockHeader()
            name = ["j1", "extra_finger"]
            position = [0.5, 0.04]
            velocity = []
            effort = []

        backend._on_joint_state_message(MalformedMsg())
        assert backend._messages_received == 2
        assert backend._valid_messages == 1
        assert backend._invalid_messages == 1
        assert backend._last_validation_error is not None
        # Existing valid state is preserved
        assert backend._latest_state.positions == (0.5, -0.5)


def test_mock_ros2_connect_disconnect_lifecycle():
    """Verify connect() allocates fresh Context and disconnect() cleanly shuts down without global calls."""
    import robotics.backends.ros2_joint_state_backend as ros2_module

    cfg = ROS2JointStateBackendConfig(
        expected_joint_names=("j1", "j2"),
        domain_id=42,
    )

    mock_context = MagicMock()
    mock_context.ok.return_value = True
    mock_node = MagicMock()
    mock_executor = MagicMock()
    mock_rclpy = MagicMock()
    mock_rclpy.context.Context.return_value = mock_context
    mock_rclpy.create_node.return_value = mock_node

    with patch.object(ros2_module, "_HAS_RCLPY", True), \
         patch.object(ros2_module, "Context", return_value=mock_context), \
         patch.object(ros2_module, "rclpy", mock_rclpy), \
         patch.object(ros2_module, "SingleThreadedExecutor", return_value=mock_executor), \
         patch("threading.Thread") as mock_thread_cls:

        mock_thread = MagicMock()
        mock_thread.is_alive.return_value = False
        mock_thread_cls.return_value = mock_thread

        backend = ros2_module.ROS2JointStateBackend(config=cfg)

        # Connect
        assert backend.connect() is True
        assert backend.is_connected()
        mock_rclpy.init.assert_called_once_with(context=mock_context, domain_id=42)
        mock_rclpy.create_node.assert_called_once()
        mock_executor.add_node.assert_called_once_with(mock_node)
        mock_thread.start.assert_called_once()

        # Connect idempotent
        assert backend.connect() is True

        # Disconnect
        backend.disconnect()
        assert not backend.is_connected()
        mock_executor.shutdown.assert_called_once()
        mock_node.destroy_node.assert_called_once()
        mock_context.shutdown.assert_called_once()
        # Verify global rclpy.shutdown was NEVER called
        assert mock_rclpy.shutdown.call_count == 0
