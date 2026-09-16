"""Real ROS2 integration tests for ROS2SimulationBackend.

These tests require an active ROS2 installation (rclpy, sensor_msgs, std_msgs).
They will be skipped automatically in environments where ROS2 is not present (e.g. Windows CI).
"""

import math
import time
import pytest

# Skip the entire module if rclpy or messages are missing
rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

sensor_msgs = pytest.importorskip("sensor_msgs")
from sensor_msgs.msg import JointState as ROSJointState
from builtin_interfaces.msg import Time as ROSTime

std_msgs = pytest.importorskip("std_msgs")
from std_msgs.msg import Float64MultiArray

from robotics.backends.base import (
    BackendCommandDisabledError,
    BackendCommandUnavailableError,
    BackendHealthStatus,
    BackendStateStaleError,
    BackendStateUnavailableError,
    ReadOnlyBackendError,
    TimestampedJointState,
    UnsupportedBackendOperationError,
)
from robotics.backends.ros2_state_mapping import (
    ROS2JointStateBackendConfig,
    ROS2SimulationBackendConfig,
)
from robotics.backends.ros2_simulation_backend import ROS2SimulationBackend


def _create_state_publisher(topic_name: str, context: Context):
    """Helper creating a test publisher for JointState telemetry."""
    node = rclpy.create_node("test_sim_state_pub", context=context)
    pub = node.create_publisher(ROSJointState, topic_name, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return node, pub, executor


def _create_command_subscriber(topic_name: str, context: Context):
    """Helper creating a test subscriber receiving Float64MultiArray commands."""
    node = rclpy.create_node("test_sim_cmd_sub", context=context)
    received_msgs = []

    def _callback(msg: Float64MultiArray):
        received_msgs.append(list(msg.data))

    sub = node.create_subscription(Float64MultiArray, topic_name, _callback, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return node, sub, executor, received_msgs


def test_ros2_simulation_position_command_streaming():
    """Verify position mode backend connects, discovers subscriber, and publishes valid Float64MultiArray."""
    pub_ctx = Context()
    sub_ctx = Context()
    rclpy.init(context=pub_ctx)
    rclpy.init(context=sub_ctx)

    try:
        # Setup telemetry publisher
        _, state_pub, state_exec = _create_state_publisher("/sim_pos_test/joint_states", pub_ctx)
        # Setup command subscriber (controller mock)
        _, cmd_sub, cmd_exec, received_commands = _create_command_subscriber(
            "/sim_pos_test/position_commands", sub_ctx
        )

        state_cfg = ROS2JointStateBackendConfig(
            expected_joint_names=("j1", "j2", "j3"),
            joint_state_topic="/sim_pos_test/joint_states",
            state_timeout_s=1.0,
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="position",
            position_command_topic="/sim_pos_test/position_commands",
            require_subscriber_ready=True,
        )

        backend = ROS2SimulationBackend(sim_cfg)
        assert backend.connect() is True

        # Publish telemetry and poll until discovery and state reception succeed
        telemetry = ROSJointState()
        telemetry.name = ["j1", "j2", "j3"]
        telemetry.position = [0.1, 0.2, 0.3]

        t_start = time.monotonic()
        ready = False
        while time.monotonic() - t_start < 3.0:
            now_ns = time.time_ns()
            telemetry.header.stamp = ROSTime(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
            state_pub.publish(telemetry)
            state_exec.spin_once(timeout_sec=0.02)
            try:
                backend.get_joint_state()
                if backend.command_endpoint_ready():
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(0.02)

        assert ready is True, "Failed to establish telemetry and command endpoint readiness within deadline"

        # Enable simulation commands
        backend.enable_simulation_commands()
        assert backend.commands_enabled is True

        # Send position command
        assert backend.command_joint_positions([0.5, 0.6, 0.7]) is True

        # Spin subscriber executor
        t_spin = time.monotonic()
        while time.monotonic() - t_spin < 3.0 and len(received_commands) == 0:
            cmd_exec.spin_once(timeout_sec=0.02)
            time.sleep(0.01)

        assert len(received_commands) >= 1
        assert received_commands[-1] == pytest.approx([0.5, 0.6, 0.7], abs=1e-5)

        backend.disconnect()
    finally:
        if pub_ctx.ok():
            pub_ctx.shutdown()
        if sub_ctx.ok():
            sub_ctx.shutdown()


def test_ros2_simulation_velocity_command_and_zero_halt():
    """Verify velocity mode backend streams velocities and sends zero-vector on halt."""
    pub_ctx = Context()
    sub_ctx = Context()
    rclpy.init(context=pub_ctx)
    rclpy.init(context=sub_ctx)

    try:
        _, state_pub, state_exec = _create_state_publisher("/sim_vel_test/joint_states", pub_ctx)
        _, cmd_sub, cmd_exec, received_commands = _create_command_subscriber(
            "/sim_vel_test/velocity_commands", sub_ctx
        )

        state_cfg = ROS2JointStateBackendConfig(
            expected_joint_names=("j1", "j2"),
            joint_state_topic="/sim_vel_test/joint_states",
            state_timeout_s=1.0,
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="velocity",
            velocity_command_topic="/sim_vel_test/velocity_commands",
            require_subscriber_ready=True,
        )

        backend = ROS2SimulationBackend(sim_cfg)
        assert backend.connect() is True

        telemetry = ROSJointState()
        telemetry.name = ["j1", "j2"]
        telemetry.position = [0.0, 0.0]

        t_start = time.monotonic()
        ready = False
        while time.monotonic() - t_start < 3.0:
            now_ns = time.time_ns()
            telemetry.header.stamp = ROSTime(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
            state_pub.publish(telemetry)
            state_exec.spin_once(timeout_sec=0.02)
            try:
                backend.get_joint_state()
                if backend.command_endpoint_ready():
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(0.02)

        assert ready is True, "Failed to establish telemetry and command endpoint readiness within deadline"

        backend.enable_simulation_commands()

        # Send ordinary velocity command
        backend.command_joint_velocities([0.25, -0.15])

        # Halt motion (publishes [0.0, 0.0])
        backend.halt_motion()

        t_spin = time.monotonic()
        while time.monotonic() - t_spin < 3.0 and len(received_commands) < 2:
            cmd_exec.spin_once(timeout_sec=0.02)
            time.sleep(0.01)

        assert len(received_commands) >= 2
        assert received_commands[0] == pytest.approx([0.25, -0.15], abs=1e-5)
        assert received_commands[-1] == pytest.approx([0.0, 0.0], abs=1e-5)

        backend.disconnect()
    finally:
        if pub_ctx.ok():
            pub_ctx.shutdown()
        if sub_ctx.ok():
            sub_ctx.shutdown()
