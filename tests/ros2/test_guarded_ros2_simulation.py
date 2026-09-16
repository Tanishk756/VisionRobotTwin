"""Real ROS 2 integration tests for GuardedRobotBackend wrapping ROS2SimulationBackend.

These tests verify fail-closed dual authorization, envelope gating, and watchdog halt
behavior in an active ROS 2 Humble runtime environment.
"""

import math
import time
import pytest

# Skip the entire module if rclpy or messages are missing
rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

sensor_msgs = pytest.importorskip("sensor_msgs")
from sensor_msgs.msg import JointState as ROSJointState
from builtin_interfaces.msg import Time as ROSTime

std_msgs = pytest.importorskip("std_msgs")
from std_msgs.msg import Float64MultiArray

from robotics.backends.base import BackendCommandDisabledError
from robotics.backends.ros2_simulation_backend import ROS2SimulationBackend
from robotics.backends.ros2_state_mapping import (
    ROS2JointStateBackendConfig,
    ROS2SimulationBackendConfig,
)
from robotics.robot_model import (
    JointMotionType,
    JointRole,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)
from robotics.safety import (
    CommandSafetyConfig,
    CommandSafetyFaultCode,
    CommandSafetyViolationError,
    GuardedRobotBackend,
    SafetyGuardState,
)


def _make_test_robot_model(dof: int = 3) -> ResolvedRobotModel:
    joints = tuple(
        ResolvedJointMetadata(
            model_index=i,
            canonical_index=i,
            name=f"joint_{i+1}",
            role=JointRole.ARM,
            motion_type=JointMotionType.REVOLUTE,
            lower_limit=-2.5,
            upper_limit=2.5,
            max_force=50.0,
            max_velocity=1.5,
            link_name=f"link_{i+1}",
        )
        for i in range(dof)
    )
    return ResolvedRobotModel(
        robot_id="test_sim_robot",
        display_name="TestSimRobot",
        all_joints=joints,
        arm_joints=joints,
        gripper_joints=(),
        ee_link_name=f"link_{dof}",
        home_joint_positions=tuple(0.0 for _ in range(dof)),
    )


def _create_state_publisher(topic_name: str, context: Context):
    node = rclpy.create_node("test_guarded_sim_state_pub", context=context)
    pub = node.create_publisher(ROSJointState, topic_name, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return node, pub, executor


def _create_command_subscriber(topic_name: str, context: Context):
    node = rclpy.create_node("test_guarded_sim_cmd_sub", context=context)
    received_msgs = []

    def _callback(msg: Float64MultiArray):
        received_msgs.append(list(msg.data))

    sub = node.create_subscription(Float64MultiArray, topic_name, _callback, 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return node, sub, executor, received_msgs


def test_guarded_ros2_simulation_position_mode_lifecycle():
    """Verify guarded position mode simulation backend with dual authorization and fault recovery."""
    pub_ctx = Context()
    sub_ctx = Context()
    rclpy.init(context=pub_ctx)
    rclpy.init(context=sub_ctx)

    try:
        _, state_pub, state_exec = _create_state_publisher("/guarded_sim/joint_states", pub_ctx)
        _, cmd_sub, cmd_exec, received_commands = _create_command_subscriber(
            "/guarded_sim/position_commands", sub_ctx
        )

        joint_names = ("joint_1", "joint_2", "joint_3")
        state_cfg = ROS2JointStateBackendConfig(
            expected_joint_names=joint_names,
            joint_state_topic="/guarded_sim/joint_states",
            state_timeout_s=1.0,
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="position",
            position_command_topic="/guarded_sim/position_commands",
            require_subscriber_ready=True,
        )

        raw_backend = ROS2SimulationBackend(sim_cfg)
        model = _make_test_robot_model(dof=3)
        safety_cfg = CommandSafetyConfig(
            state_timeout_s=1.0,
            max_position_step_by_joint=(0.3, 0.3, 0.3),
        )

        guarded = GuardedRobotBackend(
            underlying_backend=raw_backend,
            resolved_model=model,
            safety_config=safety_cfg,
        )

        assert guarded.connect() is True
        assert guarded.guard_state == SafetyGuardState.DISARMED

        # 1. Publish fresh telemetry and wait for endpoint discovery
        telemetry = ROSJointState()
        telemetry.name = list(joint_names)
        telemetry.position = [0.0, 0.0, 0.0]
        telemetry.velocity = [0.0, 0.0, 0.0]

        t_start = time.monotonic()
        ready = False
        while time.monotonic() - t_start < 3.0:
            now_ns = time.time_ns()
            telemetry.header.stamp = ROSTime(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
            state_pub.publish(telemetry)
            state_exec.spin_once(timeout_sec=0.02)
            try:
                guarded.get_joint_state()
                if raw_backend.command_endpoint_ready():
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(0.02)

        assert ready is True, "Failed to establish telemetry and command endpoint readiness within deadline"

        # 2. Command before B2 enablement fails
        with pytest.raises(BackendCommandDisabledError):
            guarded.command_joint_positions([0.1, 0.1, 0.1])

        # Enable B2 simulation commands and B3 arm
        raw_backend.enable_simulation_commands()
        guarded.arm()
        assert guarded.is_armed is True

        # 3. Safe command succeeds
        assert guarded.command_joint_positions([0.1, 0.1, 0.1]) is True

        t_spin = time.monotonic()
        while time.monotonic() - t_spin < 3.0 and len(received_commands) == 0:
            cmd_exec.spin_once(timeout_sec=0.02)
            time.sleep(0.01)

        assert len(received_commands) >= 1
        assert received_commands[-1] == pytest.approx([0.1, 0.1, 0.1])

        # 4. Position step jump violation (from 0.0 measured to 0.5 > 0.3)
        with pytest.raises(CommandSafetyViolationError, match="(?i)position step jump"):
            guarded.command_joint_positions([0.5, 0.1, 0.1])

        assert guarded.guard_state == SafetyGuardState.FAULT_LATCHED
        assert guarded.active_fault.code == CommandSafetyFaultCode.POSITION_STEP_VIOLATION

        # 5. Fault recovery: publish fresh telemetry and verify readiness
        t_rec = time.monotonic()
        while time.monotonic() - t_rec < 1.0:
            now_ns = time.time_ns()
            telemetry.header.stamp = ROSTime(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
            state_pub.publish(telemetry)
            state_exec.spin_once(timeout_sec=0.02)
            time.sleep(0.02)

        guarded.reset_fault()
        assert guarded.guard_state == SafetyGuardState.DISARMED
        guarded.arm()
        assert guarded.is_armed is True

        # 6. Disarm
        guarded.disarm()
        assert guarded.guard_state == SafetyGuardState.DISARMED

        guarded.disconnect()
        assert guarded.is_connected() is False

    finally:
        if pub_ctx.ok():
            pub_ctx.shutdown()
        if sub_ctx.ok():
            sub_ctx.shutdown()


def test_guarded_ros2_simulation_velocity_mode_and_watchdog_halt():
    """Verify guarded velocity mode simulation backend with watchdog halt dispatch."""
    pub_ctx = Context()
    sub_ctx = Context()
    rclpy.init(context=pub_ctx)
    rclpy.init(context=sub_ctx)

    try:
        _, state_pub, state_exec = _create_state_publisher("/guarded_sim_vel/joint_states", pub_ctx)
        _, cmd_sub, cmd_exec, received_commands = _create_command_subscriber(
            "/guarded_sim_vel/velocity_commands", sub_ctx
        )

        joint_names = ("joint_1", "joint_2", "joint_3")
        state_cfg = ROS2JointStateBackendConfig(
            expected_joint_names=joint_names,
            joint_state_topic="/guarded_sim_vel/joint_states",
            state_timeout_s=1.0,
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="velocity",
            velocity_command_topic="/guarded_sim_vel/velocity_commands",
            require_subscriber_ready=True,
        )

        raw_backend = ROS2SimulationBackend(sim_cfg)
        model = _make_test_robot_model(dof=3)
        safety_cfg = CommandSafetyConfig(
            state_timeout_s=1.0,
            command_watchdog_timeout_s=0.1,  # 100ms watchdog
        )

        guarded = GuardedRobotBackend(
            underlying_backend=raw_backend,
            resolved_model=model,
            safety_config=safety_cfg,
        )

        assert guarded.connect() is True

        # Telemetry and discovery loop
        telemetry = ROSJointState()
        telemetry.name = list(joint_names)
        telemetry.position = [0.0, 0.0, 0.0]
        telemetry.velocity = [0.0, 0.0, 0.0]

        t_start = time.monotonic()
        ready = False
        while time.monotonic() - t_start < 3.0:
            now_ns = time.time_ns()
            telemetry.header.stamp = ROSTime(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
            state_pub.publish(telemetry)
            state_exec.spin_once(timeout_sec=0.02)
            try:
                guarded.get_joint_state()
                if raw_backend.command_endpoint_ready():
                    ready = True
                    break
            except Exception:
                pass
            time.sleep(0.02)

        assert ready is True, "Failed to establish telemetry and command endpoint readiness within deadline"

        raw_backend.enable_simulation_commands()
        guarded.arm()

        # Dispatch valid velocity
        assert guarded.command_joint_velocities([0.5, -0.5, 0.2]) is True

        t_spin = time.monotonic()
        while time.monotonic() - t_spin < 3.0 and len(received_commands) == 0:
            cmd_exec.spin_once(timeout_sec=0.02)
            time.sleep(0.01)

        assert len(received_commands) >= 1
        assert received_commands[-1] == pytest.approx([0.5, -0.5, 0.2])

        # Wait for watchdog timeout (> 100ms)
        time.sleep(0.15)
        assert guarded.check_watchdog() is True
        assert guarded.guard_state == SafetyGuardState.FAULT_LATCHED
        assert guarded.active_fault.code == CommandSafetyFaultCode.COMMAND_WATCHDOG_TIMEOUT

        # Check that zero-velocity halt was published by underlying halt_motion()
        t_spin2 = time.monotonic()
        while time.monotonic() - t_spin2 < 3.0 and (len(received_commands) < 2 or received_commands[-1] != [0.0, 0.0, 0.0]):
            cmd_exec.spin_once(timeout_sec=0.02)
            time.sleep(0.01)

        assert len(received_commands) >= 2
        assert received_commands[-1] == pytest.approx([0.0, 0.0, 0.0])

        guarded.disconnect()

    finally:
        if pub_ctx.ok():
            pub_ctx.shutdown()
        if sub_ctx.ok():
            sub_ctx.shutdown()
