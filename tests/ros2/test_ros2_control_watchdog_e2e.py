"""Live watchdog timeout and software stop E2E test against real ros2_control simulation.

Validates that when velocity command streaming is starved, GuardedRobotBackend's
watchdog triggers, latches a fault with COMMAND_WATCHDOG_TIMEOUT, dispatches
a zero-velocity halt, and brings the simulated robot joints to a halt.
"""

import math
import time
import pytest

_HAS_ROS2: bool = False
try:
    import rclpy
    _HAS_ROS2 = True
except ImportError:
    pass

from robotics.backends.ros2_joint_state_backend import ROS2JointStateBackend
from robotics.backends.ros2_simulation_backend import ROS2SimulationBackend
from robotics.backends.ros2_state_mapping import (
    ROS2JointStateBackendConfig,
    ROS2SimulationBackendConfig,
)
from robotics.ros2_control_readiness import has_ros2_control_support
from robotics.safety import (
    CommandSafetyConfig,
    CommandSafetyFaultCode,
    CommandSafetyViolationError,
    GuardedRobotBackend,
    SafetyGuardState,
)
from tests.fixtures.rrbot_fixture import create_rrbot_model, RRBOT_CANONICAL_JOINTS
from tests.ros2.ros2_control_harness import ROS2ControlHarness


@pytest.mark.ros2_control_e2e
def test_ros2_control_watchdog_timeout_e2e():
    """Validates live watchdog timeout fault latching and software halt on real forward controller."""
    if not _HAS_ROS2 or not has_ros2_control_support():
        pytest.skip("ROS2 Humble and controller_manager_msgs required for ros2_control watchdog E2E test")

    domain_id = 45
    rrbot_model = create_rrbot_model()

    harness = ROS2ControlHarness(
        robot_controller="forward_velocity_controller",
        domain_id=domain_id,
        launch_timeout_s=30.0,
    )

    try:
        harness.start()

        # Build backend configuration
        state_cfg = ROS2JointStateBackendConfig(
            expected_joint_names=RRBOT_CANONICAL_JOINTS,
            joint_state_topic="/joint_states",
            node_name="rrbot_wd_state_sub",
            state_timeout_s=2.0,
            domain_id=domain_id,
            qos_reliability="reliable",
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="velocity",
            velocity_command_topic="/forward_velocity_controller/commands",
            require_subscriber_ready=True,
            command_node_name="rrbot_wd_cmd_pub",
            command_qos="reliable",
        )

        raw_backend = ROS2SimulationBackend(sim_cfg)
        # Test-specific 500ms watchdog for deterministic and bounded CI execution
        watchdog_timeout_s = 0.5
        safety_cfg = CommandSafetyConfig(
            state_timeout_s=2.0,
            command_watchdog_timeout_s=watchdog_timeout_s,
            velocity_limit_scale=1.0,
        )

        guarded = GuardedRobotBackend(
            underlying_backend=raw_backend,
            resolved_model=rrbot_model,
            safety_config=safety_cfg,
        )

        # Connect and establish readiness
        assert guarded.connect() is True
        t_start = time.monotonic()
        ready = False
        last_err = None
        while time.monotonic() - t_start < 15.0:
            try:
                st = guarded.get_joint_state()
                sub_count = raw_backend._cmd_publisher.get_subscription_count() if raw_backend._cmd_publisher else -1
                if sub_count >= 1:
                    ready = True
                    break
                else:
                    last_err = f"State received (positions={st.positions}), but command sub_count={sub_count}"
            except Exception as e:
                last_err = f"get_joint_state() exception: {type(e).__name__}: {e}"
            time.sleep(0.05)

        if not ready:
            logs = harness.get_logs()
            raise AssertionError(
                f"Backend failed to establish telemetry and command endpoint readiness: {last_err}\n"
                f"Controller manager logs:\n{logs}"
            )

        raw_backend.enable_simulation_commands()
        guarded.arm()
        assert guarded.guard_state == SafetyGuardState.ARMED

        # Stream valid velocities for 0.4s to establish motion
        v_cmd = [0.25, -0.25]
        q_start = list(guarded.get_joint_state().positions)
        t_stream = time.monotonic()
        while time.monotonic() - t_stream < 0.4:
            guarded.command_joint_velocities(v_cmd)
            time.sleep(0.05)

        # Starve commands: wait beyond configured watchdog timeout
        time.sleep(watchdog_timeout_s + 0.3)

        # Attempt to command new velocity - must fail closed with CommandSafetyViolationError
        with pytest.raises(CommandSafetyViolationError) as exc_info:
            guarded.command_joint_velocities(v_cmd)

        assert exc_info.value.fault.code == CommandSafetyFaultCode.COMMAND_WATCHDOG_TIMEOUT
        assert guarded.guard_state == SafetyGuardState.FAULT_LATCHED
        assert guarded.active_fault is not None
        assert guarded.active_fault.code == CommandSafetyFaultCode.COMMAND_WATCHDOG_TIMEOUT

        # Settle briefly and sample positions to verify motion has stopped
        time.sleep(0.2)
        q_stop1 = list(guarded.get_joint_state().positions)
        t_samp1 = time.monotonic()
        time.sleep(0.4)
        q_stop2 = list(guarded.get_joint_state().positions)
        t_samp2 = time.monotonic()

        dt = t_samp2 - t_samp1
        measured_vel = [
            abs(q_stop2[0] - q_stop1[0]) / dt,
            abs(q_stop2[1] - q_stop1[1]) / dt,
        ]

        assert measured_vel[0] < 0.05, f"Joint 1 still moving at {measured_vel[0]:.4f} rad/s after watchdog halt"
        assert measured_vel[1] < 0.05, f"Joint 2 still moving at {measured_vel[1]:.4f} rad/s after watchdog halt"

        guarded.disconnect()

    finally:
        harness.stop()
