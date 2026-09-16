"""Closed-loop E2E test for guarded velocity control and software stop against real ros2_control.

Validates that GuardedRobotBackend + ROS2SimulationBackend can stream safe velocity
commands to the real ros2_control simulation node, verify measured position progression,
and execute a software stop that halts robot motion.
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
    GuardedRobotBackend,
    SafetyGuardState,
)
from tests.fixtures.rrbot_fixture import create_rrbot_model, RRBOT_CANONICAL_JOINTS
from tests.ros2.ros2_control_harness import ROS2ControlHarness


@pytest.mark.ros2_control_e2e
def test_ros2_control_velocity_e2e_closed_loop_and_software_stop():
    """Validates real forward velocity control and software stop against ros2_control node."""
    if not _HAS_ROS2 or not has_ros2_control_support():
        pytest.skip("ROS2 Humble and controller_manager_msgs required for ros2_control velocity E2E test")

    domain_id = 44
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
            node_name="rrbot_vel_state_sub",
            state_timeout_s=2.0,
            domain_id=domain_id,
            qos_reliability="reliable",
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="velocity",
            velocity_command_topic="/forward_velocity_controller/commands",
            require_subscriber_ready=True,
            command_node_name="rrbot_vel_cmd_pub",
            command_qos="reliable",
        )

        raw_backend = ROS2SimulationBackend(sim_cfg)
        safety_cfg = CommandSafetyConfig(
            state_timeout_s=2.0,
            command_watchdog_timeout_s=2.0,
            velocity_limit_scale=1.0,
        )

        guarded = GuardedRobotBackend(
            underlying_backend=raw_backend,
            resolved_model=rrbot_model,
            safety_config=safety_cfg,
        )

        # Connect
        assert guarded.connect() is True

        # Wait for telemetry reception and command endpoint discovery
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
            diag = raw_backend.diagnostics()
            state_diag = raw_backend._state_backend.diagnostics()
            raise AssertionError(
                f"Backend failed to establish telemetry and command endpoint readiness: {last_err}\n"
                f"Simulation Diagnostics: {diag}\n"
                f"State Diagnostics: {state_diag}\n"
                f"Controller manager logs:\n{logs}"
            )

        # Enable simulation commands and arm guard
        raw_backend.enable_simulation_commands()
        guarded.arm()
        assert guarded.guard_state == SafetyGuardState.ARMED

        # Capture initial state
        initial_state = guarded.get_joint_state()
        q_before = list(initial_state.positions)

        v_cmd = [0.3, -0.3]

        # Stream velocity commands for 1.5s
        t_stream_start = time.monotonic()
        while time.monotonic() - t_stream_start < 1.5:
            ok = guarded.command_joint_velocities(v_cmd)
            assert ok is True
            time.sleep(0.05)

        # Capture state after streaming
        q_after_stream = list(guarded.get_joint_state().positions)

        # Assert positions moved in commanded directions (joint1 positive, joint2 negative)
        assert q_after_stream[0] > q_before[0] + 0.05, (
            f"Joint 1 did not advance positively: before={q_before[0]}, after={q_after_stream[0]}"
        )
        assert q_after_stream[1] < q_before[1] - 0.05, (
            f"Joint 2 did not advance negatively: before={q_before[1]}, after={q_after_stream[1]}"
        )

        # Execute software stop
        stop_ok = guarded.request_software_stop()
        assert stop_ok is True
        assert guarded.guard_state == SafetyGuardState.DISARMED

        # Settle briefly and collect samples to verify motion stopped / rate substantially dropped
        time.sleep(0.2)
        q_stop1 = list(guarded.get_joint_state().positions)
        t_sample1 = time.monotonic()
        time.sleep(0.5)
        q_stop2 = list(guarded.get_joint_state().positions)
        t_sample2 = time.monotonic()

        dt = t_sample2 - t_sample1
        measured_vel = [
            abs(q_stop2[0] - q_stop1[0]) / dt,
            abs(q_stop2[1] - q_stop1[1]) / dt,
        ]

        # In simulation, velocity should be near zero (substantially less than commanded 0.3 rad/s)
        assert measured_vel[0] < 0.05, f"Joint 1 still moving at {measured_vel[0]:.4f} rad/s after software stop"
        assert measured_vel[1] < 0.05, f"Joint 2 still moving at {measured_vel[1]:.4f} rad/s after software stop"

        guarded.disconnect()

    finally:
        harness.stop()
