"""Closed-loop E2E test for guarded position control against real ros2_control forward controller.

Validates that GuardedRobotBackend + ROS2SimulationBackend can command small safe
position steps to the real ros2_control simulation node and observe actual state feedback change.
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
def test_ros2_control_position_e2e_closed_loop():
    """Validates real forward position control closed loop against ros2_control node."""
    if not _HAS_ROS2 or not has_ros2_control_support():
        pytest.skip("ROS2 Humble and controller_manager_msgs required for ros2_control position E2E test")

    domain_id = 43
    rrbot_model = create_rrbot_model()

    harness = ROS2ControlHarness(
        robot_controller="forward_position_controller",
        domain_id=domain_id,
        launch_timeout_s=30.0,
    )

    try:
        harness.start()

        # Build backend configuration
        state_cfg = ROS2JointStateBackendConfig(
            expected_joint_names=RRBOT_CANONICAL_JOINTS,
            joint_state_topic="/joint_states",
            node_name="rrbot_pos_state_sub",
            state_timeout_s=2.0,
            domain_id=domain_id,
        )
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="position",
            position_command_topic="/forward_position_controller/commands",
            require_subscriber_ready=True,
            command_node_name="rrbot_pos_cmd_pub",
        )

        raw_backend = ROS2SimulationBackend(sim_cfg)
        safety_cfg = CommandSafetyConfig(
            state_timeout_s=2.0,
            max_position_step_by_joint=(0.3, 0.3),
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
            raise AssertionError(
                f"Backend failed to establish telemetry and command endpoint readiness: {last_err}\n"
                f"Controller manager logs:\n{logs}"
            )

        # Enable simulation commands and arm guard
        raw_backend.enable_simulation_commands()
        guarded.arm()
        assert guarded.guard_state == SafetyGuardState.ARMED

        # Capture initial state
        initial_state = guarded.get_joint_state()
        q_before = list(initial_state.positions)
        assert len(q_before) == 2

        # Small safe position target within RRBot joint limits
        delta_q = [0.10, -0.10]
        q_target = [q_before[0] + delta_q[0], q_before[1] + delta_q[1]]

        initial_dist = math.hypot(q_target[0] - q_before[0], q_target[1] - q_before[1])
        assert initial_dist > 0.05

        # Command position
        cmd_ok = guarded.command_joint_positions(q_target)
        assert cmd_ok is True, "command_joint_positions was rejected"

        # Observe state over bounded window and assert real closed-loop movement toward target
        t_wait = time.monotonic()
        moved = False
        error_reduced = False
        q_final = list(q_before)

        while time.monotonic() - t_wait < 8.0:
            current_st = guarded.get_joint_state()
            q_now = list(current_st.positions)
            q_final = q_now

            dist_from_before = math.hypot(q_now[0] - q_before[0], q_now[1] - q_before[1])
            dist_to_target = math.hypot(q_target[0] - q_now[0], q_target[1] - q_now[1])

            if dist_from_before > 0.01:
                moved = True
            if dist_to_target < initial_dist * 0.7:
                error_reduced = True
                break

            time.sleep(0.05)

        assert moved is True, f"Robot joints did not move from q_before {q_before}, current {q_final}"
        assert error_reduced is True, (
            f"Robot did not move toward target {q_target}. Initial dist: {initial_dist}, "
            f"final dist: {math.hypot(q_target[0] - q_final[0], q_target[1] - q_final[1])}"
        )

        # Validate software stop: position mode uses hold behavior
        stop_ok = guarded.request_software_stop()
        assert stop_ok is True
        assert guarded.guard_state == SafetyGuardState.DISARMED

        # Capture hold position and verify system remains approximately at held state
        time.sleep(0.2)
        q_held = list(guarded.get_joint_state().positions)
        time.sleep(0.3)
        q_after_hold = list(guarded.get_joint_state().positions)
        assert math.hypot(q_after_hold[0] - q_held[0], q_after_hold[1] - q_held[1]) < 0.05

        guarded.disconnect()

    finally:
        harness.stop()
