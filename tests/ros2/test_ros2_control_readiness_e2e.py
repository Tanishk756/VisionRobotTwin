"""Live controller-manager readiness probe validation E2E against real ros2_control simulation.

Validates ROS2ControlReadinessProbe against live controller_manager services
without mutating any controller or hardware state.
"""

import os
import pytest
from unittest.mock import MagicMock

_HAS_ROS2: bool = False
try:
    import rclpy
    from rclpy.node import Node
    from controller_manager_msgs.srv import ListHardwareComponents
    _HAS_ROS2 = True
except ImportError:
    pass

from robotics.backends.base import RobotBackend
from robotics.ros2_control_readiness import (
    ROS2ControlExpectedState,
    ROS2ControlReadinessProbe,
    has_ros2_control_support,
)
from tests.fixtures.rrbot_fixture import create_rrbot_model
from tests.ros2.ros2_control_harness import ROS2ControlHarness


@pytest.mark.ros2_control_e2e
def test_live_ros2_control_readiness_probe():
    """Validates ROS2ControlReadinessProbe against a live ros2_control controller manager."""
    if not _HAS_ROS2 or not has_ros2_control_support():
        pytest.skip("ROS2 Humble and controller_manager_msgs required for live readiness probe test")

    domain_id = 42
    rrbot_model = create_rrbot_model()
    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True

    harness = ROS2ControlHarness(
        robot_controller="forward_position_controller",
        domain_id=domain_id,
        launch_timeout_s=30.0,
    )

    context = None
    node = None
    try:
        harness.start()

        from rclpy.executors import SingleThreadedExecutor

        # Initialize node on same domain to run probe queries
        context = rclpy.Context()
        rclpy.init(context=context)
        node = Node("test_readiness_probe_node", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)

        # Detect active hardware component name from live controller manager
        hw_client = node.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
        assert hw_client.wait_for_service(timeout_sec=5.0)
        req = ListHardwareComponents.Request()
        fut = hw_client.call_async(req)
        executor.spin_until_future_complete(fut, timeout_sec=5.0)
        assert fut.done() and fut.exception() is None
        hw_resp = fut.result()
        assert len(hw_resp.component) > 0
        actual_hw_name = hw_resp.component[0].name
        node.destroy_client(hw_client)

        actual_ctrl_type = harness.observed_controller_type
        assert actual_ctrl_type is not None

        # 1. Positive Verification Case
        expected_valid = ROS2ControlExpectedState(
            controller_manager_name="controller_manager",
            expected_controller_name="forward_position_controller",
            expected_controller_type=actual_ctrl_type,
            expected_hardware_component=actual_hw_name,
            required_command_interfaces=("joint1/position", "joint2/position"),
            required_state_interfaces=(),
            service_timeout_s=3.0,
        )

        probe_positive = ROS2ControlReadinessProbe(expected_state=expected_valid, node=node)
        ok, reasons = probe_positive.check(backend=mock_backend, model=rrbot_model)
        assert ok is True, f"Positive probe check failed unexpectedly with reasons: {reasons}"
        assert len(reasons) == 0

        # 2. Negative Case: Nonexistent Controller Name
        expected_wrong_name = ROS2ControlExpectedState(
            controller_manager_name="controller_manager",
            expected_controller_name="nonexistent_position_controller",
            service_timeout_s=3.0,
        )
        probe_wrong_name = ROS2ControlReadinessProbe(expected_state=expected_wrong_name, node=node)
        ok, reasons = probe_wrong_name.check(backend=mock_backend, model=rrbot_model)
        assert ok is False
        assert any("not found" in r.lower() for r in reasons)

        # 3. Negative Case: Wrong Controller Type
        expected_wrong_type = ROS2ControlExpectedState(
            controller_manager_name="controller_manager",
            expected_controller_name="forward_position_controller",
            expected_controller_type="invalid_type/FakeControllerType",
            service_timeout_s=3.0,
        )
        probe_wrong_type = ROS2ControlReadinessProbe(expected_state=expected_wrong_type, node=node)
        ok, reasons = probe_wrong_type.check(backend=mock_backend, model=rrbot_model)
        assert ok is False
        assert any("type" in r.lower() for r in reasons)

        # 4. Negative Case: Unclaimed / Missing Command Interface
        expected_wrong_cmd = ROS2ControlExpectedState(
            controller_manager_name="controller_manager",
            expected_controller_name="forward_position_controller",
            required_command_interfaces=("joint1/unclaimed_interface",),
            service_timeout_s=3.0,
        )
        probe_wrong_cmd = ROS2ControlReadinessProbe(expected_state=expected_wrong_cmd, node=node)
        ok, reasons = probe_wrong_cmd.check(backend=mock_backend, model=rrbot_model)
        assert ok is False
        assert any("command interface" in r.lower() for r in reasons)

        # 5. Negative Case: Nonexistent Hardware Component
        expected_wrong_hw = ROS2ControlExpectedState(
            controller_manager_name="controller_manager",
            expected_hardware_component="NonexistentHardwareComponent",
            service_timeout_s=3.0,
        )
        probe_wrong_hw = ROS2ControlReadinessProbe(expected_state=expected_wrong_hw, node=node)
        ok, reasons = probe_wrong_hw.check(backend=mock_backend, model=rrbot_model)
        assert ok is False
        assert any("hardware component" in r.lower() for r in reasons)

    finally:
        if node is not None:
            if "executor" in locals() and executor is not None:
                executor.remove_node(node)
            node.destroy_node()
        if "executor" in locals() and executor is not None:
            executor.shutdown()
        if context is not None and context.ok():
            rclpy.shutdown(context=context)
        harness.stop()
