"""Unit tests for read-only ROS2 control readiness probe and pure evaluators."""

from types import SimpleNamespace
from unittest.mock import MagicMock
import pytest

from robotics.backends.base import OptionalDependencyError, RobotBackend
from robotics.robot_model import ResolvedRobotModel
from robotics.ros2_control_readiness import (
    ROS2ControlExpectedState,
    ROS2ControlReadinessProbe,
    evaluate_controller_list_response,
    evaluate_hardware_components_response,
    evaluate_hardware_interfaces_response,
    has_ros2_control_support,
)


def test_optional_import_behavior():
    """Verify probe construction raises OptionalDependencyError when ROS 2 is not installed and no node provided."""
    expected = ROS2ControlExpectedState(expected_controller_name="joint_trajectory_controller")
    if not has_ros2_control_support():
        with pytest.raises(OptionalDependencyError, match="(?i)requires ROS 2"):
            ROS2ControlReadinessProbe(expected_state=expected)
    else:
        # If ROS 2 is present, instantiation succeeds
        probe = ROS2ControlReadinessProbe(expected_state=expected, node=MagicMock())
        assert probe.expected_state.expected_controller_name == "joint_trajectory_controller"


def test_evaluate_controller_list_success():
    """Verify evaluation passes when controller is active, type matches, and interfaces are claimed."""
    expected = ROS2ControlExpectedState(
        expected_controller_name="forward_position_controller",
        expected_controller_type="position_controllers/JointGroupPositionController",
        required_command_interfaces=("j1/position", "j2/position"),
    )
    ctrl = SimpleNamespace(
        name="forward_position_controller",
        state="active",
        type="position_controllers/JointGroupPositionController",
        claimed_interfaces=["j1/position", "j2/position", "j3/position"],
        required_state_interfaces=[],
    )
    ok, reasons = evaluate_controller_list_response([ctrl], expected)
    assert ok is True
    assert len(reasons) == 0


def test_evaluate_controller_list_inactive():
    """Verify evaluation fails when controller is inactive."""
    expected = ROS2ControlExpectedState(expected_controller_name="forward_position_controller")
    ctrl = SimpleNamespace(
        name="forward_position_controller",
        state="inactive",
        type="position_controllers/JointGroupPositionController",
        claimed_interfaces=[],
    )
    ok, reasons = evaluate_controller_list_response([ctrl], expected)
    assert ok is False
    assert any("inactive" in r for r in reasons)


def test_evaluate_controller_list_wrong_type():
    """Verify evaluation fails when controller type does not match expected."""
    expected = ROS2ControlExpectedState(
        expected_controller_name="forward_position_controller",
        expected_controller_type="position_controllers/JointGroupPositionController",
    )
    ctrl = SimpleNamespace(
        name="forward_position_controller",
        state="active",
        type="velocity_controllers/JointGroupVelocityController",
        claimed_interfaces=[],
    )
    ok, reasons = evaluate_controller_list_response([ctrl], expected)
    assert ok is False
    assert any("velocity_controllers" in r for r in reasons)


def test_evaluate_controller_list_missing_command_interface():
    """Verify evaluation fails when required command interfaces are not claimed."""
    expected = ROS2ControlExpectedState(
        expected_controller_name="forward_position_controller",
        required_command_interfaces=("j1/position", "j2/position"),
    )
    ctrl = SimpleNamespace(
        name="forward_position_controller",
        state="active",
        type="position_controllers/JointGroupPositionController",
        claimed_interfaces=["j1/position"],  # missing j2/position
    )
    ok, reasons = evaluate_controller_list_response([ctrl], expected)
    assert ok is False
    assert any("j2/position" in r for r in reasons)


def test_evaluate_hardware_components_success_and_failures():
    """Verify hardware component lifecycle evaluation."""
    expected = ROS2ControlExpectedState(expected_hardware_component="MockRobotHardware")

    # 1. Success
    comp_ok = SimpleNamespace(name="MockRobotHardware", state=SimpleNamespace(label="active", id=3))
    ok, reasons = evaluate_hardware_components_response([comp_ok], expected)
    assert ok is True

    # 2. Inactive component
    comp_unconfigured = SimpleNamespace(name="MockRobotHardware", state=SimpleNamespace(label="unconfigured", id=1))
    ok, reasons = evaluate_hardware_components_response([comp_unconfigured], expected)
    assert ok is False
    assert any("unconfigured" in r for r in reasons)

    # 3. Missing component
    ok, reasons = evaluate_hardware_components_response([], expected)
    assert ok is False
    assert any("not found" in r for r in reasons)


def test_evaluate_hardware_interfaces():
    """Verify hardware interfaces check."""
    expected = ROS2ControlExpectedState(
        required_command_interfaces=("j1/position", "j2/position"),
        required_state_interfaces=("j1/position", "j2/position", "j1/velocity"),
    )

    cmd_ifaces = [SimpleNamespace(name="j1/position"), SimpleNamespace(name="j2/position")]
    state_ifaces = [
        SimpleNamespace(name="j1/position"),
        SimpleNamespace(name="j2/position"),
        SimpleNamespace(name="j1/velocity"),
    ]

    ok, reasons = evaluate_hardware_interfaces_response(cmd_ifaces, state_ifaces, expected)
    assert ok is True

    # Missing state interface
    ok, reasons = evaluate_hardware_interfaces_response(cmd_ifaces, state_ifaces[:2], expected)
    assert ok is False
    assert any("j1/velocity" in r for r in reasons)


def test_no_mutating_services_audit():
    """Audit runtime ros2_control_readiness code to ensure zero mutating service calls or imports exist."""
    import ast
    import inspect
    import robotics.ros2_control_readiness as module

    source = inspect.getsource(module)
    tree = ast.parse(source)

    mutating_services = {
        "SwitchController",
        "LoadController",
        "UnloadController",
        "ConfigureController",
        "SetHardwareComponentState",
        "ReloadControllerLibraries",
    }

    referenced_identifiers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            referenced_identifiers.add(node.id)
        elif isinstance(node, ast.Attribute):
            referenced_identifiers.add(node.attr)
        elif isinstance(node, ast.alias):
            referenced_identifiers.add(node.name)

    violations = mutating_services.intersection(referenced_identifiers)
    assert not violations, f"Found prohibited mutating service references in code: {violations}"

