"""Read-only ROS2 control and controller-manager readiness probe.

This module inspects ros2_control configuration and lifecycle state using strictly
read-only controller_manager query services (ListControllers, ListHardwareComponents,
ListHardwareInterfaces). It validates that expected controllers and hardware components
are active and have claimed the required command and state interfaces.

CRITICAL SAFETY MANDATE:
This probe is strictly READ-ONLY. It MUST NOT invoke mutating lifecycle or controller
management services (SwitchController, LoadController, ConfigureController,
SetHardwareComponentState, etc.).
"""

from dataclasses import dataclass
import time
from typing import Any, List, Optional, Sequence, Tuple

from robotics.backends.base import OptionalDependencyError, RobotBackend
from robotics.robot_model import ResolvedRobotModel
from robotics.safety import CommandReadinessProbe

# Guarded optional ROS2 imports
_HAS_ROS2_CONTROL_MSGS: bool = False
_ROS2_IMPORT_ERROR: Optional[Exception] = None

rclpy = None
Node = None
ListControllers = None
ListHardwareComponents = None
ListHardwareInterfaces = None

try:
    import rclpy
    from rclpy.node import Node
    from controller_manager_msgs.srv import (
        ListControllers,
        ListHardwareComponents,
        ListHardwareInterfaces,
    )
    _HAS_ROS2_CONTROL_MSGS = True
except Exception as err:
    _ROS2_IMPORT_ERROR = err


def has_ros2_control_support() -> bool:
    """Returns True if rclpy and controller_manager_msgs are available."""
    return _HAS_ROS2_CONTROL_MSGS


@dataclass(frozen=True)
class ROS2ControlExpectedState:
    """Expected configuration and lifecycle state for ros2_control preflight inspection."""
    controller_manager_name: str = "controller_manager"
    expected_controller_name: Optional[str] = None
    expected_controller_type: Optional[str] = None
    expected_hardware_component: Optional[str] = None
    expected_joint_names: Optional[Tuple[str, ...]] = None
    required_command_interfaces: Tuple[str, ...] = ()
    required_state_interfaces: Tuple[str, ...] = ()
    service_timeout_s: float = 2.0


def evaluate_controller_list_response(
    controllers: Sequence[Any],
    expected: ROS2ControlExpectedState,
) -> Tuple[bool, Tuple[str, ...]]:
    """Evaluates ListControllers.Response against expected controller state."""
    reasons: List[str] = []

    if expected.expected_controller_name is None:
        return True, ()

    matched_controller = None
    for c in controllers:
        if c.name == expected.expected_controller_name:
            matched_controller = c
            break

    if matched_controller is None:
        reasons.append(f"Expected controller '{expected.expected_controller_name}' not found in controller_manager.")
        return False, tuple(reasons)

    # 1. State check
    if getattr(matched_controller, "state", "").lower() != "active":
        reasons.append(
            f"Controller '{expected.expected_controller_name}' is '{getattr(matched_controller, 'state', '')}', expected 'active'."
        )

    # 2. Type check
    if expected.expected_controller_type is not None:
        actual_type = getattr(matched_controller, "type", "")
        if actual_type != expected.expected_controller_type:
            reasons.append(
                f"Controller '{expected.expected_controller_name}' type '{actual_type}' != expected '{expected.expected_controller_type}'."
            )

    # 3. Claimed command interfaces check
    if expected.required_command_interfaces:
        claimed = set(getattr(matched_controller, "claimed_interfaces", []) or [])
        missing_claims = [iface for iface in expected.required_command_interfaces if iface not in claimed]
        if missing_claims:
            reasons.append(
                f"Controller '{expected.expected_controller_name}' has not claimed required command interfaces: {missing_claims}."
            )

    # 4. State interfaces check
    if expected.required_state_interfaces:
        required_state = set(getattr(matched_controller, "required_state_interfaces", []) or [])
        missing_state = [iface for iface in expected.required_state_interfaces if iface not in required_state]
        if missing_state:
            reasons.append(
                f"Controller '{expected.expected_controller_name}' is missing required state interfaces: {missing_state}."
            )

    return (len(reasons) == 0), tuple(reasons)


def evaluate_hardware_components_response(
    components: Sequence[Any],
    expected: ROS2ControlExpectedState,
) -> Tuple[bool, Tuple[str, ...]]:
    """Evaluates ListHardwareComponents.Response against expected hardware component state."""
    reasons: List[str] = []

    if expected.expected_hardware_component is None:
        return True, ()

    matched_component = None
    for comp in components:
        if comp.name == expected.expected_hardware_component:
            matched_component = comp
            break

    if matched_component is None:
        reasons.append(f"Expected hardware component '{expected.expected_hardware_component}' not found.")
        return False, tuple(reasons)

    # Check lifecycle state (label in Humble HardwareComponentState)
    state_obj = getattr(matched_component, "state", None)
    state_label = getattr(state_obj, "label", "") if state_obj is not None else str(state_obj)
    if state_label.lower() != "active":
        reasons.append(
            f"Hardware component '{expected.expected_hardware_component}' lifecycle state is '{state_label}', expected 'active'."
        )

    return (len(reasons) == 0), tuple(reasons)


def evaluate_hardware_interfaces_response(
    command_interfaces: Sequence[Any],
    state_interfaces: Sequence[Any],
    expected: ROS2ControlExpectedState,
) -> Tuple[bool, Tuple[str, ...]]:
    """Evaluates ListHardwareInterfaces.Response against required command and state interfaces."""
    reasons: List[str] = []

    avail_cmd = {getattr(iface, "name", str(iface)) for iface in command_interfaces}
    avail_state = {getattr(iface, "name", str(iface)) for iface in state_interfaces}

    for req_cmd in expected.required_command_interfaces:
        if req_cmd not in avail_cmd:
            reasons.append(f"Required command interface '{req_cmd}' is not registered in hardware interfaces.")

    for req_state in expected.required_state_interfaces:
        if req_state not in avail_state:
            reasons.append(f"Required state interface '{req_state}' is not registered in hardware interfaces.")

    return (len(reasons) == 0), tuple(reasons)


def _spin_future_complete(node: Any, future: Any, timeout_sec: float) -> None:
    """Spins a future to completion safely supporting custom contexts and executors."""
    executor = getattr(node, "executor", None)
    if executor is not None:
        executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
    else:
        from rclpy.executors import SingleThreadedExecutor
        temp_executor = SingleThreadedExecutor(context=node.context)
        temp_executor.add_node(node)
        try:
            temp_executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        finally:
            temp_executor.remove_node(node)
            temp_executor.shutdown()


class ROS2ControlReadinessProbe(CommandReadinessProbe):
    """Inspects ros2_control controller_manager read-only services during software preflight checks."""

    def __init__(
        self,
        expected_state: ROS2ControlExpectedState,
        node: Optional[Any] = None,
    ) -> None:
        if not _HAS_ROS2_CONTROL_MSGS and node is None:
            raise OptionalDependencyError(
                "ROS2ControlReadinessProbe requires ROS 2 Humble packages (rclpy and controller_manager_msgs). "
                f"Import error: {_ROS2_IMPORT_ERROR}"
            )
        self._expected = expected_state
        self._node = node

    @property
    def expected_state(self) -> ROS2ControlExpectedState:
        return self._expected

    def check(self, backend: RobotBackend, model: ResolvedRobotModel) -> Tuple[bool, Tuple[str, ...]]:
        """Queries read-only controller_manager services and verifies controller and hardware readiness."""
        if not _HAS_ROS2_CONTROL_MSGS and self._node is None:
            return False, (f"ROS 2 environment unavailable for controller_manager probe: {_ROS2_IMPORT_ERROR}",)

        if self._node is None:
            return False, ("ROS 2 node instance is not configured on readiness probe.",)

        reasons: List[str] = []
        cm_prefix = f"/{self._expected.controller_manager_name}" if not self._expected.controller_manager_name.startswith("/") else self._expected.controller_manager_name

        # 1. Query ListControllers
        srv_list_controllers = f"{cm_prefix}/list_controllers"
        client_lc = self._node.create_client(ListControllers, srv_list_controllers)
        try:
            if not client_lc.wait_for_service(timeout_sec=self._expected.service_timeout_s):
                reasons.append(f"Service '{srv_list_controllers}' unavailable within {self._expected.service_timeout_s}s.")
            else:
                req = ListControllers.Request()
                future = client_lc.call_async(req)
                _spin_future_complete(self._node, future, timeout_sec=self._expected.service_timeout_s)
                if not future.done():
                    reasons.append(f"Service '{srv_list_controllers}' call timed out.")
                elif future.exception() is not None:
                    reasons.append(f"Service '{srv_list_controllers}' call failed: {future.exception()}")
                else:
                    resp = future.result()
                    ok, r = evaluate_controller_list_response(resp.controller, self._expected)
                    if not ok:
                        reasons.extend(r)
        finally:
            self._node.destroy_client(client_lc)

        # 2. Query ListHardwareComponents if expected
        if self._expected.expected_hardware_component is not None:
            srv_list_hw = f"{cm_prefix}/list_hardware_components"
            client_hw = self._node.create_client(ListHardwareComponents, srv_list_hw)
            try:
                if not client_hw.wait_for_service(timeout_sec=self._expected.service_timeout_s):
                    reasons.append(f"Service '{srv_list_hw}' unavailable within {self._expected.service_timeout_s}s.")
                else:
                    req = ListHardwareComponents.Request()
                    future = client_hw.call_async(req)
                    _spin_future_complete(self._node, future, timeout_sec=self._expected.service_timeout_s)
                    if not future.done():
                        reasons.append(f"Service '{srv_list_hw}' call timed out.")
                    elif future.exception() is not None:
                        reasons.append(f"Service '{srv_list_hw}' call failed: {future.exception()}")
                    else:
                        resp = future.result()
                        ok, r = evaluate_hardware_components_response(resp.component, self._expected)
                        if not ok:
                            reasons.extend(r)
            finally:
                self._node.destroy_client(client_hw)

        # 3. Query ListHardwareInterfaces if interfaces expected
        if self._expected.required_command_interfaces or self._expected.required_state_interfaces:
            srv_list_ifaces = f"{cm_prefix}/list_hardware_interfaces"
            client_ifaces = self._node.create_client(ListHardwareInterfaces, srv_list_ifaces)
            try:
                if not client_ifaces.wait_for_service(timeout_sec=self._expected.service_timeout_s):
                    reasons.append(f"Service '{srv_list_ifaces}' unavailable within {self._expected.service_timeout_s}s.")
                else:
                    req = ListHardwareInterfaces.Request()
                    future = client_ifaces.call_async(req)
                    _spin_future_complete(self._node, future, timeout_sec=self._expected.service_timeout_s)
                    if not future.done():
                        reasons.append(f"Service '{srv_list_ifaces}' call timed out.")
                    elif future.exception() is not None:
                        reasons.append(f"Service '{srv_list_ifaces}' call failed: {future.exception()}")
                    else:
                        resp = future.result()
                        ok, r = evaluate_hardware_interfaces_response(
                            resp.command_interfaces,
                            resp.state_interfaces,
                            self._expected,
                        )
                        if not ok:
                            reasons.extend(r)
            finally:
                self._node.destroy_client(client_ifaces)

        return (len(reasons) == 0), tuple(reasons)
