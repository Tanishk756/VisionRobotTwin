"""Unit tests for the Robot Model Registry and Capability Architecture."""

import pytest
from robotics.robot_model import RobotCapabilities, RobotModelSpec
from robotics.robot_registry import RobotRegistry, get_robot_registry, list_available_robots


def test_robot_registry_default_registration():
    """Verifies default registered robots include Panda and KUKA iiwa."""
    registry = get_robot_registry()
    robots = registry.list_robot_ids()
    assert "panda" in robots
    assert "kuka_iiwa" in robots
    assert list_available_robots() == ["panda", "kuka_iiwa"]


def test_robot_registry_panda_spec():
    """Verifies Franka Panda robot metadata and capabilities."""
    registry = get_robot_registry()
    panda_spec = registry.get_robot_spec("panda")
    assert panda_spec.robot_id == "panda"
    assert "Franka" in panda_spec.display_name or "Panda" in panda_spec.display_name
    assert panda_spec.capabilities.has_gripper is True
    assert panda_spec.capabilities.supports_pick_place is True
    assert len(panda_spec.home_joint_positions) == 7
    assert panda_spec.end_effector_link_name == "panda_grasptarget" or "hand" in panda_spec.end_effector_link_name


def test_robot_registry_kuka_spec():
    """Verifies KUKA LBR iiwa robot metadata and capabilities (no gripper)."""
    registry = get_robot_registry()
    kuka_spec = registry.get_robot_spec("kuka_iiwa")
    assert kuka_spec.robot_id == "kuka_iiwa"
    assert "KUKA" in kuka_spec.display_name or "iiwa" in kuka_spec.display_name
    assert kuka_spec.capabilities.has_gripper is False
    assert kuka_spec.capabilities.supports_pick_place is False
    assert len(kuka_spec.home_joint_positions) == 7


def test_robot_registry_unknown_robot_raises():
    """Verifies requesting an unregistered robot raises ValueError with helpful message."""
    registry = get_robot_registry()
    with pytest.raises(ValueError, match="Unsupported robot 'unknown_robot'"):
        registry.get_robot_spec("unknown_robot")
