"""Tests for Generic Robot Controller across supported manipulators."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController, PandaRobotController
from config.settings import RobotConfig


@pytest.fixture
def pybullet_direct():
    """Initializes a direct PyBullet physics simulation fixture."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)
    yield client_id
    p.disconnect(physicsClientId=client_id)


def test_generic_controller_panda_instantiation(pybullet_direct):
    """Verifies GenericRobotController loads Panda and discovers 7 arm joints + gripper."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=True,
        physicsClientId=client_id,
    )
    
    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
    )
    
    assert len(controller.arm_joint_indices) == 7
    assert len(controller.finger_joint_indices) == 2
    assert controller.capabilities.has_gripper is True
    
    # Test Home reset and FK
    controller.reset_to_home()
    q_curr = controller.get_current_joint_positions()
    assert len(q_curr) == 7
    np.testing.assert_allclose(q_curr, spec.home_joint_positions, atol=0.01)
    
    pos, orn = controller.get_end_effector_pose()
    assert len(pos) == 3 and np.all(np.isfinite(pos))
    assert len(orn) == 4 and np.all(np.isfinite(orn))


def test_generic_controller_kuka_instantiation(pybullet_direct):
    """Verifies GenericRobotController loads KUKA iiwa and discovers 7 arm joints (no gripper)."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("kuka_iiwa")
    
    body_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=True,
        physicsClientId=client_id,
    )
    
    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
    )
    
    assert len(controller.arm_joint_indices) == 7
    assert len(controller.finger_joint_indices) == 0
    assert controller.capabilities.has_gripper is False
    
    # Test Home reset and FK
    controller.reset_to_home()
    q_curr = controller.get_current_joint_positions()
    assert len(q_curr) == 7
    np.testing.assert_allclose(q_curr, spec.home_joint_positions, atol=0.01)
    
    pos, orn = controller.get_end_effector_pose()
    assert len(pos) == 3 and np.all(np.isfinite(pos))
    assert len(orn) == 4 and np.all(np.isfinite(orn))


def test_panda_robot_controller_backward_compatibility(pybullet_direct):
    """Ensures legacy PandaRobotController alias behaves identically for backward compatibility."""
    client_id = pybullet_direct
    robot_config = RobotConfig()
    
    body_id = p.loadURDF(
        robot_config.urdf_path,
        robot_config.base_position,
        robot_config.base_orientation,
        useFixedBase=True,
        physicsClientId=client_id,
    )
    
    legacy_controller = PandaRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        config=robot_config,
    )
    
    assert len(legacy_controller.arm_joint_indices) == 7
    legacy_controller.reset_to_home()
    pos, orn = legacy_controller.get_end_effector_pose()
    assert len(pos) == 3
