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


def test_generic_controller_delegates_state_to_backend(pybullet_direct):
    """Verify GenericRobotController delegates get_current_joint_positions/velocities to backend."""
    from unittest.mock import MagicMock
    from robotics.backends.base import RobotBackend, TimestampedJointState

    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=True, physicsClientId=client_id)

    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True
    fake_state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=tuple(f"panda_joint{i+1}" for i in range(7)),
        positions=(0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7),
        velocities=(0.01, 0.02, 0.03, -0.04, 0.05, 0.06, 0.07),
    )
    mock_backend.get_joint_state.return_value = fake_state

    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
        backend=mock_backend,
    )

    positions = controller.get_current_joint_positions()
    assert positions == [0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7]
    mock_backend.get_joint_state.assert_called()

    velocities = controller.get_current_joint_velocities()
    assert velocities == [0.01, 0.02, 0.03, -0.04, 0.05, 0.06, 0.07]


def test_generic_controller_delegates_positions_with_rate_limiting_to_backend(pybullet_direct):
    """Verify position slew rate limiting is performed by controller before delegating to backend."""
    from unittest.mock import MagicMock
    from robotics.backends.base import RobotBackend, TimestampedJointState

    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=True, physicsClientId=client_id)

    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True
    initial_state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=tuple(f"panda_joint{i+1}" for i in range(7)),
        positions=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        velocities=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    mock_backend.get_joint_state.return_value = initial_state

    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
        backend=mock_backend,
    )

    # Large target jump with dt=0.01
    large_target = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    commanded = controller.set_arm_joint_positions(large_target, dt=0.01, enforce_velocity_limits=True)

    # Commanded values must be scaled down by rate limiting
    assert max(commanded) < 0.1
    mock_backend.command_joint_positions.assert_called_once_with(commanded)


def test_generic_controller_delegates_velocities_and_max_force_to_backend(pybullet_direct):
    """Verify velocity clamping is performed by controller and max_force forwarded to backend."""
    from unittest.mock import MagicMock
    from robotics.backends.base import RobotBackend

    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=True, physicsClientId=client_id)

    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True

    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
        backend=mock_backend,
    )

    # Commanded velocities exceeding limits
    excess_vels = [10.0, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0]
    clamped = controller.set_arm_joint_velocities(excess_vels, max_force=88.0)

    # Clamped to individual joint velocity limits
    for i, idx in enumerate(controller.arm_joint_indices):
        assert abs(clamped[i]) <= controller.joints[idx].max_velocity + 1e-6

    mock_backend.command_joint_velocities.assert_called_once_with(clamped, effort_limit=88.0)


def test_generic_controller_with_mock_backend(pybullet_direct):
    """Verify GenericRobotController operates seamlessly with concrete MockRobotBackend."""
    from robotics.backends.mock_backend import MockRobotBackend

    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=True, physicsClientId=client_id)

    mock_backend = MockRobotBackend(
        joint_names=[f"panda_joint{i+1}" for i in range(7)],
        initial_positions=spec.home_joint_positions,
    )

    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
        backend=mock_backend,
    )

    assert controller.backend is mock_backend
    assert controller.backend.is_connected() is True

    # State read
    q = controller.get_current_joint_positions()
    np.testing.assert_allclose(q, spec.home_joint_positions)

    # Velocity dispatch with max_force
    target_v = [0.1, -0.1, 0.05, -0.05, 0.02, -0.02, 0.01]
    controller.set_arm_joint_velocities(target_v, max_force=45.0)

    assert mock_backend.last_commanded_velocities == tuple(target_v)
    assert mock_backend.last_effort_limit == 45.0


def test_generic_controller_delegates_ee_pose_to_kinematics_provider(pybullet_direct):
    """Verifies that GenericRobotController delegates get_end_effector_pose to its KinematicsProvider."""
    from unittest.mock import MagicMock
    from robotics.kinematics_provider import KinematicsProvider

    client_id = pybullet_direct
    spec = get_robot_registry().get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)

    mock_provider = MagicMock(spec=KinematicsProvider)
    mock_provider.compute_fk.return_value = (np.array([0.5, 0.1, 0.4]), np.array([1.0, 0.0, 0.0, 0.0]))

    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
        kinematics_provider=mock_provider,
    )

    assert controller.kinematics_provider is mock_provider
    pos, orn = controller.get_end_effector_pose()
    np.testing.assert_allclose(pos, [0.5, 0.1, 0.4])
    np.testing.assert_allclose(orn, [1.0, 0.0, 0.0, 0.0])
    mock_provider.compute_fk.assert_called_once()


