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


def test_generic_controller_effort_limit_override_gate(pybullet_direct):
    """Verify set_arm_joint_velocities fails if max_force requested on backend without effort_limit_override."""
    from unittest.mock import MagicMock
    from robotics.backends.base import RobotBackend, RobotBackendCapabilities, UnsupportedBackendOperationError

    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=True, physicsClientId=client_id)

    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True
    mock_backend.transport_capabilities = RobotBackendCapabilities(
        read_only=False,
        position_commands=False,
        velocity_commands=True,
        effort_limit_override=False,
        halt_motion=True,
    )

    controller = GenericRobotController(
        physics_client_id=client_id,
        robot_id=body_id,
        spec=spec,
        backend=mock_backend,
    )

    with pytest.raises(UnsupportedBackendOperationError, match="effort limit"):
        controller.set_arm_joint_velocities([0.1]*7, max_force=50.0)

    # Calling without max_force succeeds
    controller.set_arm_joint_velocities([0.1]*7, max_force=None)
    mock_backend.command_joint_velocities.assert_called_once()



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


def test_generic_controller_standalone_pure_no_pybullet():
    """MANDATORY A4 ARCHITECTURE TEST:
    Instantiates and operates GenericRobotController with ResolvedRobotModel,
    MockRobotBackend, and MockKinematicsProvider with ZERO PyBullet connection or body.
    """
    from unittest.mock import MagicMock
    from robotics.backends.mock_backend import MockRobotBackend
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
        RobotCapabilities,
    )

    arm_j0 = ResolvedJointMetadata(
        model_index=0,
        canonical_index=0,
        name="test_joint1",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-2.5,
        upper_limit=2.5,
        max_force=100.0,
        max_velocity=2.0,
        link_name="test_link1",
    )
    arm_j1 = ResolvedJointMetadata(
        model_index=1,
        canonical_index=1,
        name="test_joint2",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-1.5,
        upper_limit=1.5,
        max_force=80.0,
        max_velocity=1.5,
        link_name="test_link2",
    )

    resolved_model = ResolvedRobotModel(
        robot_id="custom_bot",
        display_name="Custom 2-DoF Robot",
        all_joints=(arm_j0, arm_j1),
        arm_joints=(arm_j0, arm_j1),
        gripper_joints=(),
        ee_link_name="test_link2",
        home_joint_positions=(0.0, 0.0),
        capabilities=RobotCapabilities(has_gripper=False, supports_pick_place=False),
    )

    mock_backend = MockRobotBackend(
        joint_names=["test_joint1", "test_joint2"],
        initial_positions=[0.0, 0.0],
    )

    mock_kinematics = MagicMock(spec=KinematicsProvider)
    mock_kinematics.compute_fk.return_value = (np.array([0.4, 0.0, 0.2]), np.array([0.0, 0.0, 0.0, 1.0]))

    # Pure decoupled construction: NO physics_client_id, NO robot_id
    controller = GenericRobotController(
        resolved_model=resolved_model,
        backend=mock_backend,
        kinematics_provider=mock_kinematics,
    )

    assert controller.model is resolved_model
    assert controller.capabilities.has_gripper is False
    assert controller.capabilities.supports_pick_place is False

    # Check joint limits
    lows, highs, ranges, rests = controller.get_joint_limits()
    assert lows == [-2.5, -1.5]
    assert highs == [2.5, 1.5]
    assert ranges == [5.0, 3.0]
    assert rests == [0.0, 0.0]

    # Check state reads
    q = controller.get_current_joint_positions()
    assert q == [0.0, 0.0]
    dq = controller.get_current_joint_velocities()
    assert dq == [0.0, 0.0]

    # Check position rate limiting
    # Large target jump with dt=0.01: max deltas are [2.0*0.01, 1.5*0.01] = [0.02, 0.015]
    commanded_q = controller.set_arm_joint_positions([1.0, 1.0], dt=0.01, enforce_velocity_limits=True)
    assert max(commanded_q) <= 0.02 + 1e-9
    assert mock_backend.last_commanded_positions == tuple(commanded_q)

    # Check velocity clamping and effort_limit
    commanded_v = controller.set_arm_joint_velocities([10.0, -10.0], max_force=55.0)
    assert commanded_v == [2.0, -1.5]
    assert mock_backend.last_commanded_velocities == (2.0, -1.5)
    assert mock_backend.last_effort_limit == 55.0

    # Check FK delegation
    ee_pos, ee_orn = controller.get_end_effector_pose()
    np.testing.assert_allclose(ee_pos, [0.4, 0.0, 0.2])
    np.testing.assert_allclose(ee_orn, [0.0, 0.0, 0.0, 1.0])

    # Check cartesian error
    err = controller.compute_cartesian_error(np.array([0.4, 0.0, 0.2]))
    assert abs(err) < 1e-9


def test_generic_controller_import_without_pybullet_subprocess():
    """Verifies that GenericRobotController and ResolvedRobotModel can be imported
    in an isolated environment where pybullet import is blocked.
    """
    import subprocess
    import sys

    code = (
        "import sys\n"
        "sys.modules['pybullet'] = None\n"
        "from robotics.robot_model import ResolvedRobotModel, JointRole, JointMotionType\n"
        "from robotics.robot_controller import GenericRobotController\n"
        "print('IMPORT_SUCCESS')\n"
    )

    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "IMPORT_SUCCESS" in result.stdout


def test_generic_controller_constructor_no_motion_side_effects():
    """Verifies that constructing GenericRobotController does NOT command any motion on backend."""
    from unittest.mock import MagicMock
    from robotics.backends.mock_backend import MockRobotBackend
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
    )

    arm_j0 = ResolvedJointMetadata(
        model_index=0,
        canonical_index=0,
        name="j1",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-1.0,
        upper_limit=1.0,
        max_force=50.0,
        max_velocity=1.0,
        link_name="l1",
    )
    model = ResolvedRobotModel(
        robot_id="bot",
        display_name="Bot",
        all_joints=(arm_j0,),
        arm_joints=(arm_j0,),
        gripper_joints=(),
        ee_link_name="l1",
        home_joint_positions=(0.0,),
    )

    mock_backend = MockRobotBackend(
        joint_names=["j1"],
        initial_positions=[0.5],  # Non-zero position
    )
    mock_kinematics = MagicMock(spec=KinematicsProvider)

    # Instantiate controller
    controller = GenericRobotController(
        resolved_model=model,
        backend=mock_backend,
        kinematics_provider=mock_kinematics,
    )

    # Verify no command was dispatched
    assert mock_backend.last_commanded_positions is None
    assert mock_backend.last_commanded_velocities is None
    assert mock_backend.is_halted is False
    # Position remains unchanged at 0.5 (was NOT teleported to home 0.0)
    assert controller.get_current_joint_positions() == [0.5]


def test_generic_controller_standalone_reset_to_home_raises():
    """Verifies that calling reset_to_home on a standalone controller without PyBullet raises RuntimeError."""
    from unittest.mock import MagicMock
    from robotics.backends.mock_backend import MockRobotBackend
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
    )

    arm_j0 = ResolvedJointMetadata(
        model_index=0,
        canonical_index=0,
        name="j1",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-1.0,
        upper_limit=1.0,
        max_force=50.0,
        max_velocity=1.0,
        link_name="l1",
    )
    model = ResolvedRobotModel(
        robot_id="bot",
        display_name="Bot",
        all_joints=(arm_j0,),
        arm_joints=(arm_j0,),
        gripper_joints=(),
        ee_link_name="l1",
        home_joint_positions=(0.0,),
    )
    mock_backend = MockRobotBackend(joint_names=["j1"], initial_positions=[0.0])
    mock_kinematics = MagicMock(spec=KinematicsProvider)

    controller = GenericRobotController(
        resolved_model=model,
        backend=mock_backend,
        kinematics_provider=mock_kinematics,
    )

    with pytest.raises(RuntimeError, match="legacy simulation teleport"):
        controller.reset_to_home()


def test_generic_controller_backend_mismatch_validation():
    """Verifies that DoF mismatch and joint name mismatch with backend raise ValueError."""
    from unittest.mock import MagicMock
    from robotics.backends.mock_backend import MockRobotBackend
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
    )

    arm_j0 = ResolvedJointMetadata(
        model_index=0,
        canonical_index=0,
        name="joint_a",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-1.0,
        upper_limit=1.0,
        max_force=50.0,
        max_velocity=1.0,
        link_name="la",
    )
    arm_j1 = ResolvedJointMetadata(
        model_index=1,
        canonical_index=1,
        name="joint_b",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-1.0,
        upper_limit=1.0,
        max_force=50.0,
        max_velocity=1.0,
        link_name="lb",
    )
    model = ResolvedRobotModel(
        robot_id="bot",
        display_name="Bot",
        all_joints=(arm_j0, arm_j1),
        arm_joints=(arm_j0, arm_j1),
        gripper_joints=(),
        ee_link_name="lb",
        home_joint_positions=(0.0, 0.0),
    )

    mock_kinematics = MagicMock(spec=KinematicsProvider)

    # 1. DoF mismatch: backend has 3 joints, model expects 2
    mock_backend_dof = MockRobotBackend(
        joint_names=["joint_a", "joint_b", "joint_c"],
        initial_positions=[0.0, 0.0, 0.0],
    )
    with pytest.raises(ValueError, match="DoF mismatch"):
        GenericRobotController(
            resolved_model=model,
            backend=mock_backend_dof,
            kinematics_provider=mock_kinematics,
        )

    # 2. Name ordering mismatch: backend has ["joint_b", "joint_a"]
    mock_backend_names = MockRobotBackend(
        joint_names=["joint_b", "joint_a"],
        initial_positions=[0.0, 0.0],
    )
    with pytest.raises(ValueError, match="names mismatch"):
        GenericRobotController(
            resolved_model=model,
            backend=mock_backend_names,
            kinematics_provider=mock_kinematics,
        )


def test_controller_deferred_backend_validation_and_subsequent_read():
    """Verifies that GenericRobotController tolerates initial BackendStateUnavailableError and validates upon first read."""
    from unittest.mock import MagicMock
    from robotics.backends.base import (
        BackendStateUnavailableError,
        RobotBackend,
        TimestampedJointState,
    )
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
    )

    arm_j0 = ResolvedJointMetadata(
        model_index=0,
        canonical_index=0,
        name="joint_1",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-1.0,
        upper_limit=1.0,
        max_force=50.0,
        max_velocity=1.0,
        link_name="l1",
    )
    model = ResolvedRobotModel(
        robot_id="bot1",
        display_name="Bot1",
        all_joints=(arm_j0,),
        arm_joints=(arm_j0,),
        gripper_joints=(),
        ee_link_name="l1",
        home_joint_positions=(0.0,),
    )

    mock_backend = MagicMock(spec=RobotBackend)
    mock_backend.is_connected.return_value = True
    # Initially state unavailable (e.g. ROS2 waiting for first message)
    mock_backend.get_joint_state.side_effect = BackendStateUnavailableError("Waiting for /joint_states")
    mock_kinematics = MagicMock(spec=KinematicsProvider)

    # Controller construction must succeed without error
    controller = GenericRobotController(
        resolved_model=model,
        backend=mock_backend,
        kinematics_provider=mock_kinematics,
    )
    assert controller.backend is mock_backend

    # When state becomes available with wrong DoF, subsequent get_current_joint_positions must raise ValueError
    invalid_state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=("wrong_name",),
        positions=(0.5,),
        velocities=(0.0,),
    )
    mock_backend.get_joint_state.side_effect = None
    mock_backend.get_joint_state.return_value = invalid_state

    with pytest.raises(ValueError, match="names mismatch"):
        controller.get_current_joint_positions()




