"""Tests for RobotBackend contract, BackendHealthStatus, and TimestampedJointState."""

import dataclasses
import numpy as np
import pytest

from robotics.backends.base import (
    BackendHealthStatus,
    RobotBackend,
    TimestampedJointState,
)


def test_timestamped_joint_state_fields_and_immutability():
    """Verify that TimestampedJointState stores immutable tuples and rejects mutation."""
    state = TimestampedJointState(
        source_timestamp_s=100.0,
        receive_timestamp_s=100.002,
        joint_names=("joint1", "joint2"),
        positions=(0.1, 0.2),
        velocities=(0.01, -0.02),
        efforts=(5.0, 10.0),
        sequence_id=42,
    )

    assert state.source_timestamp_s == 100.0
    assert state.receive_timestamp_s == 100.002
    assert state.joint_names == ("joint1", "joint2")
    assert state.positions == (0.1, 0.2)
    assert state.velocities == (0.01, -0.02)
    assert state.efforts == (5.0, 10.0)
    assert state.sequence_id == 42

    with pytest.raises(dataclasses.FrozenInstanceError):
        state.sequence_id = 43  # type: ignore[misc]


def test_timestamped_joint_state_validation():
    """Verify validation of lengths and numeric finiteness."""
    # Mismatched positions length
    with pytest.raises(ValueError, match="Length mismatch"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1", "j2"),
            positions=(0.1,),
            velocities=(0.0, 0.0),
        )

    # Mismatched velocities length
    with pytest.raises(ValueError, match="Length mismatch"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1", "j2"),
            positions=(0.1, 0.2),
            velocities=(0.0,),
        )

    # Mismatched efforts length
    with pytest.raises(ValueError, match="Length mismatch"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1", "j2"),
            positions=(0.1, 0.2),
            velocities=(0.0, 0.0),
            efforts=(1.0,),
        )

    # Non-finite position
    with pytest.raises(ValueError, match="Non-finite"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1",),
            positions=(float("nan"),),
            velocities=(0.0,),
        )

    # Non-finite velocity
    with pytest.raises(ValueError, match="Non-finite"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1",),
            positions=(0.0,),
            velocities=(float("inf"),),
        )

    # Non-finite effort
    with pytest.raises(ValueError, match="Non-finite"):
        TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=1.0,
            joint_names=("j1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(float("nan"),),
        )


def test_timestamped_joint_state_array_copies():
    """Verify that array getter methods return new, copy-safe NumPy arrays."""
    state = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=("j1", "j2"),
        positions=(0.5, -0.5),
        velocities=(0.1, -0.1),
        efforts=(2.0, 4.0),
    )

    pos_arr = state.get_positions_array()
    assert isinstance(pos_arr, np.ndarray)
    np.testing.assert_array_equal(pos_arr, [0.5, -0.5])
    pos_arr[0] = 999.0
    assert state.positions[0] == 0.5  # Original remains unaffected

    vel_arr = state.get_velocities_array()
    assert isinstance(vel_arr, np.ndarray)
    np.testing.assert_array_equal(vel_arr, [0.1, -0.1])
    vel_arr[0] = 999.0
    assert state.velocities[0] == 0.1

    eff_arr = state.get_efforts_array()
    assert isinstance(eff_arr, np.ndarray)
    np.testing.assert_array_equal(eff_arr, [2.0, 4.0])
    eff_arr[0] = 999.0
    assert state.efforts[0] == 2.0

    # Test state without efforts
    state_no_efforts = TimestampedJointState(
        source_timestamp_s=1.0,
        receive_timestamp_s=1.0,
        joint_names=("j1",),
        positions=(0.0,),
        velocities=(0.0,),
    )
    assert state_no_efforts.get_efforts_array() is None


def test_timestamped_joint_state_age_and_freshness():
    """Verify age calculation and timeout freshness evaluations."""
    state = TimestampedJointState(
        source_timestamp_s=10.0,
        receive_timestamp_s=10.0,
        joint_names=("j1",),
        positions=(0.0,),
        velocities=(0.0,),
    )

    assert state.age_s(10.0) == 0.0
    assert state.age_s(10.05) == pytest.approx(0.05)
    assert state.age_s(9.0) == 0.0  # Monotonic clamp at 0.0

    assert state.is_fresh(current_time_s=10.02, timeout_s=0.05) is True
    assert state.is_fresh(current_time_s=10.10, timeout_s=0.05) is False


def test_backend_health_status_enum():
    """Verify enum members exist and are distinct."""
    assert BackendHealthStatus.HEALTHY is not None
    assert BackendHealthStatus.DEGRADED is not None
    assert BackendHealthStatus.DISCONNECTED is not None
    assert BackendHealthStatus.ERROR is not None


def test_robot_backend_abstract_methods_cannot_be_instantiated():
    """Verify RobotBackend is an abstract base class that cannot be instantiated directly."""
    with pytest.raises(TypeError):
        RobotBackend()  # type: ignore[abstract]


def test_mock_backend_connection_lifecycle():
    """Verify MockRobotBackend connection, disconnection, and health status lifecycle."""
    from robotics.backends.mock_backend import MockRobotBackend

    backend = MockRobotBackend(joint_names=["j1", "j2"])
    assert backend.is_connected() is False
    assert backend.health_status() == BackendHealthStatus.DISCONNECTED

    assert backend.connect() is True
    assert backend.is_connected() is True
    assert backend.health_status() == BackendHealthStatus.HEALTHY

    backend.disconnect()
    assert backend.is_connected() is False
    assert backend.health_status() == BackendHealthStatus.DISCONNECTED


def test_mock_backend_joint_state_ordering_and_defaults():
    """Verify MockRobotBackend returns expected joint states and respects initial positions."""
    from robotics.backends.mock_backend import MockRobotBackend

    backend = MockRobotBackend(
        joint_names=["joint_a", "joint_b"],
        initial_positions=[0.1, -0.2],
    )
    backend.connect()

    state = backend.get_joint_state()
    assert state.joint_names == ("joint_a", "joint_b")
    assert state.positions == (0.1, -0.2)
    assert state.velocities == (0.0, 0.0)
    assert state.efforts == (0.0, 0.0)
    assert state.sequence_id >= 1


def test_mock_backend_position_command_recording():
    """Verify MockRobotBackend records commanded positions."""
    from robotics.backends.mock_backend import MockRobotBackend

    backend = MockRobotBackend(joint_names=["j1", "j2"])
    backend.connect()

    success = backend.command_joint_positions([0.5, -0.5])
    assert success is True
    assert backend.last_commanded_positions == (0.5, -0.5)
    assert ("position", (0.5, -0.5)) in backend.command_history

    # State positions update to commanded positions
    state = backend.get_joint_state()
    assert state.positions == (0.5, -0.5)


def test_mock_backend_velocity_command_and_effort_limit_recording():
    """Verify MockRobotBackend records commanded velocities and optional effort limits."""
    from robotics.backends.mock_backend import MockRobotBackend

    backend = MockRobotBackend(joint_names=["j1", "j2"])
    backend.connect()

    success = backend.command_joint_velocities([0.2, -0.3], effort_limit=50.0)
    assert success is True
    assert backend.last_commanded_velocities == (0.2, -0.3)
    assert backend.last_effort_limit == 50.0
    assert ("velocity", (0.2, -0.3), 50.0) in backend.command_history

    state = backend.get_joint_state()
    assert state.velocities == (0.2, -0.3)


def test_mock_backend_nan_inf_and_dimension_validation():
    """Verify validation of NaN, Inf, dimension mismatch, and invalid effort limits."""
    from robotics.backends.mock_backend import MockRobotBackend

    backend = MockRobotBackend(joint_names=["j1", "j2"])
    backend.connect()

    # Wrong vector length
    with pytest.raises(ValueError, match="Length mismatch"):
        backend.command_joint_positions([0.1])

    with pytest.raises(ValueError, match="Length mismatch"):
        backend.command_joint_velocities([0.1, 0.2, 0.3])

    # NaN / Inf in positions
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_positions([float("nan"), 0.0])

    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_positions([float("inf"), 0.0])

    # NaN / Inf in velocities
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_velocities([0.0, float("nan")])

    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_velocities([float("-inf"), 0.0])

    # Invalid effort limits
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_velocities([0.1, 0.1], effort_limit=float("nan"))

    with pytest.raises(ValueError, match="must be non-negative"):
        backend.command_joint_velocities([0.1, 0.1], effort_limit=-5.0)


def test_mock_backend_halt_motion():
    """Verify MockRobotBackend halt motion zeros velocities and sets is_halted flag."""
    from robotics.backends.mock_backend import MockRobotBackend

    backend = MockRobotBackend(joint_names=["j1", "j2"])
    backend.connect()
    backend.command_joint_velocities([0.5, 0.5])
    assert backend.is_halted is False

    backend.halt_motion()
    assert backend.is_halted is True
    assert backend.last_commanded_velocities == (0.0, 0.0)
    state = backend.get_joint_state()
    assert state.velocities == (0.0, 0.0)


@pytest.fixture
def pybullet_sim_fixture():
    """Provides a temporary PyBullet direct physics simulation and Panda robot body."""
    import pybullet as p
    import pybullet_data
    from robotics.robot_registry import get_robot_registry

    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    spec = get_robot_registry().get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)

    # Discover arm joint indices and names (revolute joints)
    num_joints = p.getNumJoints(body_id, physicsClientId=client_id)
    arm_indices = []
    arm_names = []
    for i in range(num_joints):
        info = p.getJointInfo(body_id, i, physicsClientId=client_id)
        joint_type = info[2]
        if joint_type == p.JOINT_REVOLUTE:
            arm_indices.append(i)
            arm_names.append(info[1].decode("utf-8"))

    yield client_id, body_id, arm_indices, arm_names, spec.max_joint_force

    if p.getConnectionInfo(physicsClientId=client_id)["isConnected"]:
        p.disconnect(physicsClientId=client_id)


def test_pybullet_backend_connection_lifecycle_non_destructive(pybullet_sim_fixture):
    """Verify Ruling B: PyBulletRobotBackend attach/detach does NOT destroy the physics client."""
    import pybullet as p
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )

    assert backend.is_connected() is False
    assert backend.health_status() == BackendHealthStatus.DISCONNECTED

    # Connect / attach
    assert backend.connect() is True
    assert backend.is_connected() is True
    assert backend.health_status() == BackendHealthStatus.HEALTHY

    # Disconnect must detach backend but NOT close PyBullet simulation client
    backend.disconnect()
    assert backend.is_connected() is False
    assert backend.health_status() == BackendHealthStatus.DISCONNECTED

    # PyBullet client MUST remain live
    client_info = p.getConnectionInfo(physicsClientId=client_id)
    assert client_info["isConnected"] == 1


def test_pybullet_backend_get_joint_state(pybullet_sim_fixture):
    """Verify PyBulletRobotBackend reads positions, velocities, and monotonic timestamps."""
    import pybullet as p
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )
    backend.connect()

    state = backend.get_joint_state()
    assert isinstance(state, TimestampedJointState)
    assert state.joint_names == tuple(arm_names)
    assert len(state.positions) == len(arm_indices)
    assert len(state.velocities) == len(arm_indices)
    assert state.efforts is not None
    assert len(state.efforts) == len(arm_indices)
    assert state.sequence_id >= 1
    assert state.receive_timestamp_s > 0.0
    assert state.source_timestamp_s == state.receive_timestamp_s


def test_pybullet_backend_init_validation(pybullet_sim_fixture):
    """Verify initialization checks for index/name length consistency."""
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    with pytest.raises(ValueError, match="Length mismatch"):
        PyBulletRobotBackend(
            physics_client_id=client_id,
            robot_body_id=body_id,
            arm_joint_indices=arm_indices,
            joint_names=arm_names[:-1],
            default_joint_force=max_force,
        )


def test_pybullet_backend_position_command_dispatch(pybullet_sim_fixture):
    """Verify PyBulletRobotBackend dispatches position control commands."""
    import pybullet as p
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )
    backend.connect()

    target_positions = [0.0, -0.3, 0.0, -1.0, 0.0, 1.0, 0.5][: len(arm_indices)]
    success = backend.command_joint_positions(target_positions)
    assert success is True

    # Step simulation to allow motors to move toward target
    for _ in range(240):
        p.stepSimulation(physicsClientId=client_id)

    state = backend.get_joint_state()
    # Check that joints moved in the direction of commanded positions
    for cur_pos, target_pos in zip(state.positions, target_positions):
        assert abs(cur_pos - target_pos) < 0.15


def test_pybullet_backend_position_command_validation(pybullet_sim_fixture):
    """Verify position command validates finiteness, vector length, and connection status."""
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )

    # Disconnected check
    with pytest.raises(RuntimeError, match="disconnected"):
        backend.command_joint_positions([0.0] * len(arm_indices))

    backend.connect()

    # Length mismatch
    with pytest.raises(ValueError, match="Length mismatch"):
        backend.command_joint_positions([0.0] * (len(arm_indices) - 1))

    # NaN / Inf
    nan_target = [0.0] * len(arm_indices)
    nan_target[0] = float("nan")
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_positions(nan_target)

    inf_target = [0.0] * len(arm_indices)
    inf_target[1] = float("inf")
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_positions(inf_target)


def test_pybullet_backend_velocity_command_dispatch_and_effort_limit(pybullet_sim_fixture):
    """Verify velocity commands apply default force or explicit effort_limit to PyBullet."""
    import pybullet as p
    from unittest.mock import patch
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )
    backend.connect()

    target_vels = [0.1] * len(arm_indices)

    # Test with default effort limit (forces == [max_force] * n)
    with patch("pybullet.setJointMotorControlArray") as mock_motor_ctrl:
        success = backend.command_joint_velocities(target_vels)
        assert success is True
        mock_motor_ctrl.assert_called_once_with(
            bodyIndex=body_id,
            jointIndices=arm_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=target_vels,
            forces=[max_force] * len(arm_indices),
            physicsClientId=client_id,
        )

    # Test with explicit effort limit override (Ruling A)
    with patch("pybullet.setJointMotorControlArray") as mock_motor_ctrl:
        success = backend.command_joint_velocities(target_vels, effort_limit=75.0)
        assert success is True
        mock_motor_ctrl.assert_called_once_with(
            bodyIndex=body_id,
            jointIndices=arm_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=target_vels,
            forces=[75.0] * len(arm_indices),
            physicsClientId=client_id,
        )


def test_pybullet_backend_velocity_validation(pybullet_sim_fixture):
    """Verify velocity commands validate finiteness, length, and effort bounds."""
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )

    with pytest.raises(RuntimeError, match="disconnected"):
        backend.command_joint_velocities([0.0] * len(arm_indices))

    backend.connect()

    # Length mismatch
    with pytest.raises(ValueError, match="Length mismatch"):
        backend.command_joint_velocities([0.0] * (len(arm_indices) + 1))

    # NaN / Inf velocity
    nan_vel = [0.0] * len(arm_indices)
    nan_vel[0] = float("nan")
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_velocities(nan_vel)

    # Non-finite effort limit
    with pytest.raises(ValueError, match="Non-finite"):
        backend.command_joint_velocities([0.0] * len(arm_indices), effort_limit=float("nan"))

    # Negative effort limit
    with pytest.raises(ValueError, match="non-negative"):
        backend.command_joint_velocities([0.0] * len(arm_indices), effort_limit=-10.0)


def test_pybullet_backend_halt_motion(pybullet_sim_fixture):
    """Verify halt_motion dispatches zero velocities to PyBullet."""
    import pybullet as p
    from unittest.mock import patch
    from robotics.backends.pybullet_backend import PyBulletRobotBackend

    client_id, body_id, arm_indices, arm_names, max_force = pybullet_sim_fixture

    backend = PyBulletRobotBackend(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=arm_indices,
        joint_names=arm_names,
        default_joint_force=max_force,
    )
    backend.connect()

    with patch("pybullet.setJointMotorControlArray") as mock_motor_ctrl:
        backend.halt_motion()
        mock_motor_ctrl.assert_called_once_with(
            bodyIndex=body_id,
            jointIndices=arm_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[0.0] * len(arm_indices),
            forces=[max_force] * len(arm_indices),
            physicsClientId=client_id,
        )




