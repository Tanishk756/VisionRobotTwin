"""Unit and integration tests for the KinematicsProvider interface and providers."""

import pytest
from typing import Sequence, Tuple, Optional
import numpy as np
import pybullet as p
import pybullet_data

from robotics.robot_registry import get_robot_registry
from tests.fixtures.kinematics_golden import (
    PANDA_GOLDEN,
    KUKA_GOLDEN,
    sign_invariant_quaternion_distance,
)


@pytest.fixture(scope="module")
def pybullet_direct_client():
    """Sets up a DIRECT PyBullet simulation client."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    yield client_id
    p.disconnect(physicsClientId=client_id)


def test_kinematics_provider_cannot_be_instantiated_directly():
    """KinematicsProvider is an abstract base class and should reject direct instantiation."""
    from robotics.kinematics_provider import KinematicsProvider

    with pytest.raises(TypeError):
        KinematicsProvider()


def test_kinematics_provider_subclass_contract():
    """A concrete implementation of KinematicsProvider must implement compute_fk, compute_jacobian, and solve_ik_raw."""
    from robotics.kinematics_provider import KinematicsProvider

    class DummyProvider(KinematicsProvider):
        def compute_fk(self, joint_positions: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
            return np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])

        def compute_jacobian(
            self, joint_positions: Sequence[float]
        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
            return np.zeros((3, 7)), np.zeros((3, 7)), np.zeros((6, 7))

        def solve_ik_raw(
            self,
            target_position: Sequence[float],
            target_orientation: Optional[Sequence[float]] = None,
            lower_limits: Optional[Sequence[float]] = None,
            upper_limits: Optional[Sequence[float]] = None,
            joint_ranges: Optional[Sequence[float]] = None,
            rest_poses: Optional[Sequence[float]] = None,
            joint_damping: Optional[Sequence[float]] = None,
            max_iterations: int = 100,
            residual_threshold: float = 1e-4,
        ) -> Tuple[float, ...]:
            return tuple([0.0] * 7)

    dummy = DummyProvider()
    pos, orn = dummy.compute_fk([0.0] * 7)
    assert pos.shape == (3,)
    assert orn.shape == (4,)

    j_lin, j_ang, j_full = dummy.compute_jacobian([0.0] * 7)
    assert j_lin.shape == (3, 7)
    assert j_ang.shape == (3, 7)
    assert j_full.shape == (6, 7)

    ik_q = dummy.solve_ik_raw([0.3, 0.0, 0.5])
    assert len(ik_q) == 7


def test_pybullet_kinematics_provider_shared_lock(pybullet_direct_client):
    """Verifies PyBulletKinematicsProvider supports injected query_lock and default private lock."""
    import threading
    from robotics.kinematics_provider import PyBulletKinematicsProvider
    from robotics.robot_registry import get_robot_registry

    client_id = pybullet_direct_client
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    robot_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)

    # 1. Default private lock
    provider_default = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=(0, 1, 2, 3, 4, 5, 6),
        end_effector_link_index=11,
    )
    assert isinstance(provider_default.query_lock, type(threading.RLock()))

    # 2. Injected shared lock
    shared_lock = threading.RLock()
    provider_shared = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=(0, 1, 2, 3, 4, 5, 6),
        end_effector_link_index=11,
        query_lock=shared_lock,
    )
    assert provider_shared.query_lock is shared_lock


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_pybullet_fk_parity(pybullet_direct_client, golden):
    """Verifies PyBulletKinematicsProvider compute_fk against golden constants."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec(golden["robot_id"])
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=golden["arm_joint_indices"],
        end_effector_link_index=golden["ee_link_index"],
    )

    # Home FK
    pos_home, orn_home = provider.compute_fk(golden["q_home"])
    np.testing.assert_allclose(pos_home, golden["fk_pos_home"], atol=1e-7)
    assert sign_invariant_quaternion_distance(orn_home, np.array(golden["fk_orn_home"])) <= 1e-7

    # Mid FK
    pos_mid, orn_mid = provider.compute_fk(golden["q_mid"])
    np.testing.assert_allclose(pos_mid, golden["fk_pos_mid"], atol=1e-7)
    assert sign_invariant_quaternion_distance(orn_mid, np.array(golden["fk_orn_mid"])) <= 1e-7


def test_pybullet_fk_state_preservation(pybullet_direct_client):
    """Verifies that compute_fk preserves active joint positions and velocities (q, dq)."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec("panda")
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    arm_joints = PANDA_GOLDEN["arm_joint_indices"]
    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=arm_joints,
        end_effector_link_index=PANDA_GOLDEN["ee_link_index"],
    )

    # Set arbitrary active positions and velocities
    init_q = [0.1, -0.5, 0.2, -1.8, 0.3, 1.2, -0.4]
    init_dq = [0.05, -0.02, 0.01, 0.04, -0.01, 0.03, -0.02]
    for idx, q_val, dq_val in zip(arm_joints, init_q, init_dq):
        p.resetJointState(robot_id, idx, targetValue=q_val, targetVelocity=dq_val, physicsClientId=client_id)

    # Query FK at a completely different candidate configuration
    candidate_q = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0]
    pos, orn = provider.compute_fk(candidate_q)
    assert pos.shape == (3,)

    # Verify original state is perfectly preserved
    states_after = p.getJointStates(robot_id, arm_joints, physicsClientId=client_id)
    q_after = [s[0] for s in states_after]
    dq_after = [s[1] for s in states_after]

    np.testing.assert_allclose(q_after, init_q, atol=1e-12)
    np.testing.assert_allclose(dq_after, init_dq, atol=1e-12)


def test_pybullet_fk_input_validation(pybullet_direct_client):
    """Verifies that compute_fk rejects wrong length, NaN, and Inf inputs."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec("panda")
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=PANDA_GOLDEN["arm_joint_indices"],
        end_effector_link_index=PANDA_GOLDEN["ee_link_index"],
    )

    # Wrong length
    with pytest.raises(ValueError, match="Expected 7 joint positions"):
        provider.compute_fk([0.0] * 6)
    with pytest.raises(ValueError, match="Expected 7 joint positions"):
        provider.compute_fk([0.0] * 8)

    # NaN
    with pytest.raises(ValueError, match="non-finite"):
        provider.compute_fk([0.0, 0.0, np.nan, 0.0, 0.0, 0.0, 0.0])

    # Inf
    with pytest.raises(ValueError, match="non-finite"):
        provider.compute_fk([0.0, 0.0, np.inf, 0.0, 0.0, 0.0, 0.0])


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_pybullet_jacobian_parity(pybullet_direct_client, golden):
    """Verifies PyBulletKinematicsProvider compute_jacobian against golden constants."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec(golden["robot_id"])
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=golden["arm_joint_indices"],
        end_effector_link_index=golden["ee_link_index"],
    )

    # Home Jacobian
    j_lin_home, j_ang_home, j_full_home = provider.compute_jacobian(golden["q_home"])
    np.testing.assert_allclose(j_lin_home, golden["j_lin_home"], atol=1e-7)
    np.testing.assert_allclose(j_ang_home, golden["j_ang_home"], atol=1e-7)
    np.testing.assert_allclose(j_full_home, golden["j_full_home"], atol=1e-7)

    # Mid Jacobian
    j_lin_mid, j_ang_mid, j_full_mid = provider.compute_jacobian(golden["q_mid"])
    np.testing.assert_allclose(j_lin_mid, golden["j_lin_mid"], atol=1e-7)
    np.testing.assert_allclose(j_ang_mid, golden["j_ang_mid"], atol=1e-7)
    np.testing.assert_allclose(j_full_mid, golden["j_full_mid"], atol=1e-7)


def test_pybullet_jacobian_input_validation(pybullet_direct_client):
    """Verifies that compute_jacobian rejects wrong length, NaN, and Inf inputs."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec("panda")
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=PANDA_GOLDEN["arm_joint_indices"],
        end_effector_link_index=PANDA_GOLDEN["ee_link_index"],
    )

    # Wrong length
    with pytest.raises(ValueError, match="Expected 7 joint positions"):
        provider.compute_jacobian([0.0] * 6)

    # NaN
    with pytest.raises(ValueError, match="non-finite"):
        provider.compute_jacobian([0.0, np.nan, 0.0, 0.0, 0.0, 0.0, 0.0])


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_pybullet_solve_ik_raw_parity(pybullet_direct_client, golden):
    """Verifies PyBulletKinematicsProvider solve_ik_raw against golden constants."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec(golden["robot_id"])
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    # Initialize joint positions to home pose
    for idx, q_val in zip(golden["arm_joint_indices"], golden["q_home"]):
        p.resetJointState(robot_id, idx, targetValue=q_val, targetVelocity=0.0, physicsClientId=client_id)

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=golden["arm_joint_indices"],
        end_effector_link_index=golden["ee_link_index"],
    )

    raw_ik_q = provider.solve_ik_raw(
        target_position=golden["ik_target_pos"],
        target_orientation=golden["ik_target_orn"],
        lower_limits=golden["lower_limits"],
        upper_limits=golden["upper_limits"],
        joint_ranges=golden["joint_ranges"],
        rest_poses=golden["rest_poses"],
    )

    assert len(raw_ik_q) == 7
    np.testing.assert_allclose(raw_ik_q, golden["raw_ik_q"], atol=1e-6)


def test_pybullet_solve_ik_raw_input_validation(pybullet_direct_client):
    """Verifies that solve_ik_raw validates target dimensions and finiteness."""
    from robotics.kinematics_provider import PyBulletKinematicsProvider

    client_id = pybullet_direct_client
    spec = get_robot_registry().get_robot_spec("panda")
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=robot_id,
        arm_joint_indices=PANDA_GOLDEN["arm_joint_indices"],
        end_effector_link_index=PANDA_GOLDEN["ee_link_index"],
    )

    # Wrong position length
    with pytest.raises(ValueError, match="Target position must have shape"):
        provider.solve_ik_raw(target_position=[0.3, 0.0])

    # NaN position
    with pytest.raises(ValueError, match="non-finite"):
        provider.solve_ik_raw(target_position=[0.3, np.nan, 0.5])

    # Wrong orientation length
    with pytest.raises(ValueError, match="Target orientation must have shape"):
        provider.solve_ik_raw(target_position=[0.3, 0.0, 0.5], target_orientation=[1.0, 0.0, 0.0])

    # NaN orientation
    with pytest.raises(ValueError, match="non-finite"):
        provider.solve_ik_raw(target_position=[0.3, 0.0, 0.5], target_orientation=[1.0, 0.0, np.nan, 0.0])
