"""Unit tests for PyBulletRobotModelResolver and simulation teleport."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.adapters.panda import PandaAdapter
from robotics.adapters.kuka_iiwa import KukaIiwaAdapter
from robotics.robot_model import JointRole, JointMotionType, ResolvedRobotModel
from robotics.pybullet_model import PyBulletRobotModelResolver, teleport_robot_to_home
from tests.fixtures.model_golden import PANDA_GOLDEN_METADATA, KUKA_GOLDEN_METADATA


@pytest.fixture
def pybullet_sim():
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
    yield cid
    p.disconnect(physicsClientId=cid)


def test_resolve_panda_model(pybullet_sim):
    """Verifies PyBulletRobotModelResolver accurately resolves Panda URDF into ResolvedRobotModel."""
    cid = pybullet_sim
    spec = get_robot_registry().get_robot_spec("panda")
    adapter = PandaAdapter(spec)
    bid = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=spec.fixed_base, physicsClientId=cid)

    model = PyBulletRobotModelResolver.resolve(cid, bid, spec, adapter)

    assert isinstance(model, ResolvedRobotModel)
    assert model.robot_id == "panda"
    assert model.dof == 7
    assert len(model.all_joints) == 12
    assert len(model.arm_joints) == 7
    assert len(model.gripper_joints) == 2
    assert model.ee_link_name == "panda_grasptarget"
    assert model.ee_link_native_index == 11
    assert model.require_arm_native_indices() == (0, 1, 2, 3, 4, 5, 6)
    assert model.require_gripper_native_indices() == (9, 10)

    # Check canonical and model indices
    for i, j in enumerate(model.arm_joints):
        assert j.canonical_index == i
        assert j.role == JointRole.ARM
        assert j.motion_type == JointMotionType.REVOLUTE

    for i, j in enumerate(model.gripper_joints):
        assert j.canonical_index == i
        assert j.role == JointRole.GRIPPER
        assert j.motion_type == JointMotionType.PRISMATIC

    # Check numerical limit parity
    np.testing.assert_allclose(model.arm_lower_limits, PANDA_GOLDEN_METADATA["arm_lower_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(model.arm_upper_limits, PANDA_GOLDEN_METADATA["arm_upper_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(model.arm_max_forces, PANDA_GOLDEN_METADATA["arm_max_forces"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(model.arm_max_velocities, PANDA_GOLDEN_METADATA["arm_max_velocities"], atol=1e-12, rtol=0)


def test_resolve_kuka_model(pybullet_sim):
    """Verifies PyBulletRobotModelResolver accurately resolves KUKA iiwa URDF into ResolvedRobotModel."""
    cid = pybullet_sim
    spec = get_robot_registry().get_robot_spec("kuka_iiwa")
    adapter = KukaIiwaAdapter(spec)
    bid = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=spec.fixed_base, physicsClientId=cid)

    model = PyBulletRobotModelResolver.resolve(cid, bid, spec, adapter)

    assert isinstance(model, ResolvedRobotModel)
    assert model.robot_id == "kuka_iiwa"
    assert model.dof == 7
    assert len(model.all_joints) == 7
    assert len(model.arm_joints) == 7
    assert len(model.gripper_joints) == 0
    assert model.ee_link_name == "lbr_iiwa_link_7"
    assert model.ee_link_native_index == 6
    assert model.require_arm_native_indices() == (0, 1, 2, 3, 4, 5, 6)

    np.testing.assert_allclose(model.arm_lower_limits, KUKA_GOLDEN_METADATA["arm_lower_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(model.arm_upper_limits, KUKA_GOLDEN_METADATA["arm_upper_limits"], atol=1e-12, rtol=0)


def test_teleport_robot_to_home(pybullet_sim):
    """Verifies teleport_robot_to_home sets joint positions in simulation without stepping physics."""
    cid = pybullet_sim
    spec = get_robot_registry().get_robot_spec("panda")
    adapter = PandaAdapter(spec)
    bid = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=spec.fixed_base, physicsClientId=cid)

    model = PyBulletRobotModelResolver.resolve(cid, bid, spec, adapter)
    teleport_robot_to_home(cid, bid, model)

    # Read joint states directly from PyBullet
    for idx, jinfo in enumerate(model.arm_joints):
        state = p.getJointState(bid, jinfo.native_index, physicsClientId=cid)
        expected_q = model.home_joint_positions[idx]
        assert abs(state[0] - expected_q) < 1e-5
        assert abs(state[1]) < 1e-5

    # Check finger positions
    for jinfo in model.gripper_joints:
        state = p.getJointState(bid, jinfo.native_index, physicsClientId=cid)
        assert abs(state[0] - 0.04) < 1e-5
