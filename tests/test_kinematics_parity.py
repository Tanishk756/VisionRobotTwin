"""Parity regression test comparing kinematics calculations against immutable Phase A1 golden baselines.

SOURCE_RUNTIME_SHA = "98133558cb5fef70b5766d58e37e748b71fde757"
"""

import pybullet as p
import pybullet_data
import numpy as np
import pytest

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.inverse_kinematics import GenericIKSolver
from robotics.kinematics import compute_fk_at_configuration, compute_jacobian
from tests.fixtures.kinematics_golden import (
    PANDA_GOLDEN,
    KUKA_GOLDEN,
    sign_invariant_quaternion_distance,
    SOURCE_RUNTIME_SHA,
)


@pytest.fixture(scope="module")
def pybullet_direct_client():
    """Sets up a DIRECT PyBullet simulation client for golden parity tests."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    yield client_id
    p.disconnect(physicsClientId=client_id)


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_golden_provenance_metadata(golden):
    """Verifies that golden test data records valid metadata."""
    assert golden["num_dof"] == 7
    assert len(golden["q_home"]) == 7
    assert len(golden["q_mid"]) == 7
    assert SOURCE_RUNTIME_SHA == "98133558cb5fef70b5766d58e37e748b71fde757"


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_fk_home_parity(pybullet_direct_client, golden):
    """Verifies Forward Kinematics at home configuration against golden constants."""
    client_id = pybullet_direct_client
    registry = get_robot_registry()
    spec = registry.get_robot_spec(golden["robot_id"])
    
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )
    ctrl = GenericRobotController(physics_client_id=client_id, robot_id=robot_id, spec=spec)
    
    pos, orn = compute_fk_at_configuration(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=ctrl.arm_joint_indices,
        joint_positions=golden["q_home"],
        ee_link_index=ctrl.ee_link_index,
    )
    
    np.testing.assert_allclose(pos, golden["fk_pos_home"], atol=1e-7)
    assert sign_invariant_quaternion_distance(orn, np.array(golden["fk_orn_home"])) <= 1e-7


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_jacobian_home_parity(pybullet_direct_client, golden):
    """Verifies Spatial Geometric Jacobian at home configuration against golden constants."""
    client_id = pybullet_direct_client
    registry = get_robot_registry()
    spec = registry.get_robot_spec(golden["robot_id"])
    
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )
    ctrl = GenericRobotController(physics_client_id=client_id, robot_id=robot_id, spec=spec)
    
    j_lin, j_ang, j_full = compute_jacobian(
        physics_client_id=client_id,
        robot_id=robot_id,
        ee_link_index=ctrl.ee_link_index,
        arm_joint_indices=ctrl.arm_joint_indices,
        joint_positions=golden["q_home"],
    )
    
    np.testing.assert_allclose(j_lin, golden["j_lin_home"], atol=1e-7)
    np.testing.assert_allclose(j_ang, golden["j_ang_home"], atol=1e-7)
    np.testing.assert_allclose(j_full, golden["j_full_home"], atol=1e-7)


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_fk_and_jacobian_mid_range_parity(pybullet_direct_client, golden):
    """Verifies FK and Jacobian at mid-range configuration against golden constants."""
    client_id = pybullet_direct_client
    registry = get_robot_registry()
    spec = registry.get_robot_spec(golden["robot_id"])
    
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )
    ctrl = GenericRobotController(physics_client_id=client_id, robot_id=robot_id, spec=spec)
    
    pos, orn = compute_fk_at_configuration(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=ctrl.arm_joint_indices,
        joint_positions=golden["q_mid"],
        ee_link_index=ctrl.ee_link_index,
    )
    np.testing.assert_allclose(pos, golden["fk_pos_mid"], atol=1e-7)
    assert sign_invariant_quaternion_distance(orn, np.array(golden["fk_orn_mid"])) <= 1e-7
    
    _, _, j_full = compute_jacobian(
        physics_client_id=client_id,
        robot_id=robot_id,
        ee_link_index=ctrl.ee_link_index,
        arm_joint_indices=ctrl.arm_joint_indices,
        joint_positions=golden["q_mid"],
    )
    np.testing.assert_allclose(j_full, golden["j_full_mid"], atol=1e-7)


@pytest.mark.parametrize("golden", [PANDA_GOLDEN, KUKA_GOLDEN], ids=["panda", "kuka_iiwa"])
def test_raw_ik_and_residual_parity(pybullet_direct_client, golden):
    """Verifies raw IK solve and residual evaluation against golden constants."""
    client_id = pybullet_direct_client
    registry = get_robot_registry()
    spec = registry.get_robot_spec(golden["robot_id"])
    
    p.resetSimulation(physicsClientId=client_id)
    robot_id = p.loadURDF(
        spec.urdf_path,
        spec.base_position,
        spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=client_id,
    )
    ctrl = GenericRobotController(physics_client_id=client_id, robot_id=robot_id, spec=spec)
    ctrl.reset_to_home()
    
    lower_limits = [ctrl.joints[i].lower_limit for i in ctrl.arm_joint_indices]
    upper_limits = [ctrl.joints[i].upper_limit for i in ctrl.arm_joint_indices]
    joint_ranges = [upper_limits[i] - lower_limits[i] for i in range(len(lower_limits))]
    rest_poses = list(spec.home_joint_positions)
    
    num_total_joints = p.getNumJoints(robot_id, physicsClientId=client_id)
    movable = [j for j in range(num_total_joints) if p.getJointInfo(robot_id, j, physicsClientId=client_id)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
    pad_len = len(movable) - len(ctrl.arm_joint_indices)
    full_lower = lower_limits + [0.0] * pad_len
    full_upper = upper_limits + [0.04] * pad_len
    full_ranges = joint_ranges + [0.04] * pad_len
    full_rests = rest_poses + [0.04] * pad_len
    
    ik_raw_full = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=ctrl.ee_link_index,
        targetPosition=golden["ik_target_pos"],
        targetOrientation=golden["ik_target_orn"],
        lowerLimits=full_lower,
        upperLimits=full_upper,
        jointRanges=full_ranges,
        restPoses=full_rests,
        maxNumIterations=100,
        residualThreshold=1e-4,
        physicsClientId=client_id,
    )
    raw_ik_q = list(ik_raw_full[:len(ctrl.arm_joint_indices)])
    
    np.testing.assert_allclose(raw_ik_q, golden["raw_ik_q"], atol=1e-6)
    
    pos_res, _ = compute_fk_at_configuration(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=ctrl.arm_joint_indices,
        joint_positions=raw_ik_q,
        ee_link_index=ctrl.ee_link_index,
    )
    pos_err = float(np.linalg.norm(np.array(golden["ik_target_pos"]) - np.array(pos_res)))
    assert abs(pos_err - golden["pos_residual"]) <= 1e-6
