"""Tests for Geometric Jacobian Computation."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.kinematics import compute_jacobian


@pytest.fixture
def pybullet_direct():
    """Initializes a direct PyBullet physics simulation fixture."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)
    yield client_id
    p.disconnect(physicsClientId=client_id)


def test_jacobian_dimensions_and_finiteness_panda(pybullet_direct):
    """Verifies Jacobian dimensions (6x7) and finite values on Panda."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    q = controller.get_current_joint_positions()
    
    J_lin, J_ang, J_full = compute_jacobian(
        physics_client_id=client_id,
        robot_id=body_id,
        ee_link_index=controller.ee_link_index,
        arm_joint_indices=controller.arm_joint_indices,
        joint_positions=q,
    )
    
    assert J_lin.shape == (3, 7)
    assert J_ang.shape == (3, 7)
    assert J_full.shape == (6, 7)
    assert np.all(np.isfinite(J_full))


def test_jacobian_dimensions_and_finiteness_kuka(pybullet_direct):
    """Verifies Jacobian dimensions (6x7) and finite values on KUKA iiwa."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("kuka_iiwa")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    q = controller.get_current_joint_positions()
    
    J_lin, J_ang, J_full = compute_jacobian(
        physics_client_id=client_id,
        robot_id=body_id,
        ee_link_index=controller.ee_link_index,
        arm_joint_indices=controller.arm_joint_indices,
        joint_positions=q,
    )
    
    assert J_lin.shape == (3, 7)
    assert J_ang.shape == (3, 7)
    assert J_full.shape == (6, 7)
    assert np.all(np.isfinite(J_full))


def test_jacobian_numerical_finite_difference_consistency(pybullet_direct):
    """Verifies that J_linear * dq matches forward kinematics finite differences delta_x."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    q0 = np.array(controller.get_current_joint_positions())
    
    J_lin, _, _ = compute_jacobian(
        physics_client_id=client_id,
        robot_id=body_id,
        ee_link_index=controller.ee_link_index,
        arm_joint_indices=controller.arm_joint_indices,
        joint_positions=list(q0),
    )
    
    # Small joint perturbation
    eps = 1e-5
    for j in range(len(q0)):
        dq = np.zeros(len(q0))
        dq[j] = eps
        
        # Perturb forward
        controller.reset_to_home()
        for idx, val in zip(controller.arm_joint_indices, q0 + dq):
            p.resetJointState(body_id, idx, val, physicsClientId=client_id)
        pos_plus, _ = controller.get_end_effector_pose()
        
        # Perturb backward
        controller.reset_to_home()
        for idx, val in zip(controller.arm_joint_indices, q0 - dq):
            p.resetJointState(body_id, idx, val, physicsClientId=client_id)
        pos_minus, _ = controller.get_end_effector_pose()
        
        # Numerical derivative
        num_dx = (pos_plus - pos_minus) / (2.0 * eps)
        ana_dx = J_lin[:, j]
        
        np.testing.assert_allclose(num_dx, ana_dx, atol=1e-3)
