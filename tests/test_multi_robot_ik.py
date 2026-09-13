"""Tests for Generic Inverse Kinematics Solver across supported manipulators."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.inverse_kinematics import GenericIKSolver, PandaIKSolver, IKStatus, IKResult


@pytest.fixture
def pybullet_direct():
    """Initializes a direct PyBullet physics simulation fixture."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)
    yield client_id
    p.disconnect(physicsClientId=client_id)


def test_generic_ik_solver_panda(pybullet_direct):
    """Tests GenericIKSolver on 7-DoF Franka Emika Panda."""
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
    
    controller = GenericRobotController(client_id, body_id, spec)
    lows, highs, ranges, rests = controller.get_joint_limits()
    
    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )
    
    # Solve for reachable target
    target_pos = np.array([0.45, 0.0, 0.35])
    target_orn = np.array([1.0, 0.0, 0.0, 0.0])
    
    res = ik_solver.solve(target_pos, target_orn)
    assert res.success is True
    assert res.status == IKStatus.SOLUTION_RETURNED
    assert len(res.joint_positions) == 7
    assert res.solve_time_ms is not None and res.solve_time_ms >= 0.0
    
    # Test FK consistency: command joints and verify end-effector position
    controller.set_arm_joint_positions(res.joint_positions)
    for _ in range(60):
        p.stepSimulation(physicsClientId=client_id)
    
    ee_pos, ee_orn = controller.get_end_effector_pose()
    pos_err = np.linalg.norm(ee_pos - target_pos)
    assert pos_err < 0.03  # Convergence within 30 mm


def test_generic_ik_solver_kuka(pybullet_direct):
    """Tests GenericIKSolver on 7-DoF KUKA LBR iiwa."""
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
    
    controller = GenericRobotController(client_id, body_id, spec)
    lows, highs, ranges, rests = controller.get_joint_limits()
    
    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )
    
    # Solve for reachable target
    target_pos = np.array([0.40, 0.0, 0.40])
    target_orn = np.array([0.0, 1.0, 0.0, 0.0])
    
    res = ik_solver.solve(target_pos, target_orn)
    assert res.success is True
    assert res.status == IKStatus.SOLUTION_RETURNED
    assert len(res.joint_positions) == 7
    assert res.solve_time_ms is not None
    
    # FK consistency check
    controller.set_arm_joint_positions(res.joint_positions)
    for _ in range(60):
        p.stepSimulation(physicsClientId=client_id)
    
    ee_pos, ee_orn = controller.get_end_effector_pose()
    pos_err = np.linalg.norm(ee_pos - target_pos)
    assert pos_err < 0.04


def test_generic_ik_solver_unreachable_and_nan_rejection(pybullet_direct):
    """Tests reachability boundaries and NaN protection."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    lows, highs, ranges, rests = controller.get_joint_limits()
    
    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
    )
    
    # Unreachable target (2.5 meters away)
    res_unreach = ik_solver.solve([2.5, 0.0, 0.0])
    assert res_unreach.success is False
    assert res_unreach.status == IKStatus.UNREACHABLE
    
    # NaN target
    res_nan = ik_solver.solve([np.nan, 0.0, 0.5])
    assert res_nan.success is False
    assert res_nan.status == IKStatus.INVALID_TARGET
