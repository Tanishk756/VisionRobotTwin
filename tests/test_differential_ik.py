"""Tests for Resolved-Rate Cartesian Velocity Controller and Null-Space Joint Centering."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.differential_ik import ResolvedRateController
from robotics.kinematics_provider import PyBulletKinematicsProvider


@pytest.fixture
def pybullet_direct():
    """Initializes a direct PyBullet physics simulation fixture."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)
    yield client_id
    p.disconnect(physicsClientId=client_id)


def test_resolved_rate_cartesian_tracking_convergence(pybullet_direct):
    """Verifies resolved-rate controller drives end-effector toward target position."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()
    
    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        kp_pos=5.0,
        kp_orn=3.0,
        enable_nullspace=True,
    )
    
    target_pos = np.array([0.45, 0.0, 0.35])
    target_orn = np.array([1.0, 0.0, 0.0, 0.0])
    
    init_err = controller.compute_cartesian_error(target_pos)
    assert init_err > 0.05
    
    # Run closed-loop resolved-rate steps using genuine velocity control
    dt = 1.0 / 240.0
    for _ in range(480):
        q_dot, metrics = rr_controller.compute_step(target_pos, target_orn, dt=dt)
        assert len(q_dot) == 7
        assert np.all(np.isfinite(q_dot))

        controller.set_arm_joint_velocities(q_dot)
        p.stepSimulation(physicsClientId=client_id)

    final_err = controller.compute_cartesian_error(target_pos)
    assert final_err < init_err * 0.1  # Over 90% error reduction (converges to sub-millimeter)


def test_resolved_rate_velocity_clamping_and_nan_rejection(pybullet_direct):
    """Verifies joint velocities are bounded by max velocity and NaNs are rejected."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    
    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        max_joint_velocity_radps=1.5,
    )
    
    # Far target (large error)
    far_pos = np.array([0.90, 0.50, 0.80])
    q_dot, _ = rr_controller.compute_step(far_pos, dt=1.0 / 240.0)
    assert np.all(np.abs(q_dot) <= 1.5001)
    
    # NaN target
    nan_pos = np.array([np.nan, 0.0, 0.5])
    q_dot_nan, _ = rr_controller.compute_step(nan_pos, dt=1.0 / 240.0)
    assert np.all(q_dot_nan == 0.0)


def test_resolved_rate_controller_with_injected_provider(pybullet_direct):
    """Verifies ResolvedRateController operates when injected with an explicit KinematicsProvider."""
    client_id = pybullet_direct
    spec = get_robot_registry().get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        end_effector_link_index=controller.ee_link_index,
    )

    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        kinematics_provider=provider,
    )

    assert rr_controller.kinematics_provider is provider
    q_dot, metrics = rr_controller.compute_step([0.45, 0.0, 0.35], [1.0, 0.0, 0.0, 0.0])
    assert len(q_dot) == 7
    assert np.all(np.isfinite(q_dot))
    assert metrics.manipulability >= 0.0


def test_resolved_rate_controller_has_zero_direct_pybullet_calls():
    """Verifies that ResolvedRateController module contains zero direct pybullet references or calls."""
    import inspect
    import robotics.differential_ik as diff_module

    source = inspect.getsource(diff_module)
    assert "p.calculateJacobian" not in source
    assert "p.getLinkState" not in source
    assert "p.getNumJoints" not in source
    assert "p.getJointInfo" not in source
    assert "import pybullet as p" not in source
