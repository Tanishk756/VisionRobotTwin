"""Six-axis model mapping, kinematics, control, and gripper regression checks."""
import numpy as np
import pybullet as p
import pytest
from config.settings import AppConfig
from robotics.simulator import PyBulletSimulator


@pytest.fixture(params=["xarm6", "xarm6_gripper"])
def sim(request):
    config = AppConfig()
    config.robot_name = request.param
    simulator = PyBulletSimulator(config, headless=True)
    try:
        yield simulator
    finally:
        simulator.close()


def test_canonical_mapping_and_jacobian(sim):
    model = sim.resolved_model
    assert model.dof == 6
    assert model.require_arm_native_indices() == (1, 2, 3, 4, 5, 6)
    q = np.array(sim.controller.get_current_joint_positions())
    linear, angular, jacobian = sim.kinematics_provider.compute_jacobian(q)
    assert jacobian.shape == (6, 6)
    for axis in range(6):
        delta = np.zeros(6); delta[axis] = 1e-5
        plus, _ = sim.kinematics_provider.compute_fk(q + delta)
        minus, _ = sim.kinematics_provider.compute_fk(q - delta)
        np.testing.assert_allclose(linear[:, axis], (plus-minus)/2e-5, atol=2e-5)
    np.testing.assert_allclose(sim.controller.get_current_joint_positions(), q)
    pos, orn = sim.controller.get_end_effector_pose()
    result = sim.ik_solver.solve(pos, orn)
    assert result.success


def test_position_and_velocity_execution(sim):
    q = np.array(sim.controller.get_current_joint_positions())
    target = q.copy(); target[0] += 0.03
    sim.controller.set_arm_joint_positions(target)
    for _ in range(90): sim.step()
    assert sim.execution_guard.active_fault is None
    assert abs(sim.controller.get_current_joint_positions()[0] - target[0]) < 0.005
    sim.controller.set_arm_joint_velocities([0.05, 0, 0, 0, 0, 0])
    for _ in range(30): sim.step()
    assert sim.execution_guard.active_fault is None
    sim.controller.backend.halt_motion()


def test_gripper_linkage_open_close(sim):
    if sim.gripper is None:
        assert not sim.robot_spec.capabilities.has_gripper
        return
    assert len(sim.controller.finger_joint_indices) == 6
    assert not sim.robot_spec.capabilities.supports_pick_place
    for operation, expected in [(sim.gripper.close, 0.75), (sim.gripper.open, 0.0)]:
        operation()
        for _ in range(120): sim.step()
        assert sim.execution_guard.active_fault is None
        values = [p.getJointState(sim.robot_id, i, physicsClientId=sim.client_id)[0] for i in sim.controller.finger_joint_indices]
        np.testing.assert_allclose(values, expected, atol=0.04)


def test_physics_step_detects_new_obstacle(sim):
    pos, _ = sim.controller.get_end_effector_pose()
    shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.08, physicsClientId=sim.client_id)
    obstacle = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, basePosition=pos, physicsClientId=sim.client_id)
    sim.collision_provider.obstacle_ids.append(obstacle)
    sim.step()
    assert sim.execution_guard.active_fault is not None
