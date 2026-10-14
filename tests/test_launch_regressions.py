"""Regression coverage for actual application startup and simulator lifecycle."""
from unittest.mock import Mock
from types import SimpleNamespace

import numpy as np
import pybullet as p
import pytest

from config.settings import get_default_config, StateMachineConfig, WorkspaceConfig
from main import VisionRobotTwinApp
from robotics.simulator import PyBulletSimulator
from robotics.state_machine import RoboticStateMachine, RobotState


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa", "xarm6"])
@pytest.mark.parametrize("scene", ["standard", "obstacles"])
def test_real_simulator_scene_startup(robot, scene):
    config = get_default_config()
    config.robot_name = robot
    config.scene_type = scene
    sim = PyBulletSimulator(config, headless=True)
    try:
        assert sim.is_connected()
        assert len(sim.obstacle_ids) == (2 if scene == "obstacles" else 0)
        sim.step()
    finally:
        sim.close()
        sim.close()
    assert not sim.is_connected()


def test_failed_scene_initialization_releases_client(monkeypatch):
    clients = []
    original = PyBulletSimulator._initialize_pybullet

    def initialize(sim):
        original(sim)
        clients.append(sim.client_id)

    def fail(sim):
        raise ValueError("scene load failed")

    monkeypatch.setattr(PyBulletSimulator, "_initialize_pybullet", initialize)
    monkeypatch.setattr(PyBulletSimulator, "_load_environment", fail)
    with pytest.raises(ValueError, match="scene load failed"):
        PyBulletSimulator(get_default_config(), headless=True)
    assert not p.isConnected(clients[0])


@pytest.mark.parametrize("during_frame", [False, True])
def test_window_close_exits_cleanly(during_frame):
    config = get_default_config()
    config.camera.synthetic_mode = True
    app = VisionRobotTwinApp(config, headless_sim=True)
    if during_frame:
        def close_window(**kwargs):
            p.disconnect(app.simulator.client_id)
            raise RuntimeError("Cannot read joint state from disconnected PyBullet backend")
        app.process_frame = close_window
    else:
        p.disconnect(app.simulator.client_id)
    app.run()
    assert not app.simulator.is_connected()


def test_connected_runtime_error_is_not_hidden():
    config = get_default_config()
    config.camera.synthetic_mode = True
    app = VisionRobotTwinApp(config, headless_sim=True)
    app.process_frame = Mock(side_effect=ValueError("real runtime failure"))
    with pytest.raises(ValueError, match="real runtime failure"):
        app.run()
    assert not app.simulator.is_connected()


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa", "xarm6"])
def test_resolved_rate_respects_guard_horizon_near_limits(robot):
    config = get_default_config()
    config.robot_name = robot
    sim = PyBulletSimulator(config, headless=True)
    try:
        ctrl = sim.controller
        q = np.asarray(ctrl.get_current_joint_positions())
        q[0] = ctrl.model.arm_upper_limits[0] - 0.0001
        for index, value in zip(ctrl.arm_joint_indices, q):
            p.resetJointState(sim.robot_id, index, value, physicsClientId=sim.client_id)
        target, _ = ctrl.get_end_effector_pose()
        sim.resolved_rate_controller.max_joint_vel = 0.001
        velocity, _ = sim.resolved_rate_controller.compute_step(
            np.asarray(target) + [10, 10, 10], target_linear_velocity=np.array([100., 100., 100.]))
        predicted = q + velocity * sim.execution_guard.velocity_horizon_s
        assert np.all(predicted <= ctrl.model.arm_upper_limits)
        assert np.all(predicted >= ctrl.model.arm_lower_limits)
        assert velocity[0] < 0  # Move back inside the margin, even for an outward target.
        assert np.all(np.abs(velocity) <= 0.001)
    finally:
        sim.close()


def test_auto_demo_repositions_object_at_each_cycle_start():
    config = StateMachineConfig(auto_demo=True)
    fsm = RoboticStateMachine(config, WorkspaceConfig())
    fsm.set_mode("AUTO")
    sync = Mock()
    for _ in range(2):
        fsm.update_auto_mode(np.array([0.45, 0., 0.3]), None, None,
                             on_targets_stabilized_fn=sync)
        assert fsm.state == RobotState.APPROACH
        fsm.reset()
    assert sync.call_count == 2
    np.testing.assert_allclose(sync.call_args.args[0][:2], [0.45, -0.20])


def test_auto_demo_completes_two_physical_pick_cycles(monkeypatch):
    import robotics.state_machine as state_machine

    clock = [0.0]
    monkeypatch.setattr(state_machine, "time", SimpleNamespace(time=lambda: clock[0]))
    config = get_default_config()
    config.camera.synthetic_mode = True
    config.state_machine.auto_demo = True
    config.mode = "auto"
    app = VisionRobotTwinApp(config, headless_sim=True)
    completed = 0
    previous = app.state_machine.state
    try:
        for _ in range(1500):
            clock[0] += 1. / 30.
            result = app.process_frame(wall_dt=1. / 30.)
            assert result.success
            assert app.state_machine.state != RobotState.ERROR
            if previous == RobotState.RETURN_HOME and app.state_machine.state == RobotState.SEARCH:
                completed += 1
            previous = app.state_machine.state
            if completed == 2:
                break
        assert completed == 2
    finally:
        app.cleanup()
