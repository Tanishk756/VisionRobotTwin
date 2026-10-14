"""Independent information paths, event timing, estimation, and replay."""
from dataclasses import asdict, replace
import json

import numpy as np
import pybullet as p
import pytest

from robotics.research.contracts import Observation, ScenarioConfig, VelocityCommand
from robotics.research.estimation import ConstantVelocityEstimator
from robotics.research.plant_twin import PlantTwinHarness
from robotics.research.transport import EventQueue


def observation(sequence=0, capture=0.0, delivery=0.1, position=(0, 0, 0)):
    return Observation(sequence, capture, delivery, ("joint",), (0.0,), position)


def test_delivery_order_ties_quantization_and_monotonic_clock():
    queue = EventQueue()
    queue.schedule(0.15, "late")
    queue.schedule(0.1, "first")
    queue.schedule(0.1, "second")
    assert not queue.drain(0.09)
    delivered = queue.drain(0.12)
    assert [item.payload for item in delivered] == ["first", "second"]
    assert delivered[0].scheduled_time_s == 0.1
    assert delivered[0].actual_time_s == 0.12
    with pytest.raises(ValueError): queue.drain(0.1)
    with pytest.raises(ValueError): queue.schedule(0.1, "past")
    assert queue.drain(0.2)[0].payload == "late"


@pytest.mark.parametrize("kwargs", [dict(duration_s=float("nan")), dict(physics_hz=0),
                                    dict(sensor_stride=1.2), dict(seed=-1),
                                    dict(dropout_probability=float("nan")),
                                    dict(plant_velocity_gain=1.2), dict(command_delay_s=-1)])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError): ScenarioConfig(**kwargs)


def test_messages_own_immutable_finite_copies():
    values = [0.1]
    message = VelocityCommand(0, 0, values)
    values[0] = 9
    assert message.velocities_rad_s == (0.1,)
    with pytest.raises(ValueError): VelocityCommand(0, 0, [float("inf")])
    with pytest.raises(ValueError): observation(capture=1, delivery=0)


def test_estimator_age_uncertainty_and_reordered_frames():
    estimator = ConstantVelocityEstimator()
    assert estimator.predict(0) is None
    assert estimator.update(observation())
    immediate = estimator.predict(0.1)
    later = estimator.predict(0.5)
    assert later.observation_age_s == 0.5
    assert np.trace(later.covariance) > np.trace(immediate.covariance)
    assert estimator.predict(0.1) == immediate  # prediction has no update side effect
    assert not estimator.update(observation())
    assert estimator.update(observation(1, 0.2, 0.3, (0.2, 0, 0)))
    assert estimator.predict(0.4).velocity_m_s[0] > 0
    assert np.linalg.eigvalsh(estimator.predict(0.4).covariance).min() >= -1e-12
    with pytest.raises(ValueError): estimator.predict(0.1)


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa", "xarm6"])
def test_clients_isolate_truth_and_observations_arrive_later(robot):
    config = ScenarioConfig(robot=robot, observation_delay_s=0.05, observation_jitter_s=0,
                            joint_noise_std_rad=0, obstacle_noise_std_m=0, dropout_probability=0)
    with PlantTwinHarness(config) as harness:
        plant, twin = harness._plant, harness._twin  # evaluator/test access only
        assert plant.client_id != twin.client_id
        assert plant.execution_guard is None
        prior = twin.controller.get_current_joint_positions()
        index = plant.controller.arm_joint_indices[0]
        p.resetJointState(plant.robot_id, index, prior[0] + 0.1, physicsClientId=plant.client_id)
        assert harness.controller_view().observation is None
        assert twin.controller.get_current_joint_positions() == prior
        harness.step()
        assert harness.controller_view().observation is None
        # A twin FK candidate query must preserve both worlds.
        plant_before = plant.controller.get_current_joint_positions()
        twin.kinematics_provider.compute_fk(np.array(prior) + 0.01)
        assert plant.controller.get_current_joint_positions() == plant_before
        assert twin.controller.get_current_joint_positions() == prior
        for _ in range(12): harness.step()
        delivered = harness.controller_view().observation
        assert delivered.capture_time_s == 0
        assert delivered.delivery_time_s == pytest.approx(0.05)
        assert delivered.joint_positions[0] == pytest.approx(prior[0] + 0.1)
        assert twin.controller.get_current_joint_positions() == list(delivered.joint_positions)
        assert not hasattr(harness.controller_view(), "plant")
    assert not p.isConnected(plant.client_id)
    assert not p.isConnected(twin.client_id)
    harness.close()
    with pytest.raises(RuntimeError): harness.step()


def test_delayed_commands_stale_commands_and_dropout():
    config = ScenarioConfig(command_delay_s=0.02, command_jitter_s=0,
                            dropout_probability=1, observation_jitter_s=0)
    with PlantTwinHarness(config) as harness:
        velocity = [0.1] + [0] * (harness.controller_view().dof - 1)
        harness.submit_velocity(velocity)
        for _ in range(5):
            assert harness.step().applied_command_sequence is None
        record = harness.step()
        assert record.applied_command_sequence == 0
        assert record.time_s >= 0.02
        assert harness.controller_view().observation is None
        # A reordered earlier command must never overwrite the newer command.
        harness._commands.schedule(harness.time_s, VelocityCommand(0, 0, velocity))
        harness.step()
        assert harness.events[-1]["kind"] == "command_stale"
        with pytest.raises(ValueError): harness.submit_velocity([100] * len(velocity))


def test_stale_observation_cannot_roll_twin_back():
    config = ScenarioConfig(dropout_probability=1)
    with PlantTwinHarness(config) as harness:
        names = harness._twin.resolved_model.arm_joint_names
        q = tuple(harness._twin.controller.get_current_joint_positions())
        newer = Observation(2, 0, 0, names, q, (1, 2, 3))
        older = replace(newer, sequence=1, obstacle_position_m=(9, 9, 9))
        harness._observations.schedule(0, newer)
        harness._observations.schedule(0, older)
        harness.step()
        assert harness.controller_view().observation == newer
        assert any(e["kind"] == "observation_stale" for e in harness.events)


def test_truth_evaluator_detects_collision_and_terminal_motion():
    config = ScenarioConfig(obstacle_velocity_m_s=(0, 0, 0), dropout_probability=1)
    with PlantTwinHarness(config) as harness:
        pos, _ = harness._plant.controller.get_end_effector_pose()
        p.resetBasePositionAndOrientation(harness._plant_obstacle, pos, (0, 0, 0, 1),
                                          physicsClientId=harness._plant.client_id)
        record = harness.evaluate()
        assert record.in_collision
        assert record.obstacle_clearance_m < 0
    config = ScenarioConfig(dropout_probability=1)
    with PlantTwinHarness(config) as harness:
        harness.step()
        expected = np.array(config.obstacle_origin_m) + harness.time_s * np.array(config.obstacle_velocity_m_s)
        np.testing.assert_allclose(harness.evaluate().plant_obstacle_position_m, expected)


def test_replay_same_seed_and_different_seed():
    def trace(seed):
        with PlantTwinHarness(ScenarioConfig(seed=seed)) as harness:
            result = []
            for _ in range(30):
                harness.submit_velocity([0.05] + [0] * (harness.controller_view().dof - 1))
                result.append(asdict(harness.step()))
            return result, harness.events
    first, events = trace(3)
    repeated, repeated_events = trace(3)
    assert json.dumps(first, sort_keys=True) == json.dumps(repeated, sort_keys=True)
    assert events == repeated_events
    _, different_events = trace(4)
    assert events != different_events


def test_mass_mismatch_is_plant_only_and_shutdown_preserves_other_clients():
    other = p.connect(p.DIRECT)
    try:
        with PlantTwinHarness(ScenarioConfig(plant_mass_scale=1.4)) as harness:
            plant, twin = harness._plant, harness._twin
            index = plant.controller.arm_joint_indices[0]
            plant_mass = p.getDynamicsInfo(plant.robot_id, index, physicsClientId=plant.client_id)[0]
            twin_mass = p.getDynamicsInfo(twin.robot_id, index, physicsClientId=twin.client_id)[0]
            assert plant_mass == pytest.approx(1.4 * twin_mass)
        assert p.isConnected(other)
    finally:
        p.disconnect(other)


def test_runner_records_terminal_state_and_preserves_output(tmp_path):
    from tools.run_plant_twin_experiment import run
    config = ScenarioConfig(duration_s=0.05, dropout_probability=0,
                            observation_delay_s=0, observation_jitter_s=0)
    summary = run(config, tmp_path)
    assert summary["records"] == 13
    records = [json.loads(line) for line in (tmp_path / "trace.jsonl").read_text().splitlines()]
    assert records[-1]["time_s"] == pytest.approx(0.05)
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["config"]["robot"] == "panda"
    assert "robotics/research/plant_twin.py" in manifest["source_files_sha256"]
    with pytest.raises(ValueError, match="empty"):
        run(config, tmp_path)


def test_different_command_delays_do_not_change_sensor_noise_stream():
    def captures(delay):
        config = ScenarioConfig(command_delay_s=delay, dropout_probability=0)
        with PlantTwinHarness(config) as harness:
            for _ in range(25):
                # Zero commands affect scheduling but not the obstacle measurements.
                harness.submit_velocity([0] * harness.controller_view().dof)
                harness.step()
            return [(e["sequence"], e["obstacle_position_m"]) for e in harness.events
                    if e["kind"] == "observation_captured"]
    assert captures(0) == captures(0.1)
