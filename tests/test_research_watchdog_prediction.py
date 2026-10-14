"""Execution expiry, latched watchdogs, and observation-only nominal prediction."""
from dataclasses import replace
from unittest.mock import Mock

import numpy as np
import pybullet as p
import pytest

from robotics.research.contracts import Observation, ScenarioConfig, VelocityCommand
from robotics.research.plant_twin import PlantTwinHarness
from robotics.research.prediction import CommandIntent, predict_joint_motion


def config(**kwargs):
    values = dict(observation_delay_s=0, observation_jitter_s=0,
                  dropout_probability=0, command_jitter_s=0)
    values.update(kwargs)
    return replace(ScenarioConfig(), **values)


def test_expired_delivery_never_reaches_plant():
    with PlantTwinHarness(config(command_delay_s=0.08, command_lifetime_s=0.03)) as harness:
        harness.step()
        dispatch = Mock(wraps=harness._plant.controller.backend.command_joint_velocities)
        harness._plant.controller.backend.command_joint_velocities = dispatch
        harness.submit_velocity([0.1] + [0] * 6)
        for _ in range(25): harness.step()
        dispatch.assert_not_called()
        assert any(e["kind"] == "command_expired" for e in harness.events)


@pytest.mark.parametrize("lifetime,watchdog,reason", [(0.03, 0.2, "active_command_expired"),
                                                     (0.2, 0.03, "command_watchdog_timeout")])
def test_active_command_stop_latches_and_reset_does_not_replay(lifetime, watchdog, reason):
    with PlantTwinHarness(config(command_delay_s=0, command_lifetime_s=lifetime,
                                 command_watchdog_s=watchdog)) as harness:
        harness.step()
        harness.submit_velocity([0.1] + [0] * 6)
        for _ in range(12): harness.step()
        assert harness.controller_view().execution_fault == reason
        with pytest.raises(RuntimeError, match="latched"):
            harness.submit_velocity([0] * 7)
        assert len(harness._commands) == 0
        harness.reset_execution_fault()
        assert harness.controller_view().execution_fault is None
        assert harness._active_command is None
        harness.step()
        assert harness._active_command is None


def test_telemetry_timeout_uses_capture_age_and_cancels_pending():
    with PlantTwinHarness(config(command_delay_s=0.2, telemetry_watchdog_s=0.04)) as harness:
        harness.step()
        harness.submit_velocity([0.1] + [0] * 6)
        # Simulate all subsequent sensing lost while the plant keeps running.
        harness._capture = lambda position: None
        for _ in range(15): harness.step()
        assert harness.controller_view().execution_fault == "telemetry_timeout"
        assert len(harness._commands) == 0
        with pytest.raises(RuntimeError, match="fresh"):
            harness.reset_execution_fault()
        assert any(e.get("cancelled_commands") == 1 for e in harness.events)


def test_startup_missing_telemetry_and_nonzero_command_rejection():
    scenario = replace(config(startup_timeout_s=0.03), dropout_probability=1)
    with PlantTwinHarness(scenario) as harness:
        with pytest.raises(RuntimeError, match="fresh"):
            harness.submit_velocity([0.1] + [0] * 6)
        harness.submit_velocity([0] * 7)
        for _ in range(10): harness.step()
        assert harness.controller_view().execution_fault == "startup_telemetry_timeout"


def test_watchdog_detects_commands_that_never_arrive():
    with PlantTwinHarness(config(command_delay_s=1, command_watchdog_s=0.03)) as harness:
        harness.step()
        harness.submit_velocity([0.1] + [0] * 6)
        for _ in range(12): harness.step()
        assert harness.controller_view().execution_fault == "command_watchdog_timeout"
        assert len(harness._commands) == 0


def test_fresh_frames_do_not_automatically_clear_fault():
    with PlantTwinHarness(config(command_delay_s=0, command_lifetime_s=0.02)) as harness:
        harness.step()
        harness.submit_velocity([0.1] + [0] * 6)
        for _ in range(30): harness.step()
        assert harness._fresh_telemetry()
        assert harness.controller_view().execution_fault == "active_command_expired"


def test_halt_failure_remains_visible():
    scenario = replace(config(startup_timeout_s=0.01), dropout_probability=1)
    with PlantTwinHarness(scenario) as harness:
        harness._plant.controller.backend.halt_motion = Mock(side_effect=RuntimeError("lost"))
        with pytest.raises(RuntimeError, match="halt failed"):
            for _ in range(10): harness.step()
        assert harness.controller_view().execution_fault == "halt_failed"


def test_piecewise_prediction_includes_commands_before_and_after_capture():
    observation = Observation(0, 0.1, 0.2, ("joint",), (0.5,), (0, 0, 0))
    first = VelocityCommand(0, 0, (1,), 0.4)
    second = VelocityCommand(1, 0.15, (-1,), 0.5)
    history = (CommandIntent(first, 0.05, 0.04, 0.06), CommandIntent(second, 0.25, 0.24, 0.26))
    states = predict_joint_motion(observation, history, 0.2, 0.4, 0.1, (-2,), (2,))
    np.testing.assert_allclose([s.joint_positions[0] for s in states], [0.6, 0.6, 0.5, 0.4, 0.4])
    assert states[-1].time_s == pytest.approx(0.6)


def test_prediction_marks_limits_without_silent_clipping():
    observation = Observation(0, 0, 0, ("joint",), (0.9,), (0, 0, 0))
    command = VelocityCommand(0, 0, (1,), 1)
    states = predict_joint_motion(observation, (CommandIntent(command, 0, 0, 0),),
                                  0, 0.2, 0.1, (-1,), (1,))
    assert states[-1].joint_limit_violation
    assert states[-1].joint_positions[0] == pytest.approx(1.1)
    with pytest.raises(ValueError):
        predict_joint_motion(None, (), 0, 0.1, 0.01, (-1,), (1,))


def test_prediction_includes_nominal_command_watchdog_deadline():
    observation = Observation(0, 0, 0, ("joint",), (0,), (0, 0, 0))
    command = VelocityCommand(0, 0, (1,), 1)
    intent = CommandIntent(command, 0, 0, 0, watchdog_deadline_s=0.05)
    states = predict_joint_motion(observation, (intent,), 0, 0.2, 0.1, (-1,), (1,))
    assert states[-1].joint_positions[0] == pytest.approx(0.05)


def test_prediction_does_not_resume_after_a_forecast_expiry_gap():
    observation = Observation(0, 0, 0, ("joint",), (0,), (0, 0, 0))
    first = CommandIntent(VelocityCommand(0, 0, (1,), 0.05), 0, 0, 0)
    late = CommandIntent(VelocityCommand(1, 0, (1,), 1), 0.1, 0.1, 0.1)
    states = predict_joint_motion(observation, (first, late), 0, 0.2, 0.01, (-1,), (1,))
    assert states[-1].joint_positions[0] == pytest.approx(0.05)


def test_prediction_forecasts_missing_initial_delivery():
    observation = Observation(0, 0, 0, ("joint",), (0,), (0, 0, 0))
    intent = CommandIntent(VelocityCommand(0, 0, (1,), 1), 0.2, 0.2, 0.2,
                           first_delivery_deadline_s=0.05)
    states = predict_joint_motion(observation, (intent,), 0, 0.4, 0.01, (-1,), (1,))
    assert states[-1].joint_positions[0] == 0


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa", "xarm6"])
def test_prediction_preserves_worlds_and_cannot_see_hidden_truth(robot):
    with PlantTwinHarness(config(robot=robot, command_delay_s=0.04)) as harness:
        harness.step()
        harness.submit_velocity([0.1] + [0] * (harness.controller_view().dof - 1))
        initial = harness.predict()
        twin_before = harness._twin.controller.get_current_joint_positions()
        plant = harness._plant
        index = plant.controller.arm_joint_indices[0]
        p.resetJointState(plant.robot_id, index, 0.3, physicsClientId=plant.client_id)
        truth = plant.controller.get_current_joint_positions()
        # Prevent accidental telemetry access during prediction.
        plant.controller.backend.get_joint_state = Mock(side_effect=AssertionError("truth leaked"))
        assert harness.predict() == initial
        assert harness._twin.controller.get_current_joint_positions() == twin_before
        actual = p.getJointState(plant.robot_id, index, physicsClientId=plant.client_id)[0]
        assert actual == truth[0]
        assert len(initial[0].joint_positions) == harness.controller_view().dof
        assert len(initial[0].end_effector_position_m) == 3


def test_nominal_prediction_does_not_use_sampled_delivery_time():
    def prediction(seed):
        scenario = config(seed=seed, command_delay_s=0.04, command_jitter_s=0.03)
        with PlantTwinHarness(scenario) as harness:
            harness.step()
            harness.submit_velocity([0.1] + [0] * 6)
            return harness.controller_view().command_history[-1], harness.events[-1]["scheduled_time_s"]
    first, arrival = prediction(0)
    second, other_arrival = prediction(1)
    assert first == second
    assert arrival != other_arrival


def test_reset_cancels_pending_commands_even_without_a_fault():
    with PlantTwinHarness(config(command_delay_s=0.1)) as harness:
        harness.step()
        harness.submit_velocity([0.1] + [0] * 6)
        harness.reset_execution_fault()
        assert len(harness._commands) == 0
        future = harness.predict(horizon_s=0.2)
        np.testing.assert_allclose(future[0].joint_positions, future[-1].joint_positions)
        for _ in range(30): harness.step()
        assert harness._active_command is None


def test_reset_halt_failure_latches_and_invalidates_prediction():
    with PlantTwinHarness(config()) as harness:
        harness.step()
        harness._plant.controller.backend.halt_motion = Mock(side_effect=RuntimeError("lost"))
        with pytest.raises(RuntimeError, match="reset software halt failed"):
            harness.reset_execution_fault()
        assert harness.controller_view().execution_fault == "halt_failed"
        with pytest.raises(RuntimeError, match="unconfirmed"):
            harness.predict()


def test_refresh_at_expiry_boundary_does_not_latch_old_command():
    with PlantTwinHarness(config(command_delay_s=0, command_lifetime_s=0.025)) as harness:
        harness.step()
        start = harness.time_s
        harness.submit_velocity([0.1] + [0] * 6)
        for _ in range(6): harness.step()
        assert harness.time_s == pytest.approx(start + 0.025)
        harness.submit_velocity([0.05] + [0] * 6)
        harness.step()
        assert harness.controller_view().execution_fault is None


@pytest.mark.parametrize("field", ["command_lifetime_s", "command_watchdog_s", "telemetry_watchdog_s", "startup_timeout_s"])
def test_deadline_configuration_rejects_zero(field):
    with pytest.raises(ValueError):
        config(**{field: 0})
