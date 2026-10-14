"""Deterministic forecast timing, query restoration, and truth isolation."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json

import numpy as np
import pybullet as p
import pytest

from robotics.research.baseline import MotionAwareStopBaseline
from robotics.research.contracts import Observation, ScenarioConfig
from robotics.research.estimation import ObstacleEstimate
from robotics.research.plant_twin import PlantTwinHarness, MotionClearance
from robotics.research.scenarios import ResearchScenario


def config(robot="panda"):
    return ScenarioConfig(robot=robot, observation_delay_s=0, observation_jitter_s=0,
                          dropout_probability=0, obstacle_noise_std_m=0, joint_noise_std_rad=0)


def estimate(harness, position, velocity):
    now = harness.time_s
    capture = harness.controller_view().observation.capture_time_s
    return ObstacleEstimate(now, now - capture, tuple(position), tuple(velocity),
                            tuple(tuple(row) for row in np.eye(6)))


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa", "xarm6"])
def test_moving_forecast_detects_future_whole_arm_contact_and_restores(robot):
    with PlantTwinHarness(config(robot)) as harness:
        harness.step()
        twin = harness._twin
        middle = np.array(p.getLinkState(twin.robot_id, 2, physicsClientId=twin.client_id)[4])
        original_pose = p.getBasePositionAndOrientation(harness._twin_obstacle, physicsClientId=twin.client_id)
        original_q = twin.controller.get_current_joint_positions()
        original_view = harness.controller_view()
        forecast = estimate(harness, middle + (0.3, 0, 0), (-3, 0, 0))
        result = harness.check_candidate_motion((0,) * original_view.dof, obstacle_estimate=forecast,
                                                environment_margin_m=0, horizon_s=0.1)
        assert not result.clear
        assert result.minimum_environment_clearance_m < 0
        assert p.getBasePositionAndOrientation(harness._twin_obstacle, physicsClientId=twin.client_id) == original_pose
        assert twin.controller.get_current_joint_positions() == original_q
        assert harness.controller_view() == original_view


def test_query_uses_forecast_at_each_sample_and_restores_on_exception():
    with PlantTwinHarness(config()) as harness:
        harness.step()
        twin = harness._twin
        original = p.getBasePositionAndOrientation(harness._twin_obstacle, physicsClientId=twin.client_id)
        seen = []
        actual = twin.collision_provider.check_collision
        def query(q):
            seen.append(p.getBasePositionAndOrientation(harness._twin_obstacle, physicsClientId=twin.client_id)[0])
            return actual(q)
        twin.collision_provider.check_collision = query
        motion = estimate(harness, (0.5, 0.5, 0.5), (0.1, 0, 0))
        harness.check_candidate_motion((0,) * 7, horizon_s=0.1, step_s=0.02, obstacle_estimate=motion)
        np.testing.assert_allclose([row[0] for row in seen], 0.5 + 0.1 * np.arange(6) * 0.02)
        twin.collision_provider.check_collision = Mock(side_effect=RuntimeError("injected"))
        with pytest.raises(RuntimeError):
            harness.check_candidate_motion((0,) * 7, obstacle_estimate=motion)
        assert p.getBasePositionAndOrientation(harness._twin_obstacle, physicsClientId=twin.client_id) == original


def test_hidden_plant_truth_and_covariance_do_not_change_motion_query():
    with PlantTwinHarness(config()) as harness:
        harness.step()
        forecast = estimate(harness, (0.6, 0.5, 0.5), (0.1, 0, 0))
        first = harness.check_candidate_motion((0,) * 7, obstacle_estimate=forecast)
        plant = harness._plant
        p.resetBasePositionAndOrientation(harness._plant_obstacle, (0, 0, 0), (0, 0, 0, 1), physicsClientId=plant.client_id)
        plant.controller.backend.get_joint_state = Mock(side_effect=AssertionError("truth access"))
        different_covariance = replace(forecast, covariance=tuple(tuple(row) for row in np.eye(6) * 1e9))
        assert harness.check_candidate_motion((0,) * 7, obstacle_estimate=different_covariance) == first


@pytest.mark.parametrize("change", [dict(time_s=10), dict(observation_age_s=5),
                                      dict(position_m=(float("nan"), 0, 0)), dict(velocity_m_s=(1, 2))])
def test_bad_estimate_rejected(change):
    with PlantTwinHarness(config()) as harness:
        harness.step()
        forecast = estimate(harness, (0.5, 0.5, 0.5), (0.1, 0, 0))
        with pytest.raises(ValueError):
            harness.check_candidate_motion((0,) * 7, obstacle_estimate=replace(forecast, **change))


def view(sequence, time, position):
    observation = Observation(sequence, time - 0.02, time, ("joint",), (0,), position)
    return SimpleNamespace(time_s=time, observation=observation, dof=1, execution_fault=None)


def test_policy_estimates_motion_from_delivered_samples_and_resets():
    policy = MotionAwareStopBaseline()
    query = Mock(return_value=MotionClearance(True, 1, 1, False, 6))
    policy.select(view(0, 0.02, (0, 0, 0)), (0.1,), query)
    policy.select(view(1, 0.12, (0.05, 0, 0)), (0.1,), query)
    forecast = query.call_args.kwargs["obstacle_estimate"]
    assert forecast.velocity_m_s[0] > 0
    assert forecast.time_s == 0.12
    assert forecast.position_m[0] > 0.05  # propagation through observation age
    policy.reset()
    assert policy.state == "WAIT" and policy.last_estimate is None


def test_estimation_failure_latches_zero_motion():
    policy = MotionAwareStopBaseline()
    policy.estimator.update = Mock(side_effect=RuntimeError("failed"))
    query = Mock()
    decision = policy.select(view(0, 0.02, (0, 0, 0)), (0.1,), query)
    assert decision.state == "FAULT" and decision.velocities_rad_s == (0,)
    assert policy.select(view(1, 0.12, (0.05, 0, 0)), (0.1,), query).state == "FAULT"
    query.assert_not_called()


def test_motion_runner_records_actual_policy_estimate(tmp_path):
    from tools.run_plant_twin_experiment import run
    cfg = replace(config(), duration_s=0.1)
    scene = ResearchScenario("motion-unit", "development", cfg, "world", ((0, 0.6, 0.5, 0.5),))
    summary = run(cfg, tmp_path, scene, "motion-aware")
    assert summary["baseline"] == "motion-aware"
    decisions = json.loads((tmp_path / "decisions.json").read_text())
    assert any(d.get("obstacle_estimate") is not None for d in decisions)
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert "covariance unused" in manifest["obstacle_prediction"]


def test_comparison_accepts_motion_mode_and_shared_parameters(tmp_path, monkeypatch):
    from tools import compare_research_baselines as module
    seen = []
    def trial(config, output, scenario, mode, **kwargs):
        seen.append((mode, kwargs))
        return {"baseline": mode, "task_completed": False}
    monkeypatch.setattr(module, "run", trial)
    path = Path(__file__).resolve().parents[1] / "config/research_scenarios/crossing-v1.json"
    result = module.compare([path], ["panda"], [0], tmp_path,
                            ("none", "fixed-horizon", "motion-aware"), horizon_s=0.2, margin_m=0.03)
    assert result["completed_trials"] == result["expected_trials"] == 3
    assert all(options == {"horizon_s": 0.2, "margin_m": 0.03} for _, options in seen)
