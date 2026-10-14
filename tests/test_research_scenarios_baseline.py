"""Scenario validation, observation-only whole-arm queries, and baseline recovery."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import json
from unittest.mock import Mock

import numpy as np
import pybullet as p
import pytest

from robotics.research.baseline import FixedHorizonStopBaseline
from robotics.research.contracts import Observation, ScenarioConfig
from robotics.research.plant_twin import MotionClearance, PlantTwinHarness
from robotics.research.scenarios import ResearchScenario, load_scenario

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("name", ["crossing", "sudden-approach", "occluded-crossing", "narrow-passage"])
def test_scenario_files_load_with_explicit_development_split(name):
    scenario = load_scenario(ROOT / f"config/research_scenarios/{name}-v1.json")
    assert scenario.schema_version == 1
    assert scenario.split == "development"
    np.testing.assert_allclose(scenario.position(0, (1, 2, 3)), np.array(scenario.waypoints[0][1:]) + (1, 2, 3))


@pytest.mark.parametrize("change", [dict(schema_version=2), dict(waypoints=((0.1, 0, 0, 0),)),
                                      dict(waypoints=((0, 0, 0, 0), (0, 1, 0, 0))),
                                      dict(occlusions=((0.2, 0.1),)), dict(fixtures=((0, 0, 0, -1),))])
def test_invalid_scenario_rejected(change):
    scenario = load_scenario(ROOT / "config/research_scenarios/crossing-v1.json")
    with pytest.raises(ValueError): replace(scenario, **change)


def test_loader_rejects_unknown_keys(tmp_path):
    data = json.loads((ROOT / "config/research_scenarios/crossing-v1.json").read_text())
    data["typo"] = True
    path = tmp_path / "bad.json"; path.write_text(json.dumps(data))
    with pytest.raises(ValueError): load_scenario(path)


def test_motion_interpolation_and_occlusion_boundaries():
    scenario = ResearchScenario("unit", "development", ScenarioConfig(), "world",
                                ((0, 0, 0, 0), (1, 1, 0, 0)), ((0.2, 0.4),))
    np.testing.assert_allclose(scenario.position(0.5), (0.5, 0, 0))
    np.testing.assert_allclose(scenario.position(2), (1, 0, 0))
    assert scenario.occluded(0.2)
    assert not scenario.occluded(0.4)


def test_occlusion_suppresses_capture_and_nominal_anchor_has_no_plant_truth():
    config = ScenarioConfig(observation_delay_s=0, observation_jitter_s=0, dropout_probability=0,
                            joint_noise_std_rad=0, obstacle_noise_std_m=0, sensor_stride=1)
    scenario = ResearchScenario("unit", "development", config, "nominal_home_ee",
                                ((0, 0, 0.3, 0), (1, 0, -0.3, 0)), ((0, 0.02),))
    with PlantTwinHarness(config, scenario) as harness:
        for _ in range(5): harness.step()
        assert harness.controller_view().observation is None
        harness.step()
        assert harness.controller_view().observation is not None
        assert sum(e["kind"] == "observation_occluded" for e in harness.events) == 5


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa", "xarm6"])
def test_query_preserves_state_rng_history_and_cannot_see_hidden_obstacle(robot):
    config = ScenarioConfig(robot=robot, observation_delay_s=0, observation_jitter_s=0, dropout_probability=0)
    with PlantTwinHarness(config) as harness:
        harness.step()
        view = harness.controller_view()
        velocity = (0.05,) + (0,) * (view.dof - 1)
        before = harness._twin.controller.get_current_joint_positions()
        rng = json.dumps(harness._command_rng.bit_generator.state, sort_keys=True)
        first = harness.check_candidate_motion(velocity)
        plant = harness._plant
        position, _ = plant.controller.get_end_effector_pose()
        p.resetBasePositionAndOrientation(harness._plant_obstacle, position, (0, 0, 0, 1), physicsClientId=plant.client_id)
        plant.controller.backend.get_joint_state = Mock(side_effect=AssertionError("plant truth leak"))
        assert harness.check_candidate_motion(velocity) == first
        assert harness.controller_view() == view
        assert harness._twin.controller.get_current_joint_positions() == before
        assert json.dumps(harness._command_rng.bit_generator.state, sort_keys=True) == rng
        with pytest.raises(ValueError): harness.check_candidate_motion(velocity, environment_margin_m=0.6)


def test_whole_arm_query_detects_contact_away_from_end_effector():
    config = ScenarioConfig(observation_delay_s=0, observation_jitter_s=0, dropout_probability=0)
    with PlantTwinHarness(config) as harness:
        harness.step()
        twin = harness._twin
        # Mid-arm link, intentionally far from the terminal grasp frame.
        middle = p.getLinkState(twin.robot_id, 2, physicsClientId=twin.client_id)[4]
        ee = twin.controller.get_end_effector_pose()[0]
        assert np.linalg.norm(ee - middle) > config.obstacle_radius_m
        observed = replace(harness.controller_view().observation, sequence=100,
                           capture_time_s=harness.time_s, delivery_time_s=harness.time_s,
                           obstacle_position_m=middle)
        harness._observations.schedule(harness.time_s, observed)
        harness.step()
        result = harness.check_candidate_motion((0,) * 7, environment_margin_m=0)
        assert not result.clear
        assert result.minimum_environment_clearance_m < 0


def view(sequence=0, time_s=0, fault=None):
    observation = Observation(sequence, time_s, time_s, ("joint",), (0,), (0, 0, 0))
    return SimpleNamespace(dof=1, observation=observation, time_s=time_s, execution_fault=fault)


def test_baseline_hold_hysteresis_unique_observations_and_recovery():
    baseline = FixedHorizonStopBaseline(minimum_hold_s=0.1, recovery_observations=2)
    blocked = Mock(return_value=MotionClearance(False, 0, 0.1, False, 6))
    assert baseline.select(view(), (0.1,), blocked).state == "HOLD"
    clear = Mock(return_value=MotionClearance(True, 0.1, 0.1, False, 6))
    assert baseline.select(view(1, 0.1), (0.1,), clear).state == "HOLD"
    assert baseline.select(view(1, 0.11), (0.1,), clear).state == "HOLD"
    assert clear.call_args.kwargs["environment_margin_m"] == pytest.approx(0.025)
    assert baseline.select(view(2, 0.12), (0.1,), clear).state == "TRACK"


def test_baseline_query_failure_and_execution_fault_latch():
    baseline = FixedHorizonStopBaseline()
    failed = Mock(side_effect=RuntimeError("query unavailable"))
    result = baseline.select(view(), (0.1,), failed)
    assert result.state == "FAULT" and result.velocities_rad_s == (0,)
    clear = Mock(return_value=MotionClearance(True, 1, 1, False, 6))
    assert baseline.select(view(1, 0.1), (0.1,), clear).state == "FAULT"
    clear.assert_not_called()
    baseline.reset()
    assert baseline.select(view(fault="telemetry_timeout"), (0.1,), clear).state == "FAULT"


def test_static_fixtures_are_known_map_and_evaluated():
    config = ScenarioConfig(observation_delay_s=0, observation_jitter_s=0, dropout_probability=0)
    scenario = ResearchScenario("fixture", "development", config, "nominal_home_ee",
                                ((0, 0, 0.4, 0),), fixtures=((0, 0, 0, 0.05),))
    with PlantTwinHarness(config, scenario) as harness:
        harness.step()
        assert not harness.check_candidate_motion((0,) * 7).clear
        assert harness.evaluate().in_collision


def test_scenario_runner_manifest_and_baseline_logs(tmp_path):
    from tools.run_plant_twin_experiment import run
    config = ScenarioConfig(duration_s=0.1, observation_delay_s=0, observation_jitter_s=0, dropout_probability=0)
    scenario = ResearchScenario("unit", "development", config, "world", ((0, 0.5, 0.4, 0.5),))
    summary = run(config, tmp_path, scenario, "fixed-horizon")
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["scenario"]["schema_version"] == 1
    assert manifest["comparator"]["name"] == "fixed-horizon"
    assert summary["control_period_s"] == config.sensor_stride / config.physics_hz
    assert len(json.loads((tmp_path / "decisions.json").read_text())) == 3


def test_comparison_records_both_methods_and_failures(tmp_path, monkeypatch):
    from tools import compare_research_baselines as module
    def trial(config, output, scenario, mode, **kwargs):
        if mode == "fixed-horizon":
            raise RuntimeError("deliberate test failure")
        return {"baseline": mode, "task_completed": False, "contact_episodes": 1}
    monkeypatch.setattr(module, "run", trial)
    paths = [ROOT / "config/research_scenarios/crossing-v1.json"]
    result = module.compare(paths, ["panda"], [0], tmp_path)
    assert result["expected_trials"] == 2
    assert result["completed_trials"] == 1
    assert len(result["errors"]) == 1
    assert result["trials"][0]["contact_episodes"] == 1
    assert (tmp_path / "comparison.csv").is_file()
