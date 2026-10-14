"""Leakage prevention, analytic coverage scores, and clustered diagnostics."""
from dataclasses import replace
from pathlib import Path
import json

import numpy as np
import pytest
from scipy.stats import chi2

from robotics.research.calibration import collect_trial, coverage_report, fit_inflation
from robotics.research.contracts import ScenarioConfig
from robotics.research.scenarios import ResearchScenario, load_scenario
from tools.calibrate_obstacle_estimator import run_calibration

ROOT = Path(__file__).resolve().parents[1]


def scenario():
    return ResearchScenario("unit-calibration", "calibration",
                            ScenarioConfig(duration_s=0.3, observation_delay_s=0.02,
                                           observation_jitter_s=0, dropout_probability=0),
                            "world", ((0, 0.4, 0, 0.4), (0.3, 0.5, 0, 0.4)))


def test_same_seed_replay_and_prediction_age():
    first, rows = collect_trial(scenario(), 1)
    repeated, repeated_rows = collect_trial(scenario(), 1)
    assert first == repeated and rows == repeated_rows
    assert rows and all(row["observation_age_s"] >= 0.02 - 1e-12 for row in rows)
    assert all(row["time_s"] + row["horizon_s"] <= 0.3 for row in rows)
    _, other = collect_trial(scenario(), 2)
    assert rows != other


def test_complete_dropout_reports_missing_predictions():
    lost = replace(scenario(), config=replace(scenario().config, dropout_probability=1))
    summary, rows = collect_trial(lost, 0)
    assert not rows
    assert summary["missing_prediction_times"] > 0
    assert summary["random_drops"] == summary["captures"]


def test_occlusion_times_are_scored_from_last_observation():
    hidden = replace(scenario(), occlusions=((0.1, 0.25),))
    summary, rows = collect_trial(hidden, 0, (0.0,))
    assert summary["occluded_captures"] > 0
    assert any(row["observation_age_s"] > 0.1 for row in rows)


@pytest.mark.parametrize("change", [dict(split="development"), dict(frame="nominal_home_ee")])
def test_wrong_scope_is_rejected(change):
    with pytest.raises(ValueError): collect_trial(replace(scenario(), **change), 0)


def test_future_truth_does_not_reach_estimator(monkeypatch):
    from robotics.research import calibration as module
    actual = module.ConstantVelocityEstimator
    seen = []
    class RecordingEstimator(actual):
        def update(self, observation):
            seen.append((observation.capture_time_s, observation.delivery_time_s))
            assert observation.delivery_time_s >= observation.capture_time_s
            return super().update(observation)
    monkeypatch.setattr(module, "ConstantVelocityEstimator", RecordingEstimator)
    _, rows = collect_trial(scenario(), 0, (0.0, 0.2))
    assert seen and rows
    # Each update is one delivered observation; forecasting more horizons
    # neither creates measurements nor changes the estimator update stream.
    first = seen.copy(); seen.clear()
    collect_trial(scenario(), 0, (0.0,))
    assert seen == first


def test_known_quantile_inflation_and_conservative_floor():
    threshold = float(chi2.ppf(0.95, 3))
    rows = [{"horizon_s": 0.0, "mahalanobis_squared": 4 * threshold}] * 20
    assert fit_inflation(rows, (0.0,))["0.0"] == pytest.approx(4)
    rows = [{"horizon_s": 0.0, "mahalanobis_squared": 0.01}] * 20
    assert fit_inflation(rows, (0.0,))["0.0"] == 1


def test_coverage_bootstraps_seeds_not_frames():
    threshold = float(chi2.ppf(0.95, 3))
    rows = []
    for seed in range(4):
        for name in ("a", "b"):
            for _ in range(3):
                rows.append(dict(trial=f"{name}-{seed}", scenario=name, seed=seed,
                                 horizon_s=0.0, mahalanobis_squared=threshold * (0.5 if seed < 2 else 2), error_m=0.01))
    report = coverage_report(rows, {"0.0": 1})["0.0"]
    repeated = coverage_report(rows * 2, {"0.0": 1})["0.0"]
    assert report["inflated_trial_mean_coverage"] == 0.5
    assert report["independent_seed_clusters"] == 4
    assert report["trials"] == 8
    assert report["seed_cluster_bootstrap_95_interval"] == repeated["seed_cluster_bootstrap_95_interval"]
    assert coverage_report(rows, {"0.0": 2})["0.0"]["inflated_trial_mean_coverage"] == 1


def test_disjoint_fit_validation_and_no_overwrite(tmp_path):
    folder = ROOT / "config/research_calibration"
    fit = [folder / "fit-linear-v1.json"]
    validation = [folder / "validation-turn-v1.json"]
    with pytest.raises(ValueError, match="disjoint"):
        run_calibration(fit, validation, [0], [0], tmp_path)
    with pytest.raises(ValueError, match="disjoint"):
        run_calibration(fit, fit, [0], [100], tmp_path)
    report = run_calibration(fit, validation, [0, 1], [100, 101], tmp_path)
    assert len(report["trials"]) == 4
    assert json.loads((tmp_path / "manifest.json").read_text())["fit_seeds"] == [0, 1]
    expected = fit_inflation([row for seed in [0, 1] for row in collect_trial(load_scenario(fit[0]), seed)[1]],
                             (0.0, 0.05, 0.1, 0.2))
    assert report["covariance_scales"] == expected  # validation cannot determine fitted scales
    with pytest.raises(ValueError, match="empty"):
        run_calibration(fit, validation, [0], [100], tmp_path)


@pytest.mark.parametrize("horizons", [(), (0, 0), (-0.1,), (float("nan"),)])
def test_bad_horizons_rejected(horizons):
    with pytest.raises(ValueError): collect_trial(scenario(), 0, horizons)


@pytest.mark.parametrize("score", [float("nan"), float("inf"), -1])
def test_invalid_scores_cannot_silently_fit(score):
    with pytest.raises(ValueError):
        fit_inflation([{"horizon_s": 0, "mahalanobis_squared": score}], (0,))
