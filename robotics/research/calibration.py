"""Pose-level estimator diagnostics and empirical covariance inflation.

    Separate fit/validation trials; frame dependence prevents interpreting the
    fitted frame quantile as a conformal or trajectory-level safety guarantee.
"""
import math

import numpy as np
from scipy.stats import chi2

from robotics.research.contracts import Observation, nonnegative
from robotics.research.estimation import ConstantVelocityEstimator
from robotics.research.transport import EventQueue


def collect_trial(scenario, seed, horizons=(0.0, 0.05, 0.1, 0.2)):
    """Evaluate forecasts without feeding future truth to the estimator.

    This fast sensor-only experiment uses the harness's noise/delay/dropout
    distributions and event queue, without an arm or collision simulation.
    """
    if scenario.split != "calibration" or scenario.frame != "world":
        raise ValueError("Calibration diagnostics require calibration-split world-frame scenarios")
    if type(seed) is not int or seed < 0:
        raise ValueError("Seed must be a nonnegative integer")
    horizons = tuple(float(h) for h in horizons)
    if not horizons or len(set(horizons)) != len(horizons):
        raise ValueError("Prediction horizons must be nonempty and unique")
    for horizon in horizons:
        nonnegative("horizon", horizon)
    config = scenario.config
    streams = np.random.SeedSequence(seed).spawn(2)
    noise, transport = [np.random.default_rng(s) for s in streams]
    queue = EventQueue()
    estimator = ConstantVelocityEstimator(measurement_std_m=max(config.obstacle_noise_std_m, 1e-6))
    sequence = captures = dropped = occluded = stale = missing = 0
    rows = []
    for tick in range(math.ceil(config.duration_s * config.physics_hz)):
        now = tick / config.physics_hz
        if tick % config.sensor_stride == 0:
            captures += 1
            if scenario.occluded(now):
                occluded += 1
            elif transport.random() < config.dropout_probability:
                dropped += 1
            else:
                measured = scenario.position(now) + noise.normal(0, config.obstacle_noise_std_m, 3)
                delay = max(0.0, config.observation_delay_s + transport.uniform(
                    -config.observation_jitter_s, config.observation_jitter_s))
                message = Observation(sequence, now, now + delay, ("pose_sensor",), (0,), measured)
                queue.schedule(now + delay, message)
            sequence += 1
        for delivery in queue.drain(now):
            message = delivery.payload
            observed = Observation(message.sequence, message.capture_time_s, delivery.actual_time_s,
                                   message.joint_names, message.joint_positions, message.obstacle_position_m)
            if not estimator.update(observed):
                stale += 1
        # Score only on the fixed sensing cadence, including occlusion periods.
        if tick % config.sensor_stride:
            continue
        if estimator.predict(now) is None:
            missing += 1
            continue
        for horizon in horizons:
            if now + horizon > config.duration_s:
                continue
            estimate = estimator.predict(now + horizon)
            truth = scenario.position(now + horizon)  # evaluator-only future truth
            error = truth - np.array(estimate.position_m)
            covariance = np.array(estimate.covariance)[:3, :3]
            score = float(error @ np.linalg.solve(covariance, error))
            if not math.isfinite(score):
                raise ValueError("Nonfinite uncertainty score")
            rows.append({"trial": f"{scenario.scenario_id}-seed{seed}", "scenario": scenario.scenario_id,
                         "seed": seed, "time_s": now, "horizon_s": horizon,
                         "observation_age_s": estimate.observation_age_s - horizon,
                         "error_m": float(np.linalg.norm(error)), "mahalanobis_squared": score,
                         "position_covariance_trace_m2": float(np.trace(covariance))})
    return {"scenario": scenario.scenario_id, "seed": seed, "captures": captures,
            "random_drops": dropped, "occluded_captures": occluded, "stale_deliveries": stale,
            "missing_prediction_times": missing, "scored_forecasts": len(rows)}, rows


def fit_inflation(rows, horizons, confidence=0.95):
    """Fit one conservative scalar per horizon using fit frames only."""
    if not 0 < confidence < 1:
        raise ValueError("Confidence must lie strictly between zero and one")
    threshold = float(chi2.ppf(confidence, df=3))
    result = {}
    for horizon in horizons:
        scores = [r["mahalanobis_squared"] for r in rows if r["horizon_s"] == horizon]
        if not scores:
            raise ValueError("Every requested horizon needs fitting observations")
        if any(not math.isfinite(score) or score < 0 for score in scores):
            raise ValueError("Uncertainty scores must be finite and nonnegative")
        result[str(horizon)] = max(1.0, float(np.quantile(scores, confidence, method="higher")) / threshold)
    return result


def coverage_report(rows, scales, confidence=0.95, bootstrap_seed=739, bootstrap_samples=1000):
    """Bootstrap independent seeds, retaining correlated scenarios/frames together."""
    if not 0 < confidence < 1 or type(bootstrap_samples) is not int or bootstrap_samples < 1:
        raise ValueError("Invalid confidence or bootstrap count")
    threshold = float(chi2.ppf(confidence, df=3))
    rng = np.random.default_rng(bootstrap_seed)
    report = {}
    for key, scale in scales.items():
        nonnegative("covariance inflation", scale, positive=True)
        selected = [r for r in rows if r["horizon_s"] == float(key)]
        if not selected:
            raise ValueError("Every requested horizon needs validation observations")
        if any(not math.isfinite(r["mahalanobis_squared"]) or r["mahalanobis_squared"] < 0 for r in selected):
            raise ValueError("Uncertainty scores must be finite and nonnegative")
        trial_ids = sorted({r["trial"] for r in selected})
        raw, calibrated = [], []
        for trial in trial_ids:
            scores = np.array([r["mahalanobis_squared"] for r in selected if r["trial"] == trial])
            raw.append(float(np.mean(scores <= threshold)))
            calibrated.append(float(np.mean(scores <= threshold * scale)))
        trial_seeds = {r["trial"]: r["seed"] for r in selected}
        seeds = sorted(set(trial_seeds.values()))
        sums = np.array([sum(value for trial, value in zip(trial_ids, calibrated) if trial_seeds[trial] == seed) for seed in seeds])
        counts = np.array([sum(trial_seeds[trial] == seed for trial in trial_ids) for seed in seeds])
        indices = rng.integers(0, len(seeds), size=(bootstrap_samples, len(seeds)))
        bootstrap = np.sum(sums[indices], axis=1) / np.sum(counts[indices], axis=1)
        report[key] = {"trials": len(trial_ids), "forecasts": len(selected), "confidence_target": confidence,
                       "covariance_scale": scale, "raw_trial_mean_coverage": float(np.mean(raw)),
                       "inflated_trial_mean_coverage": float(np.mean(calibrated)),
                       "independent_seed_clusters": len(seeds),
                       "seed_cluster_bootstrap_95_interval": [float(x) for x in np.quantile(bootstrap, [0.025, 0.975])],
                       "position_error_rmse_mm": float(np.sqrt(np.mean([r["error_m"] ** 2 for r in selected])) * 1000),
                       "position_error_p95_mm": float(np.quantile([r["error_m"] for r in selected], 0.95) * 1000),
                       "by_scenario": {scenario: float(np.mean([
                           r["mahalanobis_squared"] <= threshold * scale for r in selected if r["scenario"] == scenario]))
                           for scenario in sorted({r["scenario"] for r in selected})}}
        age_groups = {}
        for label, low, high in (("0-100ms", 0, 0.1), ("100-200ms", 0.1, 0.2),
                                 ("200-400ms", 0.2, 0.4), ("400ms-plus", 0.4, math.inf)):
            group = [r for r in selected if low <= r.get("observation_age_s", 0) < high]
            age_groups[label] = {"forecasts": len(group),
                                 "inflated_coverage": float(np.mean([r["mahalanobis_squared"] <= threshold * scale
                                                                     for r in group])) if group else None}
        report[key]["by_observation_age"] = age_groups
    return report
