"""Run development scenarios with nominal or fixed-horizon engineering control."""
import argparse
from dataclasses import asdict, replace
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import platform
from datetime import datetime, timezone
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
from robotics.research.contracts import ScenarioConfig
from robotics.research.estimation import ConstantVelocityEstimator
from robotics.research.plant_twin import PlantTwinHarness
from robotics.robot_registry import get_robot_registry, list_available_robots
from robotics.research.baseline import FixedHorizonStopBaseline, MotionAwareStopBaseline
from robotics.research.scenarios import load_scenario, scenario_snapshot


def safe_json(value):
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def run(config, output, scenario=None, baseline="none", horizon_s=0.1, margin_m=0.02):
    if baseline not in ("none", "fixed-horizon", "motion-aware"):
        raise ValueError("Unknown engineering comparator")
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Output directory must be empty to preserve experiment provenance")
    output.mkdir(parents=True, exist_ok=True)
    records, estimates, predictions, decisions = [], [], [], []
    policy_options = dict(horizon_s=horizon_s, margin_m=margin_m,
                          telemetry_timeout_s=config.telemetry_watchdog_s)
    policy = (MotionAwareStopBaseline(measurement_std_m=max(config.obstacle_noise_std_m, 1e-6), **policy_options)
              if baseline == "motion-aware" else FixedHorizonStopBaseline(**policy_options))
    estimator = ConstantVelocityEstimator(measurement_std_m=max(config.obstacle_noise_std_m, 1e-6))
    home = np.array(get_robot_registry().get_robot_spec(config.robot).home_joint_positions)
    target = home.copy()
    target[0] += scenario.target_delta_first_joint_rad if scenario else 0.08
    control_stride = config.sensor_stride if scenario else 1
    with PlantTwinHarness(config, scenario=scenario) as harness:
        for tick in range(math.ceil(config.duration_s * config.physics_hz)):
            view = harness.controller_view()
            velocity = np.zeros(view.dof)
            if view.observation is not None:
                estimator.update(view.observation)
                estimate = estimator.predict(view.time_s)
                estimates.append(asdict(estimate))
                velocity = np.clip(2 * (target - np.array(view.observation.joint_positions)),
                                   -0.2, 0.2)
                if view.time_s - view.observation.capture_time_s >= config.telemetry_watchdog_s:
                    velocity[:] = 0
            if tick % control_stride == 0:
                started = time.perf_counter()
                if baseline in ("fixed-horizon", "motion-aware"):
                    decision = policy.select(view, velocity, harness.check_candidate_motion)
                    velocity = decision.velocities_rad_s
                    decision_data = asdict(decision)
                    if baseline == "motion-aware":
                        decision_data["obstacle_estimate"] = asdict(policy.last_estimate) if policy.last_estimate else None
                else:
                    decision_data = {"state": "FAULT" if view.execution_fault else "TRACK" if view.observation else "WAIT",
                                     "intervened": False, "reason": "unfiltered_nominal",
                                     "velocities_rad_s": tuple(velocity)}
                decision_data.update(time_s=view.time_s, computation_s=time.perf_counter() - started)
                decisions.append(decision_data)
                if view.execution_fault is None:
                    harness.submit_velocity(velocity)
            if view.observation is not None and tick % control_stride == 0:
                predictions.append({"requested_time_s": view.time_s,
                                    "states": [asdict(state) for state in harness.predict()]})
            records.append(asdict(harness.step()))
        # Terminal state is evaluated without an extra physics integration.
        records.append(asdict(harness.evaluate()))
        events = list(harness.events)
    trace = "".join(json.dumps(safe_json(record), sort_keys=True, allow_nan=False) + "\n" for record in records)
    (output / "trace.jsonl").write_text(trace, encoding="utf-8")
    (output / "events.json").write_text(json.dumps(safe_json(events), indent=2, allow_nan=False), encoding="utf-8")
    (output / "estimates.json").write_text(json.dumps(safe_json(estimates), indent=2, allow_nan=False), encoding="utf-8")
    (output / "predictions.json").write_text(json.dumps(safe_json(predictions), indent=2, allow_nan=False), encoding="utf-8")
    (output / "decisions.json").write_text(json.dumps(safe_json(decisions), indent=2, allow_nan=False), encoding="utf-8")
    contact = [record["in_collision"] for record in records]
    episodes = sum(active and (i == 0 or not contact[i-1]) for i, active in enumerate(contact))
    final_error = float(np.max(np.abs(np.array(records[-1]["plant_joint_positions"]) - target)))
    completion = next((r["time_s"] for r in records if np.max(np.abs(np.array(r["plant_joint_positions"]) - target)) <= 0.02), None)
    times = [d["computation_s"] for d in decisions]
    summary = {"purpose": "development engineering comparator" if scenario else "infrastructure smoke",
               "baseline": baseline, "scenario_id": scenario.scenario_id if scenario else None,
               "records": len(records), "contact_samples": sum(contact), "contact_episodes": episodes,
               "initial_in_collision": contact[0],
               "minimum_obstacle_clearance_m": min(r["obstacle_clearance_m"] for r in records),
               "delivered_observations": sum(e["kind"] == "observation_delivered" for e in events),
               "dropped_observations": sum(e["kind"] == "observation_dropped" for e in events),
               "expired_commands": sum(e["kind"] == "command_expired" for e in events),
               "execution_faults": [e for e in events if e["kind"] == "execution_fault"],
               "occluded_observations": sum(e["kind"] == "observation_occluded" for e in events),
               "minimum_environment_clearance_m": min(r["environment_clearance_m"] for r in records),
               "intervention_fraction": float(np.mean([d["intervened"] for d in decisions])),
               "final_joint_error_rad": final_error, "completion_time_s": completion,
               "task_completed": final_error <= 0.02,
               "collision_free_task_completed": final_error <= 0.02 and not any(contact),
               "decision_time_p95_ms": float(np.percentile(times, 95) * 1000),
               "control_period_s": control_stride / config.physics_hz,
               "control_deadline_misses": sum(value > control_stride / config.physics_hz for value in times),
               "trace_sha256": hashlib.sha256(trace.encode()).hexdigest()}
    (output / "summary.json").write_text(json.dumps(safe_json(summary), indent=2, allow_nan=False), encoding="utf-8")
    def git(*args):
        result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None
    digest = hashlib.sha256()
    sources = sorted((ROOT / "robotics").rglob("*.py")) + [ROOT / "config/settings.py", Path(__file__)]
    source_hashes = {}
    for path in sources:
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(path.read_bytes())
        source_hashes[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = {"config": asdict(config), "python": platform.python_version(),
                "platform": platform.platform(), "git_head": git("rev-parse", "HEAD"),
                "working_tree_status": git("status", "--porcelain"),
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "dependencies": {name: version(name) for name in ("numpy", "pybullet")},
                "source_sha256": digest.hexdigest(), "source_files_sha256": source_hashes,
                "scenario": scenario_snapshot(scenario) if scenario else None,
                "comparator": {"name": baseline, "horizon_s": horizon_s, "margin_m": margin_m,
                               "control_stride": control_stride, "recovery_observations": 3,
                               "recovery_margin_m": 0.005, "minimum_hold_s": 0.1},
                "obstacle_prediction": ("constant-velocity mean, propagated from capture to now and horizon; covariance unused"
                                        if baseline == "motion-aware" else "last delivered obstacle position held fixed"),
                "limitations": ["pose-level sensing", "uncalibrated estimator",
                                "discrete collision evaluation", "nominal arm/obstacle models without braking guarantees",
                                "engineering comparator, no novel safety filter or physical robot"]}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=list_available_robots())
    parser.add_argument("--seed", type=int)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--observation-delay", type=float)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--baseline", choices=["none", "fixed-horizon", "motion-aware"], default="none")
    parser.add_argument("--horizon", type=float, default=0.1)
    parser.add_argument("--margin", type=float, default=0.02)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    scenario = load_scenario(args.scenario) if args.scenario else None
    overrides = {key: value for key, value in dict(robot=args.robot, seed=args.seed,
                    duration_s=args.duration, observation_delay_s=args.observation_delay).items() if value is not None}
    config = replace(scenario.config if scenario else ScenarioConfig(), **overrides)
    if scenario is not None:
        scenario = replace(scenario, config=config)
    print(json.dumps(run(config, args.output_dir, scenario, args.baseline, args.horizon, args.margin), indent=2))


if __name__ == "__main__":
    main()
