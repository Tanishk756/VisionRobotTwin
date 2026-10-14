"""Run an unsupervised simulation testbed smoke; not a safety-method benchmark."""
import argparse
from dataclasses import asdict
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import platform
from datetime import datetime, timezone
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
from robotics.research.contracts import ScenarioConfig
from robotics.research.estimation import ConstantVelocityEstimator
from robotics.research.plant_twin import PlantTwinHarness
from robotics.robot_registry import get_robot_registry, list_available_robots


def safe_json(value):
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def run(config, output):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Output directory must be empty to preserve experiment provenance")
    output.mkdir(parents=True, exist_ok=True)
    records, estimates = [], []
    estimator = ConstantVelocityEstimator(measurement_std_m=max(config.obstacle_noise_std_m, 1e-6))
    home = np.array(get_robot_registry().get_robot_spec(config.robot).home_joint_positions)
    target = home.copy()
    target[0] += 0.08
    with PlantTwinHarness(config) as harness:
        # Delayed-observation proportional reaching is deliberately unfiltered.
        # It demonstrates transport and estimation, not collision avoidance.
        for _ in range(math.ceil(config.duration_s * config.physics_hz)):
            view = harness.controller_view()
            velocity = np.zeros(view.dof)
            if view.observation is not None:
                estimator.update(view.observation)
                estimate = estimator.predict(view.time_s)
                estimates.append(asdict(estimate))
                velocity = np.clip(2 * (target - np.array(view.observation.joint_positions)),
                                   -0.2, 0.2)
            harness.submit_velocity(velocity)
            records.append(asdict(harness.step()))
        # Terminal state is evaluated without an extra physics integration.
        records.append(asdict(harness.evaluate()))
        events = list(harness.events)
    trace = "".join(json.dumps(safe_json(record), sort_keys=True, allow_nan=False) + "\n" for record in records)
    (output / "trace.jsonl").write_text(trace, encoding="utf-8")
    (output / "events.json").write_text(json.dumps(safe_json(events), indent=2, allow_nan=False), encoding="utf-8")
    (output / "estimates.json").write_text(json.dumps(safe_json(estimates), indent=2, allow_nan=False), encoding="utf-8")
    contact = [record["in_collision"] for record in records]
    episodes = sum(active and (i == 0 or not contact[i-1]) for i, active in enumerate(contact))
    summary = {"purpose": "infrastructure smoke, unfiltered proportional reaching",
               "records": len(records), "contact_samples": sum(contact), "contact_episodes": episodes,
               "minimum_obstacle_clearance_m": min(r["obstacle_clearance_m"] for r in records),
               "delivered_observations": sum(e["kind"] == "observation_delivered" for e in events),
               "dropped_observations": sum(e["kind"] == "observation_dropped" for e in events),
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
                "limitations": ["pose-level sensing", "uncalibrated estimator",
                                "discrete collision evaluation", "no safety filter or physical robot"]}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=list_available_robots(), default="panda")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--observation-delay", type=float, default=0.08)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    config = ScenarioConfig(robot=args.robot, seed=args.seed, duration_s=args.duration,
                            observation_delay_s=args.observation_delay)
    print(json.dumps(run(config, args.output_dir), indent=2))


if __name__ == "__main__":
    main()
