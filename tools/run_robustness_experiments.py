"""Run seeded simulation robustness and clearance experiments."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robotics.experiments import execute_obstacle_reach_experiment, generate_experiment_manifest
from robotics.robustness import PROFILES, RobustnessConfig, run_robustness_trial, summarize_robustness


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", nargs="+", choices=["panda", "kuka_iiwa"], default=["panda", "kuka_iiwa"])
    parser.add_argument("--controllers", nargs="+", choices=["ik", "resolved-rate"], default=["ik", "resolved-rate"])
    parser.add_argument("--profiles", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--clearance-mm", type=float, default=10.0)
    parser.add_argument("--self-clearance-mm", type=float, default=0.0)
    parser.add_argument("--planning", action="store_true", help="Also run obstacle planning for every robot/seed with the same margins")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    for values in (args.robots, args.controllers, args.profiles, args.seeds):
        if len(set(values)) != len(values):
            parser.error("Duplicate robot/controller/profile/seed selections are not allowed")
    if any(seed < 0 for seed in args.seeds):
        parser.error("Seeds must be non-negative")
    try:
        config = RobustnessConfig(duration_s=args.duration, environment_clearance_m=args.clearance_mm / 1000,
                                  self_clearance_m=args.self_clearance_mm / 1000)
    except ValueError as err:
        parser.error(str(err))
    output = args.output_dir or ROOT / "experiments" / ("robustness_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=False)
    base = generate_experiment_manifest(robot_names=args.robots, controller_types=args.controllers).to_dict()
    manifest = {key: base[key] for key in ("version", "git_commit_sha", "timestamp_utc", "environment", "provenance")}
    manifest["execution"] = {"robots": args.robots, "controllers": args.controllers,
        "random_seeds": args.seeds, "profiles": args.profiles, "physics_frequency_hz": config.physics_hz}
    manifest["suite"] = "seeded_robustness_v1"
    manifest["robustness"] = {"config": asdict(config), "seeds": args.seeds, "profiles": args.profiles,
        "planning_clearance_buffer_m": 0.005, "planning_collision_resolution_rad": 0.02,
        "sensor_injection": "pose-level Gaussian noise and contiguous detection loss; not camera pixels",
        "watchdog_clock": "simulation time with explicit polling each physics step",
        "transport_stop_model": "privileged software halt remains available during injected transport faults",
        "reference_path": "quintic translation from perturbed home FK; fixed start orientation; seeded goal offset"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8")
    results, planning, errors = [], [], []
    for robot in args.robots:
        for controller in args.controllers:
            for profile in args.profiles:
                for seed in args.seeds:
                    print(f"Running {robot}/{controller}/{profile} seed={seed}", flush=True)
                    try:
                        result = run_robustness_trial(robot, controller, profile, seed, config)
                        results.append(result)
                        print(f"  {'PASS' if result.outcome_passed else 'FAIL'} final={result.final_error_mm:.2f} mm fault={result.observed_fault}", flush=True)
                    except Exception as err:
                        errors.append({"robot": robot, "controller": controller, "profile": profile, "seed": seed,
                                       "error": f"{type(err).__name__}: {err}"})
                        print(f"  ERROR: {err}", flush=True)
        if args.planning:
            for seed in args.seeds:
                try:
                    record = execute_obstacle_reach_experiment(robot_name=robot, seed=seed,
                        environment_clearance_m=config.environment_clearance_m, self_clearance_m=config.self_clearance_m).to_dict()
                    record["seed"] = seed
                    planning.append(record)
                    print(f"Planning {robot} seed={seed}: {record['details']}", flush=True)
                except Exception as err:
                    errors.append({"robot": robot, "profile": "planning", "seed": seed, "error": f"{type(err).__name__}: {err}"})
    expected_trials = len(args.robots) * len(args.controllers) * len(args.profiles) * len(args.seeds)
    summary = {"expected_trials": expected_trials, "completed_trials": len(results),
        "outcomes_passed": sum(result.outcome_passed for result in results),
        "groups": summarize_robustness(results), "planning": planning, "errors": errors}
    summary["suite_passed"] = (not errors and len(results) == expected_trials and all(result.outcome_passed for result in results)
                               and all(item["rrt_planning_success"] and item["execution_success"] for item in planning))
    (output / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, allow_nan=False), encoding="utf-8")
    if results:
        with (output / "trials.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(results[0].to_dict()))
            writer.writeheader()
            writer.writerows(result.to_dict() for result in results)
    lines = ["# Seeded simulation robustness report", "",
        "Pose-level sensing and simulated transport faults; no physical camera or hardware evidence.", "",
        f"Completed: {len(results)}/{expected_trials}; expected outcomes passed: {summary['outcomes_passed']}; errors: {len(errors)}.", "",
        "Tracking requires both final position and orientation tolerances. Expected fault stops are assessed separately from tracking success. Fault trials must latch the specified fault, block a subsequent command, attempt halt, and finish below 0.05 rad/s with no collision/clearance or joint-limit events.", "",
        "| Robot/controller/profile | Outcomes | Tracking | Final error mean / P95 (mm) |",
        "|---|---:|---:|---:|"]
    for key, group in summary["groups"].items():
        track = f"{group['tracking_successes']}/{group['tracking_trials']}" if group["tracking_trials"] else "expected stop"
        lines.append(f"| {key} | {group['outcomes_passed']}/{group['trials']} | {track} | {group['final_error_mm_mean']:.2f} / {group['final_error_mm_p95']:.2f} |")
    lines += ["", "## Clearance planning", "",
        f"Required environment clearance: {args.clearance_mm:g} mm; self clearance: {args.self_clearance_mm:g} mm. Positive margins receive an additional 5 mm planning buffer and 0.02 rad path sampling; execution is checked against the requested margins.", "",
        "| Robot / seed | Plan | Execution | Minimum environment / self clearance (mm) |",
        "|---|---|---|---:|"]
    for record in planning:
        lines.append(f"| {record['robot_name']} / {record['seed']} | {record['rrt_planning_success']} | {record['execution_success']} | {record['min_env_clearance_m'] * 1000:.2f} / {record['min_self_clearance_m'] * 1000:.2f} |")
    lines += ["", "## Limits", "",
        "Gaussian noise is injected into pose targets, not camera images. Detection loss is one contiguous burst. Start joints and goal offsets vary by seed. These profiles do not model all real delays, optical failures, payloads, or loss of physical stop authority.", "",
        "The guard uses deterministic simulation-time polling; real background-thread and ROS2 behavior require the separate unit and middleware suites. Planning uses discrete collision samples, not a continuous collision guarantee. Unknown/unexecuted clearances serialize as null in JSON.", "",
        "See [manifest.json](manifest.json), [summary.json](summary.json), and [trials.csv](trials.csv) for configuration, source provenance, failures, and individual measurements.", ""]
    if errors:
        lines += ["## Execution errors", "", "```json", json.dumps(errors, indent=2), "```", ""]
    (output / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Artifacts: {output}; suite_passed={summary['suite_passed']}", flush=True)
    return 0 if summary["suite_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
