"""Paired development comparisons; no statistical superiority claim."""
import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robotics.research.scenarios import load_scenario
from robotics.robot_registry import list_available_robots
from tools.run_plant_twin_experiment import run, safe_json


def compare(scenario_paths, robots, seeds, output):
    scenarios = [load_scenario(path) for path in scenario_paths]
    if not scenarios or not robots or not seeds:
        raise ValueError("Scenarios, robots and seeds are required")
    for items in ([s.scenario_id for s in scenarios], robots, seeds):
        if len(set(items)) != len(items):
            raise ValueError("Duplicate comparison cases are not allowed")
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Comparison output must be empty")
    output.mkdir(parents=True, exist_ok=True)
    rows, errors = [], []
    for robot in robots:
        for scenario in scenarios:
            for seed in seeds:
                config = replace(scenario.config, robot=robot, seed=seed)
                resolved = replace(scenario, config=config)
                for mode in ("none", "fixed-horizon"):
                    case = f"{robot}-{scenario.scenario_id}-seed{seed}-{mode}"
                    print(f"Running {case}", flush=True)
                    try:
                        result = run(config, output / case, resolved, mode)
                        rows.append({"case": case, "robot": robot, "scenario": scenario.scenario_id,
                                     "split": scenario.split, "seed": seed, **result})
                    except Exception as err:
                        errors.append({"case": case, "error": f"{type(err).__name__}: {err}"})
    index = {"expected_trials": len(robots) * len(scenarios) * len(seeds) * 2,
             "completed_trials": len(rows), "errors": errors, "trials": rows,
             "note": "Engineering comparator; development outcomes include failures and are not publication evidence."}
    (output / "comparison.json").write_text(json.dumps(safe_json(index), indent=2, allow_nan=False), encoding="utf-8")
    fields = ["case", "robot", "scenario", "split", "seed", "baseline", "initial_in_collision",
              "contact_episodes", "minimum_environment_clearance_m", "task_completed",
              "collision_free_task_completed", "final_joint_error_rad", "intervention_fraction",
              "decision_time_p95_ms", "control_deadline_misses"]
    with (output / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(rows)
    return index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", nargs="+", type=Path,
                        default=sorted((ROOT / "config/research_scenarios").glob("*-v1.json")))
    parser.add_argument("--robots", nargs="+", choices=list_available_robots(), default=["panda", "kuka_iiwa", "xarm6"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[0])
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    index = compare(args.scenarios, args.robots, args.seeds, args.output_dir)
    print(f"Completed {index['completed_trials']}/{index['expected_trials']}; errors={len(index['errors'])}")
    return 1 if index["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
