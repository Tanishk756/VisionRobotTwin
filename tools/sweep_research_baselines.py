"""Paired multi-seed horizon/margin sweeps on development scenarios only."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robotics.research.contracts import nonnegative
from robotics.research.scenarios import load_scenario, scenario_snapshot
from robotics.research.sweep_analysis import analyze
from robotics.robot_registry import list_available_robots
from tools.run_plant_twin_experiment import run, safe_json


def onset_states(folder):
    decisions = json.loads((folder / "decisions.json").read_text())
    counts = {name: 0 for name in ("WAIT", "TRACK", "HOLD", "FAULT", "UNKNOWN")}
    index, previous = 0, False
    with (folder / "trace.jsonl").open() as stream:
        for line in stream:
            state = json.loads(line)
            while index+1 < len(decisions) and decisions[index+1]["time_s"] <= state["time_s"]:
                index += 1
            if state["in_collision"] and not previous:
                policy = decisions[index]["state"] if decisions and decisions[index]["time_s"] <= state["time_s"] else "UNKNOWN"
                counts[policy if policy in counts else "UNKNOWN"] += 1
            previous = state["in_collision"]
    return counts


def task_metadata(task):
    label, scenario, method, cell, horizon, margin, output = task
    return {"case": label, "robot": scenario.config.robot, "scenario": scenario.scenario_id,
           "seed": scenario.config.seed, "method": method, "cell": cell,
           "horizon_s": horizon, "margin_m": margin}


def execute_case(task):
    label, scenario, method, cell, horizon, margin, output = task
    folder = Path(output) / label
    row = task_metadata(task)
    try:
        summary = run(scenario.config, folder, scenario, method, horizon, margin)
        manifest = json.loads((folder / "manifest.json").read_text())
        return {**row, **summary, "status": "ok", "source_sha256": manifest["source_sha256"],
                "onsets_by_policy_state": onset_states(folder)}
    except Exception as err:
        return {**row, "status": "error", "error": f"{type(err).__name__}: {err}"}


def build_tasks(scenarios, robots, seeds, horizons, margins, output):
    if not scenarios or any(s.split != "development" for s in scenarios):
        raise ValueError("Sweeps require nonempty development-only scenarios")
    for values in ([s.scenario_id for s in scenarios], robots, seeds, horizons, margins):
        if not values or len(set(values)) != len(values):
            raise ValueError("Sweep dimensions must be nonempty and unique")
    if any(type(seed) is not int or seed < 0 for seed in seeds):
        raise ValueError("Seeds must be nonnegative integers")
    if any(robot not in list_available_robots() for robot in robots):
        raise ValueError("Unknown robot")
    for horizon in horizons:
        nonnegative("horizon", horizon, positive=True)
    for margin in margins:
        nonnegative("margin", margin)
        if margin + 0.005 > 0.5:
            raise ValueError("Margin exceeds query range including recovery clearance")
    tasks = []
    for robot in robots:
        for scenario in scenarios:
            for seed in seeds:
                resolved = replace(scenario, config=replace(scenario.config, robot=robot, seed=seed))
                prefix = f"{robot}-{scenario.scenario_id}-seed{seed}"
                tasks.append((prefix+"-none", resolved, "none", "nominal", 0.1, 0.02, str(output)))
                for hi, horizon in enumerate(horizons):
                    for mi, margin in enumerate(margins):
                        for method in ("fixed-horizon", "motion-aware"):
                            cell = f"{method}-h{hi}-m{mi}"
                            tasks.append((prefix+"-"+cell, resolved, method, cell, horizon, margin, str(output)))
    return tasks


def sweep(paths, robots, seeds, horizons, margins, output, workers=1):
    if type(workers) is not int or workers < 1:
        raise ValueError("workers must be a positive integer")
    scenarios = [load_scenario(path) for path in paths]
    output = Path(output)
    tasks = build_tasks(scenarios, robots, seeds, horizons, margins, output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Sweep output must be empty")
    output.mkdir(parents=True, exist_ok=True)
    source_paths = sorted((ROOT / "robotics").rglob("*.py")) + [ROOT / "config/settings.py", ROOT / "tools/run_plant_twin_experiment.py", Path(__file__)]
    hashes = {path.relative_to(ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
    def git(*args):
        result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None
    manifest = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "git_head": git("rev-parse", "HEAD"),
                "working_tree_status": git("status", "--porcelain"), "source_files_sha256": hashes,
                "scenarios": [scenario_snapshot(s) for s in scenarios], "robots": robots, "seeds": seeds,
                "horizons_s": horizons, "margins_m": margins, "workers": workers,
                "worker_blas_thread_limit": 1 if workers > 1 else "inherited",
                "expected_trials": len(tasks), "python": platform.python_version(), "platform": platform.platform(),
                "dependencies": {name: version(name) for name in ("numpy", "scipy", "pybullet")},
                "protocol": "Identical scenario/robot/seed pairs in every cell; nominal runs once per pair. Development-only; no test-set selection.",
                "timing_limit": "Concurrent workers share compute; timing is diagnostic, not a real-time benchmark."}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    rows = []
    with (output / "progress.jsonl").open("w", encoding="utf-8") as journal:
        def record(row):
            rows.append(row)
            journal.write(json.dumps(safe_json(row), allow_nan=False)+"\n"); journal.flush()
            print(f"Completed {len(rows)}/{len(tasks)}: {row['case']} [{row['status']}]", flush=True)
        if workers == 1:
            for task in tasks:
                record(execute_case(task))
        else:
            # Set only this process's environment, inherited by fresh workers.
            for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
                os.environ[name] = "1"
            with ProcessPoolExecutor(max_workers=workers) as executor:
                futures = {executor.submit(execute_case, task): task for task in tasks}
                for future in as_completed(futures):
                    try:
                        row = future.result()
                    except Exception as err:
                        row = {**task_metadata(futures[future]), "status": "error",
                               "error": f"Worker failure: {type(err).__name__}: {err}"}
                    record(row)
    rows.sort(key=lambda row: row["case"])
    expected_keys = {(task[1].config.robot, task[1].scenario_id, task[1].config.seed) for task in tasks}
    report = analyze(rows, expected_keys=expected_keys)
    index = {"expected_trials": len(tasks), "completed_trials": sum(r["status"] == "ok" for r in rows),
             "errors": [r for r in rows if r["status"] != "ok"], "trials": rows}
    (output / "results.json").write_text(json.dumps(safe_json(index), indent=2, allow_nan=False), encoding="utf-8")
    (output / "analysis.json").write_text(json.dumps(safe_json(report), indent=2, allow_nan=False), encoding="utf-8")
    lines = ["# Paired baseline parameter sweep", "",
             f"Completed {index['completed_trials']}/{len(tasks)} trials; errors: {len(index['errors'])}.", "",
             "| Cell | Horizon (ms) | Margin (mm) | Completed | Collision trials | Safe completions | Mean intervention |",
             "|---|---:|---:|---:|---:|---:|---:|"]
    for cell, group in report["groups"].items():
        intervention = group["mean_intervention_fraction"]
        lines.append(f"| {cell} | {group['horizon_s']*1000:g} | {group['margin_m']*1000:g} | {group['completed_cases']}/{group['expected_cases']} | {group['collision_trials']} | {group['collision_free_completions']} | {intervention if intervention is not None else 'unknown'} |")
    lines += ["", "Development-only exploratory results. Errors are not counted as collision-free successes.",
              "Incomplete cells cannot enter paired/Pareto analysis. No test-set setting is selected.",
              "Contact-onset policy state does not prove that the plant was stationary: commands and stops are delayed.",
              "Seed-cluster bootstrap intervals are descriptive and weak with only a few seeds.",
              "Concurrent-worker timing is diagnostic, not an end-to-end real-time benchmark."]
    (output / "REPORT.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    return index, report


def plot_report(report, output):
    """Standalone development figure; defaults are not selected on a test set."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    groups = report["groups"]
    figure, axes = plt.subplots(1, 2, figsize=(9, 3.5), constrained_layout=True)
    styles = {"fixed-horizon": ("tab:blue", "o"), "motion-aware": ("tab:orange", "s")}
    for method, (color, marker) in styles.items():
        horizons = sorted({g["horizon_s"] for g in groups.values() if g["method"] == method})
        for index, horizon in enumerate(horizons):
            selected = sorted((g for g in groups.values() if g["method"] == method and g["horizon_s"] == horizon and g["complete_pair_grid"]), key=lambda g: g["margin_m"])
            if not selected:
                continue
            x = [g["margin_m"] * 1000 for g in selected]
            label = f"{method}, {horizon*1000:g} ms"
            options = dict(color=color, marker=marker, linestyle="-" if index == 0 else "--", label=label)
            axes[0].plot(x, [g["observed_collision_fraction"] for g in selected], **options)
            axes[1].plot(x, [g["failure_adjusted_safe_completion_fraction"] for g in selected], **options)
    reference = groups.get("nominal")
    if reference and reference["complete_pair_grid"]:
        axes[0].axhline(reference["observed_collision_fraction"], color="black", linestyle=":", label="Nominal")
        axes[1].axhline(reference["failure_adjusted_safe_completion_fraction"], color="black", linestyle=":", label="Nominal")
    axes[0].set(ylabel="Fraction of trials with contact", ylim=(-0.02, 1.02))
    axes[1].set(ylabel="Collision-free completion fraction", ylim=(-0.02, 1.02))
    for axis in axes:
        axis.set_xlabel("Clearance margin (mm)")
        axis.grid(alpha=0.25)
    axes[0].legend(fontsize=7)
    figure.suptitle("Development parameter sweep; paired seeds, no test-set selection")
    for extension in ("png", "svg"):
        figure.savefig(Path(output) / f"sweep-diagnostic.{extension}", dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", nargs="+", type=Path, default=sorted((ROOT / "config/research_scenarios").glob("*-v1.json")))
    parser.add_argument("--robots", nargs="+", choices=list_available_robots(), default=["panda", "kuka_iiwa", "xarm6"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    parser.add_argument("--horizons", nargs="+", type=float, default=[0.1, 0.2])
    parser.add_argument("--margins", nargs="+", type=float, default=[0.01, 0.03])
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plot", action="store_true", help="Export PNG/SVG figure using development dependencies")
    args = parser.parse_args()
    index, analysis = sweep(args.scenarios, args.robots, args.seeds, args.horizons, args.margins, args.output_dir, args.workers)
    if args.plot:
        plot_report(analysis, args.output_dir)
    print(f"Finished {index['completed_trials']}/{index['expected_trials']}; errors={len(index['errors'])}")
    return 1 if index["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
