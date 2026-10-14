"""Fit empirical covariance inflation and diagnose disjoint validation trajectories."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robotics.research.calibration import collect_trial, coverage_report, fit_inflation
from robotics.research.scenarios import load_scenario, scenario_snapshot


def run_calibration(fit_paths, validation_paths, fit_seeds, validation_seeds, output,
                    horizons=(0.0, 0.05, 0.1, 0.2), confidence=0.95):
    fitting = [load_scenario(path) for path in fit_paths]
    validation = [load_scenario(path) for path in validation_paths]
    if not fitting or not validation or not fit_seeds or not validation_seeds:
        raise ValueError("Separate nonempty fitting and validation cases are required")
    ids = [s.scenario_id for s in (*fitting, *validation)]
    if len(set(ids)) != len(ids) or set(fit_seeds) & set(validation_seeds):
        raise ValueError("Fitting and validation scenario IDs/seeds must be disjoint")
    if any(len(set(seeds)) != len(seeds) for seeds in (fit_seeds, validation_seeds)):
        raise ValueError("Duplicate seeds are not allowed")
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Calibration output must be empty")
    output.mkdir(parents=True, exist_ok=True)
    rows, trials = {}, []
    for role, scenarios, seeds in (("fit", fitting, fit_seeds), ("validation", validation, validation_seeds)):
        rows[role] = []
        for scenario in scenarios:
            for seed in seeds:
                summary, samples = collect_trial(scenario, seed, horizons)
                trials.append({"role": role, **summary})
                rows[role].extend(samples)
    scales = fit_inflation(rows["fit"], horizons, confidence)
    reports = {role: coverage_report(samples, scales, confidence) for role, samples in rows.items()}
    report = {"confidence": confidence, "covariance_scales": scales, "fit": reports["fit"],
              "validation": reports["validation"], "trials": trials,
              "scope": "pose-level calibration diagnostic, not final paper evaluation or a safety guarantee"}
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    for role, samples in rows.items():
        with (output / f"{role}-scores.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, list(samples[0]))
            writer.writeheader(); writer.writerows(samples)
    sources = sorted((ROOT / "robotics/research").glob("*.py")) + [Path(__file__)]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    def git(*args):
        result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None
    manifest = {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "git_head": git("rev-parse", "HEAD"),
                "working_tree_status": git("status", "--porcelain"), "source_files_sha256": hashes,
                "python": platform.python_version(), "platform": platform.platform(),
                "dependencies": {name: version(name) for name in ("numpy", "scipy")},
                "fit_scenarios": [scenario_snapshot(s) for s in fitting],
                "validation_scenarios": [scenario_snapshot(s) for s in validation],
                "fit_seeds": list(fit_seeds), "validation_seeds": list(validation_seeds),
                "horizons_s": list(horizons), "confidence": confidence,
                "estimator": {"acceleration_spectral_density": 0.1, "initial_velocity_std_m_s": 0.5,
                              "measurement_std_m": "scenario noise standard deviation, floored at 1e-6"},
                "method": "per-horizon fitting-frame quantile / chi-square(3) threshold; scale floored at 1",
                "bootstrap": "1000 seed-cluster resamples, seed 739; frames/scenarios sharing a seed retained together",
                "limitations": ["pose-only sensor experiment", "validation trajectories intentionally shift motion/delay",
                                "empirical covariance scaling, not conformal calibration",
                                "no image, collision probability, physical sensing or simultaneous trajectory guarantee"]}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    lines = ["# Obstacle uncertainty diagnostic", "",
             "Separate fitting and validation scenarios/seeds; validation remains part of calibration development.", "",
             "| Horizon (ms) | Inflation | Raw validation coverage | Inflated validation coverage | Seed-cluster 95% interval | Position RMSE (mm) |",
             "|---:|---:|---:|---:|---|---:|"]
    for key, item in report["validation"].items():
        interval = item["seed_cluster_bootstrap_95_interval"]
        lines.append(f"| {float(key)*1000:g} | {item['covariance_scale']:.3f} | {item['raw_trial_mean_coverage']:.3f} | {item['inflated_trial_mean_coverage']:.3f} | [{interval[0]:.3f}, {interval[1]:.3f}] | {item['position_error_rmse_mm']:.2f} |")
    lines += ["", "Coverage is a measured pointwise diagnostic, not a collision or trajectory-level safety guarantee.",
              "Scales are fitted exclusively on fitting frames; correlated frame samples do not count as independent trials.",
              "No pass/fail is inferred from a nominal confidence target; undercoverage and distribution shift remain visible."]
    (output / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    folder = ROOT / "config/research_calibration"
    parser.add_argument("--fit-scenarios", nargs="+", type=Path, default=sorted(folder.glob("fit-*.json")))
    parser.add_argument("--validation-scenarios", nargs="+", type=Path, default=sorted(folder.glob("validation-*.json")))
    parser.add_argument("--fit-seeds", nargs="+", type=int, default=list(range(10)))
    parser.add_argument("--validation-seeds", nargs="+", type=int, default=list(range(100, 110)))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plot", action="store_true", help="Export PNG/SVG diagnostic figure (development dependencies)")
    args = parser.parse_args()
    report = run_calibration(args.fit_scenarios, args.validation_scenarios, args.fit_seeds, args.validation_seeds, args.output_dir)
    if args.plot:
        plot_report(report, args.output_dir)
    print(json.dumps(report["validation"], indent=2))


def plot_report(report, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    items = sorted(report["validation"].items(), key=lambda item: float(item[0]))
    x = np.array([float(key) * 1000 for key, _ in items])
    coverage = np.array([item["inflated_trial_mean_coverage"] for _, item in items])
    lower = np.array([item["seed_cluster_bootstrap_95_interval"][0] for _, item in items])
    upper = np.array([item["seed_cluster_bootstrap_95_interval"][1] for _, item in items])
    figure, axes = plt.subplots(1, 2, figsize=(9, 3.5), constrained_layout=True)
    axes[0].errorbar(x, coverage, yerr=np.vstack([coverage-lower, upper-coverage]), fmt="o-", capsize=4,
                     label="Validation coverage; seed-cluster interval")
    axes[0].axhline(report["confidence"], linestyle="--", color="black", label="Nominal target")
    axes[0].set(xlabel="Prediction horizon (ms)", ylabel="Pointwise region coverage", ylim=(0, 1.02))
    axes[0].legend(fontsize=7)
    axes[1].plot(x, [item["position_error_rmse_mm"] for _, item in items], "o-", color="tab:orange")
    axes[1].set(xlabel="Prediction horizon (ms)", ylabel="Position RMSE (mm)")
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.suptitle("Pose-level development validation; not a safety guarantee")
    for extension in ("svg", "png"):
        figure.savefig(Path(output) / f"coverage-diagnostic.{extension}", dpi=180)
    plt.close(figure)


if __name__ == "__main__":
    main()
