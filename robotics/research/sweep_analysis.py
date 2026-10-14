"""Exploratory paired development analysis; no test-set tuning or p-values."""
import numpy as np


def case_key(row):
    return row["robot"], row["scenario"], row["seed"]


def analyze(rows, bootstrap_samples=1000, bootstrap_seed=419, expected_keys=None):
    if not rows or type(bootstrap_samples) is not int or bootstrap_samples < 1:
        raise ValueError("Nonempty results and a positive bootstrap count are required")
    identifiers = [(r["cell"], *case_key(r)) for r in rows]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("Duplicate paired cases")
    keys = set(expected_keys) if expected_keys is not None else {case_key(r) for r in rows}
    if not keys or any(case_key(r) not in keys for r in rows):
        raise ValueError("Unexpected or empty paired case grid")
    groups = {}
    rng = np.random.default_rng(bootstrap_seed)
    for cell in sorted({r["cell"] for r in rows}):
        trial_rows = [r for r in rows if r["cell"] == cell]
        valid = [r for r in trial_rows if r["status"] == "ok"]
        complete = len(valid) == len(trial_rows) and {case_key(r) for r in trial_rows} == keys
        group = {"method": trial_rows[0]["method"], "horizon_s": trial_rows[0]["horizon_s"],
                 "margin_m": trial_rows[0]["margin_m"], "expected_cases": len(keys),
                 "recorded_cases": len(trial_rows), "completed_cases": len(valid),
                 "errors": len(trial_rows)-len(valid), "complete_pair_grid": complete,
                 "collision_trials": sum(r["contact_episodes"] > 0 for r in valid),
                 "contact_episodes": sum(r["contact_episodes"] for r in valid),
                 "task_completions": sum(r["task_completed"] for r in valid),
                 "collision_free_completions": sum(r["collision_free_task_completed"] for r in valid),
                 "watchdog_fault_trials": sum(bool(r["execution_faults"]) for r in valid),
                 "initial_collision_trials": sum(r["initial_in_collision"] for r in valid)}
        group["observed_collision_fraction"] = group["collision_trials"] / len(valid) if valid else None
        group["failure_adjusted_safe_completion_fraction"] = group["collision_free_completions"] / len(keys)
        group["mean_intervention_fraction"] = float(np.mean([r["intervention_fraction"] for r in valid])) if valid else None
        group["mean_final_joint_error_rad"] = float(np.mean([r["final_joint_error_rad"] for r in valid])) if valid else None
        group["onsets_by_policy_state"] = {state: sum(r.get("onsets_by_policy_state", {}).get(state, 0) for r in valid)
                                             for state in ("WAIT", "TRACK", "HOLD", "FAULT", "UNKNOWN")}
        group["breakdowns"] = {}
        for dimension in ("robot", "scenario"):
            group["breakdowns"][dimension] = {}
            for value in sorted({r[dimension] for r in trial_rows}):
                selected = [r for r in valid if r[dimension] == value]
                group["breakdowns"][dimension][value] = {
                    "completed_cases": len(selected),
                    "collision_trials": sum(r["contact_episodes"] > 0 for r in selected),
                    "safe_completions": sum(r["collision_free_task_completed"] for r in selected),
                    "mean_intervention_fraction": float(np.mean([r["intervention_fraction"] for r in selected])) if selected else None}
        groups[cell] = group
    reference = {case_key(r): r for r in rows if r["method"] == "none" and r["status"] == "ok"}
    paired = {}
    for cell, group in groups.items():
        if group["method"] == "none" or not group["complete_pair_grid"] or set(reference) != keys:
            continue
        selected = [r for r in rows if r["cell"] == cell]
        seeds = sorted({r["seed"] for r in selected})
        collision_deltas, safe_deltas = [], []
        for seed in seeds:
            trials = [r for r in selected if r["seed"] == seed]
            collision_deltas.append(float(np.mean([int(r["contact_episodes"] > 0) - int(reference[case_key(r)]["contact_episodes"] > 0) for r in trials])))
            safe_deltas.append(float(np.mean([int(r["collision_free_task_completed"]) - int(reference[case_key(r)]["collision_free_task_completed"]) for r in trials])))
        indices = rng.integers(0, len(seeds), size=(bootstrap_samples, len(seeds)))
        def statistic(values):
            values = np.array(values)
            return {"mean": float(np.mean(values)), "seed_cluster_95_interval": [float(v) for v in np.quantile(np.mean(values[indices], axis=1), [0.025, 0.975])]}
        paired[cell] = {"seed_clusters": len(seeds), "collision_fraction_delta_vs_nominal": statistic(collision_deltas),
                        "safe_completion_fraction_delta_vs_nominal": statistic(safe_deltas)}
    eligible = {cell: g for cell, g in groups.items() if g["complete_pair_grid"]}
    pareto = []
    for cell, group in eligible.items():
        point = (group["observed_collision_fraction"], -group["failure_adjusted_safe_completion_fraction"], group["mean_intervention_fraction"])
        dominated = False
        for other, other_group in eligible.items():
            if other == cell:
                continue
            candidate = (other_group["observed_collision_fraction"], -other_group["failure_adjusted_safe_completion_fraction"], other_group["mean_intervention_fraction"])
            if all(a <= b for a, b in zip(candidate, point)) and any(a < b for a, b in zip(candidate, point)):
                dominated = True
                break
        if not dominated:
            pareto.append(cell)
    return {"groups": groups, "paired_vs_nominal": paired, "exploratory_pareto_cells": pareto,
            "note": "Development-only; incomplete cells excluded from pairing/Pareto. Seed bootstrap is descriptive, especially with few seeds. No best-test setting or statistical superiority is claimed."}
