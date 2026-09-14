#!/usr/bin/env python3
"""CLI runner and analysis engine for reproducible task-space robotics experiments.

Executes deterministic Cartesian tracking and obstacle planning experiments
comparing Franka Emika Panda vs KUKA LBR iiwa across IK position control and
Jacobian resolved-rate control in PyBullet DIRECT simulation.
"""

import argparse
import csv
import json
import os
import platform
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# Ensure repository root is on Python path
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from robotics.experiments import (
    ALL_EXPERIMENT_NAMES,
    EXPERIMENT_DEFINITIONS,
    ExperimentDefinition,
    ExperimentMetrics,
    PlanningExperimentResult,
    check_shared_feasibility,
    execute_experiment_trial,
    execute_obstacle_reach_experiment,
    generate_experiment_manifest,
)
from utils.logger import get_logger

logger = get_logger("Robotics.ExperimentCLI")


def generate_experiment_plots(
    timeseries_data: Dict[str, List[Dict[str, Any]]],
    summary_results: Dict[str, Any],
    planning_results: Dict[str, Any],
    plots_dir: Path,
) -> List[str]:
    """Generates publication-quality matplotlib plots for the experiment run.

    Args:
        timeseries_data: Mapping of '{exp}_{robot}_{ctrl}_t{trial}' to list of time-step records.
        summary_results: Aggregate metrics summary.
        planning_results: Obstacle reach experiment results.
        plots_dir: Directory where figures will be saved.

    Returns:
        List of generated plot file paths (relative to run directory).
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        logger.warning("matplotlib not available; skipping plot generation.")
        return []

    plots_dir.mkdir(parents=True, exist_ok=True)
    generated_files: List[str] = []

    # Styling constants
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "figure.titlesize": 12,
        "figure.dpi": 200,
        "savefig.dpi": 200,
    })

    COLOR_DESIRED = "#111827"  # Dark gray / black dashed
    COLORS = {
        ("panda", "ik"): "#2563eb",            # Blue
        ("panda", "resolved-rate"): "#0284c7",  # Sky blue
        ("kuka_iiwa", "ik"): "#dc2626",        # Red
        ("kuka_iiwa", "resolved-rate"): "#ea580c",  # Orange
    }
    LABELS = {
        ("panda", "ik"): "Panda (IK)",
        ("panda", "resolved-rate"): "Panda (Resolved-Rate)",
        ("kuka_iiwa", "ik"): "KUKA iiwa (IK)",
        ("kuka_iiwa", "resolved-rate"): "KUKA iiwa (Resolved-Rate)",
    }

    # 1. 3D Trajectory Comparison Plots (per experiment)
    for exp_name in ALL_EXPERIMENT_NAMES:
        # Check if any trial data exists for this experiment
        has_data = any(f"{exp_name}_{robot}_{ctrl}_t0" in timeseries_data for (robot, ctrl) in COLORS.keys())
        if not has_data:
            continue

        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")

        # Plot desired path from first available trial
        plotted_desired = False
        for (robot, ctrl), color in COLORS.items():
            key = f"{exp_name}_{robot}_{ctrl}_t0"
            if key in timeseries_data and len(timeseries_data[key]) > 0:
                ts = timeseries_data[key]
                if not plotted_desired:
                    des_x = [row["desired_x"] for row in ts]
                    des_y = [row["desired_y"] for row in ts]
                    des_z = [row["desired_z"] for row in ts]
                    ax.plot(des_x, des_y, des_z, "k--", linewidth=2.0, label="Desired Path (Ground Truth)", zorder=10)
                    plotted_desired = True

                act_x = [row["actual_x"] for row in ts]
                act_y = [row["actual_y"] for row in ts]
                act_z = [row["actual_z"] for row in ts]
                ax.plot(act_x, act_y, act_z, color=color, linewidth=1.6, alpha=0.85, label=LABELS[(robot, ctrl)])

        ax.set_title(f"3D Cartesian Trajectory Comparison — {exp_name.replace('_', ' ').title()}\n[PYBULLET SIMULATION]", pad=15)
        ax.set_xlabel("X (m)", labelpad=8)
        ax.set_ylabel("Y (m)", labelpad=8)
        ax.set_zlabel("Z (m)", labelpad=8)
        ax.legend(loc="upper right", framealpha=0.9)
        plt.tight_layout()

        plot_path = plots_dir / f"trajectory_3d_comparison_{exp_name}.png"
        fig.savefig(plot_path, bbox_inches="tight")
        plt.close(fig)
        generated_files.append(f"plots/{plot_path.name}")

    # 2. Tracking Error vs Time Multi-panel for Line & Circle
    for exp_name in ["line", "circle", "figure_eight", "se3_sweep"]:
        if exp_name not in ALL_EXPERIMENT_NAMES:
            continue
        has_data = any(f"{exp_name}_{robot}_{ctrl}_t0" in timeseries_data for (robot, ctrl) in COLORS.keys())
        if not has_data:
            continue

        fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
        ax_pos, ax_orn = axes[0, 0], axes[0, 1]
        ax_manip, ax_sigma = axes[1, 0], axes[1, 1]

        for (robot, ctrl), color in COLORS.items():
            key = f"{exp_name}_{robot}_{ctrl}_t0"
            if key in timeseries_data and len(timeseries_data[key]) > 0:
                ts = timeseries_data[key]
                t_arr = [row["time_s"] for row in ts]
                pos_err = [row["position_error_mm"] for row in ts]
                orn_err = [row["orientation_error_deg"] for row in ts]
                manip = [row["manipulability"] for row in ts]
                sigma = [row["sigma_min"] for row in ts]

                lbl = LABELS[(robot, ctrl)]
                ax_pos.plot(t_arr, pos_err, color=color, label=lbl, linewidth=1.4)
                ax_orn.plot(t_arr, orn_err, color=color, label=lbl, linewidth=1.4)
                ax_manip.plot(t_arr, manip, color=color, label=lbl, linewidth=1.4)
                ax_sigma.plot(t_arr, sigma, color=color, label=lbl, linewidth=1.4)

        ax_pos.set_ylabel("Position Error (mm)")
        ax_pos.set_title("Cartesian Position Error vs Time")
        ax_pos.grid(True, linestyle=":", alpha=0.6)
        ax_pos.legend(loc="upper right", fontsize=8)

        ax_orn.set_ylabel("Orientation Error (deg)")
        ax_orn.set_title("Geodesic Orientation Error vs Time")
        ax_orn.grid(True, linestyle=":", alpha=0.6)

        ax_manip.set_xlabel("Time (s)")
        ax_manip.set_ylabel("Yoshikawa Manipulability")
        ax_manip.set_title("Manipulability Index vs Time")
        ax_manip.grid(True, linestyle=":", alpha=0.6)

        ax_sigma.set_xlabel("Time (s)")
        ax_sigma.set_ylabel(r"Singular Value $\sigma_{\min}$")
        ax_sigma.set_title(r"Minimum Singular Value $\sigma_{\min}$ vs Time")
        ax_sigma.grid(True, linestyle=":", alpha=0.6)

        fig.suptitle(f"Kinematic & Tracking Performance — {exp_name.replace('_', ' ').title()} [PYBULLET SIMULATION]", y=0.98)
        plt.tight_layout()
        plot_path = plots_dir / f"telemetry_timeseries_{exp_name}.png"
        fig.savefig(plot_path, bbox_inches="tight")
        plt.close(fig)
        generated_files.append(f"plots/{plot_path.name}")

    # 3. Aggregate Summary Bar Charts: Position RMSE & Joint Travel
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    experiments = [e for e in ALL_EXPERIMENT_NAMES if e in summary_results.get("tasks", {})]
    x = np.arange(len(experiments))
    width = 0.20

    combos = [("panda", "ik"), ("panda", "resolved-rate"), ("kuka_iiwa", "ik"), ("kuka_iiwa", "resolved-rate")]
    for idx, (robot, ctrl) in enumerate(combos):
        pos_rmse_means = []
        travel_means = []
        for exp in experiments:
            d = summary_results.get("tasks", {}).get(exp, {}).get(f"{robot}_{ctrl}", {})
            pos_rmse_means.append(d.get("rmse_position_error_mm", {}).get("mean", 0.0))
            travel_means.append(d.get("total_joint_travel_rad", {}).get("mean", 0.0))

        offset = (idx - 1.5) * width
        lbl = LABELS[(robot, ctrl)]
        col = COLORS[(robot, ctrl)]
        ax1.bar(x + offset, pos_rmse_means, width, label=lbl, color=col, alpha=0.9)
        ax2.bar(x + offset, travel_means, width, label=lbl, color=col, alpha=0.9)

    ax1.set_title("Root Mean Square Cartesian Position Error (mm)\n[PYBULLET SIMULATION]")
    ax1.set_xticks(x)
    ax1.set_xticklabels([e.replace('_', '\n').title() for e in experiments])
    ax1.set_ylabel("RMSE Position Error (mm)")
    ax1.grid(axis="y", linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left", fontsize=8)

    ax2.set_title("Total Joint Space Travel (rad)\n[PYBULLET SIMULATION]")
    ax2.set_xticks(x)
    ax2.set_xticklabels([e.replace('_', '\n').title() for e in experiments])
    ax2.set_ylabel("Joint Travel (rad)")
    ax2.grid(axis="y", linestyle=":", alpha=0.6)
    ax2.legend(loc="upper left", fontsize=8)

    plt.tight_layout()
    plot_path = plots_dir / "summary_benchmarks_comparison.png"
    fig.savefig(plot_path, bbox_inches="tight")
    plt.close(fig)
    generated_files.append(f"plots/{plot_path.name}")

    return generated_files


def verify_experiment_results_consistency(
    summary_results: Dict[str, Any],
    all_trial_metrics: List[Dict[str, Any]],
    manifest: Dict[str, Any],
) -> bool:
    """Programmatically verifies consistency across trial CSV records, summary JSON, and manifest.

    Raises:
        AssertionError: If any discrepancy is found between records, summary, or manifest.
    """
    tasks = summary_results.get("tasks", {})
    traj_defs = manifest.get("trajectory_definitions", {})

    for exp_name, exp_summary in tasks.items():
        if exp_summary.get("skipped", False):
            continue

        if exp_name in traj_defs:
            manifest_dur = traj_defs[exp_name].get("effective_duration_s")
            assert manifest_dur is not None, f"Manifest missing effective_duration_s for {exp_name}"

        for combo_key, stats in exp_summary.items():
            if not isinstance(stats, dict) or "trials_count" not in stats:
                continue

            if combo_key.startswith("kuka_iiwa_"):
                robot = "kuka_iiwa"
                ctrl = combo_key[len("kuka_iiwa_"):]
            elif combo_key.startswith("panda_"):
                robot = "panda"
                ctrl = combo_key[len("panda_"):]
            else:
                robot, ctrl = combo_key.split("_", 1) if "_" in combo_key else (combo_key, "")

            # Find matching trials
            matching = [
                m for m in all_trial_metrics
                if m.get("experiment_name") == exp_name and m.get("robot_name") == robot and m.get("controller_type") == ctrl
            ]
            assert stats["trials_count"] == len(matching), (
                f"Mismatch in trial count for {exp_name} {combo_key}: summary={stats['trials_count']} vs csv={len(matching)}"
            )
            success_count = sum(1 for m in matching if m.get("success", False))
            assert stats["success_count"] == success_count, (
                f"Mismatch in success count for {exp_name} {combo_key}: summary={stats['success_count']} vs csv={success_count}"
            )
            expected_rate = success_count / max(1, len(matching))
            assert abs(stats["success_rate"] - expected_rate) < 1e-6, (
                f"Mismatch in success rate for {exp_name} {combo_key}"
            )
            if matching:
                expected_rmse_mean = float(np.mean([float(m["rmse_position_error_mm"]) for m in matching]))
                actual_rmse_mean = float(stats["rmse_position_error_mm"]["mean"])
                assert abs(expected_rmse_mean - actual_rmse_mean) < 1e-4, (
                    f"Mismatch in mean RMSE for {exp_name} {combo_key}: expected={expected_rmse_mean}, actual={actual_rmse_mean}"
                )

    logger.info("Experiment results consistency verification passed successfully.")
    return True


def generate_experiment_report_markdown(
    manifest: Dict[str, Any],
    preflight_results: Dict[str, Any],
    summary_results: Dict[str, Any],
    planning_results: Dict[str, Any],
    plot_rel_paths: List[str],
    output_report_path: Path,
    all_trial_metrics: Optional[List[Dict[str, Any]]] = None,
) -> None:
    """Generates a comprehensive, conservative research-style REPORT.md artifact."""
    lines: List[str] = []
    trial_metrics = all_trial_metrics or []

    # Calculate data-derived metrics
    feasible_experiment_count = sum(1 for pf in preflight_results.values() if pf.get("shared_feasible"))
    failed_feasibility_count = sum(1 for pf in preflight_results.values() if not pf.get("shared_feasible"))
    executed_trial_count = len(trial_metrics)
    skipped_trial_count = sum(
        t.get("skipped_trial_count", 0)
        for t in summary_results.get("tasks", {}).values()
        if isinstance(t, dict) and t.get("skipped", False)
    )
    successful_trial_count = sum(1 for m in trial_metrics if m.get("success", False))
    failed_trial_count = sum(1 for m in trial_metrics if not m.get("success", False))
    self_collision_total = sum(int(m.get("self_collision_count", 0)) for m in trial_metrics)
    environment_collision_total = sum(int(m.get("environment_collision_count", 0)) for m in trial_metrics)
    singularity_warning_total = sum(int(m.get("singularity_warning_count", 0)) for m in trial_metrics)
    joint_limit_event_total = sum(int(m.get("joint_limit_violation_count", 0)) for m in trial_metrics)

    env_info = manifest.get("environment", {})
    exec_info = manifest.get("execution", {})
    tol_info = manifest.get("tolerances", {})
    traj_defs = manifest.get("trajectory_definitions", {})

    lines.append("# VisionRobotTwin Task-Space Benchmark Experiment Report")
    lines.append("")
    lines.append("> [!NOTE]")
    lines.append("> **REFERENCE PYBULLET SIMULATION RESULTS**  ")
    lines.append(f"> Generated: {manifest.get('timestamp_utc', 'N/A')} UTC  ")
    lines.append(f"> Software Version: {manifest.get('version', 'N/A')} | Commit: `{manifest.get('git_commit_sha', 'N/A')}`  ")
    lines.append(f"> Physics Engine: PyBullet package `{env_info.get('pybullet_package_version', 'unknown')}` (API: `{env_info.get('pybullet_api_version', 'unknown')}`) (`DIRECT` mode, fixed timestep dt = {exec_info.get('timestep_dt_s', 1/240):.6f}s / {exec_info.get('physics_frequency_hz', 240)} Hz)  ")
    lines.append("")
    lines.append("---")
    lines.append("")

    lines.append("## 1. Executive Summary & Objective")
    lines.append("")
    lines.append("This benchmark suite provides a mathematically rigorous, reproducible experimental comparison of:")
    lines.append("1. **Manipulators**: Franka Emika Panda (7-DoF) vs KUKA LBR iiwa (7-DoF).")
    lines.append("2. **Controllers**: IK position control with coordinated joint velocity limits vs Resolved-Rate Jacobian differential velocity control with damping.")
    lines.append("3. **Trajectories**: Identical deterministic Cartesian task-space trajectories executed under strictly matching physical simulation parameters.")
    lines.append("4. **Motion Planning**: Obstacle-blocked Cartesian reachability comparing direct joint-space interpolation against RRT-Connect planning and shortcutting.")
    lines.append("")

    lines.append("## 2. Environment & Simulation Parameters")
    lines.append("")
    lines.append("| Parameter | Value | Details |")
    lines.append("| :--- | :--- | :--- |")
    lines.append(f"| **OS** | `{env_info.get('os', 'N/A')}` | {platform.version()} |")
    lines.append(f"| **Python** | `{env_info.get('python_version', 'N/A')}` | CPython |")
    lines.append(f"| **NumPy** | `{env_info.get('numpy_version', 'N/A')}` | Vectorized linear algebra |")
    lines.append(f"| **PyBullet Package** | `{env_info.get('pybullet_package_version', 'N/A')}` | Physics client `DIRECT` |")
    lines.append(f"| **PyBullet API** | `{env_info.get('pybullet_api_version', 'N/A')}` | Internal C API version |")
    lines.append(f"| **Physics Frequency** | `{exec_info.get('physics_frequency_hz', 240)} Hz` | `dt = {exec_info.get('timestep_dt_s', 1/240):.6f}s` |")
    lines.append(f"| **Settle Tolerance** | `{tol_info.get('settle_tolerance_mm', 5.0)} mm` | Settled final position threshold |")
    lines.append(f"| **Success Position Tolerance** | `{tol_info.get('success_position_tolerance_mm', 10.0)} mm` | Trial completion position threshold |")
    lines.append(f"| **Success Orientation Tolerance** | `{tol_info.get('success_orientation_tolerance_deg', 10.0)} deg` | Trial completion orientation threshold |")
    lines.append(f"| **Planning Position Tolerance** | `{tol_info.get('planning_execution_position_tolerance_mm', 25.0)} mm` | Obstacle reach final endpoint threshold |")
    lines.append(f"| **Deterministic Repeats** | `{exec_info.get('deterministic_repeats', 1)}` | Deterministic repeatability executions |")
    lines.append(f"| **Random Seed** | `{exec_info.get('random_seed', 42)}` | Deterministic initialization seed |")
    lines.append("")

    lines.append("## 3. Shared Feasibility Preflight")
    lines.append("")
    lines.append("Before executing any trajectory trial, the entire desired task-space curve is sampled at dense intervals. Both Franka Emika Panda and KUKA LBR iiwa solvers verify that inverse kinematics solutions exist with Cartesian position residual $\\le 25\\text{ mm}$ and orientation error $\\le 10^\\circ$ across every checked waypoint.")
    lines.append("")
    lines.append("| Trajectory | Shared Feasible | Panda Feasible | KUKA Feasible | Checked Samples | Total Samples | Stride | Max Pos Residual (mm) | Max Orn Residual (deg) | Policy |")
    lines.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |")
    for exp_name, pf in preflight_results.items():
        shared_icon = "PASS" if pf.get("shared_feasible") else "FAIL"
        p_icon = "PASS" if pf.get("panda_feasible") else "FAIL"
        k_icon = "PASS" if pf.get("kuka_feasible") else "FAIL"
        chk_samples = pf.get("checked_samples", "N/A")
        tot_samples = pf.get("trajectory_total_samples", "N/A")
        stride = pf.get("sample_stride", 1)
        max_pos = max(pf.get("max_position_residual_m_panda", 0.0), pf.get("max_position_residual_m_kuka", 0.0)) * 1000.0
        max_orn = max(pf.get("max_orientation_residual_deg_panda", 0.0), pf.get("max_orientation_residual_deg_kuka", 0.0))
        policy = pf.get("policy", "EXACT_PATH")
        lines.append(f"| `{exp_name}` | **{shared_icon}** | {p_icon} | {k_icon} | {chk_samples} | {tot_samples} | {stride} | {max_pos:.2f} mm | {max_orn:.2f}° | `{policy}` |")
    lines.append("")

    all_stride_1 = all(pf.get("sample_stride", 1) == 1 for pf in preflight_results.values())
    if all_stride_1:
        lines.append("Every generated trajectory sample was checked during preflight verification (`preflight_sample_stride = 1`).")
    else:
        lines.append("Trajectory preflight verification checked subsampled waypoints according to configured stride values.")
    lines.append("")

    lines.append("## 4. Task Definitions")
    lines.append("")
    # Consume single source of truth trajectory metadata
    for idx, (name, meta) in enumerate(traj_defs.items(), 1):
        g_params = meta.get("geometry_parameters", {})
        dur = meta.get("effective_duration_s", meta.get("default_duration_s", 0.0))
        lines.append(f"{idx}. **{name.upper()}**: {meta.get('description', '')}")
        lines.append(f"   - Center Position: `{meta.get('center_position', [])}`")
        lines.append(f"   - Geometry: `{g_params}`")
        lines.append(f"   - Duration: `{dur:.1f} s` ({meta.get('sample_count', 0)} samples @ {meta.get('physics_hz', 240)} Hz)")
        lines.append(f"   - Orientation Profile: `{meta.get('orientation_profile', '')}` (Rotation Axis: `{meta.get('rotation_axis', 'None')}`)")
        lines.append("")

    lines.append("## 5. Cross-Robot & Cross-Controller Summary Matrix")
    lines.append("")
    lines.append("| Experiment | Metric | Panda (IK) | Panda (Resolved-Rate) | KUKA iiwa (IK) | KUKA iiwa (Resolved-Rate) |")
    lines.append("| :--- | :--- | :---: | :---: | :---: | :---: |")

    tasks = summary_results.get("tasks", {})
    for exp_name in ALL_EXPERIMENT_NAMES:
        if exp_name not in tasks:
            continue
        exp_data = tasks[exp_name]
        if exp_data.get("skipped", False):
            lines.append(f"| **{exp_name.replace('_', ' ').title()}** | **Status** | `SKIPPED (SHARED FEASIBILITY FAILED)` | `SKIPPED` | `SKIPPED` | `SKIPPED` |")
            continue

        p_ik = exp_data.get("panda_ik", {})
        p_rr = exp_data.get("panda_resolved-rate", {})
        k_ik = exp_data.get("kuka_iiwa_ik", {})
        k_rr = exp_data.get("kuka_iiwa_resolved-rate", {})

        def _fmt(d: Dict[str, Any], key: str, unit: str = "", scale: float = 1.0) -> str:
            val = d.get(key, {})
            if isinstance(val, dict):
                m = val.get("mean", 0.0) * scale
                return f"{m:.2f}{unit}"
            return "N/A"

        def _succ(d: Dict[str, Any]) -> str:
            rate = d.get("success_rate", 0.0) * 100.0
            return f"{rate:.0f}%"

        lines.append(f"| **{exp_name.replace('_', ' ').title()}** | **RMSE Pos (mm)** | {_fmt(p_ik, 'rmse_position_error_mm')} | {_fmt(p_rr, 'rmse_position_error_mm')} | {_fmt(k_ik, 'rmse_position_error_mm')} | {_fmt(k_rr, 'rmse_position_error_mm')} |")
        lines.append(f"| | **P95 Pos (mm)** | {_fmt(p_ik, 'p95_position_error_mm')} | {_fmt(p_rr, 'p95_position_error_mm')} | {_fmt(k_ik, 'p95_position_error_mm')} | {_fmt(k_rr, 'p95_position_error_mm')} |")
        lines.append(f"| | **Mean Orn (deg)** | {_fmt(p_ik, 'mean_orientation_error_deg', '°')} | {_fmt(p_rr, 'mean_orientation_error_deg', '°')} | {_fmt(k_ik, 'mean_orientation_error_deg', '°')} | {_fmt(k_rr, 'mean_orientation_error_deg', '°')} |")
        lines.append(f"| | **Joint Travel (rad)** | {_fmt(p_ik, 'total_joint_travel_rad', ' rad')} | {_fmt(p_rr, 'total_joint_travel_rad', ' rad')} | {_fmt(k_ik, 'total_joint_travel_rad', ' rad')} | {_fmt(k_rr, 'total_joint_travel_rad', ' rad')} |")
        lines.append(f"| | **Min Manipulability** | {_fmt(p_ik, 'min_manipulability')} | {_fmt(p_rr, 'min_manipulability')} | {_fmt(k_ik, 'min_manipulability')} | {_fmt(k_rr, 'min_manipulability')} |")
        lines.append(f"| | **Success Rate** | **{_succ(p_ik)}** | **{_succ(p_rr)}** | **{_succ(k_ik)}** | **{_succ(k_rr)}** |")

    lines.append("")

    lines.append("## 6. Visual Evidence & Trajectory Comparisons")
    lines.append("")
    for p_path in plot_rel_paths:
        title = Path(p_path).stem.replace("_", " ").title()
        lines.append(f"### {title}")
        lines.append(f"![{title}]({p_path})")
        lines.append("")

    lines.append("## 7. Obstacle-Blocked Motion Planning Experiment")
    lines.append("")
    lines.append("A dedicated obstacle avoidance experiment tests the complete collision-aware planning stack when moving between points $[0.42, -0.18, 0.35]$ and $[0.42, 0.18, 0.35]$ separated by a rigid box obstacle at $[0.42, 0.0, 0.35]$.")
    lines.append("")
    lines.append(f"Under this PyBullet configuration, planning PASS requires collision-free trajectory execution AND a final endpoint error $\\le {tol_info.get('planning_execution_position_tolerance_mm', 25.0)}\\text{{ mm}}$. This criterion validates high-level obstacle clearing and endpoint arrival, distinct from the {tol_info.get('settle_tolerance_mm', 5.0)} mm continuous settled tracking tolerance.")
    lines.append("")
    lines.append("| Robot | Direct Path State | RRT-Connect Status | Plan Time (ms) | Raw Waypoints | Smoothed Waypoints | Raw Travel (rad) | Smoothed Travel (rad) | Min Clearance (m) | Execution | Final Error (mm) |")
    lines.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")

    for r_name, p_res in planning_results.items():
        direct_str = "BLOCKED" if not p_res.get("direct_path_free", False) else "FREE"
        rrt_str = "SUCCESS" if p_res.get("rrt_planning_success", False) else "FAILED"
        exec_str = "PASS" if p_res.get("execution_success", False) else "FAIL"
        p_time = p_res.get("planning_time_ms", 0.0)
        raw_pts = p_res.get("raw_waypoints_count", 0)
        sm_pts = p_res.get("smoothed_waypoints_count", 0)
        raw_len = p_res.get("raw_joint_path_length_rad", 0.0)
        sm_len = p_res.get("smoothed_joint_path_length_rad", 0.0)
        clr = p_res.get("min_collision_clearance_m", 0.0)
        final_err = p_res.get("execution_final_pos_error_mm", 0.0)

        lines.append(f"| **{r_name.upper()}** | `{direct_str}` | `{rrt_str}` | {p_time:.1f} ms | {raw_pts} | {sm_pts} | {raw_len:.2f} rad | {sm_len:.2f} rad | {clr:.4f} m | **{exec_str}** | {final_err:.2f} mm |")

    lines.append("")

    lines.append("## 8. Failure Cases & Singularity Telemetry")
    lines.append("")
    lines.append(f"- **Feasibility**: {feasible_experiment_count} shared-feasible trajectories verified, {failed_feasibility_count} failed feasibility preflight trajectories.")
    lines.append(f"- **Trial Execution**: {executed_trial_count} executed trials ({successful_trial_count} successful, {failed_trial_count} failed), {skipped_trial_count} skipped trials due to preflight gating.")
    lines.append(f"- **Collisions**: Recorded {self_collision_total} self-collisions and {environment_collision_total} environment-collisions across all executed trials.")
    lines.append(f"- **Singularity Warnings**: {singularity_warning_total} total near-singularity warning steps (condition number $> 100$ or $\\sigma_{{\\min}} < 0.01$) observed during execution.")
    lines.append(f"- **Joint Limit Events**: {joint_limit_event_total} joint-limit violations observed during execution.")
    lines.append("")

    lines.append("## 9. Limitations & Conservative Interpretation")
    lines.append("")
    lines.append("> [!WARNING]")
    lines.append("> **Experimental Scope & Limitations:**")
    lines.append("> - All results reported herein were gathered strictly inside **PyBullet physics simulation** under idealized rigid-body dynamics.")
    lines.append("> - No claim is made regarding physical hardware performance, motor thermal limits, gear backlash, or physical friction non-linearities.")
    lines.append("> - Kinematic and controller comparisons reflect the specific URDF models, joint limits, and damping parameters configured in this environment.")
    lines.append("")

    lines.append("## 10. Reproduction Commands")
    lines.append("")
    lines.append("To reproduce the complete benchmark run deterministically:")
    lines.append("```bash")
    lines.append("python tools/run_taskspace_experiments.py --all --repeats 3 --headless")
    lines.append("```")
    lines.append("")
    lines.append("To run individual trajectories:")
    lines.append("```bash")
    lines.append("python tools/run_taskspace_experiments.py --experiment line --headless")
    lines.append("python tools/run_taskspace_experiments.py --experiment circle --headless")
    lines.append("python tools/run_taskspace_experiments.py --robot panda --controller ik --all --headless")
    lines.append("```")

    output_report_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Generated comprehensive experiment report at {output_report_path}")


def create_experiment_arg_parser() -> argparse.ArgumentParser:
    """Creates the CLI argument parser with mutually exclusive GUI/headless flags."""
    parser = argparse.ArgumentParser(
        description="VisionRobotTwin Multi-Manipulator Task-Space Experiment Suite"
    )
    parser.add_argument("--all", action="store_true", help="Execute all 5 benchmark experiments")
    parser.add_argument("--experiment", type=str, choices=ALL_EXPERIMENT_NAMES, help="Run specific experiment")
    parser.add_argument("--robot", type=str, choices=["panda", "kuka_iiwa"], help="Filter by robot")
    parser.add_argument("--controller", type=str, choices=["ik", "resolved-rate"], help="Filter by controller")
    parser.add_argument("--repeats", type=int, default=1, help="Number of deterministic repeatability runs per trial (default: 1)")
    parser.add_argument("--seed", type=int, default=42, help="Deterministic random seed")
    parser.add_argument("--trajectory-duration", type=float, default=None, help="Trajectory duration override in seconds (default: None, use per-experiment default)")
    parser.add_argument("--physics-hz", type=int, default=240, help="Physics simulation frequency in Hz")
    parser.add_argument("--preflight-stride", type=int, default=1, help="Sampling stride for preflight feasibility check (default: 1)")
    parser.add_argument("--output-dir", type=str, default=None, help="Custom output directory for experiment run")

    # Mutually exclusive GUI / Headless group
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--headless", action="store_true", help="Run in headless mode (DIRECT physics, default)")
    mode_group.add_argument("--gui", action="store_true", help="Run with PyBullet GUI")

    parser.add_argument("--publish-reference", action="store_true", help="Publish output as reference results in docs/experiments/v1.2_reference/")
    parser.add_argument("--commit-sha", type=str, default=None, help="Explicit git commit SHA to record in manifest")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = create_experiment_arg_parser()
    args = parser.parse_args(argv)

    # Resolve headless vs GUI mode (default: headless / DIRECT)
    gui_mode = bool(args.gui)

    # Determine experiments to run
    if args.all or (not args.experiment):
        experiments_to_run = list(ALL_EXPERIMENT_NAMES)
    else:
        experiments_to_run = [args.experiment]

    robots = [args.robot] if args.robot else ["panda", "kuka_iiwa"]
    controllers = [args.controller] if args.controller else ["ik", "resolved-rate"]

    # In reference publishing mode, preflight stride must strictly be 1
    if args.publish_reference:
        preflight_stride = 1
    else:
        preflight_stride = max(1, args.preflight_stride)

    # Setup output directory
    timestamp_str = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    if args.output_dir:
        run_dir = Path(args.output_dir)
    else:
        run_dir = REPO_ROOT / "experiments" / f"run_{timestamp_str}"

    timeseries_dir = run_dir / "timeseries"
    plots_dir = run_dir / "plots"
    run_dir.mkdir(parents=True, exist_ok=True)
    timeseries_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("VisionRobotTwin Task-Space Benchmark Experiment Suite")
    print(f"Target Directory: {run_dir}")
    print(f"Experiments: {experiments_to_run}")
    print(f"Robots: {robots} | Controllers: {controllers} | Deterministic Repeats: {args.repeats}")
    print(f"Preflight Stride: {preflight_stride} | Trajectory Duration Override: {args.trajectory_duration}")
    print("=" * 70)

    # 1. Shared Feasibility Preflight
    print("\n--- PHASE 1: Shared Feasibility Preflight ---")
    preflight_records: Dict[str, Any] = {}
    for exp_name in experiments_to_run:
        exp_def = EXPERIMENT_DEFINITIONS[exp_name]
        pf = check_shared_feasibility(
            exp_def,
            sample_stride=preflight_stride,
            duration_override_s=args.trajectory_duration,
            physics_hz=args.physics_hz,
        )
        preflight_records[exp_name] = pf.to_dict()
        status_str = "SHARED FEASIBLE" if pf.shared_feasible else "FEASIBILITY FAILED"
        print(f"[{exp_name.upper()}]: {status_str} (Panda: {pf.panda_feasible}, KUKA: {pf.kuka_feasible}, Samples Checked: {pf.checked_samples}/{pf.trajectory_total_samples})")

    # 2. Execute Tracking Trials (Gated by Preflight)
    print("\n--- PHASE 2: Executing Benchmark Trials ---")
    all_trial_metrics: List[Dict[str, Any]] = []
    timeseries_data: Dict[str, List[Dict[str, Any]]] = {}
    skipped_experiments: Dict[str, Dict[str, Any]] = {}

    for exp_name in experiments_to_run:
        exp_def = EXPERIMENT_DEFINITIONS[exp_name]
        pf_data = preflight_records[exp_name]

        if not pf_data.get("shared_feasible", False):
            # Gated: Skip trials for non-feasible trajectories
            expected_trials = len(robots) * len(controllers) * args.repeats
            print(f"[{exp_name.upper()}]: SKIPPING ALL {expected_trials} TRIALS (SHARED FEASIBILITY FAILED: {pf_data.get('failure_reason', '')})")
            skipped_experiments[exp_name] = {
                "status": "SHARED_FEASIBILITY_FAILED",
                "skipped": True,
                "skipped_trial_count": expected_trials,
                "failure_diagnostics": pf_data,
            }
            continue

        for robot in robots:
            for ctrl in controllers:
                for repeat in range(args.repeats):
                    trial_seed = args.seed + repeat
                    print(f"Executing: {exp_name} | {robot} | {ctrl} | Trial {repeat+1}/{args.repeats} ...", end=" ", flush=True)

                    metrics, ts_records = execute_experiment_trial(
                        experiment_def=exp_def,
                        robot_name=robot,
                        controller_type=ctrl,
                        trial_index=repeat,
                        trajectory_duration=args.trajectory_duration,
                        physics_hz=args.physics_hz,
                        gui=gui_mode,
                        seed=trial_seed,
                    )

                    m_dict = metrics.to_dict()
                    m_dict["experiment_name"] = exp_name
                    m_dict["robot_name"] = robot
                    m_dict["controller_type"] = ctrl
                    m_dict["trial_index"] = repeat
                    all_trial_metrics.append(m_dict)
                    ts_key = f"{exp_name}_{robot}_{ctrl}_t{repeat}"
                    timeseries_data[ts_key] = ts_records

                    # Save per-trial timeseries CSV
                    ts_csv_path = timeseries_dir / f"{ts_key}.csv"
                    if ts_records:
                        with open(ts_csv_path, "w", newline="", encoding="utf-8") as f:
                            writer = csv.DictWriter(f, fieldnames=list(ts_records[0].keys()))
                            writer.writeheader()
                            writer.writerows(ts_records)

                    status = "SUCCESS" if metrics.success else f"FAILED ({metrics.failure_reason})"
                    print(f"{status} (RMSE: {metrics.rmse_position_error_mm:.2f}mm, Orn: {metrics.mean_orientation_error_deg:.2f}°)")

    # 3. Execute Planning Experiment
    print("\n--- PHASE 3: Obstacle-Blocked Motion Planning Experiment ---")
    planning_records: Dict[str, Any] = {}
    for robot in robots:
        print(f"Planning Obstacle Reach on {robot} ...", end=" ", flush=True)
        try:
            p_res = execute_obstacle_reach_experiment(
                robot_name=robot,
                seed=args.seed,
                physics_hz=args.physics_hz,
                gui=gui_mode,
            )
            planning_records[robot] = p_res.to_dict()
            p_status = "SUCCESS" if p_res.execution_success else "FAILED"
            print(f"{p_status} ({p_res.details})")
        except Exception as e:
            logger.error(f"Planning experiment error on {robot}: {e}")
            planning_records[robot] = {"error": str(e), "execution_success": False}
            print(f"ERROR ({e})")

    # 4. Compute Aggregate Statistics
    summary_dict: Dict[str, Any] = {
        "timestamp_utc": timestamp_str,
        "tasks": {},
        "planning": planning_records,
    }

    for exp_name in experiments_to_run:
        if exp_name in skipped_experiments:
            summary_dict["tasks"][exp_name] = skipped_experiments[exp_name]
            continue

        summary_dict["tasks"][exp_name] = {}
        for robot in robots:
            for ctrl in controllers:
                matching = [
                    m for m in all_trial_metrics
                    if m["experiment_name"] == exp_name and m["robot_name"] == robot and m["controller_type"] == ctrl
                ]
                if not matching:
                    continue

                def _stat(key: str) -> Dict[str, float]:
                    vals = [float(m[key]) for m in matching if key in m]
                    if not vals:
                        return {"mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
                    return {
                        "mean": float(np.mean(vals)),
                        "std": float(np.std(vals)),
                        "min": float(np.min(vals)),
                        "max": float(np.max(vals)),
                    }

                successes = [m for m in matching if m["success"]]
                summary_dict["tasks"][exp_name][f"{robot}_{ctrl}"] = {
                    "trials_count": len(matching),
                    "success_count": len(successes),
                    "success_rate": len(successes) / max(1, len(matching)),
                    "rmse_position_error_mm": _stat("rmse_position_error_mm"),
                    "p95_position_error_mm": _stat("p95_position_error_mm"),
                    "max_position_error_mm": _stat("max_position_error_mm"),
                    "final_position_error_mm": _stat("final_position_error_mm"),
                    "mean_orientation_error_deg": _stat("mean_orientation_error_deg"),
                    "p95_orientation_error_deg": _stat("p95_orientation_error_deg"),
                    "total_joint_travel_rad": _stat("total_joint_travel_rad"),
                    "mean_manipulability": _stat("mean_manipulability"),
                    "min_manipulability": _stat("min_manipulability"),
                    "min_sigma_min": _stat("min_sigma_min"),
                    "singularity_warning_ratio": _stat("singularity_warning_ratio"),
                }

    # 5. Export Manifest, Summary JSON, Trial Results CSV
    manifest_obj = generate_experiment_manifest(
        robot_names=robots,
        controller_types=controllers,
        deterministic_repeats=args.repeats,
        random_seed=args.seed,
        physics_hz=args.physics_hz,
        trajectory_duration=args.trajectory_duration,
        preflight_sample_stride=preflight_stride,
        git_commit_sha=args.commit_sha,
    )
    manifest_path = run_dir / "manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest_obj.to_dict(), f, indent=2)

    summary_path = run_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary_dict, f, indent=2)

    trial_csv_path = run_dir / "trial_results.csv"
    if all_trial_metrics:
        with open(trial_csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(all_trial_metrics[0].keys()))
            writer.writeheader()
            writer.writerows(all_trial_metrics)

    # 6. Verify Results Consistency
    verify_experiment_results_consistency(summary_dict, all_trial_metrics, manifest_obj.to_dict())

    # 7. Generate Publication-Quality Plots
    print("\n--- PHASE 4: Generating Publication-Quality Visualizations ---")
    plot_rel_paths = generate_experiment_plots(
        timeseries_data=timeseries_data,
        summary_results=summary_dict,
        planning_results=planning_records,
        plots_dir=plots_dir,
    )
    for p_path in plot_rel_paths:
        print(f"Generated plot: {p_path}")

    # 8. Generate Comprehensive REPORT.md
    print("\n--- PHASE 5: Generating Research Report ---")
    report_path = run_dir / "REPORT.md"
    generate_experiment_report_markdown(
        manifest=manifest_obj.to_dict(),
        preflight_results=preflight_records,
        summary_results=summary_dict,
        planning_results=planning_records,
        plot_rel_paths=plot_rel_paths,
        output_report_path=report_path,
        all_trial_metrics=all_trial_metrics,
    )

    # 9. Reference Results Publishing
    if args.publish_reference:
        ref_dir = REPO_ROOT / "docs" / "experiments" / "v1.2_reference"
        ref_dir.mkdir(parents=True, exist_ok=True)
        ref_plots_dir = ref_dir / "plots"
        ref_plots_dir.mkdir(parents=True, exist_ok=True)

        shutil.copy2(manifest_path, ref_dir / "manifest.json")
        shutil.copy2(summary_path, ref_dir / "summary.json")
        shutil.copy2(trial_csv_path, ref_dir / "trial_results.csv")
        shutil.copy2(report_path, ref_dir / "REPORT.md")

        for p_file in plots_dir.glob("*.png"):
            shutil.copy2(p_file, ref_plots_dir / p_file.name)

        print(f"\n[REFERENCE PACKAGE PUBLISHED] -> {ref_dir}")

    print("\n" + "=" * 70)
    print("EXPERIMENT SUITE COMPLETE")
    print(f"Artifacts written to: {run_dir}")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
