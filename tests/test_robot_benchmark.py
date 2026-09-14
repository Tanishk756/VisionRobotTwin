"""Tests for Cross-Robot and Cross-Controller Benchmarking Tools."""

import pytest
import numpy as np

from tools.compare_robots import generate_shared_benchmark_targets, benchmark_robot
from tools.compare_controllers import benchmark_controller


def test_robot_benchmark_execution_and_schema():
    """Verifies robot benchmark tool runs bounded headless test and produces valid metrics."""
    targets, meta = generate_shared_benchmark_targets(seed=42, target_count=3, headless=True)
    assert len(targets) > 0
    assert meta["accepted_shared_targets"] == len(targets)

    # Benchmark Panda
    res_panda = benchmark_robot("panda", targets, headless=True)
    assert res_panda["robot_id"] == "panda"
    assert res_panda["dof"] == 7
    assert res_panda["has_gripper"] is True
    assert 0.0 <= res_panda["ik_success_rate"] <= 1.0
    assert np.isfinite(res_panda["mean_ik_solve_time_ms"])
    assert np.isfinite(res_panda["mean_manipulability"])
    assert 0.0 <= res_panda["rrt_planning_success_rate"] <= 1.0

    # Benchmark KUKA
    res_kuka = benchmark_robot("kuka_iiwa", targets, headless=True)
    assert res_kuka["robot_id"] == "kuka_iiwa"
    assert res_kuka["dof"] == 7
    assert res_kuka["has_gripper"] is False
    assert 0.0 <= res_kuka["ik_success_rate"] <= 1.0
    assert np.isfinite(res_kuka["mean_ik_solve_time_ms"])
    assert np.isfinite(res_kuka["mean_manipulability"])


def test_controller_benchmark_execution_and_schema():
    """Verifies controller benchmark tool runs bounded comparison on IK and Resolved-Rate."""
    trajectories = [
        (
            np.array([0.40, 0.0, 0.30]),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.array([0.42, 0.02, 0.32]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
    ]

    res_ik = benchmark_controller("panda", "ik", trajectories, headless=True, trajectory_duration=1.0)
    assert res_ik["controller_name"] == "IK"
    assert np.isfinite(res_ik["mean_position_tracking_error_mm"])
    assert res_ik["total_trajectories"] == 1
    assert "measured_peak_joint_velocity_radps" in res_ik
    assert "final_position_error_mm" in res_ik

    res_rr = benchmark_controller("panda", "resolved-rate", trajectories, headless=True, trajectory_duration=1.0)
    assert res_rr["controller_name"] == "RESOLVED-RATE"
    assert np.isfinite(res_rr["mean_position_tracking_error_mm"])
    assert np.isfinite(res_rr["measured_peak_joint_velocity_radps"])
    assert "measured_peak_joint_velocity_radps" in res_rr
    assert "final_position_error_mm" in res_rr
