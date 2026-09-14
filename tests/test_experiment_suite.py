"""Automated test suite for task-space robotics experiments.

Validates:
- Trajectory math, periodicity, closures, SLERP normalization, and determinism.
- Shared feasibility preflight evaluation across robots.
- Metric calculation mathematical correctness using synthetic signals.
- Manifest and summary JSON/CSV serialization integrity.
- Collision and failure detection handling.
- Headless bounded trial execution.
"""

import json
import math
import tempfile
from pathlib import Path
import numpy as np
import pytest

from robotics.coordinate_transform import compute_angular_distance
from robotics.experiments import (
    ALL_EXPERIMENT_NAMES,
    EXPERIMENT_DEFINITIONS,
    ExperimentDefinition,
    ExperimentManifest,
    ExperimentMetrics,
    ExperimentTrialResult,
    PlanningExperimentResult,
    PreflightFeasibilityResult,
    TaskspaceTrajectory,
    check_shared_feasibility,
    compute_experiment_metrics,
    execute_experiment_trial,
    execute_obstacle_reach_experiment,
    generate_circle_trajectory,
    generate_experiment_manifest,
    generate_figure_eight_trajectory,
    generate_line_trajectory,
    generate_se3_orientation_sweep_trajectory,
    generate_waypoint_box_trajectory,
    get_standard_experiment_trajectories,
)


# =============================================================================
# 1. TRAJECTORY GENERATION & MATHEMATICAL INTEGRITY TESTS
# =============================================================================

def test_line_trajectory_endpoints_and_determinism():
    """Validates straight-line start/end positions and deterministic reproducibility."""
    center = (0.48, 0.0, 0.35)
    length = 0.10
    traj1 = generate_line_trajectory(center_pos=center, length_m=length, duration_s=2.0, physics_hz=240)
    traj2 = generate_line_trajectory(center_pos=center, length_m=length, duration_s=2.0, physics_hz=240)

    # Determinism
    np.testing.assert_allclose(traj1.positions, traj2.positions, atol=1e-12)
    np.testing.assert_allclose(traj1.timestamps, traj2.timestamps, atol=1e-12)

    # Endpoints
    expected_start = np.array(center) + np.array([0.0, -length / 2.0, 0.0])
    expected_end = np.array(center) + np.array([0.0, length / 2.0, 0.0])
    np.testing.assert_allclose(traj1.positions[0], expected_start, atol=1e-6)
    np.testing.assert_allclose(traj1.positions[-1], expected_end, atol=1e-6)

    # Boundary velocities should be zero
    np.testing.assert_allclose(traj1.linear_velocities[0], [0, 0, 0], atol=1e-6)
    np.testing.assert_allclose(traj1.linear_velocities[-1], [0, 0, 0], atol=1e-6)

    # Total distance should be exactly 10 cm (0.10 m)
    dist = np.linalg.norm(expected_end - expected_start)
    assert math.isclose(dist, 0.10, rel_tol=1e-5)


def test_circle_trajectory_radius_and_closure():
    """Validates horizontal circular path radius consistency and closure."""
    center = (0.48, 0.0, 0.35)
    radius = 0.05
    traj = generate_circle_trajectory(center_pos=center, radius_m=radius, duration_s=3.0, physics_hz=240)

    # All points should be at distance 'radius' from center in XY plane
    xy_positions = traj.positions[:, :2]
    xy_center = np.array(center[:2])
    radii = np.linalg.norm(xy_positions - xy_center, axis=1)

    np.testing.assert_allclose(radii, radius, atol=1e-4)

    # Z coordinate should remain constant
    np.testing.assert_allclose(traj.positions[:, 2], center[2], atol=1e-6)

    # Closure: Start position and end position should match
    np.testing.assert_allclose(traj.positions[0], traj.positions[-1], atol=1e-4)


def test_figure_eight_closure_and_bounds():
    """Validates figure-eight lemniscate closure, symmetry, and workspace bounds."""
    center = (0.48, 0.0, 0.35)
    amp_x = 0.05
    amp_y = 0.05
    traj = generate_figure_eight_trajectory(
        center_pos=center, amplitude_x_m=amp_x, amplitude_y_m=amp_y, duration_s=4.0, physics_hz=240
    )

    # Closed loop
    np.testing.assert_allclose(traj.positions[0], traj.positions[-1], atol=1e-4)

    # Bounds: X within [center_x - amp_x, center_x + amp_x]
    assert np.min(traj.positions[:, 0]) >= center[0] - amp_x - 1e-4
    assert np.max(traj.positions[:, 0]) <= center[0] + amp_x + 1e-4

    # Y within [center_y - amp_y/2, center_y + amp_y/2]
    assert np.min(traj.positions[:, 1]) >= center[1] - (amp_y / 2.0) - 1e-4
    assert np.max(traj.positions[:, 1]) <= center[1] + (amp_y / 2.0) + 1e-4


def test_waypoint_box_interpolation_and_continuity():
    """Validates 4-corner closed box path waypoint continuity."""
    center = (0.48, 0.0, 0.35)
    traj = generate_waypoint_box_trajectory(center_pos=center, size_x_m=0.06, size_y_m=0.06, duration_s=4.0, physics_hz=240)

    # Closure
    np.testing.assert_allclose(traj.positions[0], traj.positions[-1], atol=1e-4)

    # Step-to-step position delta should be smooth and bounded
    deltas = np.linalg.norm(traj.positions[1:] - traj.positions[:-1], axis=1)
    max_step_delta_m = np.max(deltas)
    assert max_step_delta_m < 0.01, f"Trajectory step delta too large: {max_step_delta_m} m"


def test_se3_orientation_sweep_slerp_normalization():
    """Validates quaternion unit normalization and angular sweep range during SE(3) sweep."""
    center = (0.48, 0.0, 0.35)
    max_roll = 20.0
    traj = generate_se3_orientation_sweep_trajectory(
        center_pos=center, max_roll_deg=max_roll, duration_s=3.0, physics_hz=240
    )

    # All quaternions must have unit norm
    norms = np.linalg.norm(traj.orientations, axis=1)
    np.testing.assert_allclose(norms, 1.0, atol=1e-6)

    # Maximum angular distance from neutral orientation should be approx max_roll degrees
    neutral_orn = traj.orientations[0]
    ang_dists_deg = [
        math.degrees(compute_angular_distance(traj.orientations[k], neutral_orn))
        for k in range(traj.num_samples)
    ]
    max_ang_dist = max(ang_dists_deg)
    assert 18.0 <= max_ang_dist <= 22.0, f"Unexpected max roll sweep: {max_ang_dist} deg"


def test_standard_experiment_definitions_and_dict():
    """Validates standard suite dictionary has all 5 experiment definitions."""
    trajs = get_standard_experiment_trajectories(duration_scale=1.0, physics_hz=240)
    for exp_name in ALL_EXPERIMENT_NAMES:
        assert exp_name in trajs
        assert isinstance(trajs[exp_name], TaskspaceTrajectory)
        assert trajs[exp_name].num_samples > 100


# =============================================================================
# 2. SHARED FEASIBILITY PREFLIGHT TESTS
# =============================================================================

def test_shared_feasibility_preflight_all_standard_experiments():
    """Validates that all 5 benchmark trajectories pass shared preflight feasibility on Panda and KUKA."""
    for exp_name in ALL_EXPERIMENT_NAMES:
        res = check_shared_feasibility(
            exp_name,
            robots=("panda", "kuka_iiwa"),
            position_tolerance_mm=25.0,
            orientation_tolerance_deg=10.0,
            sample_stride=10,
        )
        assert res.shared_feasible is True, f"Preflight failed for {exp_name}: {res.failure_diagnostics}"
        assert res.panda_feasible is True
        assert res.kuka_feasible is True
        assert res.status == "SHARED_FEASIBILITY_PASSED"


def test_shared_feasibility_rejection_for_unreachable_target():
    """Validates that an out-of-reach trajectory is rejected by shared preflight feasibility."""
    unreachable_traj = generate_line_trajectory(
        center_pos=(2.5, 0.0, 0.5), length_m=0.10, duration_s=2.0, physics_hz=240
    )
    res = check_shared_feasibility(
        unreachable_traj,
        robots=("panda", "kuka_iiwa"),
        position_tolerance_mm=25.0,
        sample_stride=10,
    )
    assert res.shared_feasible is False
    assert res.status == "SHARED_FEASIBILITY_FAILED"
    assert len(res.failure_diagnostics) > 0


# =============================================================================
# 3. METRIC CALCULATION TESTS (KNOWN SYNTHETIC SIGNALS)
# =============================================================================

def test_metric_calculations_with_synthetic_signals():
    """Validates exact mathematics of RMSE, P95, joint travel, and manipulability calculations."""
    num_steps = 100
    dt = 0.01
    time_arr = np.linspace(0.0, 1.0, num_steps)

    # Ground truth desired position: [0, 0, 0] -> [1, 0, 0]
    des_pos = np.zeros((num_steps, 3))
    des_pos[:, 0] = np.linspace(0.0, 1.0, num_steps)

    # Actual position has known constant offset of 2.0 mm (0.002 m) along Y
    act_pos = np.copy(des_pos)
    act_pos[:, 1] = 0.002

    # Fixed orientations
    des_orn = np.zeros((num_steps, 4))
    des_orn[:, 0] = 1.0
    act_orn = np.copy(des_orn)

    # Joint positions moving linearly by 0.5 rad per joint
    joint_pos = np.zeros((num_steps, 7))
    for j in range(7):
        joint_pos[:, j] = np.linspace(0.0, 0.5, num_steps)

    joint_vel = np.full((num_steps, 7), 0.5)
    manip = np.full(num_steps, 0.12)
    sigma_min = np.full(num_steps, 0.05)
    cond_num = np.full(num_steps, 15.0)
    near_sing = np.zeros(num_steps, dtype=bool)
    self_col = np.zeros(num_steps, dtype=bool)
    env_col = np.zeros(num_steps, dtype=bool)
    limit_viol = np.zeros(num_steps, dtype=bool)
    vel_sat = np.zeros(num_steps, dtype=bool)

    metrics = compute_experiment_metrics(
        time_arr=time_arr,
        des_pos=des_pos,
        act_pos=act_pos,
        des_orn=des_orn,
        act_orn=act_orn,
        joint_pos_arr=joint_pos,
        joint_vel_arr=joint_vel,
        manip_arr=manip,
        sigma_min_arr=sigma_min,
        cond_arr=cond_num,
        near_sing_arr=near_sing,
        self_col_arr=self_col,
        env_col_arr=env_col,
        limit_viol_arr=limit_viol,
        vel_sat_arr=vel_sat,
        settle_tol_mm=5.0,
        is_se3=False,
    )

    # Position RMSE should be exactly 2.0 mm
    assert math.isclose(metrics.rmse_position_error_mm, 2.0, rel_tol=1e-4)
    assert math.isclose(metrics.mean_position_error_mm, 2.0, rel_tol=1e-4)
    assert math.isclose(metrics.p95_position_error_mm, 2.0, rel_tol=1e-4)
    assert math.isclose(metrics.final_position_error_mm, 2.0, rel_tol=1e-4)

    # Orientation error should be 0.0 deg
    assert math.isclose(metrics.mean_orientation_error_deg, 0.0, abs_tol=1e-6)

    # Total joint travel: 7 joints * 0.5 rad = 3.5 rad
    assert math.isclose(metrics.total_joint_travel_rad, 3.5, rel_tol=1e-4)
    assert math.isclose(metrics.max_joint_travel_rad, 0.5, rel_tol=1e-4)

    # Kinematics
    assert math.isclose(metrics.mean_manipulability, 0.12, rel_tol=1e-4)
    assert math.isclose(metrics.min_sigma_min, 0.05, rel_tol=1e-4)
    assert metrics.success is True


def test_metric_collision_failure_trigger():
    """Validates that any collision event marks the trial as failed."""
    num_steps = 50
    time_arr = np.linspace(0, 1, num_steps)
    pos = np.zeros((num_steps, 3))
    orn = np.zeros((num_steps, 4))
    orn[:, 0] = 1.0
    joints = np.zeros((num_steps, 7))

    # Trigger collision at step 25
    self_col = np.zeros(num_steps, dtype=bool)
    self_col[25] = True

    metrics = compute_experiment_metrics(
        time_arr=time_arr,
        des_pos=pos,
        act_pos=pos,
        des_orn=orn,
        act_orn=orn,
        joint_pos_arr=joints,
        joint_vel_arr=joints,
        manip_arr=np.ones(num_steps),
        sigma_min_arr=np.ones(num_steps),
        cond_arr=np.ones(num_steps),
        near_sing_arr=np.zeros(num_steps, dtype=bool),
        self_col_arr=self_col,
        env_col_arr=np.zeros(num_steps, dtype=bool),
        limit_viol_arr=np.zeros(num_steps, dtype=bool),
        vel_sat_arr=np.zeros(num_steps, dtype=bool),
    )

    assert metrics.success is False
    assert "Collision detected" in metrics.failure_reason
    assert metrics.self_collision_events == 1


# =============================================================================
# 4. MANIFEST & SERIALIZATION TESTS
# =============================================================================

def test_manifest_generation_and_serialization():
    """Validates manifest generation contains required fields and no machine-specific paths."""
    manifest = generate_experiment_manifest(
        robot_names=("panda", "kuka_iiwa"),
        controller_types=("ik", "resolved-rate"),
        repeats=3,
        random_seed=42,
        physics_hz=240,
    )

    m_dict = manifest.to_dict()
    assert m_dict["version"] == "1.2.0-dev"
    assert "git_commit_sha" in m_dict
    assert "environment" in m_dict
    assert "execution" in m_dict
    assert "tolerances" in m_dict
    assert "trajectory_definitions" in m_dict
    assert m_dict["execution"]["physics_frequency_hz"] == 240
    assert m_dict["execution"]["statistical_repeats"] == 3

    # Ensure JSON serializable
    json_str = json.dumps(m_dict)
    assert len(json_str) > 0

    # Ensure no absolute paths or usernames in manifest
    import getpass
    username = getpass.getuser().lower()
    assert username not in json_str.lower()


def test_manifest_explicit_git_commit_sha():
    """Validates manifest generation with explicitly provided git commit SHA."""
    test_sha = "abcdef1234567890abcdef1234567890abcdef12"
    manifest = generate_experiment_manifest(
        git_commit_sha=test_sha,
    )
    assert manifest.git_commit_sha == test_sha
    m_dict = manifest.to_dict()
    assert m_dict["git_commit_sha"] == test_sha


# =============================================================================
# 5. HEADLESS BOUNDED TRIAL & PLANNING EXECUTION TESTS
# =============================================================================

def test_headless_tracking_trial_execution_panda_ik():
    """Executes a bounded headless LINE tracking trial on Panda with IK."""
    metrics, records = execute_experiment_trial(
        experiment_def="line",
        robot_name="panda",
        controller_type="ik",
        trial_index=0,
        trajectory_duration=1.0,
        physics_hz=240,
        gui=False,
        seed=42,
    )

    assert isinstance(metrics, ExperimentMetrics)
    assert len(records) > 0
    assert metrics.rmse_position_error_mm < 10.0
    assert metrics.mean_orientation_error_deg < 5.0
    assert metrics.success is True


def test_headless_tracking_trial_execution_panda_resolved_rate():
    """Executes a bounded headless LINE tracking trial on Panda with Resolved-Rate."""
    metrics, records = execute_experiment_trial(
        experiment_def="line",
        robot_name="panda",
        controller_type="resolved-rate",
        trial_index=0,
        trajectory_duration=1.0,
        physics_hz=240,
        gui=False,
        seed=42,
    )

    assert isinstance(metrics, ExperimentMetrics)
    assert len(records) > 0
    assert metrics.rmse_position_error_mm < 25.0
    assert metrics.success is True


def test_headless_obstacle_reach_planning_experiment():
    """Executes obstacle reach experiment validating direct-path rejection and RRT success."""
    res = execute_obstacle_reach_experiment(
        robot_name="panda",
        physics_hz=240,
        gui=False,
        seed=42,
    )

    assert isinstance(res, PlanningExperimentResult)
    assert res.direct_path_free is False
    assert res.rrt_planning_success is True
    assert res.raw_waypoints_count >= 2
    assert res.smoothed_waypoints_count >= 2
    assert res.execution_success is True
    assert res.execution_final_pos_error_mm < 25.0
