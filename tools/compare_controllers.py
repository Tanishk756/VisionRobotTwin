"""Cross-Controller (IK vs Resolved-Rate Jacobian) Benchmark Tool.

Compares standard inverse kinematics position control against resolved-rate Cartesian
differential velocity control with adaptive DLS and null-space redundancy on the same robot:
- Cartesian position and orientation tracking accuracy
- Joint velocity smoothness and peak velocities
- Joint travel distance
- Manipulability and singularity warning occurrences
- Execution time and convergence

Outputs summary.json and results.csv to benchmarks/controller_comparison_<timestamp>/
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Tuple
import numpy as np
import pybullet as p
import pybullet_data

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from visionrobottwin_version import __version__
from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.inverse_kinematics import GenericIKSolver
from robotics.differential_ik import ResolvedRateController
from robotics.kinematics import compute_jacobian, compute_manipulability
from robotics.trajectory import CartesianSE3Trajectory
from utils.logger import get_logger

logger = get_logger("Tools.CompareControllers")


def benchmark_controller(
    robot_name: str,
    controller_type: str,
    trajectories: List[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]],
    headless: bool = True,
    trajectory_duration: float = 2.0,
    settle_duration: float = 0.5,
    settle_tolerance_mm: float = 5.0,
    random_seed: int = 42,
) -> Dict[str, Any]:
    """Runs trajectory tracking and convergence benchmark for a given controller type."""
    client_id = p.connect(p.DIRECT if headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)

    registry = get_robot_registry()
    spec = registry.get_robot_spec(robot_name)

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    lows, highs, ranges, rests = controller.get_joint_limits()
    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )

    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        kp_pos=6.0,
        kp_orn=3.5,
        enable_nullspace=True,
    )

    pos_errors_mm = []
    orn_errors_deg = []
    final_pos_errors_mm = []
    final_orn_errors_deg = []
    settled_flags = []
    times_to_final_goal_tolerance_s = []

    measured_peak_joint_velocities = []
    finite_diff_peak_joint_velocities = []
    total_joint_travel_rad = 0.0
    manipulabilities = []
    singularity_warnings = 0
    total_benchmark_time_s = 0.0

    dt = 1.0 / 240.0
    steps_traj = max(10, int(trajectory_duration / dt))
    steps_settle = max(1, int(settle_duration / dt)) if settle_duration > 0 else 0
    total_steps = steps_traj + steps_settle

    requested_count = len(trajectories)
    executed_count = 0
    skipped_count = 0

    for p_start, q_start, p_goal, q_goal in trajectories:
        # 1. Fair and identical initialization:
        # Solve IK for start pose and directly initialize simulator state
        ik_init = ik_solver.solve(p_start, q_start)
        if not ik_init.success:
            skipped_count += 1
            continue

        executed_count += 1
        q_init = list(ik_init.joint_positions)
        # Directly reset joint positions to start configuration
        for j_idx, pos in zip(controller.arm_joint_indices, q_init):
            p.resetJointState(body_id, j_idx, float(pos), targetVelocity=0.0, physicsClientId=client_id)
        controller.set_arm_joint_velocities([0.0] * len(controller.arm_joint_indices))

        # Settle simulation state
        for _ in range(5):
            p.stepSimulation(physicsClientId=client_id)

        rr_controller.reset()

        traj = CartesianSE3Trajectory(p_start, q_start, p_goal, q_goal, duration=trajectory_duration)
        prev_q = np.array(controller.get_current_joint_positions(), dtype=np.float64)

        time_to_goal_tol = None

        # Start timer AFTER initialization is complete
        t_start = time.perf_counter()

        for step_idx in range(total_steps):
            t_curr = step_idx * dt
            if step_idx < steps_traj:
                sample = traj.evaluate(t_curr)
                cmd_pos = sample.position
                cmd_orn = sample.orientation
            else:
                # Settle phase: continue commanding final goal
                cmd_pos = p_goal
                cmd_orn = q_goal

            if controller_type == "ik":
                # IK position control with velocity rate limiting
                res = ik_solver.solve(cmd_pos, cmd_orn)
                if res.success:
                    controller.set_arm_joint_positions(
                        list(res.joint_positions), dt=dt, enforce_velocity_limits=True
                    )
            elif controller_type == "resolved-rate":
                # Genuine velocity control command
                q_dot, m_metrics = rr_controller.compute_step(cmd_pos, cmd_orn, dt=dt)
                if m_metrics.near_singularity:
                    singularity_warnings += 1
                controller.set_arm_joint_velocities(q_dot)

            p.stepSimulation(physicsClientId=client_id)

            # Record simulated pose metrics
            curr_pos, curr_orn = controller.get_end_effector_pose()

            # Dynamic tracking error against trajectory sample
            if step_idx < steps_traj:
                pos_err_mm = float(np.linalg.norm(sample.position - curr_pos)) * 1000.0
                pos_errors_mm.append(pos_err_mm)

                dot = np.abs(np.dot(sample.orientation / np.linalg.norm(sample.orientation), curr_orn / np.linalg.norm(curr_orn)))
                orn_err_deg = float(np.degrees(2.0 * np.arccos(np.clip(dot, -1.0, 1.0))))
                orn_errors_deg.append(orn_err_deg)

            # Check convergence to final goal pose
            dist_to_goal_mm = float(np.linalg.norm(p_goal - curr_pos)) * 1000.0
            if time_to_goal_tol is None and dist_to_goal_mm <= settle_tolerance_mm:
                time_to_goal_tol = t_curr

            # Real PyBullet measured joint velocity
            meas_vels = controller.get_current_joint_velocities()
            if len(meas_vels) > 0:
                measured_peak_joint_velocities.append(float(np.max(np.abs(meas_vels))))

            # Finite-difference velocity diagnostic
            curr_q_vec = np.array(controller.get_current_joint_positions(), dtype=np.float64)
            q_dot_fd = (curr_q_vec - prev_q) / dt
            finite_diff_peak_joint_velocities.append(float(np.max(np.abs(q_dot_fd))))
            total_joint_travel_rad += float(np.sum(np.abs(curr_q_vec - prev_q)))
            prev_q = curr_q_vec.copy()

            # Manipulability
            _, _, J = compute_jacobian(
                client_id, body_id, controller.ee_link_index, controller.arm_joint_indices, list(curr_q_vec)
            )
            m = compute_manipulability(J)
            manipulabilities.append(m.manipulability)

        total_benchmark_time_s += (time.perf_counter() - t_start)

        final_pos_err = float(np.linalg.norm(p_goal - curr_pos)) * 1000.0
        final_dot = np.abs(np.dot(q_goal / np.linalg.norm(q_goal), curr_orn / np.linalg.norm(curr_orn)))
        final_orn_err = float(np.degrees(2.0 * np.arccos(np.clip(final_dot, -1.0, 1.0))))

        final_pos_errors_mm.append(final_pos_err)
        final_orn_errors_deg.append(final_orn_err)
        settled_flags.append(final_pos_err <= settle_tolerance_mm)
        times_to_final_goal_tolerance_s.append(time_to_goal_tol if time_to_goal_tol is not None else float("nan"))

    p.disconnect(physicsClientId=client_id)

    valid_tol_times = [t for t in times_to_final_goal_tolerance_s if np.isfinite(t)]

    return {
        "visionrobottwin_version": __version__,
        "controller_name": controller_type.upper(),
        "robot_name": robot_name,
        "physics_hz": 240,
        "dt": float(dt),
        "trajectory_duration_s": float(trajectory_duration),
        "settle_duration_s": float(settle_duration),
        "settle_tolerance_mm": float(settle_tolerance_mm),
        "random_seed": int(random_seed),
        "total_trajectories": int(requested_count),
        "requested_trajectories": int(requested_count),
        "executed_trajectories": int(executed_count),
        "skipped_initialization_failures": int(skipped_count),
        "completion_rate": float(executed_count / max(requested_count, 1)),
        "mean_position_tracking_error_mm": float(np.mean(pos_errors_mm)) if pos_errors_mm else 0.0,
        "p95_position_tracking_error_mm": float(np.percentile(pos_errors_mm, 95)) if pos_errors_mm else 0.0,
        "final_position_error_mm": float(np.mean(final_pos_errors_mm)) if final_pos_errors_mm else 0.0,
        "mean_orientation_error_deg": float(np.mean(orn_errors_deg)) if orn_errors_deg else 0.0,
        "final_orientation_error_deg": float(np.mean(final_orn_errors_deg)) if final_orn_errors_deg else 0.0,
        "settled_within_tolerance_rate": float(np.mean(settled_flags)) if settled_flags else 0.0,
        "time_to_final_goal_tolerance_s": float(np.mean(valid_tol_times)) if valid_tol_times else None,
        "measured_peak_joint_velocity_radps": float(np.max(measured_peak_joint_velocities)) if measured_peak_joint_velocities else 0.0,
        "finite_difference_peak_joint_velocity_radps": float(np.max(finite_diff_peak_joint_velocities)) if finite_diff_peak_joint_velocities else 0.0,
        "total_joint_travel_rad": float(total_joint_travel_rad),
        "min_manipulability": float(np.min(manipulabilities)) if manipulabilities else 0.0,
        "singularity_warning_count": int(singularity_warnings),
        "total_benchmark_time_s": float(total_benchmark_time_s),
    }


def main():
    parser = argparse.ArgumentParser(description="Cross-Controller (IK vs Resolved-Rate) Benchmark Tool.")
    parser.add_argument("--robot", type=str, default="panda", help="Robot model (panda or kuka_iiwa).")
    parser.add_argument("--trials", type=int, default=10, help="Number of test trajectories.")
    parser.add_argument("--trajectory-duration", type=float, default=2.0, help="Duration of each benchmark trajectory (s).")
    parser.add_argument("--settle-duration", type=float, default=0.5, help="Duration to settle after trajectory (s).")
    parser.add_argument("--settle-tolerance-mm", type=float, default=5.0, help="Goal settling position tolerance (mm).")
    parser.add_argument("--headless", action="store_true", default=True, help="Run simulation headless.")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory path.")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir) if args.output_dir else Path(f"benchmarks/controller_comparison_{timestamp}")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*75}")
    print(f" VisionRobotTwin Controller Benchmark Suite ({args.robot.upper()}) [PyBullet Simulation]")
    print(f" Comparing: IK Position vs Resolved-Rate Jacobian Control (Duration: {args.trajectory_duration:.1f}s, Settle: {args.settle_duration:.1f}s @ {args.settle_tolerance_mm:.1f}mm)")
    print(f"{'='*75}\n")

    # Generate reproducible benchmark trajectories
    rng = np.random.RandomState(42)
    trajectories = []
    for _ in range(args.trials):
        p0 = np.array([rng.uniform(0.35, 0.45), rng.uniform(-0.15, 0.15), rng.uniform(0.20, 0.35)])
        p1 = p0 + np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06), rng.uniform(-0.04, 0.06)])
        q0 = np.array([1.0, 0.0, 0.0, 0.0])
        q1 = np.array([0.9238795, 0.3826834, 0.0, 0.0])  # ~45 deg roll
        trajectories.append((p0, q0, p1, q1))

    results = {}
    for c_type in ["ik", "resolved-rate"]:
        print(f"[*] Benchmarking {c_type.upper()} controller ({args.trials} trajectories @ {args.trajectory_duration:.1f}s + {args.settle_duration:.1f}s settle)...", flush=True)
        res = benchmark_controller(
            args.robot,
            c_type,
            trajectories,
            headless=args.headless,
            trajectory_duration=args.trajectory_duration,
            settle_duration=args.settle_duration,
            settle_tolerance_mm=args.settle_tolerance_mm,
            random_seed=42,
        )
        results[c_type] = res

    # Save summary.json
    summary_path = out_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    # Save results.csv
    csv_path = out_dir / "results.csv"
    import csv
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = list(next(iter(results.values())).keys())
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for c_res in results.values():
            writer.writerow(c_res)

    # Print Table
    print(f"\n{'-'*85}")
    print(f"{'Metric':<38} | {'IK Position':<20} | {'Resolved-Rate':<20}")
    print(f"{'-'*85}")
    keys_to_print = [
        ("Requested / Executed", "executed_trajectories", "{:d}"),
        ("Completion Rate", "completion_rate", "{:.1%}"),
        ("Mean Pos Tracking Error (mm)", "mean_position_tracking_error_mm", "{:.3f} mm"),
        ("P95 Pos Tracking Error (mm)", "p95_position_tracking_error_mm", "{:.3f} mm"),
        ("Final Pos Error (mm)", "final_position_error_mm", "{:.3f} mm"),
        ("Mean Orientation Error (deg)", "mean_orientation_error_deg", "{:.3f} deg"),
        ("Final Orientation Error (deg)", "final_orientation_error_deg", "{:.3f} deg"),
        ("Settled Within Tolerance Rate", "settled_within_tolerance_rate", "{:.1%}"),
        ("Time to Final Goal Tolerance (s)", "time_to_final_goal_tolerance_s", "{:.3f} s"),
        ("Measured Peak Joint Vel (rad/s)", "measured_peak_joint_velocity_radps", "{:.3f} rad/s"),
        ("Total Joint Travel (rad)", "total_joint_travel_rad", "{:.3f} rad"),
        ("Min Manipulability", "min_manipulability", "{:.4f}"),
        ("Singularity Warnings", "singularity_warning_count", "{:d}"),
        ("Benchmark Wall Time (s)", "total_benchmark_time_s", "{:.3f} s"),
    ]

    for label, k, fmt in keys_to_print:
        ik_val = results.get("ik", {}).get(k, "N/A")
        rr_val = results.get("resolved-rate", {}).get(k, "N/A")
        ik_str = fmt.format(ik_val) if isinstance(ik_val, (int, float, bool)) else str(ik_val)
        rr_str = fmt.format(rr_val) if isinstance(rr_val, (int, float, bool)) else str(rr_val)
        print(f"{label:<38} | {ik_str:<20} | {rr_str:<20}")
    print(f"{'-'*85}")
    print(f"\n[+] Benchmark complete. Artifacts exported to: {out_dir}\n")


if __name__ == "__main__":
    main()
