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
) -> Dict[str, Any]:
    """Runs trajectory tracking benchmark for a given controller type."""
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
    joint_velocities = []
    joint_travel_rad = 0.0
    manipulabilities = []
    singularity_warnings = 0
    total_time_s = 0.0

    dt = 1.0 / 240.0
    steps_per_traj = 120  # 0.5s per trajectory

    for p_start, q_start, p_goal, q_goal in trajectories:
        # Reset robot to start pose using IK
        ik_init = ik_solver.solve(p_start, q_start)
        if ik_init.success:
            controller.set_arm_joint_positions(list(ik_init.joint_positions))
        p.stepSimulation(physicsClientId=client_id)

        traj = CartesianSE3Trajectory(p_start, q_start, p_goal, q_goal, duration=steps_per_traj * dt)
        prev_q = np.array(controller.get_current_joint_positions())

        t_start = time.perf_counter()

        for step in range(steps_per_traj):
            t_curr = step * dt
            sample = traj.evaluate(t_curr)

            if controller_type == "ik":
                # IK position control
                res = ik_solver.solve(sample.position, sample.orientation)
                if res.success:
                    controller.set_arm_joint_positions(list(res.joint_positions))
            elif controller_type == "resolved-rate":
                # Resolved-rate differential velocity control
                q_dot, m_metrics = rr_controller.compute_step(sample.position, sample.orientation, dt=dt)
                if m_metrics.near_singularity:
                    singularity_warnings += 1

                curr_q = np.array(controller.get_current_joint_positions())
                q_next = curr_q + q_dot * dt
                controller.set_arm_joint_positions(list(q_next))

            p.stepSimulation(physicsClientId=client_id)

            # Record metrics
            curr_pos, curr_orn = controller.get_end_effector_pose()
            pos_err = np.linalg.norm(sample.position - curr_pos) * 1000.0  # mm
            pos_errors_mm.append(pos_err)

            dot = np.abs(np.dot(sample.orientation / np.linalg.norm(sample.orientation), curr_orn / np.linalg.norm(curr_orn)))
            orn_err = np.degrees(2.0 * np.arccos(np.clip(dot, -1.0, 1.0)))
            orn_errors_deg.append(orn_err)

            curr_q_vec = np.array(controller.get_current_joint_positions())
            q_dot_est = (curr_q_vec - prev_q) / dt
            joint_velocities.append(np.max(np.abs(q_dot_est)))
            joint_travel_rad += float(np.sum(np.abs(curr_q_vec - prev_q)))
            prev_q = curr_q_vec

            # Manipulability
            _, _, J = compute_jacobian(
                client_id, body_id, controller.ee_link_index, controller.arm_joint_indices, list(curr_q_vec)
            )
            m = compute_manipulability(J)
            manipulabilities.append(m.manipulability)

        total_time_s += (time.perf_counter() - t_start)

    p.disconnect(physicsClientId=client_id)

    return {
        "controller_name": controller_type.upper(),
        "robot_name": robot_name,
        "total_trajectories": len(trajectories),
        "mean_position_tracking_error_mm": float(np.mean(pos_errors_mm)),
        "p95_position_tracking_error_mm": float(np.percentile(pos_errors_mm, 95)),
        "mean_orientation_error_deg": float(np.mean(orn_errors_deg)),
        "peak_joint_velocity_radps": float(np.max(joint_velocities)) if joint_velocities else 0.0,
        "total_joint_travel_rad": float(joint_travel_rad),
        "min_manipulability": float(np.min(manipulabilities)) if manipulabilities else 0.0,
        "singularity_warning_count": int(singularity_warnings),
        "total_benchmark_time_s": float(total_time_s),
    }


def main():
    parser = argparse.ArgumentParser(description="Cross-Controller (IK vs Resolved-Rate) Benchmark Tool.")
    parser.add_argument("--robot", type=str, default="panda", help="Robot model (panda or kuka_iiwa).")
    parser.add_argument("--trials", type=int, default=10, help="Number of test trajectories.")
    parser.add_argument("--headless", action="store_true", default=True, help="Run simulation headless.")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory path.")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir) if args.output_dir else Path(f"benchmarks/controller_comparison_{timestamp}")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*75}")
    print(f" VisionRobotTwin Controller Benchmark Suite ({args.robot.upper()})")
    print(f" Comparing: IK Position vs Resolved-Rate Jacobian Control")
    print(f"{'='*75}\n")

    # Generate benchmark trajectories
    rng = np.random.RandomState(42)
    trajectories = []
    for _ in range(args.trials):
        p0 = np.array([rng.uniform(0.35, 0.45), rng.uniform(-0.15, 0.15), rng.uniform(0.20, 0.35)])
        p1 = p0 + np.array([rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08), rng.uniform(-0.05, 0.08)])
        q0 = np.array([1.0, 0.0, 0.0, 0.0])
        q1 = np.array([0.9238795, 0.3826834, 0.0, 0.0])  # ~45 deg roll
        trajectories.append((p0, q0, p1, q1))

    results = {}
    for c_type in ["ik", "resolved-rate"]:
        print(f"[*] Benchmarking {c_type.upper()} controller ({args.trials} trajectories)...", flush=True)
        res = benchmark_controller(args.robot, c_type, trajectories, headless=args.headless)
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
    print(f"{'Metric':<35} | {'IK Position':<22} | {'Resolved-Rate':<22}")
    print(f"{'-'*85}")
    keys_to_print = [
        ("Mean Pos Tracking Error (mm)", "mean_position_tracking_error_mm", "{:.3f} mm"),
        ("P95 Pos Tracking Error (mm)", "p95_position_tracking_error_mm", "{:.3f} mm"),
        ("Mean Orientation Error (deg)", "mean_orientation_error_deg", "{:.3f} deg"),
        ("Peak Joint Velocity (rad/s)", "peak_joint_velocity_radps", "{:.3f} rad/s"),
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
        print(f"{label:<35} | {ik_str:<22} | {rr_str:<22}")
    print(f"{'-'*85}")
    print(f"\n[+] Benchmark complete. Artifacts exported to: {out_dir}\n")


if __name__ == "__main__":
    main()
