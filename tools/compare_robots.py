"""Cross-Robot Kinematic, Manipulability, and Motion Planning Benchmark Tool.

Compares Franka Emika Panda and KUKA LBR iiwa across identical reachable targets in PyBullet:
- Inverse Kinematics solve rate, solve latency, and Cartesian / angular accuracy
- Differential kinematics (Yoshikawa manipulability, Jacobian condition number, sigma_min)
- Trajectory execution tracking accuracy
- Collision-aware RRT-Connect planning success rate and latency

Outputs summary.json, results.csv, and comparison charts to benchmarks/robot_comparison_<timestamp>/
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
from robotics.kinematics import compute_jacobian, compute_manipulability, compute_fk_at_configuration
from robotics.trajectory import JointQuinticTrajectory
from robotics.collision import CollisionChecker
from robotics.planning import RRTConnectPlanner, is_joint_path_collision_free
from utils.logger import get_logger

logger = get_logger("Tools.CompareRobots")


def generate_shared_benchmark_targets(
    seed: int = 42,
    target_count: int = 15,
    max_candidates: int = 200,
    headless: bool = True,
    position_tolerance_m: float = 0.025,
    orientation_tolerance_deg: float = 10.0,
) -> Tuple[List[Tuple[np.ndarray, np.ndarray]], Dict[str, Any]]:
    """Generates true shared 6-DoF SE(3) target poses verified reachable within residual tolerances by BOTH Panda and KUKA."""
    client_id = p.connect(p.DIRECT if headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)

    registry = get_robot_registry()
    panda_spec = registry.get_robot_spec("panda")
    kuka_spec = registry.get_robot_spec("kuka_iiwa")

    panda_id = p.loadURDF(panda_spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    kuka_id = p.loadURDF(kuka_spec.urdf_path, useFixedBase=True, physicsClientId=client_id)

    panda_ctrl = GenericRobotController(client_id, panda_id, panda_spec)
    kuka_ctrl = GenericRobotController(client_id, kuka_id, kuka_spec)
    panda_ctrl.reset_to_home()
    kuka_ctrl.reset_to_home()

    p_lows, p_highs, p_rng, p_rst = panda_ctrl.get_joint_limits()
    panda_ik = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=panda_id,
        arm_joint_indices=panda_ctrl.arm_joint_indices,
        lower_limits=p_lows,
        upper_limits=p_highs,
        joint_ranges=p_rng,
        rest_poses=p_rst,
        end_effector_link_index=panda_ctrl.ee_link_index,
        max_reach_m=panda_spec.spherical_reach_m,
        min_reach_m=panda_spec.min_reach_m,
        default_ee_orientation=panda_spec.default_ee_orientation,
    )

    k_lows, k_highs, k_rng, k_rst = kuka_ctrl.get_joint_limits()
    kuka_ik = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=kuka_id,
        arm_joint_indices=kuka_ctrl.arm_joint_indices,
        lower_limits=k_lows,
        upper_limits=k_highs,
        joint_ranges=k_rng,
        rest_poses=k_rst,
        end_effector_link_index=kuka_ctrl.ee_link_index,
        max_reach_m=kuka_spec.spherical_reach_m,
        min_reach_m=kuka_spec.min_reach_m,
        default_ee_orientation=kuka_spec.default_ee_orientation,
    )

    rng = np.random.RandomState(seed)
    shared_orn = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    orn_tol_rad = float(np.radians(orientation_tolerance_deg))

    accepted_targets: List[Tuple[np.ndarray, np.ndarray]] = []
    candidate_count = 0
    rejected_pos_count = 0
    rejected_orn_count = 0

    # Grid search across the shared workspace volume
    xs = np.linspace(0.40, 0.55, 4)
    ys = np.linspace(-0.15, 0.15, 4)
    zs = np.linspace(0.20, 0.35, 3)

    grid_candidates = []
    for x in xs:
        for y in ys:
            for z in zs:
                grid_candidates.append(np.array([x, y, z], dtype=np.float64))

    for pos in grid_candidates:
        if len(accepted_targets) >= target_count:
            break
        candidate_count += 1
        res_p = panda_ik.solve(pos, shared_orn)
        res_k = kuka_ik.solve(pos, shared_orn)

        p_pos_ok = res_p.success and res_p.residual_position_m <= position_tolerance_m
        k_pos_ok = res_k.success and res_k.residual_position_m <= position_tolerance_m
        p_orn_ok = res_p.success and res_p.residual_orientation_rad <= orn_tol_rad
        k_orn_ok = res_k.success and res_k.residual_orientation_rad <= orn_tol_rad

        if p_pos_ok and k_pos_ok and p_orn_ok and k_orn_ok:
            accepted_targets.append((pos, shared_orn.copy()))
        else:
            if not (p_pos_ok and k_pos_ok):
                rejected_pos_count += 1
            elif not (p_orn_ok and k_orn_ok):
                rejected_orn_count += 1

    # Random generation if more targets needed
    while len(accepted_targets) < target_count and candidate_count < max_candidates:
        candidate_count += 1
        pos = np.array([
            rng.uniform(0.38, 0.55),
            rng.uniform(-0.18, 0.18),
            rng.uniform(0.18, 0.38),
        ], dtype=np.float64)
        res_p = panda_ik.solve(pos, shared_orn)
        res_k = kuka_ik.solve(pos, shared_orn)

        p_pos_ok = res_p.success and res_p.residual_position_m <= position_tolerance_m
        k_pos_ok = res_k.success and res_k.residual_position_m <= position_tolerance_m
        p_orn_ok = res_p.success and res_p.residual_orientation_rad <= orn_tol_rad
        k_orn_ok = res_k.success and res_k.residual_orientation_rad <= orn_tol_rad

        if p_pos_ok and k_pos_ok and p_orn_ok and k_orn_ok:
            accepted_targets.append((pos, shared_orn.copy()))
        else:
            if not (p_pos_ok and k_pos_ok):
                rejected_pos_count += 1
            elif not (p_orn_ok and k_orn_ok):
                rejected_orn_count += 1

    p.disconnect(physicsClientId=client_id)

    meta = {
        "requested_targets": target_count,
        "candidate_count": candidate_count,
        "accepted_shared_targets": len(accepted_targets),
        "rejected_position": rejected_pos_count,
        "rejected_orientation": rejected_orn_count,
        "shared_SE3_position_tolerance_mm": float(position_tolerance_m * 1000.0),
        "shared_SE3_orientation_tolerance_deg": float(orientation_tolerance_deg),
    }
    return accepted_targets, meta



def benchmark_robot(
    robot_id_name: str,
    targets: List[Tuple[np.ndarray, np.ndarray]],
    headless: bool = True,
    trajectory_duration: float = 1.5,
) -> Dict[str, Any]:
    """Runs rigorous kinematic, manipulability, dynamic trajectory, and planning benchmark on a single robot model."""
    client_id = p.connect(p.DIRECT if headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)

    # Obstacle for planning benchmark
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.12], physicsClientId=client_id)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.12], rgbaColor=[0.8, 0.2, 0.2, 1.0], physicsClientId=client_id)
    obs_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=col_box,
        baseVisualShapeIndex=vis_box,
        basePosition=[0.55, 0.0, 0.12],
        physicsClientId=client_id,
    )

    registry = get_robot_registry()
    spec = registry.get_robot_spec(robot_id_name)

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()

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

    collision_checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obs_id],
    )

    planner = RRTConnectPlanner(
        lower_limits=lows,
        upper_limits=highs,
        collision_checker=collision_checker,
        arm_joint_indices=controller.arm_joint_indices,
        step_size_rad=0.12,
        max_iterations=400,
        random_seed=42,
    )

    # Metrics collectors
    ik_successes = []
    ik_times_ms = []
    ik_pos_residuals_mm = []
    ik_orn_residuals_deg = []
    manipulabilities = []
    condition_numbers = []
    sigmas_min = []
    dynamic_pos_errors_mm = []
    dynamic_orn_errors_deg = []
    final_pos_errors_mm = []
    final_orn_errors_deg = []
    planning_successes = []
    planning_times_ms = []
    path_lengths_rad = []
    direct_free_count = 0

    dt = 1.0 / 240.0
    steps_per_traj = max(10, int(trajectory_duration / dt))

    prev_q = np.array(controller.get_current_joint_positions(), dtype=np.float64)

    for pos, orn in targets:
        # A. Measure Inverse Kinematics
        ik_res = ik_solver.solve(pos, orn)
        ik_times_ms.append(ik_res.solve_time_ms)
        ik_successes.append(1 if ik_res.success else 0)

        if ik_res.success:
            q_sol = np.array(ik_res.joint_positions, dtype=np.float64)
            ik_pos_residuals_mm.append(ik_res.residual_position_m * 1000.0)
            ik_orn_residuals_deg.append(np.degrees(ik_res.residual_orientation_rad))

            # Measure Jacobian & Manipulability at solved configuration
            _, _, J = compute_jacobian(
                client_id, body_id, controller.ee_link_index, controller.arm_joint_indices, list(q_sol)
            )
            metrics = compute_manipulability(J)
            manipulabilities.append(metrics.manipulability)
            condition_numbers.append(metrics.condition_number if np.isfinite(metrics.condition_number) else 1000.0)
            sigmas_min.append(metrics.sigma_min)

            # B. Measure Dynamic Trajectory Execution
            # Execute time-parameterized quintic joint trajectory from prev_q to q_sol
            traj = JointQuinticTrajectory(prev_q, q_sol, duration=trajectory_duration)
            for step_idx in range(steps_per_traj):
                t_curr = step_idx * dt
                sample = traj.evaluate(t_curr)
                controller.set_arm_joint_positions(
                    list(sample.position), dt=dt, enforce_velocity_limits=True
                )
                p.stepSimulation(physicsClientId=client_id)

                ee_curr_pos, ee_curr_orn = controller.get_end_effector_pose()

                # Desired pose from trajectory sample forward kinematics
                fk_des_pos, fk_des_orn = compute_fk_at_configuration(
                    client_id, body_id, controller.arm_joint_indices, list(sample.position), controller.ee_link_index
                )
                step_pos_err = float(np.linalg.norm(fk_des_pos - ee_curr_pos)) * 1000.0
                dynamic_pos_errors_mm.append(step_pos_err)

                dot_step = float(np.abs(np.dot(
                    fk_des_orn / np.linalg.norm(fk_des_orn),
                    ee_curr_orn / np.linalg.norm(ee_curr_orn),
                )))
                step_orn_err = float(np.degrees(2.0 * np.arccos(np.clip(dot_step, -1.0, 1.0))))
                dynamic_orn_errors_deg.append(step_orn_err)

            # Final trajectory endpoint errors against commanded target
            ee_final_pos, ee_final_orn = controller.get_end_effector_pose()
            final_p_err = float(np.linalg.norm(pos - ee_final_pos)) * 1000.0
            final_pos_errors_mm.append(final_p_err)

            if orn is not None:
                dot_final = float(np.abs(np.dot(
                    orn / np.linalg.norm(orn),
                    ee_final_orn / np.linalg.norm(ee_final_orn),
                )))
                final_o_err = float(np.degrees(2.0 * np.arccos(np.clip(dot_final, -1.0, 1.0))))
            else:
                final_o_err = 0.0
            final_orn_errors_deg.append(final_o_err)

            # C. Collision-Aware Planning from prev_q to q_sol
            is_free, _ = is_joint_path_collision_free(
                prev_q, q_sol, collision_checker, arm_joint_indices=controller.arm_joint_indices
            )
            if is_free:
                direct_free_count += 1

            plan_res = planner.plan(prev_q, q_sol)
            planning_successes.append(1 if plan_res.success else 0)
            planning_times_ms.append(plan_res.planning_time_ms)
            if plan_res.success:
                path_lengths_rad.append(plan_res.path_length_joint_rad)

            prev_q = q_sol.copy()
        else:
            ik_pos_residuals_mm.append(float("nan"))
            ik_orn_residuals_deg.append(float("nan"))
            manipulabilities.append(0.0)
            condition_numbers.append(float("nan"))
            sigmas_min.append(0.0)
            planning_successes.append(0)

    p.disconnect(physicsClientId=client_id)

    valid_ik_pos = [v for v in ik_pos_residuals_mm if np.isfinite(v)]
    valid_ik_orn = [v for v in ik_orn_residuals_deg if np.isfinite(v)]
    valid_cond = [v for v in condition_numbers if np.isfinite(v)]
    valid_dyn_pos = [v for v in dynamic_pos_errors_mm if np.isfinite(v)]
    valid_dyn_orn = [v for v in dynamic_orn_errors_deg if np.isfinite(v)]
    valid_fin_pos = [v for v in final_pos_errors_mm if np.isfinite(v)]
    valid_fin_orn = [v for v in final_orn_errors_deg if np.isfinite(v)]

    result = {
        "robot_id": robot_id_name,
        "display_name": spec.display_name,
        "dof": len(controller.arm_joint_indices),
        "has_gripper": spec.capabilities.has_gripper,
        "total_targets": len(targets),
        "ik_success_rate": float(np.mean(ik_successes)),
        "mean_ik_solve_time_ms": float(np.mean(ik_times_ms)),
        "p95_ik_solve_time_ms": float(np.percentile(ik_times_ms, 95)),
        "mean_ik_residual_position_mm": float(np.mean(valid_ik_pos)) if valid_ik_pos else 0.0,
        "p95_ik_residual_position_mm": float(np.percentile(valid_ik_pos, 95)) if valid_ik_pos else 0.0,
        "mean_ik_residual_orientation_deg": float(np.mean(valid_ik_orn)) if valid_ik_orn else 0.0,
        "min_manipulability": float(np.min(manipulabilities)) if manipulabilities else 0.0,
        "mean_manipulability": float(np.mean(manipulabilities)) if manipulabilities else 0.0,
        "max_jacobian_condition_number": float(np.max(valid_cond)) if valid_cond else 0.0,
        "dynamic_samples_count": len(valid_dyn_pos),
        "mean_dynamic_position_error_mm": float(np.mean(valid_dyn_pos)) if valid_dyn_pos else 0.0,
        "p95_dynamic_position_error_mm": float(np.percentile(valid_dyn_pos, 95)) if valid_dyn_pos else 0.0,
        "mean_dynamic_orientation_error_deg": float(np.mean(valid_dyn_orn)) if valid_dyn_orn else 0.0,
        "p95_dynamic_orientation_error_deg": float(np.percentile(valid_dyn_orn, 95)) if valid_dyn_orn else 0.0,
        "mean_final_position_error_mm": float(np.mean(valid_fin_pos)) if valid_fin_pos else 0.0,
        "p95_final_position_error_mm": float(np.percentile(valid_fin_pos, 95)) if valid_fin_pos else 0.0,
        "mean_final_orientation_error_deg": float(np.mean(valid_fin_orn)) if valid_fin_orn else 0.0,
        "p95_final_orientation_error_deg": float(np.percentile(valid_fin_orn, 95)) if valid_fin_orn else 0.0,
        "mean_dynamic_tracking_error_mm": float(np.mean(valid_dyn_pos)) if valid_dyn_pos else 0.0,
        "p95_dynamic_tracking_error_mm": float(np.percentile(valid_dyn_pos, 95)) if valid_dyn_pos else 0.0,
        "collision_free_direct_path_rate": float(direct_free_count / max(len(targets), 1)),
        "rrt_planning_success_rate": float(np.mean(planning_successes)),
        "mean_planning_time_ms": float(np.mean(planning_times_ms)) if planning_times_ms else 0.0,
        "mean_joint_path_length_rad": float(np.mean(path_lengths_rad)) if path_lengths_rad else 0.0,
    }
    return result


def main():
    parser = argparse.ArgumentParser(description="Cross-Robot Kinematic and Planning Benchmark Tool.")
    parser.add_argument("--robots", nargs="+", default=["panda", "kuka_iiwa"], help="Robots to benchmark.")
    parser.add_argument("--trials", type=int, default=15, help="Number of benchmark target poses.")
    parser.add_argument("--trajectory-duration", type=float, default=2.0, help="Trajectory duration (s).")
    parser.add_argument("--headless", action="store_true", default=True, help="Run simulation headless.")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory path.")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir) if args.output_dir else Path(f"benchmarks/robot_comparison_{timestamp}")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*75}")
    print(f" VisionRobotTwin Cross-Robot Benchmark Suite (PyBullet Simulation)")
    print(f" Target Robots: {', '.join(args.robots)} | Target Trials: {args.trials}")
    print(f" Trajectory Duration: {args.trajectory_duration:.1f}s")
    print(f"{'='*75}\n")

    targets, target_meta = generate_shared_benchmark_targets(
        seed=42, target_count=args.trials, headless=args.headless
    )
    print(f"[*] Target Generation: Candidates={target_meta['candidate_count']}, Accepted={target_meta['accepted_shared_targets']}, Rejected Pos={target_meta['rejected_position']}, Rejected Orn={target_meta['rejected_orientation']}")
    results = {}

    for r_id in args.robots:
        print(f"[*] Benchmarking {r_id} ({len(targets)} shared targets)...", flush=True)
        res = benchmark_robot(
            r_id,
            targets,
            headless=args.headless,
            trajectory_duration=args.trajectory_duration,
        )
        res["shared_target_generation_metadata"] = target_meta
        results[r_id] = res

    # Save summary.json
    summary_path = out_dir / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    # Save results.csv
    csv_path = out_dir / "results.csv"
    import csv
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        csv_rows = []
        for r_res in results.values():
            flat_res = {k: v for k, v in r_res.items() if not isinstance(v, dict)}
            csv_rows.append(flat_res)
        if csv_rows:
            fieldnames = list(csv_rows[0].keys())
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in csv_rows:
                writer.writerow(row)

    # Print Table
    print(f"\n{'-'*95}")
    print(f"{'Metric':<38} | {'Panda':<25} | {'KUKA iiwa':<25}")
    print(f"{'-'*95}")
    keys_to_print = [
        ("DoF", "dof", "{:d}"),
        ("Gripper Support", "has_gripper", "{}"),
        ("IK Success Rate", "ik_success_rate", "{:.1%}"),
        ("Mean IK Solve Time (ms)", "mean_ik_solve_time_ms", "{:.3f} ms"),
        ("P95 IK Solve Time (ms)", "p95_ik_solve_time_ms", "{:.3f} ms"),
        ("Mean IK Position Residual (mm)", "mean_ik_residual_position_mm", "{:.3f} mm"),
        ("P95 IK Position Residual (mm)", "p95_ik_residual_position_mm", "{:.3f} mm"),
        ("Mean IK Orientation Residual (deg)", "mean_ik_residual_orientation_deg", "{:.3f} deg"),
        ("Dynamic Tracking Samples", "dynamic_samples_count", "{:d}"),
        ("Mean Dynamic Position Error (mm)", "mean_dynamic_position_error_mm", "{:.3f} mm"),
        ("P95 Dynamic Position Error (mm)", "p95_dynamic_position_error_mm", "{:.3f} mm"),
        ("Mean Dynamic Orientation Error (deg)", "mean_dynamic_orientation_error_deg", "{:.3f} deg"),
        ("Mean Final Position Error (mm)", "mean_final_position_error_mm", "{:.3f} mm"),
        ("Mean Final Orientation Error (deg)", "mean_final_orientation_error_deg", "{:.3f} deg"),
        ("Mean Manipulability (Yoshikawa)", "mean_manipulability", "{:.4f}"),
        ("Min Manipulability", "min_manipulability", "{:.4f}"),
        ("Max Jacobian Condition", "max_jacobian_condition_number", "{:.2f}"),
        ("Direct Free Path Rate", "collision_free_direct_path_rate", "{:.1%}"),
        ("RRT Planning Success Rate", "rrt_planning_success_rate", "{:.1%}"),
        ("Mean Planning Time (ms)", "mean_planning_time_ms", "{:.2f} ms"),
        ("Mean Joint Path Length (rad)", "mean_joint_path_length_rad", "{:.3f} rad"),
    ]

    for label, k, fmt in keys_to_print:
        p_val = results.get("panda", {}).get(k, "N/A")
        k_val = results.get("kuka_iiwa", {}).get(k, "N/A")
        p_str = fmt.format(p_val) if isinstance(p_val, (int, float, bool)) else str(p_val)
        k_str = fmt.format(k_val) if isinstance(k_val, (int, float, bool)) else str(k_val)
        print(f"{label:<38} | {p_str:<25} | {k_str:<25}")
    print(f"{'-'*95}")
    print(f"\n[+] Benchmark complete. Artifacts exported to: {out_dir}\n")


if __name__ == "__main__":
    main()
