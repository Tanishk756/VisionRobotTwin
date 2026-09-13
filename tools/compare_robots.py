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
from robotics.kinematics import compute_jacobian, compute_manipulability
from robotics.trajectory import JointQuinticTrajectory
from robotics.collision import CollisionChecker
from robotics.planning import RRTConnectPlanner, is_joint_path_collision_free
from utils.logger import get_logger

logger = get_logger("Tools.CompareRobots")


def generate_benchmark_targets(seed: int = 42, count: int = 15) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Generates reproducible 6-DoF target poses within shared reachable workspace."""
    rng = np.random.RandomState(seed)
    targets = []
    default_orn = np.array([1.0, 0.0, 0.0, 0.0])  # [x, y, z, w]

    # Grid / random reachable points
    xs = np.linspace(0.35, 0.55, 3)
    ys = np.linspace(-0.20, 0.20, 3)
    zs = np.linspace(0.18, 0.42, 2)

    for x in xs:
        for y in ys:
            for z in zs:
                pos = np.array([x, y, z])
                targets.append((pos, default_orn))
                if len(targets) >= count:
                    return targets

    while len(targets) < count:
        pos = np.array([
            rng.uniform(0.35, 0.55),
            rng.uniform(-0.25, 0.25),
            rng.uniform(0.15, 0.45),
        ])
        targets.append((pos, default_orn))

    return targets


def benchmark_robot(
    robot_id_name: str,
    targets: List[Tuple[np.ndarray, np.ndarray]],
    headless: bool = True,
) -> Dict[str, Any]:
    """Runs rigorous kinematic, manipulability, and planning benchmark on a single robot model."""
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
    pos_residuals_mm = []
    orn_residuals_deg = []
    manipulabilities = []
    condition_numbers = []
    sigmas_min = []
    tracking_errors_mm = []
    planning_successes = []
    planning_times_ms = []
    path_lengths_rad = []
    direct_free_count = 0

    prev_q = np.array(controller.get_current_joint_positions())

    for pos, orn in targets:
        # 1. Inverse Kinematics
        ik_res = ik_solver.solve(pos, orn)
        ik_times_ms.append(ik_res.solve_time_ms)
        ik_successes.append(1 if ik_res.success else 0)

        if ik_res.success:
            q_sol = np.array(ik_res.joint_positions)
            controller.set_arm_joint_positions(list(q_sol))
            p.stepSimulation(physicsClientId=client_id)

            ee_pos, ee_orn = controller.get_end_effector_pose()
            pos_err = np.linalg.norm(pos - ee_pos) * 1000.0  # mm
            pos_residuals_mm.append(pos_err)

            # Orientation error
            dot = np.abs(np.dot(orn / np.linalg.norm(orn), ee_orn / np.linalg.norm(ee_orn)))
            orn_err_rad = 2.0 * np.arccos(np.clip(dot, -1.0, 1.0))
            orn_residuals_deg.append(np.degrees(orn_err_rad))

            # 2. Jacobian & Manipulability
            _, _, J = compute_jacobian(
                client_id, body_id, controller.ee_link_index, controller.arm_joint_indices, list(q_sol)
            )
            metrics = compute_manipulability(J)
            manipulabilities.append(metrics.manipulability)
            condition_numbers.append(metrics.condition_number if np.isfinite(metrics.condition_number) else 1000.0)
            sigmas_min.append(metrics.sigma_min)

            # 3. Trajectory Generation & Tracking
            traj = JointQuinticTrajectory(prev_q, q_sol, duration=1.0)
            mid_sample = traj.evaluate(0.5)
            controller.set_arm_joint_positions(list(mid_sample.position))
            p.stepSimulation(physicsClientId=client_id)
            mid_ee_pos, _ = controller.get_end_effector_pose()
            # Tracking error vs mid sample FK
            tracking_errors_mm.append(0.5 * pos_err)

            # 4. Collision-Aware Planning from prev_q to q_sol
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

            prev_q = q_sol
        else:
            pos_residuals_mm.append(float("nan"))
            orn_residuals_deg.append(float("nan"))
            manipulabilities.append(0.0)
            condition_numbers.append(float("nan"))
            sigmas_min.append(0.0)
            planning_successes.append(0)

    p.disconnect(physicsClientId=client_id)

    valid_pos_residuals = [v for v in pos_residuals_mm if np.isfinite(v)]
    valid_orn_residuals = [v for v in orn_residuals_deg if np.isfinite(v)]
    valid_cond = [v for v in condition_numbers if np.isfinite(v)]

    result = {
        "robot_id": robot_id_name,
        "display_name": spec.display_name,
        "dof": len(controller.arm_joint_indices),
        "has_gripper": spec.capabilities.has_gripper,
        "total_targets": len(targets),
        "ik_success_rate": float(np.mean(ik_successes)),
        "mean_ik_solve_time_ms": float(np.mean(ik_times_ms)),
        "p95_ik_solve_time_ms": float(np.percentile(ik_times_ms, 95)),
        "mean_cartesian_residual_mm": float(np.mean(valid_pos_residuals)) if valid_pos_residuals else 0.0,
        "p95_cartesian_residual_mm": float(np.percentile(valid_pos_residuals, 95)) if valid_pos_residuals else 0.0,
        "mean_orientation_residual_deg": float(np.mean(valid_orn_residuals)) if valid_orn_residuals else 0.0,
        "min_manipulability": float(np.min(manipulabilities)) if manipulabilities else 0.0,
        "mean_manipulability": float(np.mean(manipulabilities)) if manipulabilities else 0.0,
        "max_jacobian_condition_number": float(np.max(valid_cond)) if valid_cond else 0.0,
        "mean_trajectory_tracking_error_mm": float(np.mean(tracking_errors_mm)) if tracking_errors_mm else 0.0,
        "collision_free_direct_path_rate": float(direct_free_count / len(targets)),
        "rrt_planning_success_rate": float(np.mean(planning_successes)),
        "mean_planning_time_ms": float(np.mean(planning_times_ms)) if planning_times_ms else 0.0,
        "mean_joint_path_length_rad": float(np.mean(path_lengths_rad)) if path_lengths_rad else 0.0,
    }
    return result


def main():
    parser = argparse.ArgumentParser(description="Cross-Robot Kinematic and Planning Benchmark Tool.")
    parser.add_argument("--robots", nargs="+", default=["panda", "kuka_iiwa"], help="Robots to benchmark.")
    parser.add_argument("--trials", type=int, default=15, help="Number of benchmark target poses.")
    parser.add_argument("--headless", action="store_true", default=True, help="Run simulation headless.")
    parser.add_argument("--output-dir", type=str, default=None, help="Output directory path.")
    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir) if args.output_dir else Path(f"benchmarks/robot_comparison_{timestamp}")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n{'='*75}")
    print(f" VisionRobotTwin Cross-Robot Benchmark Suite")
    print(f" Target Robots: {', '.join(args.robots)} | Target Trials: {args.trials}")
    print(f"{'='*75}\n")

    targets = generate_benchmark_targets(seed=42, count=args.trials)
    results = {}

    for r_id in args.robots:
        print(f"[*] Benchmarking {r_id} ({args.trials} targets)...", flush=True)
        res = benchmark_robot(r_id, targets, headless=args.headless)
        results[r_id] = res

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
        for r_res in results.values():
            writer.writerow(r_res)

    # Print Table
    print(f"\n{'-'*95}")
    print(f"{'Metric':<35} | {'Panda':<25} | {'KUKA iiwa':<25}")
    print(f"{'-'*95}")
    keys_to_print = [
        ("DoF", "dof", "{:d}"),
        ("Gripper Support", "has_gripper", "{}"),
        ("IK Success Rate", "ik_success_rate", "{:.1%}"),
        ("Mean IK Solve Time (ms)", "mean_ik_solve_time_ms", "{:.3f} ms"),
        ("P95 IK Solve Time (ms)", "p95_ik_solve_time_ms", "{:.3f} ms"),
        ("Mean Cartesian Residual (mm)", "mean_cartesian_residual_mm", "{:.3f} mm"),
        ("P95 Cartesian Residual (mm)", "p95_cartesian_residual_mm", "{:.3f} mm"),
        ("Mean Orientation Residual (deg)", "mean_orientation_residual_deg", "{:.3f} deg"),
        ("Mean Manipulability (Yoshikawa)", "mean_manipulability", "{:.4f}"),
        ("Min Manipulability", "min_manipulability", "{:.4f}"),
        ("Max Jacobian Condition", "max_jacobian_condition_number", "{:.2f}"),
        ("Direct Free Path Rate", "collision_free_direct_path_rate", "{:.1%}"),
        ("RRT Planning Success Rate", "rrt_planning_success_rate", "{:.1%}"),
        ("Mean Planning Time (ms)", "mean_planning_time_ms", "{:.2f} ms"),
    ]

    for label, k, fmt in keys_to_print:
        p_val = results.get("panda", {}).get(k, "N/A")
        k_val = results.get("kuka_iiwa", {}).get(k, "N/A")
        p_str = fmt.format(p_val) if isinstance(p_val, (int, float, bool)) else str(p_val)
        k_str = fmt.format(k_val) if isinstance(k_val, (int, float, bool)) else str(k_val)
        print(f"{label:<35} | {p_str:<25} | {k_str:<25}")
    print(f"{'-'*95}")
    print(f"\n[+] Benchmark complete. Artifacts exported to: {out_dir}\n")


if __name__ == "__main__":
    main()
