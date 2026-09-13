"""Physical & Synthetic Robotics Benchmark Suite V2.

Performs scientifically rigorous, reproducible benchmarking across perception and control:
Modes:
- 'stationary': Standstill precision and jitter analysis (XYZ std mm, norm mm, geodesic orientation std deg, percentiles).
- 'tracking': Dynamic motion tracking continuity, EE tracking error, detection rate, loss events.
- 'standard': Combined full-pipeline operational evaluation.

Outputs structured artifacts to benchmarks/YYYYMMDD_HHMMSS/:
- summary.json (comprehensive metrics + safe system manifest)
- frames.csv (per-frame time-series measurements)
- plot_position_jitter.png (if matplotlib available)
- plot_tracking_error.png (if matplotlib available)
"""

import argparse
import csv
import json
import platform
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional
import cv2
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from visionrobottwin_version import __version__
from config.settings import get_default_config
from main import VisionRobotTwinApp
from robotics.coordinate_transform import compute_angular_distance
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.BenchmarkV2")


def build_system_manifest(app: VisionRobotTwinApp, camera_index: int, duration_s: float) -> Dict[str, Any]:
    """Constructs an anonymous, reproducible execution manifest without user PII."""
    try:
        pybullet_ver = p.__version__ if hasattr(p, "__version__") else "unknown"
    except Exception:
        pybullet_ver = "unknown"

    calib_file = Path("calibration/camera_calibration.npz")
    ext_file = Path("calibration/extrinsics.json")

    return {
        "visionrobottwin_version": __version__,
        "python_version": sys.version.split()[0],
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "pybullet_version": pybullet_ver,
        "os": f"{platform.system()} {platform.release()} ({platform.machine()})",
        "camera_index": int(camera_index),
        "requested_camera_resolution": f"{app.config.camera.width}x{app.config.camera.height}",
        "actual_camera_resolution": f"{app.config.camera.width}x{app.config.camera.height}",
        "benchmark_start_timestamp": datetime.now().isoformat(),
        "benchmark_duration_s": float(duration_s),
        "calibration_status": "CALIBRATED" if app.calibration.is_calibrated else "FALLBACK PINHOLE",
        "calibration_file": str(calib_file.name) if calib_file.exists() else "none",
        "extrinsics_status": "CALIBRATED" if app.workspace_mapper.tf_config.is_calibrated_extrinsics else "NOMINAL",
        "extrinsics_file": str(ext_file.name) if ext_file.exists() else "none",
        "control_mode": app.config.control_mode,
        "transform_mode": app.config.transform.transform_mode,
    }


def generate_benchmark_plots(
    session_dir: Path,
    timestamps: List[float],
    cam_positions: List[List[float]],
    tracking_errors_m: List[float],
) -> None:
    """Generates visual diagnostic plots if matplotlib is installed."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        # 1. Jitter Plot
        if len(cam_positions) > 5 and len(timestamps) == len(cam_positions):
            pos_arr = np.array(cam_positions) * 1000.0  # Convert to mm
            pos_zeroed = pos_arr - np.mean(pos_arr, axis=0)

            fig, ax = plt.subplots(figsize=(8, 4), dpi=150)
            ax.plot(timestamps, pos_zeroed[:, 0], label="X Deviation (mm)", color="#e74c3c", alpha=0.85)
            ax.plot(timestamps, pos_zeroed[:, 1], label="Y Deviation (mm)", color="#2ecc71", alpha=0.85)
            ax.plot(timestamps, pos_zeroed[:, 2], label="Z Deviation (mm)", color="#3498db", alpha=0.85)
            ax.set_title("Optical Position Jitter (Zero-Centered Residuals)", fontsize=11, fontweight="bold")
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.set_ylabel("Displacement (mm)", fontsize=9)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.legend(loc="upper right", fontsize=8)
            plt.tight_layout()

            jitter_plot_path = session_dir / "plot_position_jitter.png"
            plt.savefig(jitter_plot_path)
            plt.close(fig)
            logger.info(f"Generated jitter plot -> {jitter_plot_path}")

        # 2. Tracking Error Plot
        if len(tracking_errors_m) > 5:
            err_mm = np.array(tracking_errors_m) * 1000.0
            fig, ax = plt.subplots(figsize=(8, 4), dpi=150)
            ax.plot(err_mm, label="Command-to-Sim EE Error (mm)", color="#9b59b6", lw=1.2)
            ax.axhline(np.mean(err_mm), color="#e67e22", linestyle="--", label=f"Mean: {np.mean(err_mm):.1f} mm")
            ax.axhline(np.percentile(err_mm, 95), color="#c0392b", linestyle=":", label=f"95th %tile: {np.percentile(err_mm, 95):.1f} mm")
            ax.set_title("Digital Twin End-Effector Tracking Error", fontsize=11, fontweight="bold")
            ax.set_xlabel("Tracked Frame Sample", fontsize=9)
            ax.set_ylabel("Cartesian Error (mm)", fontsize=9)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.legend(loc="upper right", fontsize=8)
            plt.tight_layout()

            err_plot_path = session_dir / "plot_tracking_error.png"
            plt.savefig(err_plot_path)
            plt.close(fig)
            logger.info(f"Generated tracking error plot -> {err_plot_path}")

    except ImportError:
        logger.debug("matplotlib not installed; skipping plot generation.")
    except Exception as e:
        logger.warning(f"Plot generation failed: {e}")


def run_benchmark(
    camera_index: int = 0,
    duration_s: float = 10.0,
    synthetic: bool = False,
    benchmark_mode: str = "stationary",
    control_mode: str = "6dof",
    transform_mode: str = "relative",
    output_dir: Path = Path("benchmarks"),
) -> dict:
    """Executes a structured benchmark session and outputs JSON summary, CSV, and plots."""
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = output_dir / timestamp_str
    session_dir.mkdir(parents=True, exist_ok=True)

    config = get_default_config()
    config.camera.camera_index = camera_index
    config.camera.synthetic_mode = synthetic
    config.control_mode = control_mode
    config.transform.transform_mode = transform_mode
    config.simulation.gui = False  # Headless execution

    print("=" * 70)
    print(" VISIONROBOTTWIN BENCHMARK SUITE V2")
    print(f" Source         : {'SYNTHETIC STREAM' if synthetic else f'PHYSICAL CAMERA (Index {camera_index})'}")
    print(f" Benchmark Mode : {benchmark_mode.upper()}")
    print(f" Control Mode   : {control_mode.upper()} | Transform Mode: {transform_mode.upper()}")
    print(f" Target Duration: {duration_s:.1f} seconds")
    print(f" Session Output : {session_dir}")
    print("=" * 70)

    try:
        app = VisionRobotTwinApp(config, headless_sim=True)
    except Exception as e:
        logger.error(f"Failed to initialize benchmark app: {e}")
        return {"status": "FAILED", "error": str(e)}

    manifest = build_system_manifest(app, camera_index, duration_s)

    # Telemetry collectors
    timestamps = []
    fps_records = []
    detection_flags = []
    cam_positions = []
    cam_quaternions = []
    tracking_errors_m = []
    physics_substeps_records = []
    frame_intervals_s = []
    tracking_loss_events = 0
    was_previously_tracking = False

    csv_path = session_dir / "frames.csv"
    csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp_s", "detected", "fps", "substeps",
        "cam_x_m", "cam_y_m", "cam_z_m",
        "cam_qx", "cam_qy", "cam_qz", "cam_qw",
        "ee_x_m", "ee_y_m", "ee_z_m",
        "tracking_error_mm",
    ])

    start_time = time.time()
    last_loop_perf = time.perf_counter()
    frames_processed = 0

    try:
        while time.time() - start_time < duration_s:
            t_loop_start = time.perf_counter()
            frame_dt = t_loop_start - last_loop_perf
            last_loop_perf = t_loop_start
            frames_processed += 1
            frame_intervals_s.append(frame_dt)

            res = app.process_frame()
            if not res.success or res.frame is None:
                time.sleep(0.01)
                continue

            t_rel = time.time() - start_time
            timestamps.append(t_rel)
            fps_records.append(res.fps)
            physics_substeps_records.append(res.physics_substeps)

            det_flag = 1 if res.target_pose is not None else 0
            detection_flags.append(det_flag)

            c_pos = [res.target_pose.x, res.target_pose.y, res.target_pose.z] if res.target_pose else [None, None, None]
            c_quat = list(res.target_pose.quaternion_xyzw) if res.target_pose else [None, None, None, None]
            ee_p = list(res.ee_position) if res.ee_position is not None else [None, None, None]
            err_mm = (res.tracking_error_m * 1000.0) if res.tracking_error_m is not None else None

            if res.target_pose is not None:
                cam_positions.append(c_pos)
                cam_quaternions.append(c_quat)
                if res.tracking_error_m is not None:
                    tracking_errors_m.append(res.tracking_error_m)
                was_previously_tracking = True
            else:
                if was_previously_tracking:
                    tracking_loss_events += 1
                    was_previously_tracking = False

            csv_writer.writerow([
                f"{t_rel:.4f}", det_flag, f"{res.fps:.1f}", res.physics_substeps,
                f"{c_pos[0]:.4f}" if c_pos[0] is not None else "",
                f"{c_pos[1]:.4f}" if c_pos[1] is not None else "",
                f"{c_pos[2]:.4f}" if c_pos[2] is not None else "",
                f"{c_quat[0]:.4f}" if c_quat[0] is not None else "",
                f"{c_quat[1]:.4f}" if c_quat[1] is not None else "",
                f"{c_quat[2]:.4f}" if c_quat[2] is not None else "",
                f"{c_quat[3]:.4f}" if c_quat[3] is not None else "",
                f"{ee_p[0]:.4f}" if ee_p[0] is not None else "",
                f"{ee_p[1]:.4f}" if ee_p[1] is not None else "",
                f"{ee_p[2]:.4f}" if ee_p[2] is not None else "",
                f"{err_mm:.2f}" if err_mm is not None else "",
            ])

            # Pacing
            elapsed = time.perf_counter() - t_loop_start
            if elapsed < 0.016:
                time.sleep(0.016 - elapsed)

    finally:
        csv_file.close()
        app.cleanup()

    total_elapsed = time.time() - start_time

    # --- Compute Statistical Metrics ---
    pos_arr = np.array(cam_positions) if cam_positions else np.empty((0, 3))
    quat_arr = np.array(cam_quaternions) if cam_quaternions else np.empty((0, 4))
    err_arr_mm = (np.array(tracking_errors_m) * 1000.0) if tracking_errors_m else np.empty((0,))

    detection_rate_pct = (sum(detection_flags) / len(detection_flags) * 100.0) if detection_flags else 0.0
    mean_fps = float(np.mean(fps_records[5:])) if len(fps_records) > 5 else 0.0
    mean_detection_interval_ms = float(np.mean(frame_intervals_s) * 1000.0) if frame_intervals_s else 0.0
    mean_substeps = float(np.mean(physics_substeps_records)) if physics_substeps_records else 1.0
    total_substeps = sum(physics_substeps_records)
    measured_step_rate_hz = float(total_substeps / max(total_elapsed, 1e-4))

    # Position Jitter
    if len(pos_arr) > 5:
        pos_std_xyz_mm = (np.std(pos_arr, axis=0) * 1000.0).tolist()
        pos_jitter_norm_mm = float(np.linalg.norm(np.std(pos_arr, axis=0)) * 1000.0)
        # Deviations from median
        median_pos = np.median(pos_arr, axis=0)
        pos_devs_mm = np.linalg.norm(pos_arr - median_pos, axis=1) * 1000.0
        max_pos_dev_mm = float(np.max(pos_devs_mm))
        p95_pos_dev_mm = float(np.percentile(pos_devs_mm, 95))
    else:
        pos_std_xyz_mm = [0.0, 0.0, 0.0]
        pos_jitter_norm_mm = 0.0
        max_pos_dev_mm = 0.0
        p95_pos_dev_mm = 0.0

    # Orientation Geodesic Jitter
    if len(quat_arr) > 5:
        # Align quaternion signs
        for i in range(len(quat_arr)):
            q_norm = np.linalg.norm(quat_arr[i])
            if q_norm > 1e-6:
                quat_arr[i] /= q_norm
            if i > 0 and np.dot(quat_arr[i], quat_arr[0]) < 0.0:
                quat_arr[i] = -quat_arr[i]
        mean_rot = Rotation.from_quat(quat_arr).mean()
        ref_q = mean_rot.as_quat()
        ang_diffs_deg = [
            np.degrees(compute_angular_distance(q, ref_q)) for q in quat_arr
        ]
        orientation_jitter_deg = float(np.std(ang_diffs_deg))
    else:
        orientation_jitter_deg = 0.0

    # EE Tracking Error
    if len(err_arr_mm) > 0:
        mean_ee_err_mm = float(np.mean(err_arr_mm))
        max_ee_err_mm = float(np.max(err_arr_mm))
        p95_ee_err_mm = float(np.percentile(err_arr_mm, 95))
    else:
        mean_ee_err_mm = 0.0
        max_ee_err_mm = 0.0
        p95_ee_err_mm = 0.0

    summary = {
        "manifest": manifest,
        "benchmark_mode": benchmark_mode,
        "session_metrics": {
            "duration_seconds": round(total_elapsed, 2),
            "frames_captured": frames_processed,
            "frames_tracked": len(cam_positions),
            "detection_rate_pct": round(detection_rate_pct, 2),
            "tracking_loss_events_count": tracking_loss_events,
            "mean_detection_interval_ms": round(mean_detection_interval_ms, 2),
            "actual_camera_fps": round(mean_fps, 2),
            "processed_frame_fps": round(frames_processed / max(total_elapsed, 1e-4), 2),
            "physics_substeps_per_frame": round(mean_substeps, 2),
            "physics_substeps_per_second_hz": round(measured_step_rate_hz, 2),
        },
        "optical_jitter": {
            "position_std_xyz_mm": {
                "x": round(pos_std_xyz_mm[0], 3),
                "y": round(pos_std_xyz_mm[1], 3),
                "z": round(pos_std_xyz_mm[2], 3),
            },
            "position_jitter_norm_mm": round(pos_jitter_norm_mm, 3),
            "max_position_deviation_mm": round(max_pos_dev_mm, 3),
            "p95_position_deviation_mm": round(p95_pos_dev_mm, 3),
            "orientation_geodesic_jitter_std_deg": round(orientation_jitter_deg, 3),
        },
        "digital_twin_ee_tracking": {
            "mean_error_mm": round(mean_ee_err_mm, 2),
            "max_error_mm": round(max_ee_err_mm, 2),
            "p95_error_mm": round(p95_ee_err_mm, 2),
        },
    }

    summary_file = session_dir / "summary.json"
    with open(summary_file, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    logger.info(f"Saved benchmark summary to {summary_file}")

    # Generate diagnostic plots
    generate_benchmark_plots(
        session_dir=session_dir,
        timestamps=timestamps,
        cam_positions=cam_positions,
        tracking_errors_m=tracking_errors_m,
    )

    # Console Output
    print("\n" + "=" * 65)
    print("   BENCHMARK V2 RESULTS SUMMARY")
    print("=" * 65)
    print(f"Benchmark Mode       : {benchmark_mode.upper()}")
    print(f"Processed Frames     : {frames_processed} ({summary['session_metrics']['processed_frame_fps']:.1f} FPS)")
    print(f"Detection Rate       : {detection_rate_pct:.1f}% ({len(cam_positions)}/{frames_processed} tracked)")
    print(f"Tracking Loss Events : {tracking_loss_events}")
    print(f"Position Jitter Norm : {pos_jitter_norm_mm:.2f} mm (X={pos_std_xyz_mm[0]:.2f}, Y={pos_std_xyz_mm[1]:.2f}, Z={pos_std_xyz_mm[2]:.2f} mm)")
    print(f"P95 Pos Deviation    : {p95_pos_dev_mm:.2f} mm (Max: {max_pos_dev_mm:.2f} mm)")
    print(f"Orientation Jitter   : {orientation_jitter_deg:.2f} deg (geodesic)")
    print(f"Digital Twin EE Error: Mean: {mean_ee_err_mm:.1f} mm | P95: {p95_ee_err_mm:.1f} mm")
    print(f"Physics Stepping     : {measured_step_rate_hz:.0f} Hz ({mean_substeps:.1f} steps/f)")
    print(f"Session Artifacts    : {session_dir}")
    print("=" * 65 + "\n")

    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="VisionRobotTwin Benchmark Suite V2",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--camera", type=int, default=0, help="Physical camera index")
    parser.add_argument("--duration", type=float, default=5.0, help="Benchmark duration in seconds")
    parser.add_argument("--synthetic", action="store_true", help="Run benchmark on synthetic test stream")
    parser.add_argument(
        "--benchmark-mode",
        type=str,
        choices=["stationary", "tracking", "standard"],
        default="stationary",
        help="Benchmark mode: stationary (standstill jitter), tracking (motion error), standard",
    )
    parser.add_argument("--control-mode", type=str, choices=["6dof", "3dof"], default="6dof", help="Control mode")
    parser.add_argument("--transform-mode", type=str, choices=["relative", "se3"], default="relative", help="Transform mode")
    parser.add_argument("--output-dir", type=str, default="benchmarks", help="Output directory for session artifacts")
    args = parser.parse_args()

    run_benchmark(
        camera_index=args.camera,
        duration_s=args.duration,
        synthetic=args.synthetic,
        benchmark_mode=args.benchmark_mode,
        control_mode=args.control_mode,
        transform_mode=args.transform_mode,
        output_dir=Path(args.output_dir),
    )
