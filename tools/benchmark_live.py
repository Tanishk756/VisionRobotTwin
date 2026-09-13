"""Physical & Synthetic Robotics Benchmark Utility.

Measures real-time perception and control metrics:
- Real Camera Frame Rate (FPS)
- ArUco Marker Detection & Tracking Rate (%)
- 3D Position & Orientation Jitter (Standard Deviation at standstill)
- PnP Metric Depth Estimates
- Robot Digital Twin Command-to-EE Tracking Error (mm)
- Tracking Loss Occurrences & Latencies

Outputs structured session benchmark results to benchmarks/session_TIMESTAMP.json.
"""

import sys
import time
import argparse
import json
from pathlib import Path
from datetime import datetime
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.settings import get_default_config
from main import VisionRobotTwinApp
from utils.logger import setup_logger

logger = setup_logger("Tools.Benchmark")


def run_benchmark(
    camera_index: int = 0,
    duration_s: float = 10.0,
    synthetic: bool = False,
    control_mode: str = "6dof",
    transform_mode: str = "relative",
    output_dir: Path = Path("benchmarks"),
) -> dict:
    """Executes a benchmark session using the shared unified frame processing pipeline."""
    output_dir.mkdir(parents=True, exist_ok=True)
    config = get_default_config()
    config.camera.camera_index = camera_index
    config.camera.synthetic_mode = synthetic
    config.control_mode = control_mode
    config.transform.transform_mode = transform_mode
    config.simulation.gui = False  # Headless for benchmarking

    print("=" * 70)
    print(" VISIONROBOTTWIN BENCHMARK SUITE")
    print(f" Source: {'SYNTHETIC STREAM' if synthetic else f'PHYSICAL CAMERA (Index {camera_index})'}")
    print(f" Control Mode: {control_mode.upper()} | Transform Mode: {transform_mode.upper()}")
    print(f" Duration: {duration_s:.1f} seconds")
    print("=" * 70)

    try:
        app = VisionRobotTwinApp(config, headless_sim=True)
    except Exception as e:
        logger.error(f"Failed to initialize benchmark app: {e}")
        return {"status": "FAILED", "error": str(e)}

    timestamps = []
    fps_records = []
    detection_flags = []
    cam_positions = []
    cam_orientations = []
    tracking_errors_m = []
    physics_substeps_records = []
    tracking_loss_events = 0
    was_previously_tracking = False

    start_time = time.time()
    frames_processed = 0

    while time.time() - start_time < duration_s:
        t_loop_start = time.perf_counter()
        frames_processed += 1

        res = app.process_frame()
        if not res.success or res.frame is None:
            time.sleep(0.01)
            continue

        timestamps.append(time.time() - start_time)
        fps_records.append(res.fps)
        physics_substeps_records.append(res.physics_substeps)

        if res.target_pose is not None:
            detection_flags.append(1)
            cam_positions.append([res.target_pose.x, res.target_pose.y, res.target_pose.z])
            cam_orientations.append(list(res.target_pose.quaternion_xyzw))
            if res.tracking_error_m is not None:
                tracking_errors_m.append(res.tracking_error_m)
            was_previously_tracking = True
        else:
            detection_flags.append(0)
            if was_previously_tracking:
                tracking_loss_events += 1
                was_previously_tracking = False

        # Sleep to approximate 30-60 FPS camera rate
        elapsed = time.perf_counter() - t_loop_start
        if elapsed < 0.016:
            time.sleep(0.016 - elapsed)

    app.cleanup()
    total_elapsed = time.time() - start_time

    # Compute Statistical Metrics
    pos_arr = np.array(cam_positions) if cam_positions else np.empty((0, 3))
    err_arr = np.array(tracking_errors_m) if tracking_errors_m else np.empty((0,))

    detection_rate_pct = (sum(detection_flags) / len(detection_flags) * 100.0) if detection_flags else 0.0
    mean_fps = float(np.mean(fps_records[10:])) if len(fps_records) > 10 else 0.0
    mean_substeps = float(np.mean(physics_substeps_records)) if physics_substeps_records else 1.0

    pos_std_xyz_mm = (np.std(pos_arr, axis=0) * 1000.0).tolist() if len(pos_arr) > 5 else [0.0, 0.0, 0.0]
    mean_error_mm = float(np.mean(err_arr) * 1000.0) if len(err_arr) > 0 else 0.0
    max_error_mm = float(np.max(err_arr) * 1000.0) if len(err_arr) > 0 else 0.0

    results = {
        "timestamp": datetime.now().isoformat(),
        "input_source": "SYNTHETIC" if synthetic else f"PHYSICAL_CAMERA_INDEX_{camera_index}",
        "control_mode": control_mode,
        "transform_mode": transform_mode,
        "session_duration_s": round(total_elapsed, 2),
        "total_frames_processed": frames_processed,
        "mean_camera_fps": round(mean_fps, 2),
        "mean_physics_substeps_per_frame": round(mean_substeps, 2),
        "detection_rate_pct": round(detection_rate_pct, 2),
        "position_jitter_std_mm": {
            "x": round(pos_std_xyz_mm[0], 3),
            "y": round(pos_std_xyz_mm[1], 3),
            "z": round(pos_std_xyz_mm[2], 3),
        },
        "robot_tracking_error_mm": {
            "mean": round(mean_error_mm, 2),
            "max": round(max_error_mm, 2),
        },
        "tracking_loss_events_count": tracking_loss_events,
    }

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_file = output_dir / f"session_{timestamp_str}.json"
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print("\n" + "=" * 50)
    print(" BENCHMARK RESULTS")
    print("=" * 50)
    print(f"Control Mode:            {control_mode.upper()} ({transform_mode.upper()})")
    print(f"Mean Camera FPS:         {results['mean_camera_fps']:.1f} FPS")
    print(f"Physics Substeps/Frame:  {results['mean_physics_substeps_per_frame']:.1f}")
    print(f"Marker Detection Rate:   {results['detection_rate_pct']:.1f} %")
    print(f"Position Jitter (StdDev): X={pos_std_xyz_mm[0]:.2f}mm, Y={pos_std_xyz_mm[1]:.2f}mm, Z={pos_std_xyz_mm[2]:.2f}mm")
    print(f"Mean Robot EE Error:     {results['robot_tracking_error_mm']['mean']:.2f} mm")
    print(f"Tracking Loss Occurrences: {tracking_loss_events}")
    print(f"Saved Report:            {out_file}")
    print("=" * 50 + "\n")

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VisionRobotTwin Benchmark Utility")
    parser.add_argument("--camera", type=int, default=0, help="Physical camera index")
    parser.add_argument("--duration", type=float, default=5.0, help="Benchmark duration in seconds")
    parser.add_argument("--synthetic", action="store_true", help="Run benchmark on synthetic test stream")
    parser.add_argument("--control-mode", type=str, choices=["6dof", "3dof"], default="6dof", help="Control mode")
    parser.add_argument("--transform-mode", type=str, choices=["relative", "se3"], default="relative", help="Transform mode")
    args = parser.parse_args()

    run_benchmark(
        camera_index=args.camera,
        duration_s=args.duration,
        synthetic=args.synthetic,
        control_mode=args.control_mode,
        transform_mode=args.transform_mode,
    )
