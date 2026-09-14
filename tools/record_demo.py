"""Physical & Simulated Demo Video Recorder Tool.

Records high-quality demo video of the VisionRobotTwin HUD session to:
demo/recordings/demo_YYYYMMDD_HHMMSS.mp4 (with AVI fallback).

Usage:
    python tools/record_demo.py --camera 0 --duration 15
    python tools/record_demo.py --synthetic --duration 10 --headless
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple
import cv2
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config.settings import get_default_config
from main import VisionRobotTwinApp
from utils.telemetry import TelemetryData, TelemetryOverlay
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.RecordDemo")


def create_video_writer(
    output_path: Path,
    width: int,
    height: int,
    fps: float = 30.0,
) -> Tuple[cv2.VideoWriter, Path]:
    """Creates OpenCV VideoWriter with MP4 (mp4v/avc1) or fallback to AVI (MJPG)."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Try MP4 codecs first
    for fourcc_str in ["mp4v", "avc1", "H264"]:
        fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
        if writer.isOpened():
            logger.info(f"Initialized video writer with codec '{fourcc_str}': {output_path}")
            return writer, output_path

    # Fallback to AVI / MJPG
    avi_path = output_path.with_suffix(".avi")
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(str(avi_path), fourcc, fps, (width, height))
    if writer.isOpened():
        logger.warning(f"MP4 codec unavailable. Falling back to AVI with MJPG: {avi_path}")
        return writer, avi_path

    raise RuntimeError(f"Could not initialize any OpenCV VideoWriter for {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="VisionRobotTwin Demo Video Recorder",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--synthetic", action="store_true", help="Run with synthetic frame generator")
    parser.add_argument("--mode", type=str, choices=["manual", "auto"], default="manual", help="Operation mode")
    parser.add_argument("--control-mode", type=str, choices=["6dof", "3dof"], default="6dof", help="Control mode")
    parser.add_argument("--transform-mode", type=str, choices=["relative", "se3"], default="relative", help="Transform mode")
    parser.add_argument("--duration", type=float, default=15.0, help="Recording duration in seconds (0 = until keypress)")
    parser.add_argument("--output-dir", type=str, default="demo/recordings", help="Output directory for recordings")
    parser.add_argument("--headless", action="store_true", help="Run in headless simulation mode")
    return parser.parse_args()


def record_demo(args: argparse.Namespace) -> bool:
    """Runs application and records output video frames."""
    config = get_default_config()
    config.camera.camera_index = args.camera
    config.camera.synthetic_mode = args.synthetic
    config.mode = args.mode
    config.control_mode = args.control_mode
    config.transform.transform_mode = args.transform_mode
    if args.headless:
        config.simulation.gui = False

    app = VisionRobotTwinApp(config=config, headless_sim=args.headless)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.output_dir)
    out_mp4 = out_dir / f"demo_{timestamp}.mp4"

    logger.info("=" * 65)
    logger.info("   VISION ROBOT TWIN DEMO RECORDER")
    logger.info("=" * 65)
    logger.info(f"Target Duration : {args.duration:.1f}s" if args.duration > 0 else "Target Duration : Manual [Q]")
    logger.info(f"Target Output   : {out_mp4}")
    logger.info("=" * 65)

    writer: Optional[cv2.VideoWriter] = None
    actual_path = out_mp4
    start_time = time.time()
    frames_recorded = 0

    try:
        while True:
            elapsed = time.time() - start_time
            if args.duration > 0 and elapsed >= args.duration:
                logger.info(f"Target duration of {args.duration:.1f}s reached.")
                break

            res = app.process_frame()
            if not res.success or res.frame is None:
                time.sleep(0.01)
                continue

            display_frame = res.display_frame if res.display_frame is not None else res.frame.copy()

            # Render Telemetry overlay
            telem_data = TelemetryData(
                mode=res.mode,
                state=res.state_name,
                marker_id=res.target_pose.marker_id if res.target_pose else None,
                tracking_active=(res.target_pose is not None and res.state_name == "TRACK"),
                raw_pos_cam=res.raw_pos_cam,
                filtered_pos_cam=res.filtered_pos_cam,
                marker_distance_m=res.target_pose.distance_m if res.target_pose else None,
                robot_target_pos=tuple(res.commanded_position) if res.commanded_position is not None else None,
                robot_ee_pos=(float(res.ee_position[0]), float(res.ee_position[1]), float(res.ee_position[2])) if res.ee_position is not None else None,
                tracking_error_m=res.tracking_error_m,
                ik_status=res.ik_status,
                fps=res.fps,
                lost_tracking_time_s=app.state_machine.lost_tracking_duration_s,
                is_calibrated=app.calibration.is_calibrated,
                is_calibrated_extrinsics=app.workspace_mapper.tf_config.is_calibrated_extrinsics,
                transform_mode=app.workspace_mapper.tf_config.transform_mode,
                workspace_clamped=res.was_clamped,
            )
            app.telemetry_overlay.render(display_frame, telem_data)

            # Lazy-init writer on first frame
            if writer is None:
                fh, fw = display_frame.shape[:2]
                writer, actual_path = create_video_writer(out_mp4, fw, fh, fps=30.0)

            # Add recording indicator (red dot)
            rec_color = (0, 0, 255) if int(elapsed * 2) % 2 == 0 else (100, 100, 255)
            cv2.circle(display_frame, (fw - 30, 25), 8, rec_color, -1)
            cv2.putText(display_frame, "REC", (fw - 65, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

            writer.write(display_frame)
            frames_recorded += 1

            if not args.headless:
                cv2.imshow("VisionRobotTwin - Recording Demo", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    logger.info("Recording stopped by user.")
                    break

    finally:
        if writer is not None:
            writer.release()
            logger.info(f"Video saved ({frames_recorded} frames) -> {actual_path}")
        app.cleanup()

    return frames_recorded > 0


if __name__ == "__main__":
    cli_args = parse_args()
    success = record_demo(cli_args)
    sys.exit(0 if success else 1)
