"""World-Anchor Camera-to-Robot Extrinsic Calibration Tool.

Performs Camera-to-Virtual-Robot Extrinsic Calibration using a known physical/synthetic
ArUco World Anchor Marker (default ID 10).

Usage:
    python tools/calibrate_extrinsics.py --camera 0 --marker-id 10
    python tools/calibrate_extrinsics.py --synthetic --samples 30
"""

import argparse
import sys
import time
from pathlib import Path
import cv2
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config.settings import AppConfig, ArUcoConfig, CameraConfig
from vision.calibration import load_or_create_calibration
from vision.camera import Camera, SyntheticFrameGenerator
from vision.aruco_detector import ArUcoDetector
from vision.pose_estimator import PoseEstimator
from vision.extrinsics import (
    ExtrinsicCalibration,
    compute_robot_to_camera_transform,
    aggregate_camera_poses,
)
from robotics.coordinate_transform import (
    create_homogeneous_matrix,
    rotation_matrix_to_quaternion,
    rotation_matrix_to_euler,
)
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.CalibrateExtrinsics")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="World-Anchor Camera-to-Robot Extrinsic Calibration Tool",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--marker-id", type=int, default=10, help="ArUco World Anchor Marker ID")
    parser.add_argument("--marker-size", type=float, default=0.05, help="Physical marker side length in meters")
    parser.add_argument("--anchor-x", type=float, default=0.50, help="Anchor X position in robot base frame (m)")
    parser.add_argument("--anchor-y", type=float, default=0.00, help="Anchor Y position in robot base frame (m)")
    parser.add_argument("--anchor-z", type=float, default=0.00, help="Anchor Z position in robot base frame (m)")
    parser.add_argument("--anchor-roll", type=float, default=float(np.pi), help="Anchor Roll in radians (robot frame)")
    parser.add_argument("--anchor-pitch", type=float, default=0.00, help="Anchor Pitch in radians (robot frame)")
    parser.add_argument("--anchor-yaw", type=float, default=0.00, help="Anchor Yaw in radians (robot frame)")
    parser.add_argument("--samples", type=int, default=30, help="Number of stable marker pose samples to collect")
    parser.add_argument("--output", type=str, default="calibration/extrinsics.json", help="Output JSON calibration file")
    parser.add_argument("--synthetic", action="store_true", help="Use synthetic frame generator for automated testing")
    parser.add_argument("--headless", action="store_true", help="Run without OpenCV GUI window")
    return parser.parse_args()


def run_extrinsic_calibration(args: argparse.Namespace) -> bool:
    """Executes world-anchor camera-to-robot extrinsic calibration."""
    logger.info("=" * 65)
    logger.info("   WORLD-ANCHOR CAMERA-TO-ROBOT EXTRINSIC CALIBRATION")
    logger.info("=" * 65)
    logger.info(f"Target Anchor Marker ID : {args.marker_id}")
    logger.info(f"Physical Marker Size    : {args.marker_size * 100:.1f} cm")
    logger.info(f"Robot Anchor Position   : X={args.anchor_x:.3f}m, Y={args.anchor_y:.3f}m, Z={args.anchor_z:.3f}m")
    logger.info(f"Robot Anchor RPY (rad)  : Roll={args.anchor_roll:.3f}, Pitch={args.anchor_pitch:.3f}, Yaw={args.anchor_yaw:.3f}")
    logger.info(f"Target Stable Samples   : {args.samples}")
    logger.info(f"Mode                    : {'SYNTHETIC' if args.synthetic else 'PHYSICAL CAMERA'}")
    logger.info("=" * 65)

    # 1. Load Camera Intrinsics
    calib_file = Path("calibration/camera_calibration.npz")
    calibration = load_or_create_calibration(calib_file, 1280, 720)
    logger.info(
        f"Camera Intrinsics Loaded: {'CALIBRATED' if calibration.is_calibrated else 'PINHOLE FALLBACK'} "
        f"(fx={calibration.fx:.1f}, fy={calibration.fy:.1f})"
    )

    # 2. Build T_robot_anchor from parameters
    T_robot_anchor = create_homogeneous_matrix(
        rotation=(args.anchor_roll, args.anchor_pitch, args.anchor_yaw),
        translation=(args.anchor_x, args.anchor_y, args.anchor_z),
    )

    # 3. Initialize Vision Pipeline
    aruco_cfg = ArUcoConfig(marker_size_m=args.marker_size)
    detector = ArUcoDetector(aruco_cfg)
    estimator = PoseEstimator(calibration, aruco_cfg)

    # 4. Open Camera or Synthetic Generator
    cam_cfg = CameraConfig(camera_index=args.camera, synthetic_mode=args.synthetic)
    try:
        camera = Camera(cam_cfg, aruco_cfg)
    except Exception as e:
        logger.error(f"Failed to open camera: {e}")
        return False

    collected_transforms: list[np.ndarray] = []
    window_name = "VisionRobotTwin - Extrinsics Calibration"

    try:
        frame_idx = 0
        while len(collected_transforms) < args.samples:
            frame_idx += 1
            ret, frame = camera.read(active_marker_id=args.marker_id)
            if not ret or frame is None:
                logger.warning("Camera read returned None")
                time.sleep(0.01)
                continue

            # Detect marker
            detections = detector.detect(frame)
            anchor_detection = next((d for d in detections if d.id == args.marker_id), None)

            annotated_frame = frame.copy()
            if anchor_detection is not None:
                detector.draw_detections(annotated_frame, [anchor_detection])
                pose = estimator.estimate_pose(anchor_detection)
                if pose is not None:
                    estimator.draw_axes(annotated_frame, pose)
                    collected_transforms.append(pose.transform_matrix)
                    status_text = f"Anchor Detected: {len(collected_transforms)}/{args.samples} samples"
                    color = (0, 220, 0)
                else:
                    status_text = "Anchor Detected (PnP solve failed)"
                    color = (0, 165, 255)
            else:
                status_text = f"Searching for Anchor Marker ID {args.marker_id}..."
                color = (0, 0, 255)

            # Render HUD overlay
            cv2.putText(
                annotated_frame,
                status_text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
                cv2.LINE_AA,
            )
            progress_bar_w = int((len(collected_transforms) / args.samples) * (annotated_frame.shape[1] - 40))
            cv2.rectangle(annotated_frame, (20, 60), (20 + progress_bar_w, 75), (0, 200, 0), -1)
            cv2.rectangle(annotated_frame, (20, 60), (annotated_frame.shape[1] - 20, 75), (200, 200, 200), 1)

            if not args.headless:
                cv2.imshow(window_name, annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    logger.info("Calibration aborted by user.")
                    return False

            if args.synthetic and frame_idx > args.samples * 3:
                logger.error("Synthetic calibration timed out without sufficient samples.")
                break

    finally:
        if camera is not None:
            camera.release()
        if not args.headless:
            cv2.destroyAllWindows()

    if len(collected_transforms) < max(5, args.samples // 2):
        logger.error(f"Insufficient samples collected ({len(collected_transforms)}/{args.samples}). Calibration failed.")
        return False

    # 5. Robust Pose Aggregation & Outlier Rejection
    robust_T_cam_anchor, rejected, t_std_mm, r_std_deg = aggregate_camera_poses(
        collected_transforms,
        max_pos_deviation_m=0.05,
        max_rot_deviation_deg=15.0,
    )

    # 6. Solve T_robot_camera = T_robot_anchor @ inv(T_cam_anchor)
    T_robot_camera = compute_robot_to_camera_transform(T_robot_anchor, robust_T_cam_anchor)

    # Extract translation and quaternion
    t_robot_camera = T_robot_camera[:3, 3]
    q_robot_camera = rotation_matrix_to_quaternion(T_robot_camera[:3, :3])
    rpy_robot_camera = rotation_matrix_to_euler(T_robot_camera[:3, :3])

    # 7. Create & Save ExtrinsicCalibration
    calib = ExtrinsicCalibration(
        camera_index=args.camera,
        anchor_marker_id=args.marker_id,
        anchor_marker_size_m=args.marker_size,
        anchor_pose_in_robot_base={
            "translation_m": [float(args.anchor_x), float(args.anchor_y), float(args.anchor_z)],
            "euler_rpy_rad": [float(args.anchor_roll), float(args.anchor_pitch), float(args.anchor_yaw)],
        },
        T_robot_camera_matrix=T_robot_camera,
        T_robot_camera_translation=t_robot_camera,
        T_robot_camera_quaternion=q_robot_camera,
        sample_count=len(collected_transforms),
        rejected_samples=rejected,
        translation_std_mm=t_std_mm,
        rotation_std_deg=r_std_deg,
    )

    out_path = Path(args.output)
    calib.save(out_path)

    # 8. Display Results Summary
    logger.info("=" * 65)
    logger.info("   EXTRINSIC CALIBRATION RESULTS SUMMARY")
    logger.info("=" * 65)
    logger.info(f"Accepted Samples       : {len(collected_transforms) - rejected}/{len(collected_transforms)} (Rejected: {rejected})")
    logger.info(f"Translation StDev      : {t_std_mm:.2f} mm")
    logger.info(f"Rotation Geodesic StDev: {r_std_deg:.2f} deg")
    logger.info(f"Camera Translation (m) : X={t_robot_camera[0]:.4f}, Y={t_robot_camera[1]:.4f}, Z={t_robot_camera[2]:.4f}")
    logger.info(f"Camera Quaternion [xyzw]: [{q_robot_camera[0]:.4f}, {q_robot_camera[1]:.4f}, {q_robot_camera[2]:.4f}, {q_robot_camera[3]:.4f}]")
    logger.info(f"Camera Euler RPY (rad) : Roll={rpy_robot_camera[0]:.3f}, Pitch={rpy_robot_camera[1]:.3f}, Yaw={rpy_robot_camera[2]:.3f}")
    logger.info(f"Saved Calibration File : {out_path}")
    logger.info("=" * 65)
    return True


if __name__ == "__main__":
    cli_args = parse_args()
    success = run_extrinsic_calibration(cli_args)
    sys.exit(0 if success else 1)
