"""World-Anchor Extrinsic Calibration Validation Tool.

Validates the accuracy of the calibrated camera-to-robot transform by detecting
the world anchor ArUco marker (ID 10) in the camera frame, transforming its pose
into the robot base frame, and comparing against the known configured anchor location.

Reports measured repeatability and calibration residual error:
- Translation error (mm)
- Orientation angular error (degrees)

NOTE: This is a measured validation of the calibration anchor only.
Do not generalize this into whole-workspace robot accuracy.
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
from vision.camera import Camera
from vision.aruco_detector import ArUcoDetector
from vision.pose_estimator import PoseEstimator
from vision.extrinsics import ExtrinsicCalibration, validate_extrinsic_transform
from robotics.coordinate_transform import (
    create_homogeneous_matrix,
    rotation_matrix_to_quaternion,
    rotation_matrix_to_euler,
)
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.ValidateExtrinsics")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="World-Anchor Extrinsic Calibration Validation Tool",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--marker-id", type=int, default=10, help="ArUco World Anchor Marker ID")
    parser.add_argument("--marker-size", type=float, default=0.05, help="Physical marker side length in meters")
    parser.add_argument("--extrinsics", type=str, default="calibration/extrinsics.json", help="Path to extrinsics JSON")
    parser.add_argument("--samples", type=int, default=15, help="Number of validation frames to evaluate")
    parser.add_argument("--synthetic", action="store_true", help="Use synthetic frame generator for automated testing")
    parser.add_argument("--headless", action="store_true", help="Run without OpenCV GUI window")
    return parser.parse_args()


def run_extrinsic_validation(args: argparse.Namespace) -> bool:
    """Executes validation of camera-to-robot extrinsics."""
    logger.info("=" * 65)
    logger.info("   WORLD-ANCHOR EXTRINSIC CALIBRATION VALIDATION")
    logger.info("=" * 65)

    # 1. Load Extrinsics
    ext_path = Path(args.extrinsics)
    if not ext_path.exists():
        logger.error(f"Extrinsics file not found: {ext_path}. Run 'python tools/calibrate_extrinsics.py' first.")
        return False

    try:
        calib_ext = ExtrinsicCalibration.load(ext_path)
    except Exception as e:
        logger.error(f"Failed to load extrinsics from {ext_path}: {e}")
        return False

    logger.info(f"Loaded Extrinsics from     : {ext_path} (Version: {calib_ext.version})")
    logger.info(f"Anchor Marker ID           : {calib_ext.anchor_marker_id}")

    # Build expected T_robot_anchor
    anchor_info = calib_ext.anchor_pose_in_robot_base
    t_anchor_exp = anchor_info.get("translation_m", [0.50, 0.0, 0.0])
    rpy_anchor_exp = anchor_info.get("euler_rpy_rad", [np.pi, 0.0, 0.0])
    T_robot_anchor_expected = create_homogeneous_matrix(
        rotation=tuple(rpy_anchor_exp),
        translation=tuple(t_anchor_exp),
    )

    # 2. Load Camera Intrinsics
    calib_file = Path("calibration/camera_calibration.npz")
    calibration = load_or_create_calibration(calib_file, 1280, 720)

    # 3. Vision Pipeline
    aruco_cfg = ArUcoConfig(marker_size_m=args.marker_size)
    detector = ArUcoDetector(aruco_cfg)
    estimator = PoseEstimator(calibration, aruco_cfg)

    # 4. Open Camera
    cam_cfg = CameraConfig(camera_index=args.camera, synthetic_mode=args.synthetic)
    try:
        camera = Camera(cam_cfg, aruco_cfg)
    except Exception as e:
        logger.error(f"Failed to open camera: {e}")
        return False

    translation_errors_mm: list[float] = []
    rotation_errors_deg: list[float] = []
    window_name = "VisionRobotTwin - Extrinsics Validation"

    try:
        frame_idx = 0
        while len(translation_errors_mm) < args.samples:
            frame_idx += 1
            ret, frame = camera.read(active_marker_id=args.marker_id)
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            detections = detector.detect(frame)
            anchor_det = next((d for d in detections if d.id == args.marker_id), None)

            annotated_frame = frame.copy()
            if anchor_det is not None:
                detector.draw_detections(annotated_frame, [anchor_det])
                pose = estimator.estimate_pose(anchor_det)
                if pose is not None:
                    estimator.draw_axes(annotated_frame, pose)
                    
                    t_err_mm, r_err_deg = validate_extrinsic_transform(
                        T_robot_camera=calib_ext.T_robot_camera_matrix,
                        T_camera_anchor_measured=pose.transform_matrix,
                        T_robot_anchor_expected=T_robot_anchor_expected,
                    )
                    translation_errors_mm.append(t_err_mm)
                    rotation_errors_deg.append(r_err_deg)

                    status_text = f"Validating: {len(translation_errors_mm)}/{args.samples} (Err: {t_err_mm:.1f}mm, {r_err_deg:.1f}deg)"
                    color = (0, 220, 0)
                else:
                    status_text = "Anchor Detected (PnP solve failed)"
                    color = (0, 165, 255)
            else:
                status_text = f"Searching for Anchor Marker ID {args.marker_id}..."
                color = (0, 0, 255)

            cv2.putText(
                annotated_frame,
                status_text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                color,
                2,
                cv2.LINE_AA,
            )

            if not args.headless:
                cv2.imshow(window_name, annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    logger.info("Validation aborted by user.")
                    return False

            if args.synthetic and frame_idx > args.samples * 3:
                break

    finally:
        camera.release()
        if not args.headless:
            cv2.destroyAllWindows()

    if not translation_errors_mm:
        logger.error("No valid marker detections acquired during validation.")
        return False

    mean_t_err = float(np.mean(translation_errors_mm))
    max_t_err = float(np.max(translation_errors_mm))
    mean_r_err = float(np.mean(rotation_errors_deg))
    max_r_err = float(np.max(rotation_errors_deg))

    logger.info("=" * 65)
    logger.info("   EXTRINSIC CALIBRATION VALIDATION REPORT")
    logger.info("=" * 65)
    logger.info(f"Evaluated Frames        : {len(translation_errors_mm)}")
    logger.info(f"Mean Translation Error  : {mean_t_err:.2f} mm")
    logger.info(f"Max Translation Error   : {max_t_err:.2f} mm")
    logger.info(f"Mean Orientation Error  : {mean_r_err:.2f} deg")
    logger.info(f"Max Orientation Error   : {max_r_err:.2f} deg")
    logger.info("-" * 65)
    logger.info("NOTE: This validation reflects anchor-point repeatability and residual error.")
    logger.info("It does NOT imply millimeter ground-truth robot accuracy across the entire workspace.")
    logger.info("=" * 65)
    return True


if __name__ == "__main__":
    cli_args = parse_args()
    success = run_extrinsic_validation(cli_args)
    sys.exit(0 if success else 1)
