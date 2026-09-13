"""Interactive Camera Calibration Tool using Chessboard Pattern.

Computes camera intrinsic matrix and lens distortion coefficients using standard
pinhole + Brown-Conrady lens distortion model. Reports OpenCV RMS reprojection error,
mean point reprojection error, per-frame residuals, sample diversity score, and saves:
- calibration/camera_calibration.npz
- calibration/camera_calibration_report.json
- calibration/calibration_diagnostics.png
"""

import sys
import argparse
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple
import cv2
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from visionrobottwin_version import __version__
from vision.calibration import (
    CameraCalibration,
    CalibrationReport,
    calculate_image_sharpness,
    calculate_board_area_ratio,
    calculate_sample_diversity,
    is_duplicate_sample,
)
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.CalibrateCamera")


def render_diagnostics_image(
    image_size: Tuple[int, int],
    all_corners: List[np.ndarray],
    per_frame_errors: List[float],
    output_path: Path,
) -> None:
    """Renders a 2-panel calibration diagnostic visualization without external heavy GUI dependencies."""
    w, h = image_size
    diag_w, diag_h = 1280, 640
    canvas = np.zeros((diag_h, diag_w, 3), dtype=np.uint8)
    canvas[:] = (25, 25, 25)

    # Panel 1: Corner Spatial Distribution Overlay (Left 640x640)
    p1_x, p1_y, p1_w, p1_h = 20, 50, 580, 560
    cv2.rectangle(canvas, (p1_x, p1_y), (p1_x + p1_w, p1_y + p1_h), (45, 45, 45), -1)
    cv2.rectangle(canvas, (p1_x, p1_y), (p1_x + p1_w, p1_y + p1_h), (80, 80, 80), 1)
    cv2.putText(canvas, "Corner Spatial Coverage Map", (p1_x + 10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)

    # Draw grid quadrants
    mid_x = p1_x + p1_w // 2
    mid_y = p1_y + p1_h // 2
    cv2.line(canvas, (mid_x, p1_y), (mid_x, p1_y + p1_h), (60, 60, 60), 1)
    cv2.line(canvas, (p1_x, mid_y), (p1_x + p1_w, mid_y), (60, 60, 60), 1)

    scale_x = p1_w / float(w)
    scale_y = p1_h / float(h)

    for i, corners in enumerate(all_corners):
        pts = corners.reshape(-1, 2)
        color = (
            int(50 + (i * 205) // max(len(all_corners), 1)),
            int(255 - (i * 180) // max(len(all_corners), 1)),
            int(100 + (i * 120) // max(len(all_corners), 1)),
        )
        for pt in pts:
            px = int(p1_x + pt[0] * scale_x)
            py = int(p1_y + pt[1] * scale_y)
            cv2.circle(canvas, (px, py), 2, color, -1)

    # Panel 2: Per-Frame Reprojection Error Bar Chart (Right 640x640)
    p2_x, p2_y, p2_w, p2_h = 640, 50, 610, 560
    cv2.rectangle(canvas, (p2_x, p2_y), (p2_x + p2_w, p2_y + p2_h), (45, 45, 45), -1)
    cv2.rectangle(canvas, (p2_x, p2_y), (p2_x + p2_w, p2_y + p2_h), (80, 80, 80), 1)
    cv2.putText(canvas, "Per-Frame Reprojection Error (px)", (p2_x + 10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)

    if per_frame_errors:
        max_err = max(max(per_frame_errors), 1.0)
        n_bars = len(per_frame_errors)
        bar_width = max(int((p2_w - 40) / max(n_bars, 1)) - 4, 4)

        # Draw 0.5px and 1.0px reference lines
        for ref_val, ref_col in [(0.5, (0, 200, 0)), (1.0, (0, 165, 255))]:
            if ref_val <= max_err:
                ref_y = int(p2_y + p2_h - 40 - (ref_val / max_err) * (p2_h - 80))
                cv2.line(canvas, (p2_x + 30, ref_y), (p2_x + p2_w - 20, ref_y), ref_col, 1, cv2.LINE_AA)
                cv2.putText(canvas, f"{ref_val:.1f}px", (p2_x + 5, ref_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, ref_col, 1)

        for i, err in enumerate(per_frame_errors):
            bx = p2_x + 40 + i * (bar_width + 4)
            bar_h = int((err / max_err) * (p2_h - 80))
            by = p2_y + p2_h - 40 - bar_h
            b_col = (0, 255, 0) if err < 0.5 else ((0, 165, 255) if err < 1.0 else (0, 0, 255))
            cv2.rectangle(canvas, (bx, by), (bx + bar_width, p2_y + p2_h - 40), b_col, -1)
            cv2.putText(canvas, f"{i+1}", (bx, p2_y + p2_h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), canvas)
    logger.info(f"Saved calibration diagnostic chart to {output_path}")


def generate_synthetic_calibration_frames(
    width: int,
    height: int,
    pattern_size: Tuple[int, int],
    square_size_m: float,
    num_frames: int = 15,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """Generates synthetic ground-truth calibration object/image point pairs for automated tests."""
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m

    # Nominal intrinsics
    fx = fy = float(width) * 0.8
    cx, cy = float(width) / 2.0, float(height) / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)

    all_obj = []
    all_img = []

    np.random.seed(42)
    for i in range(num_frames):
        # Vary translation and rotation across view volume
        tx = (i % 3 - 1) * 0.12 + np.random.uniform(-0.02, 0.02)
        ty = ((i // 3) % 3 - 1) * 0.08 + np.random.uniform(-0.02, 0.02)
        tz = 0.55 + (i * 0.03) + np.random.uniform(-0.02, 0.02)
        tvec = np.array([[tx], [ty], [tz]], dtype=np.float64)

        rvec = np.array([
            np.radians(np.random.uniform(-25, 25)),
            np.radians(np.random.uniform(-25, 25)),
            np.radians(np.random.uniform(-15, 15)),
        ], dtype=np.float64)

        img_pts, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
        all_obj.append(objp)
        all_img.append(img_pts.astype(np.float32))

    return all_obj, all_img


def calibrate(
    camera_index: int = 0,
    width: int = 1280,
    height: int = 720,
    pattern_size: Tuple[int, int] = (9, 6),
    square_size_m: float = 0.025,
    min_frames: int = 15,
    output_path: Path = Path("calibration/camera_calibration.npz"),
    report_path: Path = Path("calibration/camera_calibration_report.json"),
    diagnostics_path: Path = Path("calibration/calibration_diagnostics.png"),
    synthetic: bool = False,
) -> Optional[CalibrationReport]:
    """Runs calibration pipeline with quality heuristics, reports, and diagnostics."""
    print("=" * 70)
    print(" CAMERA CALIBRATION UTILITY (v1.2)")
    print("=" * 70)
    print(f" Source: {'SYNTHETIC GENERATOR' if synthetic else f'PHYSICAL WEBCAM (Index {camera_index})'}")
    print(f" Pattern Size: {pattern_size[0]} cols x {pattern_size[1]} rows | Square: {square_size_m * 1000.0:.1f} mm")
    print(f" Target Frames: {min_frames}")
    print(f" Outputs: {output_path} | {report_path}")
    print("=" * 70)

    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m

    obj_points: List[np.ndarray] = []
    img_points: List[np.ndarray] = []

    if synthetic:
        logger.info("Executing synthetic camera calibration...")
        obj_points, img_points = generate_synthetic_calibration_frames(
            width=width,
            height=height,
            pattern_size=pattern_size,
            square_size_m=square_size_m,
            num_frames=min_frames,
        )
    else:
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(camera_index)

        if not cap.isOpened():
            logger.error(f"Cannot open webcam index {camera_index}.")
            print("\n[ERROR] No physical camera found for calibration.")
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        captured_count = 0
        warning_message = ""
        warning_timer = 0.0

        print("\nControls:")
        print("  [SPACE] - Capture frame when chessboard corners are detected")
        print("  [C]     - Compute calibration using captured frames")
        print("  [R]     - Reset captured frames")
        print("  [Q/ESC] - Quit without saving\n")

        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            display_frame = frame.copy()

            found, corners = cv2.findChessboardCorners(
                gray,
                pattern_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            sharpness = calculate_image_sharpness(gray)
            status_col = (0, 165, 255)
            status_text = f"Searching for {cols}x{rows} chessboard... ({captured_count}/{min_frames} captured)"

            refined_corners = None
            if found:
                refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(display_frame, pattern_size, refined_corners, found)
                area_ratio = calculate_board_area_ratio(refined_corners, (height, width))

                if sharpness < 25.0:
                    status_text = f"Pattern BLURRY (sharpness {sharpness:.0f} < 25). Hold still!"
                    status_col = (0, 0, 255)
                elif area_ratio < 0.08:
                    status_text = f"Pattern TOO FAR ({area_ratio*100:.1f}% area < 8%). Move closer!"
                    status_col = (0, 165, 255)
                else:
                    status_text = f"Pattern READY! Press [SPACE] to capture ({captured_count}/{min_frames})"
                    status_col = (0, 255, 0)

            # Header overlay
            cv2.rectangle(display_frame, (10, 10), (display_frame.shape[1] - 10, 50), (20, 20, 20), -1)
            cv2.putText(display_frame, status_text, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_col, 2)

            if warning_message:
                cv2.putText(display_frame, warning_message, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.imshow("Camera Calibration", display_frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                print("Calibration cancelled.")
                cap.release()
                cv2.destroyAllWindows()
                return None

            elif key == ord(" ") and found and refined_corners is not None:
                if is_duplicate_sample(refined_corners, img_points):
                    warning_message = "[WARNING] Near-duplicate view! Please vary board tilt/depth."
                    print(warning_message)
                else:
                    warning_message = ""
                    obj_points.append(objp)
                    img_points.append(refined_corners)
                    captured_count += 1
                    print(f"[CAPTURED] Frame {captured_count}/{min_frames} (sharpness: {sharpness:.0f})")

            elif key == ord("r"):
                obj_points.clear()
                img_points.clear()
                captured_count = 0
                warning_message = ""
                print("[RESET] Cleared all captured frames.")

            elif key == ord("c") or (captured_count >= min_frames and key == 13):
                if captured_count < 5:
                    print(f"[WARNING] Need at least 5 frames to calibrate, only {captured_count} captured.")
                    continue
                break

        cap.release()
        cv2.destroyAllWindows()

    if len(obj_points) < 5:
        logger.error(f"Insufficient calibration samples ({len(obj_points)} < 5).")
        return None

    print("\nComputing camera matrix and distortion coefficients...")
    ret_val, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (width, height), None, None
    )

    rms_reprojection_error = float(ret_val)

    # Compute per-frame and overall mean point reprojection errors
    per_frame_errors: List[float] = []
    total_point_error = 0.0

    for i in range(len(obj_points)):
        imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        err_norm = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2)
        mean_frame_err = float(err_norm / len(imgpoints2))
        per_frame_errors.append(mean_frame_err)
        total_point_error += mean_frame_err

    mean_point_error = total_point_error / len(obj_points)
    diversity_score, div_metrics = calculate_sample_diversity(img_points, (height, width))

    print("\n" + "=" * 60)
    print(" CALIBRATION RESULTS & QUALITY REPORT")
    print("=" * 60)
    print(f"OpenCV RMS Reprojection Error: {rms_reprojection_error:.4f} px")
    print(f"Mean Point Reprojection Error: {mean_point_error:.4f} px")
    print(f"Max Per-Frame Error:           {max(per_frame_errors):.4f} px (Frame {np.argmax(per_frame_errors)+1})")
    print(f"Sample Diversity Score:        {diversity_score*100.0:.1f} / 100")
    print(f"Accepted Frame Count:          {len(obj_points)}")
    print(f"Focal Length (fx, fy):         ({mtx[0, 0]:.2f}, {mtx[1, 1]:.2f})")
    print(f"Principal Point (cx, cy):       ({mtx[0, 2]:.2f}, {mtx[1, 2]:.2f})")
    print(f"Distortion Coeffs (k1,k2,p1,p2,k3): {dist.ravel()[:5]}")
    print("=" * 60)

    # Save .npz calibration archive
    calib = CameraCalibration(
        camera_matrix=mtx,
        dist_coeffs=dist,
        image_size=(width, height),
        reprojection_error=rms_reprojection_error,
        rms_reprojection_error_px=rms_reprojection_error,
        mean_reprojection_error_px=mean_point_error,
        is_calibrated=True,
    )
    calib.save(output_path)

    # Save machine-readable JSON report
    report = CalibrationReport(
        version=__version__,
        timestamp=datetime.now().isoformat(),
        camera_index=camera_index,
        image_size=(width, height),
        chessboard_pattern=pattern_size,
        square_size_m=square_size_m,
        accepted_frames_count=len(obj_points),
        opencv_rms_reprojection_error_px=round(rms_reprojection_error, 4),
        mean_point_reprojection_error_px=round(mean_point_error, 4),
        per_frame_reprojection_errors_px=[round(e, 4) for e in per_frame_errors],
        diversity_score=round(diversity_score, 3),
        camera_matrix=mtx.tolist(),
        dist_coeffs=dist.ravel().tolist(),
    )
    report.save(report_path)

    # Save diagnostic visualization
    render_diagnostics_image(
        image_size=(width, height),
        all_corners=img_points,
        per_frame_errors=per_frame_errors,
        output_path=diagnostics_path,
    )

    print(f"\n[SUCCESS] Calibration saved to {output_path} and {report_path}!\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Camera Calibration Tool")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--cols", type=int, default=9, help="Interior corners along width")
    parser.add_argument("--rows", type=int, default=6, help="Interior corners along height")
    parser.add_argument("--square-size", type=float, default=0.025, help="Square size in meters")
    parser.add_argument("--synthetic", action="store_true", help="Run synthetic calibration without camera")
    args = parser.parse_args()

    calibrate(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        pattern_size=(args.cols, args.rows),
        square_size_m=args.square_size,
        synthetic=args.synthetic,
    )
