"""Interactive Camera Calibration Tool using Chessboard Pattern.

Computes camera intrinsic matrix and lens distortion coefficients using standard
pinhole + Brown-Conrady lens distortion model. Reports RMS reprojection error
and saves calibration parameters to calibration/camera_calibration.npz.
"""

import sys
import argparse
from pathlib import Path
import cv2
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.settings import CalibrationConfig
from vision.calibration import CameraCalibration
from utils.logger import setup_logger, get_logger

setup_logger()
logger = get_logger("Tools.CalibrateCamera")


def calibrate(
    camera_index: int = 0,
    width: int = 1280,
    height: int = 720,
    pattern_size: tuple = (9, 6),
    square_size_m: float = 0.025,
    min_frames: int = 15,
    output_path: Path = Path("calibration/camera_calibration.npz"),
) -> None:
    """Runs interactive calibration loop with live webcam."""
    print("=" * 70)
    print(" CAMERA CALIBRATION UTILITY")
    print("=" * 70)
    print(f"Pattern Size (interior corners): {pattern_size[0]} cols x {pattern_size[1]} rows")
    print(f"Square Size: {square_size_m * 1000.0:.1f} mm")
    print(f"Target Captured Frames: {min_frames}")
    print(f"Output File: {output_path}")
    print("\nControls:")
    print("  [SPACE] - Capture frame when chessboard corners are detected")
    print("  [C]     - Compute calibration using captured frames")
    print("  [R]     - Reset captured frames")
    print("  [Q/ESC] - Quit without saving\n")

    # Generate 3D object points in chessboard coordinate system (Z = 0)
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m

    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        logger.error(f"Cannot open webcam index {camera_index}.")
        print("\n[ERROR] No physical camera found for calibration.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    captured_count = 0

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display_frame = frame.copy()

        # Find chessboard corners
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if found:
            refined_corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            cv2.drawChessboardCorners(display_frame, pattern_size, refined_corners, found)
            status_text = f"Pattern DETECTED! Press [SPACE] to capture ({captured_count}/{min_frames})"
            status_col = (0, 255, 0)
        else:
            status_text = f"Searching for {cols}x{rows} chessboard... ({captured_count}/{min_frames} captured)"
            status_col = (0, 165, 255)

        # Header overlay
        cv2.rectangle(display_frame, (10, 10), (display_frame.shape[1] - 10, 50), (20, 20, 20), -1)
        cv2.putText(display_frame, status_text, (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_col, 2)

        cv2.imshow("Camera Calibration", display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            print("Calibration cancelled.")
            break
        elif key == ord(" ") and found:
            obj_points.append(objp)
            img_points.append(refined_corners)
            captured_count += 1
            print(f"[CAPTURED] Frame {captured_count}/{min_frames}")
        elif key == ord("r"):
            obj_points.clear()
            img_points.clear()
            captured_count = 0
            print("[RESET] Cleared all captured frames.")
        elif key == ord("c") or (captured_count >= min_frames and key == 13):
            if captured_count < 5:
                print(f"[WARNING] Need at least 5 frames to calibrate, only {captured_count} captured.")
                continue

            print("\nComputing camera matrix and distortion coefficients...")
            h, w = gray.shape[:2]
            ret_val, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                obj_points, img_points, (w, h), None, None
            )

            # OpenCV ret_val is the overall RMS reprojection error (px)
            rms_reprojection_error = float(ret_val)

            # Compute arithmetic mean point reprojection error across all captured images
            total_error = 0.0
            for i in range(len(obj_points)):
                imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                total_error += error
            mean_error = total_error / len(obj_points)

            print("\n" + "=" * 50)
            print(" CALIBRATION RESULTS")
            print("=" * 50)
            print(f"OpenCV RMS Reprojection Error: {rms_reprojection_error:.4f} pixels")
            print(f"Mean Point Reprojection Error: {mean_error:.4f} pixels")
            print(f"Focal Length (fx, fy):         ({mtx[0, 0]:.2f}, {mtx[1, 1]:.2f})")
            print(f"Principal Point (cx, cy):       ({mtx[0, 2]:.2f}, {mtx[1, 2]:.2f})")
            print(f"Distortion Coeffs (k1,k2,p1,p2,k3): {dist.ravel()[:5]}")
            print("=" * 50)

            calib = CameraCalibration(
                camera_matrix=mtx,
                dist_coeffs=dist,
                image_size=(w, h),
                reprojection_error=rms_reprojection_error,
                rms_reprojection_error_px=rms_reprojection_error,
                mean_reprojection_error_px=mean_error,
                is_calibrated=True,
            )
            calib.save(output_path)
            print(f"\n[SUCCESS] Calibration saved to {output_path}!\n")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Camera Calibration Tool")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--cols", type=int, default=9, help="Interior corners along width")
    parser.add_argument("--rows", type=int, default=6, help="Interior corners along height")
    parser.add_argument("--square-size", type=float, default=0.025, help="Square size in meters")
    args = parser.parse_args()

    calibrate(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        pattern_size=(args.cols, args.rows),
        square_size_m=args.square_size,
    )
