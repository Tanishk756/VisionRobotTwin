import json
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
import math
import cv2
import numpy as np
from utils.logger import get_logger

logger = get_logger("Vision.Calibration")


@dataclass
class CalibrationReport:
    """Structured machine-readable camera calibration report."""
    version: str
    timestamp: str
    camera_index: int
    image_size: Tuple[int, int]
    chessboard_pattern: Tuple[int, int]
    square_size_m: float
    accepted_frames_count: int
    opencv_rms_reprojection_error_px: float
    mean_point_reprojection_error_px: float
    per_frame_reprojection_errors_px: List[float]
    diversity_score: float
    camera_matrix: List[List[float]]
    dist_coeffs: List[float]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def save(self, file_path: Path) -> bool:
        path = Path(file_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.to_dict(), f, indent=2)
            logger.info(f"Saved calibration report JSON to {path}")
            return True
        except Exception as e:
            logger.error(f"Failed to save calibration report JSON to {path}: {e}")
            return False

    @classmethod
    def load(cls, file_path: Path) -> Optional["CalibrationReport"]:
        path = Path(file_path)
        if not path.exists():
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return cls(
                version=data.get("version", "unknown"),
                timestamp=data.get("timestamp", ""),
                camera_index=int(data.get("camera_index", 0)),
                image_size=tuple(data.get("image_size", (1280, 720))),
                chessboard_pattern=tuple(data.get("chessboard_pattern", (9, 6))),
                square_size_m=float(data.get("square_size_m", 0.025)),
                accepted_frames_count=int(data.get("accepted_frames_count", 0)),
                opencv_rms_reprojection_error_px=float(data.get("opencv_rms_reprojection_error_px", 0.0)),
                mean_point_reprojection_error_px=float(data.get("mean_point_reprojection_error_px", 0.0)),
                per_frame_reprojection_errors_px=[float(x) for x in data.get("per_frame_reprojection_errors_px", [])],
                diversity_score=float(data.get("diversity_score", 0.0)),
                camera_matrix=data.get("camera_matrix", []),
                dist_coeffs=data.get("dist_coeffs", []),
            )
        except Exception as e:
            logger.error(f"Failed to load calibration report from {path}: {e}")
            return None


def calculate_image_sharpness(gray_img: np.ndarray) -> float:
    """Computes focus sharpness via Laplacian variance."""
    return float(cv2.Laplacian(gray_img, cv2.CV_64F).var())


def calculate_board_area_ratio(corners: np.ndarray, image_shape: Tuple[int, int]) -> float:
    """Calculates fraction of image area occupied by detected chessboard corners."""
    h, w = image_shape[:2]
    img_area = float(w * h)
    hull = cv2.convexHull(corners.reshape(-1, 2))
    hull_area = cv2.contourArea(hull)
    return float(hull_area / max(img_area, 1.0))


def calculate_sample_diversity(all_corners: List[np.ndarray], image_shape: Tuple[int, int]) -> Tuple[float, Dict[str, Any]]:
    """Evaluates spatial, scale, and tilt diversity of captured calibration samples."""
    if not all_corners:
        return 0.0, {}

    h, w = image_shape[:2]
    cx_grid = []
    cy_grid = []
    area_ratios = []

    for c in all_corners:
        pts = c.reshape(-1, 2)
        mean_pt = np.mean(pts, axis=0)
        cx_grid.append(mean_pt[0] / max(w, 1))
        cy_grid.append(mean_pt[1] / max(h, 1))
        area_ratios.append(calculate_board_area_ratio(c, image_shape))

    # Spatial coverage score across 4 quadrants + center
    cx_arr = np.array(cx_grid)
    cy_arr = np.array(cy_grid)
    spatial_spread = float(np.std(cx_arr) + np.std(cy_arr))
    scale_spread = float(np.std(area_ratios))

    # Normalized diversity score [0.0 - 1.0]
    diversity_score = float(np.clip((spatial_spread * 2.0) + (scale_spread * 5.0), 0.0, 1.0))

    metrics = {
        "spatial_spread": spatial_spread,
        "scale_spread": scale_spread,
        "mean_area_ratio": float(np.mean(area_ratios)),
        "diversity_score": diversity_score,
    }
    return diversity_score, metrics


def is_duplicate_sample(
    corners: np.ndarray,
    existing_corners: List[np.ndarray],
    min_center_dist_px: float = 35.0,
    min_area_diff_ratio: float = 0.08,
) -> bool:
    """Detects whether current sample is nearly identical to an already captured frame."""
    if not existing_corners:
        return False

    curr_pts = corners.reshape(-1, 2)
    curr_center = np.mean(curr_pts, axis=0)
    curr_area = cv2.contourArea(cv2.convexHull(curr_pts))

    for prev in existing_corners:
        prev_pts = prev.reshape(-1, 2)
        prev_center = np.mean(prev_pts, axis=0)
        prev_area = cv2.contourArea(cv2.convexHull(prev_pts))

        dist = float(np.linalg.norm(curr_center - prev_center))
        area_diff = abs(curr_area - prev_area) / max(prev_area, 1.0)

        if dist < min_center_dist_px and area_diff < min_area_diff_ratio:
            return True

    return False


class CameraCalibration:
    """Encapsulates camera intrinsic parameters and lens distortion coefficients."""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_size: Tuple[int, int],
        reprojection_error: Optional[float] = None,
        rms_reprojection_error_px: Optional[float] = None,
        mean_reprojection_error_px: Optional[float] = None,
        is_calibrated: bool = False,
    ):
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        self.image_size = image_size  # (width, height)
        self.rms_reprojection_error_px = (
            rms_reprojection_error_px
            if rms_reprojection_error_px is not None
            else reprojection_error
        )
        self.reprojection_error = self.rms_reprojection_error_px  # Compatibility alias
        self.mean_reprojection_error_px = mean_reprojection_error_px
        self.is_calibrated = is_calibrated

        if self.camera_matrix.shape != (3, 3):
            raise ValueError(f"Camera matrix must be (3, 3), got {self.camera_matrix.shape}")

    @property
    def fx(self) -> float:
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.camera_matrix[1, 2])

    @classmethod
    def from_fov(cls, width: int, height: int, fov_deg: float = 65.0) -> "CameraCalibration":
        """Constructs an uncalibrated default pinhole camera model based on field of view."""
        fov_rad = math.radians(fov_deg)
        focal_length = (width / 2.0) / math.tan(fov_rad / 2.0)
        cx = width / 2.0
        cy = height / 2.0

        camera_matrix = np.array([
            [focal_length, 0.0, cx],
            [0.0, focal_length, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        logger.info(
            f"Generated default pinhole camera model: fx={focal_length:.1f}, fy={focal_length:.1f}, "
            f"cx={cx:.1f}, cy={cy:.1f}, FOV={fov_deg:.1f}°"
        )
        return cls(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=(width, height),
            reprojection_error=None,
            rms_reprojection_error_px=None,
            mean_reprojection_error_px=None,
            is_calibrated=False,
        )

    @classmethod
    def load(cls, file_path: Path) -> Optional["CameraCalibration"]:
        """Loads calibration from an .npz archive."""
        path = Path(file_path)
        if not path.exists():
            return None

        try:
            data = np.load(str(path))
            camera_matrix = data["camera_matrix"]
            dist_coeffs = data["dist_coeffs"]
            image_size = tuple(data["image_size"]) if "image_size" in data else (1280, 720)
            rms_err = None
            if "rms_reprojection_error_px" in data:
                rms_err = float(data["rms_reprojection_error_px"])
            elif "reprojection_error" in data:
                rms_err = float(data["reprojection_error"])

            mean_err = float(data["mean_reprojection_error_px"]) if "mean_reprojection_error_px" in data else None

            logger.info(
                f"Successfully loaded camera calibration from {path} "
                f"(RMS Reprojection Error: {rms_err:.4f} px)" if rms_err is not None else f"from {path}"
            )
            return cls(
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                image_size=image_size,
                reprojection_error=rms_err,
                rms_reprojection_error_px=rms_err,
                mean_reprojection_error_px=mean_err,
                is_calibrated=True,
            )
        except Exception as e:
            logger.error(f"Failed to load calibration file {path}: {e}")
            return None

    def save(self, file_path: Path) -> bool:
        """Saves calibration parameters to an .npz archive."""
        path = Path(file_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            save_dict = {
                "camera_matrix": self.camera_matrix,
                "dist_coeffs": self.dist_coeffs,
                "image_size": np.array(self.image_size),
                "reprojection_error": self.rms_reprojection_error_px if self.rms_reprojection_error_px is not None else 0.0,
                "rms_reprojection_error_px": self.rms_reprojection_error_px if self.rms_reprojection_error_px is not None else 0.0,
            }
            if self.mean_reprojection_error_px is not None:
                save_dict["mean_reprojection_error_px"] = self.mean_reprojection_error_px

            np.savez_compressed(str(path), **save_dict)
            logger.info(f"Saved calibration parameters to {path}")
            return True
        except Exception as e:
            logger.error(f"Failed to save calibration parameters to {path}: {e}")
            return False


def load_or_create_calibration(
    calibration_file: Path,
    width: int,
    height: int,
    default_fov: float = 65.0,
) -> CameraCalibration:
    """Loads existing calibration if present, or creates a default pinhole model."""
    calib = CameraCalibration.load(calibration_file)
    if calib is not None:
        return calib

    logger.warning(
        f"Calibration file '{calibration_file}' not found. Falling back to default pinhole model. "
        "For metrically accurate 6-DoF tracking, please run 'python tools/calibrate_camera.py'."
    )
    return CameraCalibration.from_fov(width, height, default_fov)
