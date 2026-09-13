"""Camera Intrinsic Calibration Loader and Manager.

Provides facilities to load, save, and validate camera intrinsic matrices and distortion
coefficients. Automatically generates pinhole camera model intrinsics from horizontal
field-of-view when physical calibration is unavailable.
"""

from pathlib import Path
from typing import Optional, Tuple
import math
import numpy as np
from utils.logger import get_logger

logger = get_logger("Vision.Calibration")


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
