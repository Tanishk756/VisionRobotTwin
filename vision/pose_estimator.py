"""6-DoF Pose Estimation for ArUco Markers.

Uses Perspective-n-Point (PnP) optimization with exact marker physical geometry,
camera intrinsics, and distortion parameters to resolve 3D translation (tvec)
and 3D orientation (rvec, rotation matrix, and quaternion).
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from config.settings import ArUcoConfig
from vision.calibration import CameraCalibration
from vision.aruco_detector import MarkerDetection
from utils.logger import get_logger

logger = get_logger("Vision.PoseEstimator")


@dataclass
class MarkerPose:
    """Represents full 6-DoF pose of a marker in camera coordinate frame."""
    marker_id: int
    rvec: np.ndarray          # (3, 1) Rodrigues rotation vector (radians)
    tvec: np.ndarray          # (3, 1) Cartesian translation vector (meters: X, Y, Z)
    rotation_matrix: np.ndarray  # (3, 3) SO(3) rotation matrix
    quaternion_xyzw: np.ndarray  # (4,) Unit quaternion [x, y, z, w]
    transform_matrix: np.ndarray # (4, 4) SE(3) homogeneous transformation matrix
    distance_m: float         # Euclidean distance from camera origin

    @property
    def x(self) -> float:
        return float(self.tvec[0, 0])

    @property
    def y(self) -> float:
        return float(self.tvec[1, 0])

    @property
    def z(self) -> float:
        return float(self.tvec[2, 0])


class PoseEstimator:
    """Estimates metric 6-DoF pose of detected markers using camera calibration."""

    def __init__(self, calibration: CameraCalibration, aruco_config: ArUcoConfig):
        self.calibration = calibration
        self.aruco_config = aruco_config
        self.marker_size = aruco_config.marker_size_m

        # Define 3D object points of marker corners in marker coordinate frame (Z=0)
        # Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        s = self.marker_size / 2.0
        self.obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float32)

    def estimate_pose(self, detection: MarkerDetection) -> Optional[MarkerPose]:
        """Solves PnP problem for a single detected marker.

        Args:
            detection: MarkerDetection with 2D corner coordinates.

        Returns:
            MarkerPose if PnP converged, otherwise None.
        """
        image_points = detection.corners.astype(np.float32)

        # IPPE_SQUARE is the optimal, fast, closed-form solver for planar square fiducials
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                image_points,
                self.calibration.camera_matrix,
                self.calibration.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except Exception:
            # Fallback to standard iterative Levenberg-Marquardt solver
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                image_points,
                self.calibration.camera_matrix,
                self.calibration.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        if not success or rvec is None or tvec is None:
            return None

        # Convert Rodrigues vector to 3x3 rotation matrix
        rot_mat, _ = cv2.Rodrigues(rvec)

        # Convert rotation matrix to unit quaternion [x, y, z, w]
        rot = Rotation.from_matrix(rot_mat)
        quat_xyzw = rot.as_quat()

        # Build 4x4 SE(3) homogeneous transform
        t_matrix = np.eye(4, dtype=np.float64)
        t_matrix[:3, :3] = rot_mat
        t_matrix[:3, 3] = tvec.flatten()

        distance = float(np.linalg.norm(tvec))

        return MarkerPose(
            marker_id=detection.id,
            rvec=rvec,
            tvec=tvec,
            rotation_matrix=rot_mat,
            quaternion_xyzw=quat_xyzw,
            transform_matrix=t_matrix,
            distance_m=distance,
        )

    def draw_axes(
        self,
        frame: np.ndarray,
        pose: MarkerPose,
        length: Optional[float] = None,
    ) -> np.ndarray:
        """Renders 3D coordinate frame axes (X=Red, Y=Green, Z=Blue) on marker center."""
        axis_len = length if length is not None else self.aruco_config.draw_axes_length_m
        cv2.drawFrameAxes(
            frame,
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
            pose.rvec,
            pose.tvec,
            axis_len,
            2,
        )
        return frame
