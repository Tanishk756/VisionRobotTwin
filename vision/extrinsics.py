"""World-Anchor Camera-to-Robot Extrinsic Calibration.

Implements Camera-to-Virtual-Robot Extrinsic Calibration via a known World Anchor ArUco marker:
Given:
- T_robot_anchor: Known anchor marker pose in robot base frame (user configured)
- T_camera_anchor: Measured anchor marker pose in camera optical frame
Solves:
- T_robot_camera = T_robot_anchor @ inverse(T_camera_anchor)

Includes robust sample aggregation, outlier rejection, repeatability metrics (std mm and deg),
JSON serialization, and analytical validation.
"""

from dataclasses import dataclass, field, asdict
from datetime import datetime
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np
from scipy.spatial.transform import Rotation

from robotics.coordinate_transform import (
    create_homogeneous_matrix,
    invert_homogeneous_matrix,
    rotation_matrix_to_quaternion,
    rotation_matrix_to_euler,
    quaternion_to_rotation_matrix,
    compute_angular_distance,
    is_valid_se3,
)
from utils.logger import get_logger

logger = get_logger("Vision.Extrinsics")


@dataclass
class ExtrinsicCalibration:
    """Strongly-typed representation of calibrated Camera-to-Robot extrinsics."""
    version: str = "1.2.0"
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    camera_index: int = 0
    anchor_marker_id: int = 10
    anchor_marker_size_m: float = 0.05
    anchor_pose_in_robot_base: Dict[str, Any] = field(default_factory=lambda: {
        "translation_m": [0.50, 0.0, 0.0],
        "euler_rpy_rad": [float(np.pi), 0.0, 0.0],
    })
    T_robot_camera_matrix: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float64))
    T_robot_camera_translation: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    T_robot_camera_quaternion: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64))
    sample_count: int = 0
    rejected_samples: int = 0
    translation_std_mm: float = 0.0
    rotation_std_deg: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        """Serializes calibration metadata to JSON-compatible dictionary."""
        return {
            "version": self.version,
            "timestamp": self.timestamp,
            "camera_index": int(self.camera_index),
            "anchor_marker_id": int(self.anchor_marker_id),
            "anchor_marker_size_m": float(self.anchor_marker_size_m),
            "anchor_pose_in_robot_base": self.anchor_pose_in_robot_base,
            "T_robot_camera": {
                "translation_m": [float(v) for v in self.T_robot_camera_translation],
                "quaternion_xyzw": [float(v) for v in self.T_robot_camera_quaternion],
                "matrix_4x4": [[float(val) for val in row] for row in self.T_robot_camera_matrix],
            },
            "sample_count": int(self.sample_count),
            "rejected_samples": int(self.rejected_samples),
            "translation_std_mm": float(self.translation_std_mm),
            "rotation_std_deg": float(self.rotation_std_deg),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ExtrinsicCalibration":
        """Constructs ExtrinsicCalibration from dictionary with strict schema validation."""
        if not isinstance(data, dict):
            raise ValueError("Extrinsic calibration data must be a dictionary")

        if "T_robot_camera" not in data:
            raise ValueError("Missing 'T_robot_camera' key in extrinsics data")

        t_data = data["T_robot_camera"]
        if "matrix_4x4" not in t_data:
            raise ValueError("Missing 'matrix_4x4' in T_robot_camera")

        mat = np.array(t_data["matrix_4x4"], dtype=np.float64)
        if mat.shape != (4, 4):
            raise ValueError(f"T_robot_camera matrix must be (4, 4), got {mat.shape}")

        if not np.all(np.isfinite(mat)):
            raise ValueError("T_robot_camera matrix contains NaN or Inf values")

        if not is_valid_se3(mat, tol=1e-3):
            raise ValueError("T_robot_camera matrix is not a valid SE(3) transformation")

        trans = np.array(t_data.get("translation_m", mat[:3, 3]), dtype=np.float64)
        quat = np.array(t_data.get("quaternion_xyzw", rotation_matrix_to_quaternion(mat[:3, :3])), dtype=np.float64)

        # Normalize quaternion
        q_norm = np.linalg.norm(quat)
        if q_norm < 1e-6 or not np.isfinite(q_norm):
            raise ValueError("T_robot_camera quaternion is zero or invalid")
        quat = quat / q_norm

        return cls(
            version=str(data.get("version", "1.2.0")),
            timestamp=str(data.get("timestamp", "")),
            camera_index=int(data.get("camera_index", 0)),
            anchor_marker_id=int(data.get("anchor_marker_id", 10)),
            anchor_marker_size_m=float(data.get("anchor_marker_size_m", 0.05)),
            anchor_pose_in_robot_base=data.get("anchor_pose_in_robot_base", {}),
            T_robot_camera_matrix=mat,
            T_robot_camera_translation=trans,
            T_robot_camera_quaternion=quat,
            sample_count=int(data.get("sample_count", 0)),
            rejected_samples=int(data.get("rejected_samples", 0)),
            translation_std_mm=float(data.get("translation_std_mm", 0.0)),
            rotation_std_deg=float(data.get("rotation_std_deg", 0.0)),
        )

    def save(self, path: Union[str, Path]) -> None:
        """Saves calibration to a JSON file."""
        target_path = Path(path)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        with open(target_path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
        logger.info(f"Saved extrinsics calibration to {target_path}")

    @classmethod
    def load(cls, path: Union[str, Path]) -> "ExtrinsicCalibration":
        """Loads calibration from a JSON file."""
        target_path = Path(path)
        if not target_path.exists():
            raise FileNotFoundError(f"Extrinsics calibration file not found: {target_path}")

        try:
            with open(target_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return cls.from_dict(data)
        except json.JSONDecodeError as e:
            raise ValueError(f"Corrupted extrinsics JSON file {target_path}: {e}")


def compute_robot_to_camera_transform(
    T_robot_anchor: np.ndarray,
    T_camera_anchor: np.ndarray,
) -> np.ndarray:
    """Computes T_robot_camera = T_robot_anchor @ inv(T_camera_anchor).

    Args:
        T_robot_anchor: (4, 4) Homogeneous matrix of anchor marker in robot base frame.
        T_camera_anchor: (4, 4) Homogeneous matrix of anchor marker in camera optical frame.

    Returns:
        (4, 4) T_robot_camera homogeneous matrix.
    """
    if T_robot_anchor.shape != (4, 4) or T_camera_anchor.shape != (4, 4):
        raise ValueError("Both transforms must be (4, 4) matrices")

    if not np.all(np.isfinite(T_robot_anchor)) or not np.all(np.isfinite(T_camera_anchor)):
        raise ValueError("Transform matrices contain NaN or Inf")

    T_anchor_camera = invert_homogeneous_matrix(T_camera_anchor)
    T_robot_camera = T_robot_anchor @ T_anchor_camera
    return T_robot_camera


def aggregate_camera_poses(
    transforms_cam_anchor: List[np.ndarray],
    max_pos_deviation_m: float = 0.05,
    max_rot_deviation_deg: float = 15.0,
) -> Tuple[np.ndarray, int, float, float]:
    """Aggregates multiple observed T_camera_anchor transforms with outlier rejection.

    Args:
        transforms_cam_anchor: List of (4, 4) SE(3) matrices.
        max_pos_deviation_m: Translation distance cutoff from median for outlier rejection.
        max_rot_deviation_deg: Angular geodesic distance cutoff from median in degrees.

    Returns:
        Tuple of:
        - robust_T_cam_anchor (4, 4)
        - rejected_count (int)
        - translation_std_mm (float)
        - rotation_std_deg (float)
    """
    if not transforms_cam_anchor:
        raise ValueError("Cannot aggregate empty transform list")

    translations = np.array([T[:3, 3] for T in transforms_cam_anchor], dtype=np.float64)
    quaternions = np.array([
        rotation_matrix_to_quaternion(T[:3, :3]) for T in transforms_cam_anchor
    ], dtype=np.float64)

    # Normalize quaternions and align signs for consistent averaging
    for i in range(len(quaternions)):
        q_norm = np.linalg.norm(quaternions[i])
        if q_norm > 1e-6:
            quaternions[i] /= q_norm
        if i > 0 and np.dot(quaternions[i], quaternions[0]) < 0.0:
            quaternions[i] = -quaternions[i]

    # Compute median translation & median rotation
    median_t = np.median(translations, axis=0)
    
    # Robust rotation estimate via scipy Rotation mean
    rotations = Rotation.from_quat(quaternions)
    mean_rot = rotations.mean()
    median_q = mean_rot.as_quat()

    # Outlier filtering
    accepted_indices = []
    for i in range(len(transforms_cam_anchor)):
        t_dist = float(np.linalg.norm(translations[i] - median_t))
        r_dist_deg = np.degrees(compute_angular_distance(quaternions[i], median_q))

        if t_dist <= max_pos_deviation_m and r_dist_deg <= max_rot_deviation_deg:
            accepted_indices.append(i)

    rejected_count = len(transforms_cam_anchor) - len(accepted_indices)

    if not accepted_indices:
        # If all filtered out, fallback to median
        accepted_indices = list(range(len(transforms_cam_anchor)))
        rejected_count = 0

    acc_translations = translations[accepted_indices]
    acc_quaternions = quaternions[accepted_indices]

    final_t = np.mean(acc_translations, axis=0)
    final_rot = Rotation.from_quat(acc_quaternions).mean()
    final_R = final_rot.as_matrix()

    # Calculate standard deviations
    t_std_mm = float(np.linalg.norm(np.std(acc_translations, axis=0)) * 1000.0)
    
    # Geodesic rotation dispersion
    ref_q = final_rot.as_quat()
    ang_diffs_deg = [
        np.degrees(compute_angular_distance(q, ref_q)) for q in acc_quaternions
    ]
    r_std_deg = float(np.std(ang_diffs_deg)) if len(ang_diffs_deg) > 1 else 0.0

    robust_T = np.eye(4, dtype=np.float64)
    robust_T[:3, :3] = final_R
    robust_T[:3, 3] = final_t

    return robust_T, rejected_count, t_std_mm, r_std_deg


def validate_extrinsic_transform(
    T_robot_camera: np.ndarray,
    T_camera_anchor_measured: np.ndarray,
    T_robot_anchor_expected: np.ndarray,
) -> Tuple[float, float]:
    """Validates camera-to-robot extrinsics by measuring reprojection of anchor marker.

    Args:
        T_robot_camera: (4, 4) Calibrated camera-to-robot base matrix.
        T_camera_anchor_measured: (4, 4) Live measured marker in camera frame.
        T_robot_anchor_expected: (4, 4) Ground truth anchor in robot base frame.

    Returns:
        Tuple of (translation_error_mm, orientation_error_deg).
    """
    T_robot_anchor_est = T_robot_camera @ T_camera_anchor_measured

    t_est = T_robot_anchor_est[:3, 3]
    t_exp = T_robot_anchor_expected[:3, 3]
    trans_error_mm = float(np.linalg.norm(t_est - t_exp) * 1000.0)

    q_est = rotation_matrix_to_quaternion(T_robot_anchor_est[:3, :3])
    q_exp = rotation_matrix_to_quaternion(T_robot_anchor_expected[:3, :3])
    rot_error_deg = float(np.degrees(compute_angular_distance(q_est, q_exp)))

    return trans_error_mm, rot_error_deg
