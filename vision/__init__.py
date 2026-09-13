"""Vision package for camera capture, ArUco marker detection, calibration, and 6-DoF pose estimation."""

from vision.calibration import CameraCalibration, load_or_create_calibration
from vision.extrinsics import (
    ExtrinsicCalibration,
    compute_robot_to_camera_transform,
    aggregate_camera_poses,
    validate_extrinsic_transform,
)
from vision.camera import Camera, SyntheticFrameGenerator
from vision.aruco_detector import ArUcoDetector, MarkerDetection
from vision.pose_estimator import PoseEstimator, MarkerPose

__all__ = [
    "CameraCalibration",
    "load_or_create_calibration",
    "ExtrinsicCalibration",
    "compute_robot_to_camera_transform",
    "aggregate_camera_poses",
    "validate_extrinsic_transform",
    "Camera",
    "SyntheticFrameGenerator",
    "ArUcoDetector",
    "MarkerDetection",
    "PoseEstimator",
    "MarkerPose",
]
