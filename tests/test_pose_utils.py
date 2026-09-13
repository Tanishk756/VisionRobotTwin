"""Unit tests for camera calibration, ArUco detection, and 6-DoF PnP pose estimation."""

import cv2
import numpy as np
import pytest

from config.settings import ArUcoConfig
from vision.calibration import CameraCalibration
from vision.aruco_detector import ArUcoDetector, MarkerDetection
from vision.pose_estimator import PoseEstimator


def test_pinhole_camera_intrinsics():
    """Verify pinhole calibration generation from horizontal FOV."""
    calib = CameraCalibration.from_fov(width=1280, height=720, fov_deg=65.0)
    assert calib.camera_matrix.shape == (3, 3)
    assert calib.cx == 640.0
    assert calib.cy == 360.0
    assert calib.fx > 500.0  # Reasonable focal length for 1280x720


def test_aruco_detection_and_pnp_pose_estimation():
    """Verify end-to-end marker detection and 6-DoF pose estimation on generated synthetic image."""
    aruco_cfg = ArUcoConfig(dictionary_name="DICT_4X4_50", marker_size_m=0.05)
    calib = CameraCalibration.from_fov(width=640, height=480, fov_deg=60.0)

    detector = ArUcoDetector(aruco_cfg)
    estimator = PoseEstimator(calib, aruco_cfg)

    # Generate a test image containing marker ID 0
    img = np.full((480, 640, 3), 255, dtype=np.uint8)
    marker_raw = np.zeros((150, 150), dtype=np.uint8)
    cv2.aruco.generateImageMarker(detector.dictionary, 0, 150, marker_raw, 1)
    img[165:315, 245:395] = cv2.cvtColor(marker_raw, cv2.COLOR_GRAY2BGR)

    # Detect
    detections = detector.detect(img)
    assert len(detections) == 1
    det = detections[0]
    assert det.id == 0
    assert np.isclose(det.center[0], 320.0, atol=5.0)
    assert np.isclose(det.center[1], 240.0, atol=5.0)

    # Estimate 6-DoF pose
    pose = estimator.estimate_pose(det)
    assert pose is not None
    assert pose.marker_id == 0
    # Marker centered in camera FOV should have near-zero X and Y, and positive Z distance
    assert abs(pose.x) < 0.05
    assert abs(pose.y) < 0.05
    assert pose.z > 0.10  # Positive depth
    assert pose.transform_matrix.shape == (4, 4)
