"""Unit tests for camera calibration quality heuristics, reports, and sample filtering."""

import json
from pathlib import Path
import pytest
import numpy as np
import cv2

from vision.calibration import (
    CalibrationReport,
    calculate_image_sharpness,
    calculate_board_area_ratio,
    calculate_sample_diversity,
    is_duplicate_sample,
)


def test_sharpness_and_area_heuristics():
    """Validates sharpness and area ratio calculations on synthetic test patterns."""
    # Blurry uniform image
    flat_img = np.full((480, 640), 128, dtype=np.uint8)
    sharpness_flat = calculate_image_sharpness(flat_img)
    assert sharpness_flat < 1.0

    # High-contrast chessboard-like synthetic image
    checker_img = np.zeros((480, 640), dtype=np.uint8)
    checker_img[::20, :] = 255
    checker_img[:, ::20] = 255
    sharpness_checker = calculate_image_sharpness(checker_img)
    assert sharpness_checker > 100.0

    # Board area ratio
    corners = np.array([
        [[100.0, 100.0]],
        [[300.0, 100.0]],
        [[300.0, 250.0]],
        [[100.0, 250.0]],
    ], dtype=np.float32)
    area_ratio = calculate_board_area_ratio(corners, image_shape=(480, 640))
    # Area = 200 * 150 = 30000; Total = 480 * 640 = 307200; Ratio ~ 0.097
    assert 0.08 < area_ratio < 0.12


def test_duplicate_sample_rejection():
    """Asserts that near-identical chessboard corner sets are identified as duplicates."""
    corners_a = np.array([
        [[100.0, 100.0]],
        [[200.0, 100.0]],
        [[200.0, 200.0]],
        [[100.0, 200.0]],
    ], dtype=np.float32)

    # Very small displacement (< 2px)
    corners_near = corners_a + 1.0
    assert is_duplicate_sample(corners_near, [corners_a], min_center_dist_px=35.0)

    # Significant displacement (> 50px)
    corners_far = corners_a + 60.0
    assert not is_duplicate_sample(corners_far, [corners_a], min_center_dist_px=35.0)


def test_calibration_report_json_serialization(tmp_path: Path):
    """Validates CalibrationReport JSON export and import fidelity."""
    report = CalibrationReport(
        version="1.2.0",
        timestamp="2026-09-13T12:00:00",
        camera_index=0,
        image_size=(1280, 720),
        chessboard_pattern=(9, 6),
        square_size_m=0.025,
        accepted_frames_count=20,
        opencv_rms_reprojection_error_px=0.28,
        mean_point_reprojection_error_px=0.25,
        per_frame_reprojection_errors_px=[0.2, 0.3, 0.25],
        diversity_score=0.88,
        camera_matrix=[[1000.0, 0.0, 640.0], [0.0, 1000.0, 360.0], [0.0, 0.0, 1.0]],
        dist_coeffs=[-0.05, 0.02, 0.001, 0.001, 0.0],
    )

    out_json = tmp_path / "camera_calibration_report.json"
    report.save(out_json)

    loaded = CalibrationReport.load(out_json)
    assert loaded is not None
    assert loaded.accepted_frames_count == 20
    assert loaded.opencv_rms_reprojection_error_px == 0.28
    assert loaded.diversity_score == 0.88
    assert loaded.camera_matrix[0][0] == 1000.0
