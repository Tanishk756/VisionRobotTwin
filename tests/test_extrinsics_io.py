"""Unit tests for ExtrinsicCalibration JSON serialization and deserialization."""

import json
from pathlib import Path
import pytest
import numpy as np

from vision.extrinsics import ExtrinsicCalibration
from robotics.coordinate_transform import create_homogeneous_matrix


def test_extrinsics_save_load_roundtrip(tmp_path: Path):
    """Validates full save and load fidelity of ExtrinsicCalibration."""
    T_robot_camera = create_homogeneous_matrix(
        rotation=(np.pi, 0.25, -0.10),
        translation=(0.65, -0.05, 0.42),
    )

    calib = ExtrinsicCalibration(
        version="1.2.0-dev",
        camera_index=1,
        anchor_marker_id=10,
        anchor_marker_size_m=0.05,
        anchor_pose_in_robot_base={
            "translation_m": [0.50, 0.0, 0.0],
            "euler_rpy_rad": [float(np.pi), 0.0, 0.0],
        },
        T_robot_camera_matrix=T_robot_camera,
        T_robot_camera_translation=T_robot_camera[:3, 3],
        T_robot_camera_quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        sample_count=25,
        rejected_samples=2,
        translation_std_mm=1.45,
        rotation_std_deg=0.32,
    )

    file_path = tmp_path / "extrinsics_test.json"
    calib.save(file_path)

    loaded = ExtrinsicCalibration.load(file_path)

    assert loaded.version == "1.2.0-dev"
    assert loaded.camera_index == 1
    assert loaded.anchor_marker_id == 10
    assert loaded.sample_count == 25
    assert loaded.rejected_samples == 2
    assert np.isclose(loaded.translation_std_mm, 1.45)
    assert np.isclose(loaded.rotation_std_deg, 0.32)
    assert np.allclose(loaded.T_robot_camera_matrix, T_robot_camera, atol=1e-6)
    assert np.allclose(loaded.T_robot_camera_translation, T_robot_camera[:3, 3], atol=1e-6)


def test_extrinsics_missing_file_raises():
    """Asserts FileNotFoundError for non-existent file."""
    with pytest.raises(FileNotFoundError):
        ExtrinsicCalibration.load("non_existent_file_path.json")


def test_extrinsics_invalid_json(tmp_path: Path):
    """Asserts ValueError when loading corrupt JSON."""
    bad_file = tmp_path / "corrupt.json"
    bad_file.write_text("{ not valid json !!! }", encoding="utf-8")

    with pytest.raises(ValueError, match="Corrupted extrinsics JSON"):
        ExtrinsicCalibration.load(bad_file)


def test_extrinsics_invalid_schema_missing_keys():
    """Asserts ValueError when essential schema fields are missing."""
    with pytest.raises(ValueError, match="Missing 'T_robot_camera'"):
        ExtrinsicCalibration.from_dict({"version": "1.2.0-dev"})


def test_extrinsics_nan_rejection():
    """Asserts ValueError when matrix contains NaN."""
    bad_dict = {
        "T_robot_camera": {
            "matrix_4x4": [
                [1.0, 0.0, 0.0, np.nan],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        }
    }
    with pytest.raises(ValueError, match="NaN or Inf"):
        ExtrinsicCalibration.from_dict(bad_dict)
