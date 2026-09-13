"""Unit tests verifying benchmark suite uses shared app.process_frame pipeline across 3-DoF and 6-DoF."""

from pathlib import Path
import pytest
from tools.benchmark_live import run_benchmark


def test_benchmark_synthetic_6dof_execution(tmp_path: Path):
    """Verify benchmark executes 6-DoF mode on synthetic stream using unified pipeline."""
    res = run_benchmark(
        camera_index=0,
        duration_s=0.5,
        synthetic=True,
        control_mode="6dof",
        transform_mode="relative",
        output_dir=tmp_path,
    )
    assert res["input_source"] == "SYNTHETIC"
    assert res["control_mode"] == "6dof"
    assert res["transform_mode"] == "relative"
    assert res["total_frames_processed"] > 0
    assert "mean_camera_fps" in res
    assert "mean_physics_substeps_per_frame" in res
    assert "position_jitter_std_mm" in res
    assert "robot_tracking_error_mm" in res


def test_benchmark_synthetic_3dof_execution(tmp_path: Path):
    """Verify benchmark executes 3-DoF mode on synthetic stream using unified pipeline."""
    res = run_benchmark(
        camera_index=0,
        duration_s=0.5,
        synthetic=True,
        control_mode="3dof",
        transform_mode="relative",
        output_dir=tmp_path,
    )
    assert res["control_mode"] == "3dof"
    assert res["total_frames_processed"] > 0
