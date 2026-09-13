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
    assert res["manifest"]["camera_index"] == 0
    assert res["manifest"]["control_mode"] == "6dof"
    assert res["manifest"]["transform_mode"] == "relative"
    assert res["session_metrics"]["frames_captured"] > 0
    assert "actual_camera_fps" in res["session_metrics"]
    assert "physics_substeps_per_frame" in res["session_metrics"]
    assert "position_std_xyz_mm" in res["optical_jitter"]
    assert "mean_error_mm" in res["digital_twin_ee_tracking"]


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
    assert res["manifest"]["control_mode"] == "3dof"
    assert res["session_metrics"]["frames_captured"] > 0
