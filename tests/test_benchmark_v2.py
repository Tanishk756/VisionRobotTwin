"""Unit tests for Benchmark Suite V2 statistical calculations and session manifest."""

from pathlib import Path
import pytest
import numpy as np

from tools.benchmark_live import run_benchmark


def test_benchmark_synthetic_stationary_execution(tmp_path: Path):
    """Executes a short synthetic stationary benchmark and validates output schema."""
    summary = run_benchmark(
        camera_index=0,
        duration_s=1.0,
        synthetic=True,
        benchmark_mode="stationary",
        control_mode="6dof",
        transform_mode="relative",
        output_dir=tmp_path,
    )

    assert summary["benchmark_mode"] == "stationary"
    assert "manifest" in summary
    assert summary["manifest"]["visionrobottwin_version"] == "1.2.0-dev"
    assert "optical_jitter" in summary
    assert "session_metrics" in summary
    assert summary["session_metrics"]["frames_captured"] > 0
    assert "digital_twin_ee_tracking" in summary


def test_benchmark_synthetic_tracking_execution(tmp_path: Path):
    """Executes a short synthetic tracking benchmark."""
    summary = run_benchmark(
        camera_index=0,
        duration_s=1.0,
        synthetic=True,
        benchmark_mode="tracking",
        control_mode="6dof",
        transform_mode="relative",
        output_dir=tmp_path,
    )

    assert summary["benchmark_mode"] == "tracking"
    assert summary["session_metrics"]["detection_rate_pct"] == 100.0
    assert summary["session_metrics"]["frames_tracked"] > 0
