"""Unit tests for Camera abstraction, physical/synthetic mode isolation, and mid-stream disconnects."""

from unittest.mock import MagicMock
import numpy as np
import pytest
from config.settings import CameraConfig, ArUcoConfig
from vision.camera import Camera, CameraStreamError


def test_camera_explicit_synthetic_mode():
    """Verify synthetic mode initializes and produces valid non-empty frames."""
    cfg = CameraConfig(synthetic_mode=True, width=640, height=480)
    cam = Camera(cfg, ArUcoConfig())
    assert cam.is_synthetic is True

    ret, frame = cam.read(active_marker_id=0, state="MANUAL")
    assert ret is True
    assert frame is not None
    assert frame.shape == (480, 640, 3)
    cam.release()


def test_camera_failure_raises_when_fallback_disabled():
    """Verify that an invalid camera index raises RuntimeError when fallback is disabled."""
    cfg = CameraConfig(camera_index=999, synthetic_mode=False, allow_synthetic_fallback=False)
    with pytest.raises(RuntimeError) as exc_info:
        Camera(cfg, ArUcoConfig())
    assert "could not be opened" in str(exc_info.value)


def test_camera_midstream_disconnect_raises_error():
    """Verify that consecutive read failures on open physical camera raise CameraStreamError."""
    cfg = CameraConfig(
        camera_index=0,
        synthetic_mode=False,
        allow_synthetic_fallback=False,
        max_consecutive_read_failures=5,
    )
    cam = Camera.__new__(Camera)
    cam.config = cfg
    cam.aruco_config = ArUcoConfig()
    cam.is_synthetic = False
    cam._synthetic_gen = None
    cam._consecutive_read_failures = 0
    cam.max_consecutive_read_failures = 5

    # Mock an open physical camera capture that suddenly starts returning failure
    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_cap.read.return_value = (False, None)
    cam.cap = mock_cap

    # First 4 failures return (False, None)
    for _ in range(4):
        ret, frame = cam.read()
        assert ret is False
        assert frame is None

    # 5th failure exceeds threshold and raises CameraStreamError
    with pytest.raises(CameraStreamError) as exc_info:
        cam.read()
    assert "disconnected" in str(exc_info.value)


def test_camera_midstream_failure_fallback_when_enabled():
    """Verify that if fallback is enabled, camera transitions to synthetic generator on frame failure."""
    cfg = CameraConfig(
        camera_index=0,
        synthetic_mode=False,
        allow_synthetic_fallback=True,
    )
    cam = Camera.__new__(Camera)
    cam.config = cfg
    cam.aruco_config = ArUcoConfig()
    cam.is_synthetic = False
    cam._synthetic_gen = None
    cam._consecutive_read_failures = 0
    cam.max_consecutive_read_failures = 30

    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = True
    mock_cap.read.return_value = (False, None)
    cam.cap = mock_cap

    ret, frame = cam.read(active_marker_id=0, state="MANUAL")
    assert ret is True
    assert frame is not None
    assert cam.is_synthetic is True
