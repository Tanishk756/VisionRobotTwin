"""Unit tests for Camera abstraction and physical/synthetic mode isolation."""

import pytest
from config.settings import CameraConfig, ArUcoConfig
from vision.camera import Camera


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
