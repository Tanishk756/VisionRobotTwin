"""Unit tests for demo recording and screenshot metadata capture."""

import json
from pathlib import Path
import pytest
import numpy as np

from config.settings import get_default_config
from main import VisionRobotTwinApp


def test_screenshot_metadata_generation(tmp_path: Path):
    """Asserts that _capture_screenshots produces valid JSON metadata alongside images."""
    config = get_default_config()
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.screenshots_dir = tmp_path / "screenshots"

    app = VisionRobotTwinApp(config=config, headless_sim=True)

    # Process one frame to populate _last_frame_result
    res = app.process_frame()
    app._capture_screenshots(last_display_frame=res.display_frame, last_result=res)

    app.cleanup()

    json_files = list(config.screenshots_dir.glob("session_*.json"))
    assert len(json_files) == 1

    with open(json_files[0], "r", encoding="utf-8") as f:
        meta = json.load(f)

    assert meta["version"] == "1.2.0"
    assert "camera_pose" in meta
    assert "calibration_status" in meta
    assert "extrinsics_status" in meta
    assert "robot_target_pos" in meta
