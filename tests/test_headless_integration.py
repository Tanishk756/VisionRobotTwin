"""Integration tests for bounded headless runtime execution."""

import pytest
from config.settings import get_default_config
from main import VisionRobotTwinApp


def test_bounded_headless_run():
    """Verify application runs the full perception-kinematics-simulation loop and terminates cleanly."""
    config = get_default_config()
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.max_frames = 25  # Bounded execution

    app = VisionRobotTwinApp(config, headless_sim=True, record_data=False)
    app.run()
    assert app._frames_processed >= 25
