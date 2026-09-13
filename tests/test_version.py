"""Tests for project version reporting."""

import subprocess
import sys
from visionrobottwin_version import __version__


def test_version_string():
    """Verify version string conforms to semantic versioning."""
    assert __version__ == "1.2.0-dev"


def test_cli_version_flag():
    """Verify python main.py --version returns expected version string."""
    result = subprocess.run(
        [sys.executable, "main.py", "--version"],
        capture_output=True,
        text=True,
        check=False,
    )
    output = result.stdout + result.stderr
    assert "VisionRobotTwin 1.2.0-dev" in output
