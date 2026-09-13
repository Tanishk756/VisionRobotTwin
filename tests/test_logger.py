"""Unit tests for logger hierarchy, propagation, and handler deduplication."""

import logging
from pathlib import Path
import pytest

from utils.logger import setup_logger, get_logger


def test_logger_hierarchy_and_propagation():
    """Verify get_logger returns children under VisionRobotTwin root and propagates events."""
    base_logger = setup_logger(name="VisionRobotTwin", debug=True)
    child_logger = get_logger("Robotics.IK")

    assert child_logger.name == "VisionRobotTwin.Robotics.IK"
    assert child_logger.propagate is True


def test_logger_no_duplicate_handlers_on_reinit():
    """Verify repeated setup_logger calls do not create duplicate handlers."""
    logger1 = setup_logger(name="VisionRobotTwin", debug=True)
    initial_handler_count = len(logger1.handlers)

    logger2 = setup_logger(name="VisionRobotTwin", debug=True)
    assert len(logger2.handlers) == initial_handler_count


def test_logger_child_message_logged_to_file(tmp_path: Path):
    """Verify child module log messages propagate to the base logger's file handler."""
    log_file = tmp_path / "test_session.log"
    base_logger = setup_logger(name="VisionRobotTwin", log_file=log_file, debug=True)

    ik_logger = get_logger("Robotics.IK")
    test_msg = "Kinematic limit test event 12345"
    ik_logger.info(test_msg)

    # Flush handlers
    for h in base_logger.handlers:
        h.flush()

    assert log_file.exists()
    content = log_file.read_text(encoding="utf-8")
    assert test_msg in content
    assert "VisionRobotTwin.Robotics.IK" in content


def test_logger_reconfig_updates_debug_level():
    """Verify calling setup_logger(debug=True) updates root and handler levels to DEBUG."""
    # 1. Initialize INFO
    logger = setup_logger(name="VisionRobotTwin", debug=False)
    assert logger.level == logging.INFO

    # 2. Re-configure with debug=True
    logger = setup_logger(name="VisionRobotTwin", debug=True)
    assert logger.level == logging.DEBUG
    for h in logger.handlers:
        if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
            assert h.level == logging.DEBUG

    child = get_logger("Robotics.Test")
    assert child.isEnabledFor(logging.DEBUG)
