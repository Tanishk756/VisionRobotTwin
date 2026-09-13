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
