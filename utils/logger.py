"""Structured Logger Hierarchy for VisionRobotTwin.

Configures the root 'VisionRobotTwin' logger with console formatting and file handlers.
All module loggers (e.g., 'VisionRobotTwin.Perception.Camera', 'VisionRobotTwin.Robotics.IK')
are created as descendants of 'VisionRobotTwin', guaranteeing proper propagation and eliminating
lost log messages or duplicate handlers.
"""

import logging
import sys
from pathlib import Path
from typing import Optional

ROOT_LOGGER_NAME = "VisionRobotTwin"


def setup_logger(
    name: str = ROOT_LOGGER_NAME,
    log_file: Optional[Path] = Path("logs/vision_robot_twin.log"),
    level: int = logging.INFO,
    debug: bool = False,
) -> logging.Logger:
    """Configures and returns the root application logger.

    Args:
        name: Root logger name (defaults to 'VisionRobotTwin').
        log_file: Optional file path for persistent logs.
        level: Default log level.
        debug: If True, sets level to DEBUG.

    Returns:
        Configured logging.Logger instance.
    """
    if debug:
        level = logging.DEBUG

    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Formatting
    console_format = logging.Formatter(
        fmt="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    file_format = logging.Formatter(
        fmt="%(asctime)s [%(levelname)s] %(name)s (%(filename)s:%(lineno)d): %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # Ensure handlers
    has_console = False
    for h in logger.handlers:
        if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
            has_console = True
            h.setLevel(level)

    if not has_console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(console_format)
        logger.addHandler(console_handler)

    # File Handler
    if log_file:
        log_path = Path(log_file)
        already_has_file = any(
            isinstance(h, logging.FileHandler) and Path(getattr(h, "baseFilename", "")).resolve() == log_path.resolve()
            for h in logger.handlers
        )
        if not already_has_file:
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                file_handler = logging.FileHandler(log_path, mode="a", encoding="utf-8")
                file_handler.setLevel(logging.DEBUG)  # Always capture debug in logfile
                file_handler.setFormatter(file_format)
                logger.addHandler(file_handler)
            except Exception as e:
                logger.warning(f"Could not initialize file logger at {log_file}: {e}")

    return logger


def get_logger(module_name: str = "") -> logging.Logger:
    """Retrieves a descendant logger under the 'VisionRobotTwin' namespace.

    Example:
        get_logger("Robotics.IK") -> logging.getLogger("VisionRobotTwin.Robotics.IK")
    """
    # Ensure root is initialized with default config if not already setup
    root_logger = logging.getLogger(ROOT_LOGGER_NAME)
    if not root_logger.handlers:
        setup_logger(ROOT_LOGGER_NAME)

    if not module_name or module_name == ROOT_LOGGER_NAME:
        return root_logger

    if module_name.startswith(f"{ROOT_LOGGER_NAME}."):
        full_name = module_name
    else:
        full_name = f"{ROOT_LOGGER_NAME}.{module_name}"

    return logging.getLogger(full_name)
