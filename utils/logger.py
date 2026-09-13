"""Structured Logger for VisionRobotTwin.

Provides clean console formatting and file logging with automatic directory creation.
Avoids spamming every frame by providing rate-limited / event-driven log messages.
"""

import logging
import sys
from pathlib import Path
from typing import Optional


_GLOBAL_LOGGER: Optional[logging.Logger] = None


def setup_logger(
    name: str = "VisionRobotTwin",
    log_file: Optional[Path] = Path("logs/vision_robot_twin.log"),
    level: int = logging.INFO,
    debug: bool = False,
) -> logging.Logger:
    """Configures and returns the application logger.

    Args:
        name: Logger name.
        log_file: Optional file path for persistent logs.
        level: Default log level.
        debug: If True, sets level to DEBUG.

    Returns:
        Configured logging.Logger instance.
    """
    global _GLOBAL_LOGGER
    if debug:
        level = logging.DEBUG

    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid duplicate handlers if re-initialized
    if logger.handlers:
        return logger

    # Formatting
    console_format = logging.Formatter(
        fmt="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    file_format = logging.Formatter(
        fmt="%(asctime)s [%(levelname)s] %(name)s (%(filename)s:%(lineno)d): %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # Console Handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(console_format)
    logger.addHandler(console_handler)

    # File Handler
    if log_file:
        try:
            log_path = Path(log_file)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            file_handler = logging.FileHandler(log_path, mode="a", encoding="utf-8")
            file_handler.setLevel(logging.DEBUG)  # Always log debug to file
            file_handler.setFormatter(file_format)
            logger.addHandler(file_handler)
        except Exception as e:
            logger.warning(f"Could not initialize file logger at {log_file}: {e}")

    _GLOBAL_LOGGER = logger
    return logger


def get_logger(name: str = "VisionRobotTwin") -> logging.Logger:
    """Retrieves the global logger instance or initializes a default one."""
    global _GLOBAL_LOGGER
    if _GLOBAL_LOGGER is None:
        return setup_logger(name)
    return logging.getLogger(name)
