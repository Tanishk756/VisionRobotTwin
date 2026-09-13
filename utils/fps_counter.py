"""Real-time FPS calculation utility with sliding-window exponential averaging."""

import time
from collections import deque
from typing import Optional


class FPSCounter:
    """Accurately calculates instantaneous and smoothed frames per second (FPS)."""

    def __init__(self, window_size: int = 30, smoothing: float = 0.9):
        """Initializes the FPS counter.

        Args:
            window_size: Number of frame intervals to track for windowed average.
            smoothing: Exponential smoothing factor (0.0 to 1.0).
        """
        self._window_size = window_size
        self._smoothing = smoothing
        self._timestamps: deque = deque(maxlen=window_size)
        self._last_time: Optional[float] = None
        self._smoothed_fps: float = 0.0
        self._frame_count: int = 0
        self._start_time: float = time.perf_counter()

    def update(self) -> float:
        """Records a new frame and returns the current smoothed FPS."""
        now = time.perf_counter()
        self._frame_count += 1
        self._timestamps.append(now)

        if self._last_time is not None:
            dt = now - self._last_time
            if dt > 0.0:
                instantaneous_fps = 1.0 / dt
                if self._smoothed_fps == 0.0:
                    self._smoothed_fps = instantaneous_fps
                else:
                    self._smoothed_fps = (
                        self._smoothing * self._smoothed_fps
                        + (1.0 - self._smoothing) * instantaneous_fps
                    )

        self._last_time = now
        return self._smoothed_fps

    @property
    def fps(self) -> float:
        """Returns the current smoothed FPS value."""
        return self._smoothed_fps

    @property
    def average_fps(self) -> float:
        """Calculates total average FPS since initialization."""
        elapsed = time.perf_counter() - self._start_time
        if elapsed > 0 and self._frame_count > 0:
            return self._frame_count / elapsed
        return 0.0

    def reset(self) -> None:
        """Resets the counter state."""
        self._timestamps.clear()
        self._last_time = None
        self._smoothed_fps = 0.0
        self._frame_count = 0
        self._start_time = time.perf_counter()
