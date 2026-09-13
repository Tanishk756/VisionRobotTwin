"""Pose Filtering and Smoothing Algorithms.

Provides:
- ExponentialMovingAverageFilter (EMA): Simple, fast, low-overhead recursive filter.
- OneEuroFilter: Adaptive 1 Euro filter (Casiez et al., CHI 2012) designed to eliminate
  jitter at low speeds while maintaining low latency during fast motions.
- PoseFilter: Unified 6-DoF filter handling Cartesian translations and orientation smoothing.
"""

import math
import numpy as np
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation, Slerp


class ExponentialMovingAverageFilter:
    """Exponential Moving Average (EMA) filter for N-dimensional vectors."""

    def __init__(self, alpha: float = 0.35, initial_value: Optional[np.ndarray] = None):
        """Initializes the EMA filter.

        Args:
            alpha: Weight for new observation (0 < alpha <= 1.0).
                   Higher = more responsive / less smooth. Lower = smoother / more lag.
            initial_value: Optional starting state vector.
        """
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"Alpha must be in (0.0, 1.0], got {alpha}")
        self._alpha = float(alpha)
        self._state: Optional[np.ndarray] = (
            np.array(initial_value, dtype=np.float64) if initial_value is not None else None
        )

    @property
    def alpha(self) -> float:
        return self._alpha

    @alpha.setter
    def alpha(self, val: float) -> None:
        if not (0.0 < val <= 1.0):
            raise ValueError(f"Alpha must be in (0.0, 1.0], got {val}")
        self._alpha = float(val)

    def update(self, measurement: np.ndarray) -> np.ndarray:
        """Filters a new measurement vector."""
        measurement = np.asarray(measurement, dtype=np.float64)
        if self._state is None:
            self._state = measurement.copy()
        else:
            self._state = self._alpha * measurement + (1.0 - self._alpha) * self._state
        return self._state.copy()

    @property
    def value(self) -> Optional[np.ndarray]:
        return self._state.copy() if self._state is not None else None

    def reset(self) -> None:
        self._state = None


class LowPassFilter:
    """Helper low-pass filter used in OneEuroFilter."""

    def __init__(self, alpha: float = 1.0):
        self._alpha = alpha
        self._y: Optional[np.ndarray] = None

    def filter(self, x: np.ndarray, alpha: Optional[float] = None) -> np.ndarray:
        if alpha is not None:
            self._alpha = alpha
        if self._y is None:
            self._y = np.array(x, dtype=np.float64)
        else:
            self._y = self._alpha * np.asarray(x, dtype=np.float64) + (1.0 - self._alpha) * self._y
        return self._y.copy()

    def last_value(self) -> Optional[np.ndarray]:
        return self._y.copy() if self._y is not None else None

    def reset(self) -> None:
        self._y = None


class OneEuroFilter:
    """1 Euro Filter for adaptive noise suppression and low latency tracking.

    Reference:
    Gery Casiez, Nicolas Roussel, Daniel Vogel.
    1 € Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems.
    ACM Conference on Human Factors in Computing Systems (CHI), May 2012, Austin, TX, USA.
    """

    def __init__(
        self,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0,
        freq: float = 30.0,
    ):
        """Initializes the 1 Euro filter.

        Args:
            min_cutoff: Minimum cutoff frequency (Hz). Lower = more smoothing at standstill.
            beta: Speed coefficient. Higher = less lag during rapid movement.
            d_cutoff: Cutoff frequency for derivative calculation (Hz).
            freq: Sampling frequency (Hz).
        """
        self._min_cutoff = float(min_cutoff)
        self._beta = float(beta)
        self._d_cutoff = float(d_cutoff)
        self._freq = float(freq)

        self._x_filter = LowPassFilter()
        self._dx_filter = LowPassFilter()
        self._last_time: Optional[float] = None

    def _alpha(self, cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def update(self, x: np.ndarray, timestamp: Optional[float] = None) -> np.ndarray:
        """Filters a signal vector using time-based or frequency-based rate."""
        x = np.asarray(x, dtype=np.float64)

        if self._last_time is not None and timestamp is not None:
            dt = timestamp - self._last_time
            if dt <= 1e-5:
                dt = 1.0 / self._freq
        else:
            dt = 1.0 / self._freq

        self._last_time = timestamp

        # Compute instantaneous derivative (speed)
        prev_x = self._x_filter.last_value()
        if prev_x is None:
            dx = np.zeros_like(x)
        else:
            dx = (x - prev_x) / dt

        # Filter derivative
        a_d = self._alpha(self._d_cutoff, dt)
        filtered_dx = self._dx_filter.filter(dx, a_d)

        # Dynamic cutoff based on speed
        speed = np.linalg.norm(filtered_dx)
        cutoff = self._min_cutoff + self._beta * speed

        # Filter signal with adaptive cutoff
        a = self._alpha(cutoff, dt)
        return self._x_filter.filter(x, a)

    def reset(self) -> None:
        self._x_filter.reset()
        self._dx_filter.reset()
        self._last_time = None


class PoseFilter:
    """Unified 6-DoF Filter managing 3D Cartesian position and orientation quaternion."""

    def __init__(
        self,
        filter_type: str = "ema",
        ema_alpha_pos: float = 0.35,
        ema_alpha_rot: float = 0.25,
        one_euro_min_cutoff: float = 1.0,
        one_euro_beta: float = 0.007,
    ):
        self._filter_type = filter_type.lower()
        self._ema_alpha_rot = ema_alpha_rot

        if self._filter_type == "one_euro":
            self._pos_filter = OneEuroFilter(
                min_cutoff=one_euro_min_cutoff,
                beta=one_euro_beta,
            )
        else:
            self._pos_filter = ExponentialMovingAverageFilter(alpha=ema_alpha_pos)

        self._filtered_quat: Optional[np.ndarray] = None

    def update_position(self, pos: np.ndarray, timestamp: Optional[float] = None) -> np.ndarray:
        """Filters Cartesian translation."""
        if isinstance(self._pos_filter, OneEuroFilter):
            return self._pos_filter.update(pos, timestamp)
        return self._pos_filter.update(pos)

    def update_orientation_quaternion(self, quat_xyzw: np.ndarray) -> np.ndarray:
        """Smooths orientation using Spherical Linear Interpolation (SLERP)."""
        quat = np.asarray(quat_xyzw, dtype=np.float64)
        norm = np.linalg.norm(quat)
        if norm > 1e-6:
            quat = quat / norm

        if self._filtered_quat is None:
            self._filtered_quat = quat.copy()
            return self._filtered_quat.copy()

        # Ensure shortest path on quaternion hypersphere
        if np.dot(quat, self._filtered_quat) < 0.0:
            quat = -quat

        try:
            key_rots = Rotation.from_quat([self._filtered_quat, quat])
            slerp = Slerp([0.0, 1.0], key_rots)
            interpolated = slerp([self._ema_alpha_rot])
            self._filtered_quat = interpolated.as_quat()[0]
        except Exception:
            # Fallback to normalized linear blend if SLERP numerical edge occurs
            blended = (1.0 - self._ema_alpha_rot) * self._filtered_quat + self._ema_alpha_rot * quat
            self._filtered_quat = blended / (np.linalg.norm(blended) + 1e-8)

        return self._filtered_quat.copy()

    def update(
        self,
        position: np.ndarray,
        quaternion_xyzw: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """Filters full 6-DoF pose."""
        filt_pos = self.update_position(position, timestamp)
        filt_quat = (
            self.update_orientation_quaternion(quaternion_xyzw)
            if quaternion_xyzw is not None
            else None
        )
        return filt_pos, filt_quat

    def reset(self) -> None:
        self._pos_filter.reset()
        self._filtered_quat = None
