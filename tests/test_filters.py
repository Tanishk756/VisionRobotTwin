"""Unit tests for pose filters (EMA, 1 Euro, and 6-DoF PoseFilter)."""

import numpy as np
import pytest

from utils.filters import (
    ExponentialMovingAverageFilter,
    OneEuroFilter,
    PoseFilter,
)


def test_ema_filter_convergence():
    """Verify EMA filter smoothly converges toward a step input."""
    filter_ema = ExponentialMovingAverageFilter(alpha=0.5)
    step_val = np.array([10.0, 20.0, 30.0])

    val = filter_ema.update(step_val)
    # First update should equal measurement
    assert np.allclose(val, step_val)

    # Subsequent updates with zero input should decay exponentially
    zero_val = np.array([0.0, 0.0, 0.0])
    for _ in range(10):
        val = filter_ema.update(zero_val)

    assert np.all(val < 0.1)


def test_ema_filter_noise_reduction():
    """Verify EMA filter suppresses high-frequency noise variance."""
    np.random.seed(42)
    true_val = np.array([1.0, 2.0, 3.0])
    noise = np.random.normal(0, 0.5, (100, 3))
    noisy_signal = true_val + noise

    filter_ema = ExponentialMovingAverageFilter(alpha=0.2)
    filtered_outputs = [filter_ema.update(sample) for sample in noisy_signal]

    raw_var = np.var(noisy_signal[20:], axis=0)
    filt_var = np.var(filtered_outputs[20:], axis=0)

    # Filtered signal variance should be significantly lower
    assert np.all(filt_var < raw_var * 0.4)


def test_one_euro_filter():
    """Verify 1 Euro filter maintains low jitter at standstill and responsive tracking on steps."""
    filter_1euro = OneEuroFilter(min_cutoff=1.0, beta=0.007, freq=30.0)

    # Static signal with jitter
    static_val = np.array([0.5, 0.5, 0.5])
    jitter = np.random.normal(0, 0.02, (50, 3))
    for sample in static_val + jitter:
        out = filter_1euro.update(sample)

    assert np.allclose(out, static_val, atol=0.05)


def test_pose_filter_6dof():
    """Verify 6-DoF PoseFilter smooths both translation and quaternion orientation."""
    pose_filter = PoseFilter(filter_type="ema", ema_alpha_pos=0.3, ema_alpha_rot=0.3)

    pos1 = np.array([0.1, 0.2, 0.5])
    quat1 = np.array([0.0, 0.0, 0.0, 1.0])

    filt_pos, filt_quat = pose_filter.update(pos1, quat1)
    assert np.allclose(filt_pos, pos1)
    assert np.allclose(filt_quat, quat1)

    # Perturbed step
    pos2 = np.array([0.2, 0.4, 0.6])
    quat2 = np.array([0.0, 0.7071, 0.0, 0.7071])

    filt_pos2, filt_quat2 = pose_filter.update(pos2, quat2)
    # Filtered translation should be between pos1 and pos2
    assert filt_pos2[0] < pos2[0]
    assert filt_pos2[0] > pos1[0]
    assert np.isclose(np.linalg.norm(filt_quat2), 1.0, atol=1e-5)
