"""Unit tests for SimulationClock multi-substep accumulator and physics scheduling."""

import pytest
from utils.simulation_clock import SimulationClock


def test_simulation_clock_substeps_calculation():
    """Verify that given 1/30s frame delta and 1/240s physics timestep, ~8 substeps are executed."""
    clock = SimulationClock(
        target_physics_hz=240.0,
        simulation_time_step=1.0 / 240.0,
        max_substeps_per_iteration=20,
    )

    steps_called = 0

    def dummy_step():
        nonlocal steps_called
        steps_called += 1

    # wall_dt = 1/30s = 0.033333s
    # 0.033333 / (1/240) = 8.0 substeps
    substeps = clock.step(wall_dt=1.0 / 30.0, step_fn=dummy_step)
    assert substeps == 8
    assert steps_called == 8
    assert clock.physics_step_count == 8


def test_simulation_clock_accumulator_carries_remainder():
    """Verify that fractional remainder is accumulated and discharged in subsequent calls."""
    clock = SimulationClock(
        target_physics_hz=240.0,
        simulation_time_step=1.0 / 240.0,
        max_substeps_per_iteration=20,
    )

    # Injected dt = 0.005s (slightly more than 1.2 substeps: 0.005 * 240 = 1.2)
    s1 = clock.step(wall_dt=0.005)
    assert s1 == 1
    # Remainder should be approx 0.005 - 1/240 = 0.0008333s
    assert clock.accumulator > 0.0008

    # Second step of 0.005s -> Total accumulated = 0.005 + 0.0008333 = 0.0058333s (1.4 substeps)
    s2 = clock.step(wall_dt=0.005)
    assert s2 == 1

    # Third step of 0.005s -> Total accumulated = 0.005 + 0.001666 = 0.006666s (1.6 substeps)
    s3 = clock.step(wall_dt=0.005)
    assert s3 == 1

    # Fourth step of 0.005s -> Total accumulated = 0.005 + 0.0025 = 0.0075s (1.8 substeps)
    s4 = clock.step(wall_dt=0.005)
    assert s4 == 1

    # Fifth step of 0.005s -> Total accumulated = 0.005 + 0.00333 = 0.00833s = 2.0 substeps!
    s5 = clock.step(wall_dt=0.005)
    assert s5 == 2


def test_simulation_clock_max_substeps_bound():
    """Verify that huge stalls (e.g. 5 seconds) are capped to prevent spiral of death."""
    clock = SimulationClock(
        target_physics_hz=240.0,
        simulation_time_step=1.0 / 240.0,
        max_substeps_per_iteration=15,
    )

    substeps = clock.step(wall_dt=5.0)
    assert substeps == 15
    assert clock.physics_step_count == 15
