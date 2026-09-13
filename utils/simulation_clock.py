"""Physics Simulation Clock and Multi-Substep Accumulator.

Decouples camera perception frame rate (~30 FPS) from PyBullet physics stepping rate (240 Hz).
Advances physics by exact 1/240-second substeps using a high-precision wall-time accumulator,
preventing physics slowdown while preserving simulation determinism and stability.
"""

from typing import Callable, Optional, Tuple
import time


class SimulationClock:
    """Manages multi-substep physics scheduling using a fractional time accumulator."""

    def __init__(
        self,
        target_physics_hz: float = 240.0,
        simulation_time_step: Optional[float] = None,
        max_substeps_per_frame: int = 20,
        max_substeps_per_iteration: Optional[int] = None,
        max_accumulated_time_s: float = 0.20,
    ):
        """Initializes the simulation clock.

        Args:
            target_physics_hz: Fixed physics engine stepping frequency (Hz).
            simulation_time_step: Optional direct timestep in seconds (1/240s).
            max_substeps_per_frame: Safety limit preventing CPU stalls/spiral of death.
            max_substeps_per_iteration: Alias for max_substeps_per_frame.
            max_accumulated_time_s: Maximum allowable time buffer in accumulator.
        """
        if simulation_time_step is not None:
            self.physics_dt = float(simulation_time_step)
            self.target_physics_hz = 1.0 / self.physics_dt if self.physics_dt > 0 else float(target_physics_hz)
        else:
            self.physics_dt = 1.0 / float(target_physics_hz)
            self.target_physics_hz = float(target_physics_hz)

        self.max_substeps = int(max_substeps_per_iteration if max_substeps_per_iteration is not None else max_substeps_per_frame)
        self.max_accumulated_time_s = float(max_accumulated_time_s)

        self.accumulator: float = 0.0
        self.physics_step_count: int = 0
        self.total_physics_steps: int = 0
        self.last_time: Optional[float] = None
        self._step_window_start: float = time.perf_counter()
        self._step_window_count: int = 0
        self._actual_step_rate: float = self.target_physics_hz

    def step(
        self,
        wall_dt: Optional[float] = None,
        step_fn: Optional[Callable[[], None]] = None,
        physics_step_fn: Optional[Callable[[], None]] = None,
        current_time: Optional[float] = None,
        injected_dt: Optional[float] = None,
    ) -> int:
        """Advances physics by the required number of substeps.

        Args:
            wall_dt: Optional elapsed wall-clock time in seconds.
            step_fn: Optional step function callback executed each substep.
            physics_step_fn: Alias for step_fn.
            current_time: Optional wall-clock timestamp (seconds).
            injected_dt: Alias for wall_dt.

        Returns:
            Number of physics substeps executed.
        """
        fn = step_fn or physics_step_fn or (lambda: None)
        effective_dt = wall_dt if wall_dt is not None else injected_dt
        now = current_time if current_time is not None else time.perf_counter()

        if effective_dt is not None:
            dt = float(effective_dt)
        elif self.last_time is not None:
            dt = now - self.last_time
        else:
            dt = self.physics_dt  # Default first step

        self.last_time = now

        # Clamp dt to avoid spiral of death on long freezes/breakpoints
        dt = min(dt, self.max_accumulated_time_s)
        self.accumulator += dt

        substeps = 0
        while self.accumulator >= self.physics_dt and substeps < self.max_substeps:
            fn()
            self.accumulator -= self.physics_dt
            substeps += 1
            self.physics_step_count += 1
            self.total_physics_steps += 1
            self._step_window_count += 1

        # Prevent runaway accumulation if simulation cannot keep up
        if self.accumulator > self.max_accumulated_time_s:
            self.accumulator = 0.0

        # Calculate sliding-window physics steps per second
        elapsed = now - self._step_window_start
        if elapsed >= 0.5:
            self._actual_step_rate = self._step_window_count / elapsed
            self._step_window_start = now
            self._step_window_count = 0

        return substeps

    def reset(self) -> None:
        """Resets accumulator and timers."""
        self.accumulator = 0.0
        self.physics_step_count = 0
        self.total_physics_steps = 0
        self.last_time = None
        self._step_window_start = time.perf_counter()
        self._step_window_count = 0
        self._actual_step_rate = self.target_physics_hz
