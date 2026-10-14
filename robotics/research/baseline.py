"""Engineering fixed-horizon stop comparator, not a novel safety algorithm."""
from dataclasses import dataclass

from robotics.research.contracts import nonnegative, vector


@dataclass(frozen=True)
class BaselineDecision:
    velocities_rad_s: tuple
    state: str
    reason: str
    intervened: bool
    clearance: object = None


class FixedHorizonStopBaseline:
    def __init__(self, horizon_s=0.1, step_s=0.02, margin_m=0.02,
                 recovery_margin_m=0.005, recovery_observations=3,
                 minimum_hold_s=0.1, telemetry_timeout_s=0.3):
        for name, value in (("horizon", horizon_s), ("step", step_s), ("telemetry timeout", telemetry_timeout_s)):
            nonnegative(name, value, positive=True)
        for name, value in (("margin", margin_m), ("recovery margin", recovery_margin_m), ("hold", minimum_hold_s)):
            nonnegative(name, value)
        if margin_m + recovery_margin_m > 0.5 or type(recovery_observations) is not int or recovery_observations < 1:
            raise ValueError("Invalid recovery settings")
        self.horizon_s, self.step_s, self.margin_m = horizon_s, step_s, margin_m
        self.recovery_margin_m, self.recovery_observations = recovery_margin_m, recovery_observations
        self.minimum_hold_s, self.telemetry_timeout_s = minimum_hold_s, telemetry_timeout_s
        self.reset()

    def reset(self):
        """Explicit policy reset; does not reset the execution supervisor."""
        self.state = "WAIT"
        self._held_since = None
        self._clear_count = 0
        self._last_clear_sequence = None

    def select(self, view, nominal, query):
        nominal = vector("nominal velocities", nominal, view.dof)
        zero = (0.0,) * view.dof
        if self.state == "FAULT" or view.execution_fault is not None:
            self.state = "FAULT"
            return BaselineDecision(zero, self.state, view.execution_fault or "policy_fault_latched", True)
        if view.observation is None or view.time_s - view.observation.capture_time_s >= self.telemetry_timeout_s:
            if self.state != "WAIT":
                if self.state != "HOLD":
                    self._held_since = view.time_s
                self.state = "HOLD"
            self._clear_count = 0
            self._last_clear_sequence = None
            return BaselineDecision(zero, self.state, "missing_or_stale_observation", True)
        margin = self.margin_m + (self.recovery_margin_m if self.state == "HOLD" else 0)
        try:
            clearance = query(nominal, horizon_s=self.horizon_s, step_s=self.step_s,
                              environment_margin_m=margin)
        except Exception as err:
            self.state = "FAULT"
            return BaselineDecision(zero, self.state, f"query_failed: {err}", True)
        if not clearance.clear:
            if self.state != "HOLD":
                self._held_since = view.time_s
            self.state = "HOLD"
            self._clear_count = 0
            self._last_clear_sequence = None
            return BaselineDecision(zero, self.state, "predicted_clearance_or_limit_violation", True, clearance)
        if self.state == "HOLD":
            if view.observation.sequence != self._last_clear_sequence:
                self._clear_count += 1
                self._last_clear_sequence = view.observation.sequence
            if self._clear_count < self.recovery_observations or view.time_s - self._held_since < self.minimum_hold_s:
                return BaselineDecision(zero, self.state, "awaiting_clear_observations", True, clearance)
        self.state = "TRACK"
        return BaselineDecision(nominal, self.state, "sampled_path_clear", False, clearance)
