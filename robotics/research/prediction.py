"""Nominal piecewise-constant velocity prediction from controller information.

This is kinematic extrapolation, not a dynamic rollout or safety certificate.
Actual transport realizations and plant state must never enter this module.
"""
from dataclasses import dataclass
import math
import numpy as np

from robotics.research.contracts import VelocityCommand, nonnegative


@dataclass(frozen=True)
class CommandIntent:
    command: VelocityCommand
    nominal_application_s: float
    earliest_application_s: float
    latest_application_s: float
    watchdog_deadline_s: float | None = None
    first_delivery_deadline_s: float | None = None

    def __post_init__(self):
        for name in ("nominal_application_s", "earliest_application_s", "latest_application_s"):
            nonnegative(name, getattr(self, name))
        if not self.command.created_time_s <= self.earliest_application_s <= self.nominal_application_s <= self.latest_application_s:
            raise ValueError("Command arrival bounds must follow creation and contain nominal arrival")
        if self.watchdog_deadline_s is not None:
            nonnegative("watchdog_deadline_s", self.watchdog_deadline_s)
            if self.watchdog_deadline_s <= self.nominal_application_s:
                raise ValueError("Watchdog deadline must follow nominal application")
        if self.first_delivery_deadline_s is not None:
            nonnegative("first_delivery_deadline_s", self.first_delivery_deadline_s)
            if self.first_delivery_deadline_s <= self.command.created_time_s:
                raise ValueError("First delivery deadline must follow creation")


@dataclass(frozen=True)
class PredictedState:
    time_s: float
    joint_positions: tuple
    joint_limit_violation: bool
    end_effector_position_m: tuple = ()
    end_effector_orientation_xyzw: tuple = ()


def predict_joint_motion(observation, intents, now_s, horizon_s, step_s,
                         lower_limits, upper_limits, stopped_at_s=None):
    nonnegative("now_s", now_s)
    nonnegative("horizon_s", horizon_s)
    nonnegative("step_s", step_s, positive=True)
    if observation is None:
        raise ValueError("Prediction requires a delivered observation")
    if now_s < observation.delivery_time_s:
        raise ValueError("Cannot predict using an observation not yet delivered")
    if math.ceil(horizon_s / step_s) > 10000:
        raise ValueError("Prediction sample budget exceeded")
    q = np.array(observation.joint_positions, dtype=float)
    lower, upper = np.array(lower_limits), np.array(upper_limits)
    if q.shape != lower.shape or lower.shape != upper.shape:
        raise ValueError("Prediction limits must match observed DoF")
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower >= upper):
        raise ValueError("Prediction limits must be finite and ordered")
    intents = tuple(intents)
    for intent in intents:
        if intent.command.created_time_s > now_s or len(intent.command.velocities_rad_s) != len(q):
            raise ValueError("Prediction history contains future or mismatched commands")
    start, end = observation.capture_time_s, now_s + horizon_s
    samples = [now_s]
    count = math.ceil(horizon_s / step_s)
    samples.extend(min(now_s + index * step_s, end) for index in range(1, count + 1))
    boundaries = {start, *samples}
    for intent in intents:
        boundaries.add(max(start, min(end, intent.nominal_application_s)))
        if intent.command.valid_until_s is not None:
            boundaries.add(max(start, min(end, intent.command.valid_until_s)))
        if intent.watchdog_deadline_s is not None:
            boundaries.add(max(start, min(end, intent.watchdog_deadline_s)))
        if intent.first_delivery_deadline_s is not None:
            boundaries.add(max(start, min(end, intent.first_delivery_deadline_s)))
    if stopped_at_s is not None:
        nonnegative("stopped_at_s", stopped_at_s)
        boundaries.add(max(start, min(end, stopped_at_s)))
    previous = start
    result = []
    sample_set = set(samples)
    forecast_latched = False
    for time_s in sorted(boundaries):
        eligible = [intent for intent in intents if intent.nominal_application_s <= previous
                    and (intent.command.valid_until_s is None or intent.nominal_application_s < intent.command.valid_until_s)]
        active = max(eligible, key=lambda intent: intent.command.sequence, default=None)
        # Forecast a latched stop rather than allowing a late intent to restart
        # motion after a nominal expiry gap. A visible supervisor stop/reset
        # marker overrides that forecast; it never requests nonzero motion.
        if active is not None and active.command.valid_until_s is None:
            forecast_latched = False
        elif active is not None:
            if (active.command.valid_until_s is not None and previous >= active.command.valid_until_s
                    or active.watchdog_deadline_s is not None and previous >= active.watchdog_deadline_s):
                forecast_latched = True
        for intent in intents:
            if (intent.first_delivery_deadline_s is not None and previous >= intent.first_delivery_deadline_s
                    and intent.nominal_application_s > intent.first_delivery_deadline_s):
                if active is None or active.command.sequence <= intent.command.sequence:
                    forecast_latched = True
        velocity = np.zeros_like(q)
        if active is not None and (active.command.valid_until_s is None or previous < active.command.valid_until_s):
            velocity = np.array(active.command.velocities_rad_s)
            if active.watchdog_deadline_s is not None and previous >= active.watchdog_deadline_s:
                velocity = np.zeros_like(q)
        if stopped_at_s is not None and previous >= stopped_at_s:
            velocity = np.zeros_like(q)
        if forecast_latched:
            velocity = np.zeros_like(q)
        q += (time_s - previous) * velocity
        previous = time_s
        if time_s in sample_set:
            result.append(PredictedState(time_s, tuple(q), bool(np.any(q < lower) or np.any(q > upper))))
    return tuple(result)
