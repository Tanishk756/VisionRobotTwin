"""Baseline constant-velocity Kalman estimator; not yet empirically calibrated."""
from dataclasses import dataclass
import numpy as np

from robotics.research.contracts import nonnegative


@dataclass(frozen=True)
class ObstacleEstimate:
    time_s: float
    observation_age_s: float
    position_m: tuple
    velocity_m_s: tuple
    covariance: tuple


class ConstantVelocityEstimator:
    """Filter measurements at capture time, then predict without mutating state.

    White acceleration spectral density is a model choice, not a bound. No
    collision-probability guarantee follows from this Gaussian covariance.
    """
    def __init__(self, measurement_std_m=0.003, acceleration_spectral_density=0.1,
                 initial_velocity_std_m_s=0.5):
        nonnegative("measurement_std_m", measurement_std_m, positive=True)
        nonnegative("acceleration_spectral_density", acceleration_spectral_density)
        nonnegative("initial_velocity_std_m_s", initial_velocity_std_m_s, positive=True)
        self._r = np.eye(3) * measurement_std_m ** 2
        self._spectral = acceleration_spectral_density
        self._velocity_variance = initial_velocity_std_m_s ** 2
        self._state = self._covariance = self._last = None

    def _transition(self, duration):
        identity = np.eye(3)
        transition = np.eye(6)
        transition[:3, 3:] = duration * identity
        process = self._spectral * np.block([
            [identity * duration ** 3 / 3, identity * duration ** 2 / 2],
            [identity * duration ** 2 / 2, identity * duration]])
        return transition, process

    def update(self, observation):
        if self._last is not None and (observation.sequence <= self._last.sequence
                                      or observation.capture_time_s <= self._last.capture_time_s
                                      or observation.delivery_time_s < self._last.delivery_time_s):
            return False
        measurement = np.array(observation.obstacle_position_m)
        if self._state is None:
            self._state = np.concatenate([measurement, np.zeros(3)])
            self._covariance = np.zeros((6, 6))
            self._covariance[:3, :3] = self._r
            self._covariance[3:, 3:] = np.eye(3) * self._velocity_variance
        else:
            transition, process = self._transition(observation.capture_time_s - self._last.capture_time_s)
            predicted = transition @ self._state
            covariance = transition @ self._covariance @ transition.T + process
            innovation_covariance = covariance[:3, :3] + self._r
            gain = np.linalg.solve(innovation_covariance, covariance[:3, :]).T
            self._state = predicted + gain @ (measurement - predicted[:3])
            correction = np.eye(6)
            correction[:, :3] -= gain
            self._covariance = correction @ covariance @ correction.T + gain @ self._r @ gain.T
        self._last = observation
        return True

    def predict(self, now_s):
        nonnegative("now_s", now_s)
        if self._last is None:
            return None
        if now_s < self._last.delivery_time_s:
            raise ValueError("Prediction requested before the observation was available")
        age = now_s - self._last.capture_time_s
        transition, process = self._transition(age)
        mean = transition @ self._state
        covariance = transition @ self._covariance @ transition.T + process
        return ObstacleEstimate(now_s, age, tuple(mean[:3]), tuple(mean[3:]),
                                tuple(tuple(row) for row in covariance))
