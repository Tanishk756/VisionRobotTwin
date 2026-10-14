"""Non-safety-rated collision supervision for simulation execution.

Checks sampled command paths and measured state. Finite sampling and velocity
lookahead do not establish continuous collision guarantees or braking safety.
"""
from dataclasses import dataclass
import math
import threading

import numpy as np

from robotics.backends.base import BackendError, BackendHealthStatus, RobotBackend
from robotics.planning import is_joint_path_collision_free


class CollisionSafetyError(BackendError):
    """Movement rejected by the latched execution collision guard."""


@dataclass(frozen=True)
class ExecutionCollisionFault:
    code: str
    detail: str


class CollisionGuardBackend(RobotBackend):
    """Supervise position segments and predicted velocity motion through I/O."""
    def __init__(self, backend, provider, model, velocity_horizon_s=0.1, resolution_rad=0.01):
        if any(not math.isfinite(v) or v <= 0 for v in (velocity_horizon_s, resolution_rad)):
            raise ValueError("Collision horizon and sampling resolution must be positive and finite")
        self.backend = backend
        self.provider = provider
        self.model = model
        self.velocity_horizon_s = velocity_horizon_s
        self.resolution_rad = resolution_rad
        self._fault = None
        self._lock = threading.RLock()
        self.accepted_commands = 0
        self.rejected_commands = 0

    @property
    def active_fault(self):
        with self._lock:
            return self._fault

    @property
    def transport_capabilities(self):
        return self.backend.transport_capabilities

    def connect(self):
        return self.backend.connect()

    def disconnect(self):
        self.backend.disconnect()

    def is_connected(self):
        return self.backend.is_connected()

    def health_status(self):
        return BackendHealthStatus.ERROR if self.active_fault is not None else self.backend.health_status()

    def get_joint_state(self):
        return self.backend.get_joint_state()

    def halt_motion(self):
        # Stop remains privileged even with a latched collision fault.
        try:
            self.backend.halt_motion()
        except Exception as err:
            self._fault = ExecutionCollisionFault("halt_failed", str(err))
            raise CollisionSafetyError(f"Collision software halt failed: {err}") from err

    def _latch(self, code, detail):
        if self._fault is None:
            self._fault = ExecutionCollisionFault(code, detail)
            self.halt_motion()

    def monitor(self):
        """Check measured clearance without automatically clearing a fault."""
        with self._lock:
            if self._fault is not None:
                return False
            try:
                state = self.backend.get_joint_state()
                if state.joint_names != self.model.arm_joint_names:
                    raise ValueError("Telemetry joints do not match the collision model")
                result = self.provider.check_collision(state.positions)
                if result.in_collision:
                    self._latch("measured_collision", result.details or "Measured collision/clearance violation")
                    return False
                return True
            except CollisionSafetyError:
                raise
            except Exception as err:
                self._latch("collision_query_failed", str(err))
                return False

    def reset_fault(self):
        """Explicit reset requires a connected, queryable, currently clear robot."""
        with self._lock:
            if not self.backend.is_connected():
                raise CollisionSafetyError("Cannot reset collision fault while disconnected")
            state = self.backend.get_joint_state()
            if state.joint_names != self.model.arm_joint_names or self.provider.check_collision(state.positions).in_collision:
                raise CollisionSafetyError("Cannot reset collision fault while model/state is unsafe")
            self.halt_motion()
            self._fault = None

    def _validate_path(self, targets, velocity=False):
        if self._fault is not None:
            self.rejected_commands += 1
            raise CollisionSafetyError(f"Collision fault is latched: {self._fault.detail}")
        try:
            target = np.asarray(targets, dtype=np.float64)
            if target.shape != (self.model.dof,) or not np.all(np.isfinite(target)):
                raise ValueError("Collision command must be finite and match model DoF")
            if not self.monitor():
                raise CollisionSafetyError(self._fault.detail)
            state = self.backend.get_joint_state()
            current = np.asarray(state.positions)
            candidate = current + target * self.velocity_horizon_s if velocity else target
            if np.any(candidate < self.model.arm_lower_limits) or np.any(candidate > self.model.arm_upper_limits):
                self._latch("joint_limit_prediction", "Candidate motion exceeds joint limits")
                raise CollisionSafetyError(self._fault.detail)
            clear, _ = is_joint_path_collision_free(current, candidate, self.provider, resolution_rad=self.resolution_rad)
            if not clear:
                self._latch("predicted_collision", "Command path violates collision or clearance policy")
                raise CollisionSafetyError(self._fault.detail)
        except Exception as err:
            self.rejected_commands += 1
            if self._fault is None:
                self._latch("collision_query_failed", str(err))
            if isinstance(err, CollisionSafetyError):
                raise
            raise CollisionSafetyError(str(err)) from err

    def command_joint_positions(self, targets):
        with self._lock:
            self._validate_path(targets)
            return self._dispatch(targets, False)

    def command_joint_velocities(self, targets, effort_limit=None):
        with self._lock:
            self._validate_path(targets, True)
            return self._dispatch(targets, True, effort_limit)

    def _dispatch(self, targets, velocity, effort_limit=None):
        try:
            result = (self.backend.command_joint_velocities(targets, effort_limit)
                      if velocity else self.backend.command_joint_positions(targets))
            if not result:
                raise BackendError("Execution transport rejected command")
        except Exception as err:
            self.rejected_commands += 1
            self._latch("dispatch_failed", str(err))
            raise CollisionSafetyError(str(err)) from err
        self.accepted_commands += 1
        return True
