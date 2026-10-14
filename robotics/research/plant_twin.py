"""Two-client, simulation-only plant/twin harness.

Controllers should receive controller_view() and submit_velocity(), never the
harness or evaluator. Private attributes are an API convention, not a sandbox.
The twin is an observation-synchronized kinematic model in this first version,
not an autonomous dynamic predictor. No novel safety algorithm is implemented.
"""
from dataclasses import dataclass, replace
import math

import numpy as np
import pybullet as p

from config.settings import AppConfig
from robotics.research.contracts import Observation, ScenarioConfig, TrialRecord, VelocityCommand
from robotics.research.transport import EventQueue
from robotics.simulator import PyBulletSimulator
from robotics.robot_registry import get_robot_registry
from robotics.research.prediction import CommandIntent, predict_joint_motion


@dataclass(frozen=True)
class ControllerView:
    time_s: float
    observation: Observation | None
    dof: int
    joint_velocity_limits_rad_s: tuple
    command_history: tuple = ()
    execution_fault: str | None = None


class PlantTwinHarness:
    def __init__(self, config: ScenarioConfig):
        get_robot_registry().get_robot_spec(config.robot)
        self.config = config
        self._plant = self._twin = None
        self._closed = False
        self._tick = 0
        self._observation_sequence = 0
        self._command_sequence = 0
        self._applied_sequence = -1
        self._latest = None
        self._active_command = None
        self._last_application_s = None
        self._first_submission_s = None
        self._received_sequence = -1
        self._fault = None
        self._history = []
        self._observations = EventQueue()
        self._commands = EventQueue()
        # Transport streams cannot consume/change the sensing-noise stream.
        streams = np.random.SeedSequence(config.seed).spawn(3)
        self._sensing_rng, self._observation_rng, self._command_rng = [np.random.default_rng(s) for s in streams]
        self.events = []
        try:
            self._plant = self._make_world(config.plant_velocity_gain)
            self._twin = self._make_world(1.0)
            for link in range(-1, p.getNumJoints(self._plant.robot_id, physicsClientId=self._plant.client_id)):
                mass = p.getDynamicsInfo(self._plant.robot_id, link, physicsClientId=self._plant.client_id)[0]
                if mass > 0:
                    p.changeDynamics(self._plant.robot_id, link, mass=mass * config.plant_mass_scale,
                                     physicsClientId=self._plant.client_id)
            self._plant_obstacle = self._make_obstacle(self._plant)
            self._twin_obstacle = self._make_obstacle(self._twin)
        except Exception:
            self.close()
            raise

    def _make_world(self, velocity_gain):
        app = AppConfig(robot_name=self.config.robot, execution_collision_checking=False)
        app.simulation.gui = False
        app.simulation.time_step = 1.0 / self.config.physics_hz
        app.robot.velocity_gain = velocity_gain
        return PyBulletSimulator(app, headless=True)

    def _make_obstacle(self, world):
        shape = p.createCollisionShape(p.GEOM_SPHERE, radius=self.config.obstacle_radius_m,
                                       physicsClientId=world.client_id)
        body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                 basePosition=self.config.obstacle_origin_m,
                                 physicsClientId=world.client_id)
        world.collision_provider.obstacle_ids.append(body)
        return body

    @property
    def time_s(self):
        return self._tick / self.config.physics_hz

    def _require_open(self):
        if self._closed:
            raise RuntimeError("Plant/twin harness is closed")

    def controller_view(self):
        self._require_open()
        return ControllerView(self.time_s, self._latest, self._twin.resolved_model.dof,
                              self._twin.resolved_model.arm_max_velocities,
                              tuple(self._history), self._fault)

    def _fresh_telemetry(self):
        return self._latest is not None and self.time_s - self._latest.capture_time_s < self.config.telemetry_watchdog_s

    def _latch_stop(self, reason):
        if self._fault is not None:
            return
        self._fault = reason
        cancelled = self._commands.clear()
        self._active_command = None
        # The supervisor's stop request is controller-visible information;
        # it is not confirmation that the physical plant has stopped.
        stop = VelocityCommand(self._command_sequence, self.time_s,
                               (0.0,) * self._twin.resolved_model.dof)
        self._command_sequence += 1
        self._received_sequence = stop.sequence
        self._history.append(CommandIntent(stop, self.time_s, self.time_s, self.time_s))
        try:
            self._plant.controller.backend.halt_motion()
        except Exception as err:
            self._fault = "halt_failed"
            self.events.append({"kind": "execution_fault", "reason": self._fault,
                                "time_s": self.time_s, "cancelled_commands": cancelled})
            raise RuntimeError("Research plant software halt failed") from err
        self.events.append({"kind": "execution_fault", "reason": reason,
                            "time_s": self.time_s, "cancelled_commands": cancelled})

    def reset_execution_fault(self):
        """Explicit reset requires fresh telemetry; canceled commands stay canceled."""
        self._require_open()
        if not self._fresh_telemetry():
            raise RuntimeError("Cannot reset without fresh delivered telemetry")
        try:
            self._plant.controller.backend.halt_motion()
        except Exception as err:
            self._fault = "halt_failed"
            self._commands.clear()
            self._active_command = None
            self.events.append({"kind": "execution_fault", "reason": "halt_failed", "time_s": self.time_s})
            raise RuntimeError("Research reset software halt failed") from err
        cancelled = self._commands.clear()
        self._active_command = None
        self._last_application_s = None
        self._first_submission_s = None
        stop = VelocityCommand(self._command_sequence, self.time_s,
                               (0.0,) * self._twin.resolved_model.dof)
        self._command_sequence += 1
        self._received_sequence = stop.sequence
        self._history.append(CommandIntent(stop, self.time_s, self.time_s, self.time_s))
        self._fault = None
        self.events.append({"kind": "execution_reset", "time_s": self.time_s,
                            "cancelled_commands": cancelled})

    def predict(self, horizon_s=0.1, step_s=0.02):
        """Nominal arm prediction/FK using only controller-visible information.

        Arrival times use configured delay bounds, not sampled transport outcomes.
        No plant query occurs. FK candidates preserve the observed twin state.
        """
        self._require_open()
        if self._fault == "halt_failed":
            raise RuntimeError("Prediction unavailable after an unconfirmed halt")
        model = self._twin.resolved_model
        states = predict_joint_motion(self._latest, tuple(self._history), self.time_s,
                                      horizon_s, step_s, model.arm_lower_limits, model.arm_upper_limits)
        result = []
        for state in states:
            position, orientation = self._twin.kinematics_provider.compute_fk(state.joint_positions)
            result.append(replace(state, end_effector_position_m=tuple(position),
                                  end_effector_orientation_xyzw=tuple(orientation)))
        return tuple(result)

    def submit_velocity(self, values):
        self._require_open()
        if self._fault is not None:
            raise RuntimeError(f"Execution fault is latched: {self._fault}")
        command = VelocityCommand(self._command_sequence, self.time_s, values,
                                  self.time_s + self.config.command_lifetime_s)
        limits = self._twin.resolved_model.arm_max_velocities
        if len(command.velocities_rad_s) != len(limits) or any(abs(v) > limit for v, limit in zip(command.velocities_rad_s, limits)):
            raise ValueError("Command exceeds model dimensions or velocity limits")
        if any(command.velocities_rad_s) and not self._fresh_telemetry():
            if self._latest is not None:
                self._latch_stop("telemetry_timeout")
            raise RuntimeError("Movement requires fresh delivered telemetry")
        delay = max(0.0, self.config.command_delay_s + self._command_rng.uniform(
            -self.config.command_jitter_s, self.config.command_jitter_s))
        self._commands.schedule(self.time_s + delay, command)
        first_delivery_deadline = None
        if self._first_submission_s is None:
            self._first_submission_s = self.time_s
            first_delivery_deadline = self.time_s + self.config.command_watchdog_s
        def quantize(value):
            return math.ceil(value * self.config.physics_hz - 1e-12) / self.config.physics_hz
        base = self.config.command_delay_s
        jitter = self.config.command_jitter_s
        self._history.append(CommandIntent(command, quantize(self.time_s + base),
            quantize(self.time_s + max(0, base - jitter)), quantize(self.time_s + base + jitter),
            quantize(self.time_s + base) + self.config.command_watchdog_s, first_delivery_deadline))
        self.events.append({"kind": "command_submitted", "sequence": command.sequence,
                            "time_s": self.time_s, "scheduled_time_s": self.time_s + delay,
                            "velocities_rad_s": command.velocities_rad_s,
                            "valid_until_s": command.valid_until_s})
        self._command_sequence += 1
        return command.sequence

    def _capture(self, obstacle_position):
        sequence = self._observation_sequence
        self._observation_sequence += 1
        if self._observation_rng.random() < self.config.dropout_probability:
            self.events.append({"kind": "observation_dropped", "sequence": sequence, "time_s": self.time_s})
            return
        state = self._plant.controller.backend.get_joint_state()
        q = np.array(state.positions) + self._sensing_rng.normal(0, self.config.joint_noise_std_rad, len(state.positions))
        obstacle = obstacle_position + self._sensing_rng.normal(0, self.config.obstacle_noise_std_m, 3)
        delay = max(0.0, self.config.observation_delay_s + self._observation_rng.uniform(
            -self.config.observation_jitter_s, self.config.observation_jitter_s))
        observation = Observation(sequence, self.time_s, self.time_s + delay, state.joint_names, q, obstacle)
        self._observations.schedule(observation.delivery_time_s, observation)
        self.events.append({"kind": "observation_captured", "sequence": sequence, "time_s": self.time_s,
                            "scheduled_time_s": observation.delivery_time_s,
                            "joint_positions": observation.joint_positions,
                            "obstacle_position_m": observation.obstacle_position_m})

    def step(self):
        """Capture, deliver observations, apply commands, record truth, integrate.

        Return the evaluator record at the pre-integration timestamp. The next
        step evaluates the state resulting from this integration.
        """
        self._require_open()
        obstacle = np.array(self.config.obstacle_origin_m) + self.time_s * np.array(self.config.obstacle_velocity_m_s)
        p.resetBasePositionAndOrientation(self._plant_obstacle, obstacle, (0, 0, 0, 1), physicsClientId=self._plant.client_id)
        if self._tick % self.config.sensor_stride == 0:
            self._capture(obstacle)
        for delivery in self._observations.drain(self.time_s):
            observation = replace(delivery.payload, delivery_time_s=delivery.actual_time_s)
            stale = self._latest is not None and (observation.sequence <= self._latest.sequence
                    or observation.capture_time_s <= self._latest.capture_time_s)
            self.events.append({"kind": "observation_stale" if stale else "observation_delivered",
                                "sequence": observation.sequence, "time_s": self.time_s,
                                "scheduled_time_s": delivery.scheduled_time_s})
            if stale:
                continue
            if observation.joint_names != self._twin.resolved_model.arm_joint_names:
                raise ValueError("Observed joints differ from twin model")
            self._latest = observation
            for index, value in zip(self._twin.controller.arm_joint_indices, observation.joint_positions):
                p.resetJointState(self._twin.robot_id, index, value, physicsClientId=self._twin.client_id)
            p.resetBasePositionAndOrientation(self._twin_obstacle, observation.obstacle_position_m,
                                              (0, 0, 0, 1), physicsClientId=self._twin.client_id)
        if self._latest is None:
            if self.time_s >= self.config.startup_timeout_s:
                self._latch_stop("startup_telemetry_timeout")
        elif not self._fresh_telemetry():
            self._latch_stop("telemetry_timeout")
        for delivery in self._commands.drain(self.time_s):
            command = delivery.payload
            stale = command.sequence <= self._received_sequence
            expired = command.valid_until_s is not None and self.time_s >= command.valid_until_s
            self._received_sequence = max(self._received_sequence, command.sequence)
            kind = "command_stale" if stale else "command_expired" if expired else "command_applied"
            if self._fault is not None:
                kind = "command_blocked"
            self.events.append({"kind": kind,
                                "sequence": command.sequence, "time_s": self.time_s,
                                "scheduled_time_s": delivery.scheduled_time_s})
            if kind == "command_applied":
                try:
                    if not self._plant.controller.backend.command_joint_velocities(command.velocities_rad_s):
                        raise RuntimeError("Simulated plant rejected velocity command")
                except Exception:
                    self._latch_stop("command_dispatch_failed")
                    raise
                self._applied_sequence = command.sequence
                self._active_command = command
                self._last_application_s = self.time_s
        if self._fault is None and self._active_command is not None:
            if self._active_command.valid_until_s is not None and self.time_s >= self._active_command.valid_until_s:
                self._latch_stop("active_command_expired")
            elif self.time_s - self._last_application_s >= self.config.command_watchdog_s:
                self._latch_stop("command_watchdog_timeout")
        elif self._fault is None and self._first_submission_s is not None:
            if self.time_s - self._first_submission_s >= self.config.command_watchdog_s:
                self._latch_stop("command_watchdog_timeout")
        record = self.evaluate()
        # Only the plant advances dynamically. Twin queries cannot step the plant.
        p.stepSimulation(physicsClientId=self._plant.client_id)
        self._tick += 1
        next_position = np.array(self.config.obstacle_origin_m) + self.time_s * np.array(self.config.obstacle_velocity_m_s)
        p.resetBasePositionAndOrientation(self._plant_obstacle, next_position, (0, 0, 0, 1),
                                          physicsClientId=self._plant.client_id)
        return record

    def evaluate(self):
        """Evaluator-only truth access; never pass this record to a controller."""
        self._require_open()
        points = p.getClosestPoints(self._plant.robot_id, self._plant_obstacle, distance=10.0,
                                     physicsClientId=self._plant.client_id)
        clearance = min((point[8] for point in points), default=math.inf)
        result = self._plant.collision_provider.check_collision()
        position, _ = p.getBasePositionAndOrientation(self._plant_obstacle, physicsClientId=self._plant.client_id)
        return TrialRecord(self.time_s,
                           tuple(self._plant.controller.get_current_joint_positions()),
                           tuple(self._twin.controller.get_current_joint_positions()), tuple(position),
                           self._latest.obstacle_position_m if self._latest else None,
                           self._latest.sequence if self._latest else None,
                           self.time_s - self._latest.capture_time_s if self._latest else None,
                           self._applied_sequence if self._applied_sequence >= 0 else None,
                           result.in_collision, clearance, self._fault)

    def close(self):
        if not self._closed:
            for world in (self._twin, self._plant):
                if world is not None:
                    world.close()
            self._closed = True

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, *args):
        self.close()
