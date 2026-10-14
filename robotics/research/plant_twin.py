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


@dataclass(frozen=True)
class ControllerView:
    time_s: float
    observation: Observation | None
    dof: int
    joint_velocity_limits_rad_s: tuple


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
                              self._twin.resolved_model.arm_max_velocities)

    def submit_velocity(self, values):
        self._require_open()
        command = VelocityCommand(self._command_sequence, self.time_s, values)
        limits = self._twin.resolved_model.arm_max_velocities
        if len(command.velocities_rad_s) != len(limits) or any(abs(v) > limit for v, limit in zip(command.velocities_rad_s, limits)):
            raise ValueError("Command exceeds model dimensions or velocity limits")
        delay = max(0.0, self.config.command_delay_s + self._command_rng.uniform(
            -self.config.command_jitter_s, self.config.command_jitter_s))
        self._commands.schedule(self.time_s + delay, command)
        self.events.append({"kind": "command_submitted", "sequence": command.sequence,
                            "time_s": self.time_s, "scheduled_time_s": self.time_s + delay,
                            "velocities_rad_s": command.velocities_rad_s})
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
            stale = self._latest is not None and observation.sequence <= self._latest.sequence
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
        for delivery in self._commands.drain(self.time_s):
            command = delivery.payload
            stale = command.sequence <= self._applied_sequence
            self.events.append({"kind": "command_stale" if stale else "command_applied",
                                "sequence": command.sequence, "time_s": self.time_s,
                                "scheduled_time_s": delivery.scheduled_time_s})
            if not stale:
                if not self._plant.controller.backend.command_joint_velocities(command.velocities_rad_s):
                    raise RuntimeError("Simulated plant rejected velocity command")
                self._applied_sequence = command.sequence
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
                           result.in_collision, clearance)

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
