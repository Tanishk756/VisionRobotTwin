"""Immutable messages and validated configuration for plant/twin experiments."""
from dataclasses import dataclass
import math


def nonnegative(name, value, positive=False):
    if not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")


def vector(name, values, length=None):
    result = tuple(float(v) for v in values)
    if not result or (length is not None and len(result) != length) or not all(map(math.isfinite, result)):
        raise ValueError(f"Invalid finite vector: {name}")
    return result


@dataclass(frozen=True)
class Observation:
    sequence: int
    capture_time_s: float
    delivery_time_s: float
    joint_names: tuple
    joint_positions: tuple
    obstacle_position_m: tuple
    frame: str = "world"

    def __post_init__(self):
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        nonnegative("capture_time_s", self.capture_time_s)
        nonnegative("delivery_time_s", self.delivery_time_s)
        if self.delivery_time_s < self.capture_time_s:
            raise ValueError("Delivery cannot precede capture")
        names = tuple(self.joint_names)
        if not names or len(set(names)) != len(names) or any(not isinstance(n, str) or not n for n in names):
            raise ValueError("Joint names must be nonempty and unique")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "joint_positions", vector("joints", self.joint_positions, len(names)))
        object.__setattr__(self, "obstacle_position_m", vector("obstacle", self.obstacle_position_m, 3))
        if self.frame != "world":
            raise ValueError("Initial harness only supports world-frame observations")


@dataclass(frozen=True)
class VelocityCommand:
    sequence: int
    created_time_s: float
    velocities_rad_s: tuple
    valid_until_s: float | None = None

    def __post_init__(self):
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        nonnegative("created_time_s", self.created_time_s)
        object.__setattr__(self, "velocities_rad_s", vector("velocities", self.velocities_rad_s))
        if self.valid_until_s is not None:
            nonnegative("valid_until_s", self.valid_until_s)
            if self.valid_until_s <= self.created_time_s:
                raise ValueError("Command validity must end after creation")


@dataclass(frozen=True)
class ScenarioConfig:
    robot: str = "panda"
    seed: int = 0
    physics_hz: int = 240
    sensor_stride: int = 8
    duration_s: float = 2.0
    observation_delay_s: float = 0.08
    observation_jitter_s: float = 0.02
    command_delay_s: float = 0.04
    command_jitter_s: float = 0.01
    dropout_probability: float = 0.1
    joint_noise_std_rad: float = 0.001
    obstacle_noise_std_m: float = 0.003
    plant_mass_scale: float = 1.2
    plant_velocity_gain: float = 0.8
    obstacle_origin_m: tuple = (0.45, -0.3, 0.35)
    obstacle_velocity_m_s: tuple = (0.0, 0.2, 0.0)
    obstacle_radius_m: float = 0.05
    command_lifetime_s: float = 0.15
    command_watchdog_s: float = 0.15
    telemetry_watchdog_s: float = 0.3
    startup_timeout_s: float = 0.3

    def __post_init__(self):
        for name in ("seed", "physics_hz", "sensor_stride"):
            value = getattr(self, name)
            if type(value) is not int or value < (0 if name == "seed" else 1):
                raise ValueError(f"Invalid integer {name}")
        for name in ("duration_s", "plant_mass_scale", "plant_velocity_gain", "obstacle_radius_m"):
            nonnegative(name, getattr(self, name), positive=True)
        for name in ("command_lifetime_s", "command_watchdog_s", "telemetry_watchdog_s", "startup_timeout_s"):
            nonnegative(name, getattr(self, name), positive=True)
        for name in ("observation_delay_s", "observation_jitter_s", "command_delay_s",
                     "command_jitter_s", "joint_noise_std_rad", "obstacle_noise_std_m"):
            nonnegative(name, getattr(self, name))
        if not 0 <= self.dropout_probability <= 1 or self.plant_velocity_gain > 1:
            raise ValueError("Invalid probability or motor gain")
        for name in ("obstacle_origin_m", "obstacle_velocity_m_s"):
            object.__setattr__(self, name, vector(name, getattr(self, name), 3))


@dataclass(frozen=True)
class TrialRecord:
    time_s: float
    plant_joint_positions: tuple
    twin_joint_positions: tuple
    plant_obstacle_position_m: tuple
    observed_obstacle_position_m: tuple | None
    observation_sequence: int | None
    observation_age_s: float | None
    applied_command_sequence: int | None
    in_collision: bool
    obstacle_clearance_m: float
    execution_fault: str | None = None
    environment_clearance_m: float = float("inf")
