"""Seeded PyBullet robustness trials; pose-level sensing and transport injection.

These experiments exercise software behavior, not optical camera accuracy or
physical safety. The watchdog runs on deterministic simulation time.
"""
from dataclasses import asdict, dataclass, replace
from typing import Optional

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from config.settings import AppConfig
from robotics.backends.base import BackendError, BackendHealthStatus, RobotBackend
from robotics.safety import CommandSafetyConfig, GuardedRobotBackend, SafetyGuardState
from robotics.simulator import PyBulletSimulator
from robotics.trajectory import CartesianSE3Trajectory


PROFILES = ("nominal", "pose_noise", "detection_dropout", "stale_telemetry",
            "transport_disconnect", "dispatch_failure", "command_starvation")
EXPECTED_FAULTS = {
    "stale_telemetry": "state_stale",
    "transport_disconnect": "backend_unhealthy",
    "dispatch_failure": "command_dispatch_failed",
    "command_starvation": "command_watchdog_timeout",
}


@dataclass(frozen=True)
class RobustnessConfig:
    duration_s: float = 1.0
    settle_s: float = 0.5
    physics_hz: int = 240
    start_jitter_rad: float = 0.03
    position_noise_std_m: float = 0.003
    orientation_noise_std_rad: float = 0.01
    telemetry_delay_s: float = 0.2
    environment_clearance_m: float = 0.01
    self_clearance_m: float = 0.0
    goal_tolerance_m: float = 0.01
    goal_orientation_tolerance_deg: float = 10.0

    def __post_init__(self):
        if not isinstance(self.physics_hz, int) or self.physics_hz <= 0:
            raise ValueError("Physics frequency must be a positive integer")
        for name, value in asdict(self).items():
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.duration_s <= 0 or self.goal_tolerance_m <= 0 or self.goal_orientation_tolerance_deg <= 0 or self.settle_s < 0.2:
            raise ValueError("Duration/tolerance must be positive and settling at least 0.2 s")
        if self.telemetry_delay_s <= 0.1:
            raise ValueError("Stale-telemetry delay must exceed the 0.1 s state timeout")


class InjectedSimulationBackend(RobotBackend):
    """Test-only wrapper with a simulated receive clock and explicit failure hooks."""
    def __init__(self, backend, clock):
        self.backend = backend
        self.clock = clock
        self.delay_s = 0.0
        self.disconnected = False
        self.fail_dispatch = False
        self.dispatch_attempts = 0
        self.halt_attempts = 0

    @property
    def transport_capabilities(self):
        return self.backend.transport_capabilities

    def connect(self):
        return self.backend.connect()

    def disconnect(self):
        self.backend.disconnect()

    def is_connected(self):
        return not self.disconnected and self.backend.is_connected()

    def health_status(self):
        return BackendHealthStatus.DISCONNECTED if self.disconnected else self.backend.health_status()

    def get_joint_state(self):
        return replace(self.backend.get_joint_state(), receive_timestamp_s=self.clock[0] - self.delay_s)

    def command_joint_positions(self, targets):
        self.dispatch_attempts += 1
        return False if self.fail_dispatch else self.backend.command_joint_positions(targets)

    def command_joint_velocities(self, targets, effort_limit=None):
        self.dispatch_attempts += 1
        return False if self.fail_dispatch else self.backend.command_joint_velocities(targets, effort_limit)

    def halt_motion(self):
        # Independent privileged stop path remains available in the injected
        # disconnect scenario. This is not a model of a broken physical E-stop.
        self.halt_attempts += 1
        self.backend.halt_motion()


@dataclass
class RobustnessResult:
    robot: str
    controller: str
    profile: str
    seed: int
    outcome_passed: bool
    tracking_success: Optional[bool]
    expected_fault: Optional[str]
    observed_fault: Optional[str]
    final_error_mm: float
    rmse_mm: float
    p95_error_mm: float
    peak_measured_velocity_rad_s: float
    final_measured_velocity_rad_s: float
    collision_or_clearance_events: int
    joint_limit_events: int
    dropout_frames: int
    halt_attempts: int
    post_fault_command_blocked: Optional[bool]
    reason: str
    final_orientation_error_deg: float = 0.0
    fault_detection_delay_s: Optional[float] = None

    def to_dict(self):
        return asdict(self)


def run_robustness_trial(robot="panda", controller="resolved-rate", profile="nominal",
                         seed=0, config=None):
    """Run a perturbed start/path with one explicit fault/noise profile."""
    if profile not in PROFILES or controller not in ("ik", "resolved-rate"):
        raise ValueError("Unknown robustness profile or controller")
    config = config or RobustnessConfig()
    rng = np.random.default_rng(seed)
    app = AppConfig(robot_name=robot, environment_clearance_m=config.environment_clearance_m,
                    self_clearance_m=config.self_clearance_m)
    app.simulation.gui = False
    app.simulation.time_step = 1.0 / config.physics_hz
    sim = PyBulletSimulator(app, headless=True)
    try:
        ctrl = sim.controller
        low, high, _, home = ctrl.get_joint_limits()
        start_q = np.clip(np.array(home) + rng.normal(0, config.start_jitter_rad, len(home)),
                          np.array(low) + 1e-6, np.array(high) - 1e-6)
        for index, value in zip(ctrl.arm_joint_indices, start_q):
            p.resetJointState(sim.robot_id, index, value, physicsClientId=sim.client_id)
        start_pos, start_orn = ctrl.get_end_effector_pose()
        goal = start_pos + np.array([0.03, 0.04, -0.03]) + rng.uniform(-0.005, 0.005, 3)
        trajectory = CartesianSE3Trajectory(start_pos, start_orn, goal, start_orn, config.duration_s)
        clock = [0.0]
        injected = InjectedSimulationBackend(ctrl.backend, clock)
        guard = GuardedRobotBackend(injected, ctrl.model, CommandSafetyConfig(
            state_timeout_s=0.1, command_watchdog_timeout_s=0.1,
            watchdog_auto_start=False, max_position_step_by_joint=(0.1,) * ctrl.model.dof,
        ), time_source=lambda: clock[0])
        ctrl.backend = guard
        guard.arm()
        dt = 1.0 / config.physics_hz
        steps = int(round((config.duration_s + config.settle_s) * config.physics_hz))
        errors, speeds = [], []
        collision_events = limit_events = dropout_frames = 0
        observed = None
        fault_delay = None
        blocked = None
        reason = ""
        hold_q = None
        injection_at = config.duration_s * 0.4
        for step in range(steps):
            clock[0] = step * dt
            sample = trajectory.evaluate(clock[0])
            active = clock[0] >= injection_at
            if active and profile == "stale_telemetry":
                injected.delay_s = config.telemetry_delay_s
            if active and profile == "transport_disconnect":
                injected.disconnected = True
            if active and profile == "dispatch_failure":
                injected.fail_dispatch = True
            try:
                guard.check_watchdog()
                if guard.guard_state != SafetyGuardState.FAULT_LATCHED:
                    if active and profile == "command_starvation":
                        pass
                    elif profile == "detection_dropout" and 0.35 * config.duration_s <= clock[0] <= 0.55 * config.duration_s:
                        if hold_q is None:
                            hold_q = list(injected.backend.get_joint_state().positions)
                        guard.command_joint_positions(hold_q)
                        dropout_frames += 1
                    else:
                        target, orientation = sample.position.copy(), sample.orientation.copy()
                        if profile == "pose_noise":
                            target += rng.normal(0, config.position_noise_std_m, 3)
                            orientation = (Rotation.from_rotvec(rng.normal(0, config.orientation_noise_std_rad, 3))
                                           * Rotation.from_quat(orientation)).as_quat()
                        if controller == "resolved-rate":
                            velocities, _ = sim.resolved_rate_controller.compute_step(
                                target, orientation, dt,
                                target_linear_velocity=sample.linear_velocity,
                                target_angular_velocity=sample.angular_velocity,
                            )
                            ctrl.set_arm_joint_velocities(velocities)
                        else:
                            result = sim.ik_solver.solve(target, orientation)
                            if not result.success:
                                raise BackendError(f"IK failure: {result.status_message}")
                            ctrl.set_arm_joint_positions(result.joint_positions, dt=dt)
            except BackendError as err:
                reason = str(err)
                if guard.guard_state != SafetyGuardState.FAULT_LATCHED:
                    guard.request_software_stop()
                    raise
            if guard.active_fault is not None and observed is None:
                observed = guard.active_fault.code.value
                fault_delay = clock[0] - injection_at
                attempts = injected.dispatch_attempts
                try:
                    guard.command_joint_velocities([0.0] * ctrl.model.dof)
                    blocked = False
                except BackendError:
                    blocked = injected.dispatch_attempts == attempts
            p.stepSimulation(physicsClientId=sim.client_id)
            # Truth measurements bypass injected telemetry; the controller does not.
            truth = injected.backend.get_joint_state()
            pos, truth_orn = ctrl.kinematics_provider.compute_fk(truth.positions)
            errors.append(float(np.linalg.norm(pos - sample.position) * 1000))
            speeds.append(float(np.max(np.abs(truth.velocities))))
            limit_events += int(np.any(np.array(truth.positions) < np.array(low) - 0.01)
                                or np.any(np.array(truth.positions) > np.array(high) + 0.01))
            collision_events += int(sim.collision_provider.check_collision().in_collision)
        final_error = float(np.linalg.norm(pos - goal) * 1000)
        orientation_error = float(np.degrees((Rotation.from_quat(start_orn) * Rotation.from_quat(truth_orn).inv()).magnitude()))
        expected = EXPECTED_FAULTS.get(profile)
        safe = collision_events == 0 and limit_events == 0
        tracking = None if expected else (safe and observed is None and final_error <= config.goal_tolerance_m * 1000
                                         and orientation_error <= config.goal_orientation_tolerance_deg)
        passed = (safe and observed == expected and blocked is True and speeds[-1] < 0.05
                  and injected.halt_attempts > 0) if expected else bool(tracking)
        result = RobustnessResult(robot, controller, profile, seed, passed, tracking,
            expected, observed, final_error, float(np.sqrt(np.mean(np.square(errors)))),
            float(np.percentile(errors, 95)), max(speeds), speeds[-1], collision_events,
            limit_events, dropout_frames, injected.halt_attempts, blocked,
            reason if observed or not passed else "", final_orientation_error_deg=orientation_error,
            fault_detection_delay_s=fault_delay)
        guard.disconnect()
        return result
    finally:
        sim.close()


def summarize_robustness(results):
    """Keep expected-stop outcomes separate from tracking success statistics."""
    groups = {}
    for result in results:
        groups.setdefault(f"{result.robot}/{result.controller}/{result.profile}", []).append(result)
    summary = {}
    for key, group in groups.items():
        errors = [result.final_error_mm for result in group]
        tracking = [result.tracking_success for result in group if result.tracking_success is not None]
        summary[key] = {
            "trials": len(group), "outcomes_passed": sum(result.outcome_passed for result in group),
            "outcome_pass_rate": sum(result.outcome_passed for result in group) / len(group),
            "tracking_trials": len(tracking), "tracking_successes": sum(tracking),
            "tracking_success_rate": sum(tracking) / len(tracking) if tracking else None,
            "expected_stop_trials": len(group) - len(tracking),
            "final_error_mm_mean": float(np.mean(errors)), "final_error_mm_p95": float(np.percentile(errors, 95)),
            "final_error_mm_max": max(errors),
            "final_error_mm_std": float(np.std(errors)),
            "rmse_mm_mean": float(np.mean([result.rmse_mm for result in group])),
            "rmse_mm_p95": float(np.percentile([result.rmse_mm for result in group], 95)),
            "final_orientation_error_deg_max": max(result.final_orientation_error_deg for result in group),
            "fault_detection_delay_s_max": max((result.fault_detection_delay_s for result in group
                                                if result.fault_detection_delay_s is not None), default=None),
            "collision_or_clearance_events": sum(result.collision_or_clearance_events for result in group),
        }
    return summary
