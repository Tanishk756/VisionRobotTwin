"""Behavioral regressions for the October software audit."""
from dataclasses import replace
from unittest.mock import MagicMock

import numpy as np
import pybullet as p
import pybullet_data
import pytest
from scipy.spatial.transform import Rotation

from robotics.backends.base import BackendError, BackendHealthStatus, TimestampedJointState
from robotics.backends.physical_identity import ExpectedPhysicalRobotIdentity, SafetySignalState
from robotics.backends.physical_readiness import PhysicalReadinessEvaluator
from robotics.backends.physical_state_backend import PhysicalRobotStateBackend
from robotics.backends.physical_validation import JointLimitSource, LimitProvenanceEngine, LimitSourceKind
from robotics.inverse_kinematics import IKResult, IKStatus
from robotics.differential_ik import ResolvedRateController
from robotics.kinematics_provider import KinematicsProvider, PyBulletKinematicsProvider
from robotics.motion_manager import MotionManager, MotionState
from robotics.robot_controller import GenericRobotController
from robotics.robot_registry import get_robot_registry
from robotics.safety import CommandSafetyConfig, CommandSafetyFaultCode, GuardedRobotBackend, SafetyGuardState
from robotics.trajectory import CartesianSE3Trajectory, JointQuinticTrajectory, TrajectoryExecutor
from tests.mocks.mock_physical_adapter import MockPhysicalTelemetryAdapter
from tests.test_guarded_backend import _make_model_and_backend
from tests.test_physical_readiness import _make_test_robot_model
from vision.extrinsics import ExtrinsicCalibration


@pytest.mark.parametrize("mode", ["position", "velocity"])
@pytest.mark.parametrize("problem", ["stale", "future", "disconnect", "health", "joint_names", "missing"])
def test_guard_rechecks_state_after_arming(mode, problem):
    model, backend = _make_model_and_backend()
    guard = GuardedRobotBackend(backend, model, time_source=lambda: 100.0)
    state = replace(backend.get_joint_state.return_value, receive_timestamp_s=100.0)
    backend.get_joint_state.return_value = state
    guard.arm()
    if problem in ("stale", "future"):
        backend.get_joint_state.return_value = replace(state, receive_timestamp_s=99.0 if problem == "stale" else 101.0)
    elif problem == "disconnect":
        backend.is_connected.return_value = False
    elif problem == "health":
        backend.health_status.return_value = BackendHealthStatus.ERROR
    elif problem == "joint_names":
        backend.get_joint_state.return_value = replace(state, joint_names=("wrong1", "wrong2"))
    else:
        backend.get_joint_state.side_effect = BackendError("Telemetry unavailable")
    command = guard.command_joint_positions if mode == "position" else guard.command_joint_velocities
    with pytest.raises(BackendError):
        command([0.1, 0.1])
    backend.command_joint_positions.assert_not_called()
    backend.command_joint_velocities.assert_not_called()
    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED


@pytest.mark.parametrize("mode", ["position", "velocity"])
@pytest.mark.parametrize("failure", [False, RuntimeError("transport failed")])
def test_dispatch_failure_stops_and_latches(mode, failure):
    model, backend = _make_model_and_backend()
    method = backend.command_joint_positions if mode == "position" else backend.command_joint_velocities
    if isinstance(failure, Exception):
        method.side_effect = failure
    else:
        method.return_value = failure
    guard = GuardedRobotBackend(backend, model)
    guard.arm()
    command = guard.command_joint_positions if mode == "position" else guard.command_joint_velocities
    with pytest.raises(BackendError, match="dispatch failed"):
        command([0.1, 0.1])
    assert guard.active_fault.code == CommandSafetyFaultCode.COMMAND_DISPATCH_FAILED
    assert guard.accepted_command_sequence == 0
    backend.halt_motion.assert_called_once()
    guard.disarm()
    guard.disconnect()
    guard.connect()
    assert guard.guard_state == SafetyGuardState.FAULT_LATCHED
    with pytest.raises(BackendError):
        guard.arm()


def test_position_margin_and_step_configuration_are_enforced():
    model, backend = _make_model_and_backend()
    with pytest.raises(ValueError, match="DoF"):
        GuardedRobotBackend(backend, model, CommandSafetyConfig(max_position_step_by_joint=(0.1,)))
    guard = GuardedRobotBackend(backend, model, CommandSafetyConfig(position_limit_margin_rad=0.1))
    guard.arm()
    with pytest.raises(BackendError):
        guard.command_joint_positions([1.95, 0])
    backend.command_joint_positions.assert_not_called()


@pytest.mark.parametrize("field", ["state_timeout_s", "command_watchdog_timeout_s", "position_limit_margin_rad"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1.0])
def test_nonfinite_guard_configuration_rejected(field, value):
    with pytest.raises(ValueError):
        CommandSafetyConfig(**{field: value})


def test_joint_snapshot_copies_mutable_inputs_and_rejects_future_age():
    positions = [0.1]
    names = ["j1"]
    state = TimestampedJointState(100, 100, names, positions)
    positions[0] = 99
    names[0] = "changed"
    assert state.positions == (0.1,) and state.joint_names == ("j1",)
    assert not state.is_fresh(99, 1)


@pytest.mark.parametrize("robot", ["panda", "kuka_iiwa"])
@pytest.mark.parametrize("rotated", [False, True])
def test_fk_and_jacobian_use_same_world_link_frame(robot, rotated):
    client = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        spec = get_robot_registry().get_robot_spec(robot)
        orientation = Rotation.from_euler("xyz", [0.2, -0.1, 0.7]).as_quat() if rotated else [0, 0, 0, 1]
        body = p.loadURDF(spec.urdf_path, basePosition=[0.1, 0.2, 0.3], baseOrientation=orientation, useFixedBase=True, physicsClientId=client)
        ee = 11 if robot == "panda" else 6
        # Reversed joint order exposes assumptions that the arm occupies the
        # first n columns in canonical order.
        joints = tuple(reversed(range(7)))
        provider = PyBulletKinematicsProvider(client, body, joints, ee)
        q = np.array(spec.home_joint_positions)[::-1]
        linear, angular, _ = provider.compute_jacobian(q)
        epsilon = 1e-5
        for i in range(7):
            delta = np.eye(7)[i] * epsilon
            plus, plus_orn = provider.compute_fk(q + delta)
            minus, minus_orn = provider.compute_fk(q - delta)
            np.testing.assert_allclose((plus - minus) / (2 * epsilon), linear[:, i], atol=1e-5)
            measured_angular = (Rotation.from_quat(plus_orn) * Rotation.from_quat(minus_orn).inv()).as_rotvec() / (2 * epsilon)
            np.testing.assert_allclose(measured_angular, angular[:, i], atol=1e-5)
    finally:
        p.disconnect(client)


@pytest.mark.parametrize("omission", ["identity", "limits", "stale_safety", "future_safety", "drives", "brakes", "mode"])
def test_physical_readiness_requires_explicit_fresh_evidence(omission):
    provider = MockPhysicalTelemetryAdapter.create_nominal(now_s=100)
    expected = ExpectedPhysicalRobotIdentity("MockVendor", "MockArm", ("joint1", "joint2"), 2)
    if omission in ("stale_safety", "future_safety"):
        provider.set_safety_status(replace(provider.read_safety_status(), receive_timestamp_s=90 if omission == "stale_safety" else 101))
    elif omission in ("drives", "brakes", "mode"):
        field = {"drives": "drives_state_safe_or_known", "brakes": "brakes_state_safe_or_known", "mode": "operational_mode_safe"}[omission]
        provider.set_safety_status(replace(provider.read_safety_status(), **{field: SafetySignalState.UNKNOWN}))
    report = PhysicalReadinessEvaluator().evaluate(provider, _make_test_robot_model(),
        expected_identity=None if omission == "identity" else expected,
        limits_resolved=omission != "limits", now_s=100.1)
    assert report.ready_for_observation
    assert not report.eligible_for_future_command_commissioning
    assert provider.command_side_effect_count == 0


def test_physical_backend_rejects_future_telemetry_and_empty_limit_evidence():
    provider = MockPhysicalTelemetryAdapter.create_nominal(now_s=101)
    backend = PhysicalRobotStateBackend(provider, _make_test_robot_model(), time_source=lambda: 100)
    with pytest.raises(BackendError):
        backend.get_joint_state()
    assert not LimitProvenanceEngine().resolve_robot_limits({})[0]
    source = JointLimitSource("unverified", LimitSourceKind.OTHER, -1, 1, 1, trusted_for_command=False)
    assert not LimitProvenanceEngine().resolve_joint_limits("joint1", [source]).is_valid


@pytest.mark.parametrize("mode", ["position", "velocity"])
def test_controller_propagates_dispatch_rejection_and_rejects_bad_dimensions(mode):
    model, backend = _make_model_and_backend()
    controller = GenericRobotController(resolved_model=model, backend=backend, kinematics_provider=MagicMock(spec=KinematicsProvider))
    command = controller.set_arm_joint_positions if mode == "position" else controller.set_arm_joint_velocities
    with pytest.raises(ValueError):
        command([0, 0, 0])
    with pytest.raises(ValueError):
        command([float("nan"), 0])
    backend.command_joint_positions.return_value = False
    backend.command_joint_velocities.return_value = False
    with pytest.raises(BackendError):
        command([0, 0])


def test_motion_completion_waits_for_feedback_and_times_out():
    model, backend = _make_model_and_backend()
    controller = GenericRobotController(resolved_model=model, backend=backend, kinematics_provider=MagicMock(spec=KinematicsProvider))
    solver = MagicMock()
    solver.solve.return_value = IKResult(True, [0.5, 0.5], IKStatus.SOLUTION_RETURNED, "OK")
    manager = MotionManager(controller, solver, trajectory_mode="direct", settling_timeout_s=0.2)
    assert manager.plan_motion_to_pose([0.3, 0, 0.3])
    assert not manager.step(0.1)[0]
    assert manager.state == MotionState.DIRECT
    assert not manager.step(0.1)[0]
    assert manager.state == MotionState.FAILED
    backend.halt_motion.assert_called_once()
    calls = backend.command_joint_positions.call_count
    manager.step(0.1)
    assert backend.command_joint_positions.call_count == calls


def test_motion_dispatch_failure_cannot_report_completion():
    model, backend = _make_model_and_backend()
    controller = GenericRobotController(resolved_model=model, backend=backend, kinematics_provider=MagicMock(spec=KinematicsProvider))
    solver = MagicMock()
    solver.solve.return_value = IKResult(True, [0.5, 0.5], IKStatus.SOLUTION_RETURNED, "OK")
    manager = MotionManager(controller, solver)
    manager.plan_motion_to_pose([0.3, 0, 0.3])
    backend.command_joint_positions.return_value = False
    assert not manager.step(0.1)[0]
    assert manager.state == MotionState.FAILED


def test_cartesian_angular_velocity_matches_orientation_derivative():
    start = Rotation.from_euler("xyz", [0.3, 0.2, -0.4])
    goal = Rotation.from_euler("xyz", [-0.2, 0.6, 0.5])
    trajectory = CartesianSE3Trajectory([0, 0, 0], start.as_quat(), [1, 1, 1], goal.as_quat(), duration=2)
    epsilon = 1e-5
    plus = Rotation.from_quat(trajectory.evaluate(1 + epsilon).orientation)
    minus = Rotation.from_quat(trajectory.evaluate(1 - epsilon).orientation)
    velocity = (plus * minus.inv()).as_rotvec() / (2 * epsilon)
    np.testing.assert_allclose(trajectory.evaluate(1).angular_velocity, velocity, atol=1e-6)
    np.testing.assert_allclose(trajectory.evaluate(2).angular_velocity, 0)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1, 0])
def test_trajectory_rejects_invalid_duration_and_steps(value):
    with pytest.raises(ValueError):
        JointQuinticTrajectory([0], [1], duration=value)
    with pytest.raises(ValueError):
        TrajectoryExecutor(JointQuinticTrajectory([0], [1])).step(value)


@pytest.mark.parametrize("field,value", [("translation_m", [1, 0, 0]), ("translation_m", [float("nan"), 0, 0]), ("quaternion_xyzw", [1, 0, 0, 0]), ("quaternion_xyzw", [1, 0, 0])])
def test_extrinsic_redundant_fields_cannot_disagree(field, value):
    data = ExtrinsicCalibration().to_dict()
    data["T_robot_camera"][field] = value
    with pytest.raises(ValueError):
        ExtrinsicCalibration.from_dict(data)


def test_watchdog_stops_starvation_without_another_command():
    import threading
    model, backend = _make_model_and_backend()
    halted = threading.Event()
    backend.halt_motion.side_effect = halted.set
    guard = GuardedRobotBackend(backend, model, CommandSafetyConfig(command_watchdog_timeout_s=0.03))
    guard.arm()
    guard.command_joint_velocities([0.1, 0.1])
    assert halted.wait(1.0), "Default guard must monitor even when command caller stops running"
    assert guard.active_fault.code == CommandSafetyFaultCode.COMMAND_WATCHDOG_TIMEOUT
    guard.disconnect()
    assert guard._watchdog_thread is None


@pytest.mark.parametrize("orientation", [[0, 0, 0, 0], [1, 0, 0], [float("nan"), 0, 0, 1]])
def test_resolved_rate_invalid_orientation_produces_no_motion(orientation):
    model, backend = _make_model_and_backend()
    provider = MagicMock(spec=KinematicsProvider)
    controller = GenericRobotController(resolved_model=model, backend=backend, kinematics_provider=provider)
    resolved = ResolvedRateController(robot_controller=controller)
    velocities, _ = resolved.compute_step([0.3, 0, 0.3], orientation)
    np.testing.assert_array_equal(velocities, [0, 0])
    provider.compute_fk.assert_not_called()


def test_resolved_rate_feedforward_can_move_at_zero_position_error():
    model, backend = _make_model_and_backend()
    provider = MagicMock(spec=KinematicsProvider)
    provider.compute_fk.return_value = (np.zeros(3), np.array([0, 0, 0, 1]))
    jacobian = np.zeros((6, 2))
    jacobian[0, 0] = jacobian[1, 1] = 1
    provider.compute_jacobian.return_value = (jacobian[:3], jacobian[3:], jacobian)
    controller = GenericRobotController(resolved_model=model, backend=backend, kinematics_provider=provider)
    resolved = ResolvedRateController(robot_controller=controller, enable_nullspace=False)
    velocities, _ = resolved.compute_step([0, 0, 0], target_linear_velocity=np.array([0.1, 0.2, 0]))
    assert velocities[0] > 0 and velocities[1] > velocities[0]


@pytest.mark.parametrize("violation", ["limits", "orientation"])
def test_experiment_success_requires_limits_and_orientation(violation):
    from robotics.experiments import compute_experiment_metrics
    zeros = np.zeros(2)
    orientations = np.tile([0, 0, 0, 1], (2, 1))
    actual = np.tile([1, 0, 0, 0], (2, 1)) if violation == "orientation" else orientations.copy()
    metrics = compute_experiment_metrics(
        np.array([0, 0.1]), np.zeros((2, 3)), np.zeros((2, 3)), orientations, actual,
        np.zeros((2, 2)), np.zeros((2, 2)), np.ones(2), np.ones(2), np.ones(2),
        zeros, zeros, zeros, np.ones(2) if violation == "limits" else zeros, zeros,
        is_se3=False,
    )
    assert not metrics.success


def test_simulation_position_gains_reach_motor_transport():
    from unittest.mock import patch
    from robotics.backends.pybullet_backend import PyBulletRobotBackend
    backend = PyBulletRobotBackend(0, 0, [0, 1], ["j1", "j2"], position_gain=0.75, velocity_gain=1.0)
    with patch.object(backend, "is_connected", return_value=True), patch("pybullet.setJointMotorControlArray") as dispatch:
        backend.command_joint_positions([0.1, 0.2])
    assert dispatch.call_args.kwargs["positionGains"] == [0.75, 0.75]
    assert dispatch.call_args.kwargs["velocityGains"] == [1.0, 1.0]
