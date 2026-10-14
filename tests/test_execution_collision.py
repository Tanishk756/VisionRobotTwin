"""Dispatch prevention, fault recovery, and measured-state supervision."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from robotics.execution_collision import CollisionGuardBackend, CollisionSafetyError
from robotics.collision_provider import CollisionResult
from robotics.backends.base import TimestampedJointState


def make_guard():
    backend = Mock()
    backend.is_connected.return_value = True
    backend.get_joint_state.return_value = TimestampedJointState(0, 0, ("joint",), (0.0,))
    backend.command_joint_positions.return_value = True
    backend.command_joint_velocities.return_value = True
    provider = Mock()
    provider.check_collision.side_effect = lambda q: CollisionResult(abs(q[0]) >= 0.05)
    model = SimpleNamespace(dof=1, arm_joint_names=("joint",), arm_lower_limits=(-1,), arm_upper_limits=(1,))
    return CollisionGuardBackend(backend, provider, model), backend, provider


@pytest.mark.parametrize("velocity", [False, True])
def test_predicted_collision_blocks_transport_and_latches(velocity):
    guard, backend, provider = make_guard()
    command = guard.command_joint_velocities if velocity else guard.command_joint_positions
    with pytest.raises(CollisionSafetyError):
        command([0.8])
    backend.command_joint_positions.assert_not_called()
    backend.command_joint_velocities.assert_not_called()
    backend.halt_motion.assert_called_once()
    provider.check_collision.side_effect = lambda q: CollisionResult(False)
    with pytest.raises(CollisionSafetyError):
        command([0.0])
    guard.reset_fault()
    assert command([0.0])
    assert guard.active_fault is None


def test_measured_collision_and_unsafe_reset():
    guard, backend, provider = make_guard()
    provider.check_collision.side_effect = lambda q: CollisionResult(True)
    assert not guard.monitor()
    assert guard.active_fault.code == "measured_collision"
    with pytest.raises(CollisionSafetyError):
        guard.reset_fault()
    assert guard.active_fault is not None


def test_query_failure_stops_and_latches():
    guard, backend, provider = make_guard()
    provider.check_collision.side_effect = RuntimeError("query lost")
    assert not guard.monitor()
    assert guard.active_fault.code == "collision_query_failed"
    backend.halt_motion.assert_called_once()


def test_transport_rejection_stops():
    guard, backend, provider = make_guard()
    backend.command_joint_positions.return_value = False
    with pytest.raises(CollisionSafetyError):
        guard.command_joint_positions([0.01])
    assert guard.active_fault.code == "dispatch_failed"


def test_stop_failure_is_visible():
    guard, backend, provider = make_guard()
    backend.halt_motion.side_effect = RuntimeError("stop lost")
    with pytest.raises(CollisionSafetyError, match="halt failed"):
        guard.command_joint_positions([0.8])
    assert guard.active_fault.code == "halt_failed"


@pytest.mark.parametrize("mode,controller", [("manual", "ik"), ("manual", "resolved-rate"), ("auto", "ik"), ("auto", "resolved-rate")])
def test_application_fault_survives_frames_modes_and_unsafe_reset(mode, controller):
    from config.settings import AppConfig
    from main import VisionRobotTwinApp
    from robotics.state_machine import RobotState
    config = AppConfig()
    config.mode = mode
    config.controller_type = controller
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    app = VisionRobotTwinApp(config, headless_sim=True)
    try:
        guard = app.simulator.execution_guard
        original = guard.provider.check_collision
        guard.provider.check_collision = lambda q=None: CollisionResult(True)
        assert not guard.monitor()
        frame = app.process_frame(wall_dt=0.01)
        assert frame.success
        assert app.state_machine.state == RobotState.ERROR
        assert "COLLISION_STOP" in frame.ik_status
        app._handle_keypress(ord("r"))
        assert guard.active_fault is not None
        app._handle_keypress(ord("m"))
        app.process_frame(wall_dt=0.01)
        assert app.state_machine.state == RobotState.ERROR
        guard.provider.check_collision = original
        app._handle_keypress(ord("r"))
        assert guard.active_fault is None
    finally:
        app.cleanup()


def test_intermediate_collision_rejected_even_when_endpoint_is_clear():
    guard, backend, provider = make_guard()
    provider.check_collision.side_effect = lambda q: CollisionResult(0.03 < q[0] < 0.07)
    with pytest.raises(CollisionSafetyError):
        guard.command_joint_positions([0.1])
    backend.command_joint_positions.assert_not_called()
    assert guard.active_fault.code == "predicted_collision"
