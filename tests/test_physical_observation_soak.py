"""Motionless observation soak session tests and zero-command audits for Phase B4.1."""

from dataclasses import dataclass
import math
from typing import Callable, List, Optional, Tuple

import pytest

from robotics.backends.base import (
    BackendHealthStatus,
    ReadOnlyBackendError,
    TimestampedJointState,
)
from robotics.backends.physical_identity import (
    PhysicalRobotIdentity,
    PhysicalSafetyStatus,
    SafetySignalState,
)
from robotics.backends.physical_state_backend import PhysicalRobotStateBackend
from robotics.robot_model import (
    JointMotionType,
    JointRole,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)
from tests.mocks.mock_physical_adapter import MockPhysicalTelemetryAdapter


@dataclass(frozen=True)
class PhysicalObservationSummary:
    """Consolidated metrics from a motionless physical observation session."""

    total_samples: int
    first_timestamp_s: Optional[float]
    last_timestamp_s: Optional[float]
    max_receive_gap_s: float
    mean_interval_s: float
    stale_incidents: int
    identity_change_incidents: int
    model_mismatch_incidents: int
    safety_transitions: int
    backend_error_incidents: int
    command_side_effects_detected: int


class PhysicalObservationSession:
    """Manages deterministic motionless observation data collection over a configured interval."""

    def __init__(
        self,
        backend: PhysicalRobotStateBackend,
        physical_state_timeout_s: float = 0.5,
    ) -> None:
        self._backend = backend
        self._timeout_s = physical_state_timeout_s

    def run_discrete(
        self,
        sample_timestamps_s: List[float],
        time_source_setter: Optional[Callable[[float], None]] = None,
    ) -> PhysicalObservationSummary:
        """Executes observation collection over a pre-scheduled list of discrete evaluation timestamps."""
        total_samples = 0
        first_t: Optional[float] = None
        last_t: Optional[float] = None
        max_gap = 0.0
        interval_sum = 0.0
        intervals_count = 0
        stale_count = 0
        ident_change_count = 0
        model_mismatch_count = 0
        safety_transition_count = 0
        error_count = 0

        last_ident: Optional[PhysicalRobotIdentity] = None
        last_safety: Optional[PhysicalSafetyStatus] = None

        for t_eval in sample_timestamps_s:
            if time_source_setter is not None:
                time_source_setter(t_eval)

            if first_t is None:
                first_t = t_eval
            if last_t is not None:
                gap = t_eval - last_t
                if gap > max_gap:
                    max_gap = gap
                interval_sum += gap
                intervals_count += 1
            last_t = t_eval

            # Read joint state
            try:
                state = self._backend.get_joint_state()
                total_samples += 1
            except Exception:
                error_count += 1
                stale_count += 1

            # Read identity
            try:
                ident = self._backend.read_identity()
                if last_ident is not None and ident != last_ident:
                    ident_change_count += 1
                last_ident = ident
            except Exception:
                error_count += 1

            # Read safety status
            try:
                safety = self._backend.read_safety_status()
                if last_safety is not None and not safety.signals_equal(last_safety):
                    safety_transition_count += 1
                last_safety = safety
            except Exception:
                error_count += 1

        mean_interval = (interval_sum / intervals_count) if intervals_count > 0 else 0.0
        cmd_side_effects = self._backend._provider.command_side_effect_count

        return PhysicalObservationSummary(
            total_samples=total_samples,
            first_timestamp_s=first_t,
            last_timestamp_s=last_t,
            max_receive_gap_s=max_gap,
            mean_interval_s=mean_interval,
            stale_incidents=stale_count,
            identity_change_incidents=ident_change_count,
            model_mismatch_incidents=model_mismatch_count,
            safety_transitions=safety_transition_count,
            backend_error_incidents=error_count,
            command_side_effects_detected=cmd_side_effects,
        )


def _make_test_robot_model() -> ResolvedRobotModel:
    joints = tuple(
        ResolvedJointMetadata(
            model_index=i,
            canonical_index=i,
            name=f"joint{i+1}",
            role=JointRole.ARM,
            motion_type=JointMotionType.REVOLUTE,
            lower_limit=-3.14,
            upper_limit=3.14,
            max_force=50.0,
            max_velocity=2.0,
            link_name=f"link{i+1}",
        )
        for i in range(2)
    )
    return ResolvedRobotModel(
        robot_id="test_robot",
        display_name="TestRobot",
        all_joints=joints,
        arm_joints=joints,
        gripper_joints=(),
        ee_link_name="link2",
    )


class TestPhysicalObservationSoak:
    """Tests for motionless observation soak collection and zero-command audit."""

    def test_nominal_stable_observation_stream(self):
        current_time = [100.0]
        adapter = MockPhysicalTelemetryAdapter.create_nominal(now_s=100.0)
        model = _make_test_robot_model()
        backend = PhysicalRobotStateBackend(
            adapter,
            model,
            state_timeout_s=0.5,
            time_source=lambda: current_time[0],
        )
        backend.connect()

        session = PhysicalObservationSession(backend, physical_state_timeout_s=0.5)

        # Schedule 50 samples at 20ms intervals (100 Hz simulation)
        timestamps = [100.0 + i * 0.02 for i in range(50)]

        def advance_time(t: float):
            current_time[0] = t
            # Adapter receives fresh state at each evaluation
            adapter.set_joint_state(
                TimestampedJointState(
                    source_timestamp_s=t,
                    receive_timestamp_s=t,
                    joint_names=("joint1", "joint2"),
                    positions=(0.0, 0.0),
                    sequence_id=1,
                )
            )

        summary = session.run_discrete(timestamps, time_source_setter=advance_time)

        assert summary.total_samples == 50
        assert summary.max_receive_gap_s == pytest.approx(0.02)
        assert summary.stale_incidents == 0
        assert summary.identity_change_incidents == 0
        assert summary.safety_transitions == 0
        assert summary.backend_error_incidents == 0
        assert summary.command_side_effects_detected == 0

    def test_excessive_gap_detected_and_accounted(self):
        current_time = [100.0]
        adapter = MockPhysicalTelemetryAdapter.create_nominal(now_s=100.0)
        model = _make_test_robot_model()
        backend = PhysicalRobotStateBackend(
            adapter,
            model,
            state_timeout_s=0.5,
            time_source=lambda: current_time[0],
        )
        backend.connect()

        session = PhysicalObservationSession(backend, physical_state_timeout_s=0.5)

        # Jump from 100.0 to 101.0 (1.0s gap > 0.5s timeout) without updating adapter state
        timestamps = [100.0, 101.0]

        def advance_time(t: float):
            current_time[0] = t

        summary = session.run_discrete(timestamps, time_source_setter=advance_time)

        assert summary.max_receive_gap_s == pytest.approx(1.0)
        assert summary.stale_incidents == 1  # Second sample failed staleness
        assert summary.command_side_effects_detected == 0

    def test_safety_status_transition_detected(self):
        current_time = [100.0]
        adapter = MockPhysicalTelemetryAdapter.create_nominal(now_s=100.0)
        model = _make_test_robot_model()
        backend = PhysicalRobotStateBackend(
            adapter,
            model,
            state_timeout_s=0.5,
            time_source=lambda: current_time[0],
        )
        backend.connect()

        session = PhysicalObservationSession(backend, physical_state_timeout_s=0.5)
        timestamps = [100.0, 100.05, 100.10]

        def advance_time(t: float):
            current_time[0] = t
            adapter.set_joint_state(
                TimestampedJointState(
                    source_timestamp_s=t,
                    receive_timestamp_s=t,
                    joint_names=("joint1", "joint2"),
                    positions=(0.0, 0.0),
                    sequence_id=1,
                )
            )
            if t >= 100.05:
                # Transition safety status to faulted
                faulted = PhysicalSafetyStatus(
                    communication_healthy=SafetySignalState.SAFE,
                    vendor_fault_clear=SafetySignalState.UNSAFE,
                    protective_stop_clear=SafetySignalState.SAFE,
                    emergency_stop_clear=SafetySignalState.SAFE,
                    external_control_ready=SafetySignalState.SAFE,
                    drives_state_safe_or_known=SafetySignalState.SAFE,
                    operational_mode_safe=SafetySignalState.SAFE,
                    brakes_state_safe_or_known=SafetySignalState.SAFE,
                    receive_timestamp_s=t,
                )
                adapter.set_safety_status(faulted)

        summary = session.run_discrete(timestamps, time_source_setter=advance_time)

        assert summary.safety_transitions == 1
        assert summary.command_side_effects_detected == 0
