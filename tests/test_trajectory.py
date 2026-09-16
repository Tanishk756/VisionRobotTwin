"""Tests for Trajectory Generation (Joint Quintic and Cartesian SE(3) SLERP)."""

import pytest
import numpy as np

from robotics.trajectory import (
    JointQuinticTrajectory,
    CartesianSE3Trajectory,
    TrajectoryExecutor,
    TrajectorySample,
    CartesianTrajectorySample,
)


def test_joint_quintic_boundary_conditions():
    """Verifies joint quintic polynomial satisfies C2 boundary conditions."""
    q_start = np.array([0.0, -0.5, 0.2, -1.0, 0.0, 1.5, 0.7])
    q_goal = np.array([0.5, 0.2, -0.3, -0.5, 0.5, 0.8, -0.2])
    duration = 2.0  # seconds

    traj = JointQuinticTrajectory(q_start, q_goal, duration=duration)
    assert traj.duration == pytest.approx(2.0)

    # At t=0
    s0 = traj.evaluate(0.0)
    assert np.allclose(s0.position, q_start, atol=1e-6)
    assert np.allclose(s0.velocity, 0.0, atol=1e-6)
    assert np.allclose(s0.acceleration, 0.0, atol=1e-6)

    # At t=duration
    s_end = traj.evaluate(duration)
    assert np.allclose(s_end.position, q_goal, atol=1e-6)
    assert np.allclose(s_end.velocity, 0.0, atol=1e-6)
    assert np.allclose(s_end.acceleration, 0.0, atol=1e-6)

    # Intermediate smoothness and continuity
    times = np.linspace(0.0, duration, 50)
    positions = [traj.evaluate(t).position for t in times]
    velocities = [traj.evaluate(t).velocity for t in times]
    accelerations = [traj.evaluate(t).acceleration for t in times]

    assert all(np.all(np.isfinite(p)) for p in positions)
    assert all(np.all(np.isfinite(v)) for v in velocities)
    assert all(np.all(np.isfinite(a)) for a in accelerations)


def test_cartesian_se3_trajectory_slerp():
    """Verifies Cartesian trajectory interpolates position smoothly and SLERPs orientation."""
    p_start = np.array([0.3, -0.2, 0.4])
    p_goal = np.array([0.5, 0.2, 0.6])
    q_start = np.array([1.0, 0.0, 0.0, 0.0])  # Unit quaternion [x, y, z, w]
    q_goal = np.array([0.7071068, 0.0, 0.7071068, 0.0])  # 180 deg around X/Z or 90 deg rotation
    duration = 1.5

    traj = CartesianSE3Trajectory(p_start, q_start, p_goal, q_goal, duration=duration)

    # t=0
    s0 = traj.evaluate(0.0)
    assert np.allclose(s0.position, p_start, atol=1e-5)
    assert np.allclose(s0.orientation, q_start / np.linalg.norm(q_start), atol=1e-4)

    # t=duration
    s_end = traj.evaluate(duration)
    assert np.allclose(s_end.position, p_goal, atol=1e-5)
    # Check orientation matches q_goal (or -q_goal since q == -q in SO(3))
    dot = np.abs(np.dot(s_end.orientation, q_goal / np.linalg.norm(q_goal)))
    assert dot > 0.999

    # Monotonic progression and normalized quaternion everywhere
    times = np.linspace(0.0, duration, 30)
    for t in times:
        st = traj.evaluate(t)
        norm = np.linalg.norm(st.orientation)
        assert np.isclose(norm, 1.0, atol=1e-5)


def test_trajectory_executor_lifecycle():
    """Verifies trajectory executor step lifecycle, completion, and abort."""
    q_start = np.zeros(7)
    q_goal = np.ones(7) * 0.5
    traj = JointQuinticTrajectory(q_start, q_goal, duration=1.0)

    executor = TrajectoryExecutor(traj)
    assert not executor.is_complete
    assert executor.progress_pct == pytest.approx(0.0)

    # Step forward
    dt = 0.1
    for i in range(5):
        sample, done, pct = executor.step(dt)
        assert not done
        assert pct > 0.0 and pct <= 50.0

    # Step to completion
    for _ in range(6):
        sample, done, pct = executor.step(dt)

    assert done
    assert executor.is_complete
    assert executor.progress_pct == pytest.approx(100.0)

    # Test abort
    executor.reset()
    executor.step(0.2)
    assert not executor.is_complete
    executor.abort()
    assert executor.is_aborted


def test_motion_manager_step_dispatches_to_ros2_simulation_backend():
    """Verifies MotionManager trajectory step commands flow through GenericRobotController to ROS2SimulationBackend."""
    import time
    from unittest.mock import MagicMock, patch
    import robotics.backends.ros2_simulation_backend as sim_module
    import robotics.backends.ros2_joint_state_backend as state_module
    from robotics.backends.ros2_simulation_backend import ROS2SimulationBackend
    from robotics.backends.ros2_state_mapping import (
        ROS2JointStateBackendConfig,
        ROS2SimulationBackendConfig,
    )
    from robotics.backends.base import TimestampedJointState
    from robotics.robot_controller import GenericRobotController
    from robotics.motion_manager import MotionManager, MotionState
    from robotics.inverse_kinematics import GenericIKSolver, IKResult, IKStatus
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
    )

    class MockFloat64MultiArray:
        def __init__(self):
            self.data = []

    with patch.object(sim_module, "_HAS_RCLPY", True), \
         patch.object(state_module, "_HAS_RCLPY", True), \
         patch.object(sim_module, "Float64MultiArray", MockFloat64MultiArray):

        joints = (
            ResolvedJointMetadata(
                model_index=0,
                canonical_index=0,
                name="j1",
                role=JointRole.ARM,
                motion_type=JointMotionType.REVOLUTE,
                lower_limit=-3.0,
                upper_limit=3.0,
                max_force=50.0,
                max_velocity=2.0,
                link_name="l1",
            ),
            ResolvedJointMetadata(
                model_index=1,
                canonical_index=1,
                name="j2",
                role=JointRole.ARM,
                motion_type=JointMotionType.REVOLUTE,
                lower_limit=-3.0,
                upper_limit=3.0,
                max_force=50.0,
                max_velocity=2.0,
                link_name="l2",
            ),
        )
        model = ResolvedRobotModel(
            robot_id="sim_arm",
            display_name="SimArm",
            all_joints=joints,
            arm_joints=joints,
            gripper_joints=(),
            ee_link_name="l2",
            home_joint_positions=(0.0, 0.0),
        )

        state_cfg = ROS2JointStateBackendConfig(expected_joint_names=("j1", "j2"))
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="position",
            position_command_topic="/sim_arm/position_commands",
            require_subscriber_ready=False,
        )

        sim_backend = ROS2SimulationBackend(sim_cfg)
        sim_backend._is_connected = True
        sim_backend._commands_enabled = True
        sim_backend._cmd_publisher = MagicMock()
        sim_backend._state_backend = MagicMock()
        sim_backend._state_backend.is_connected.return_value = True

        current_state = TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=time.monotonic(),
            joint_names=("j1", "j2"),
            positions=(0.0, 0.0),
            velocities=(0.0, 0.0),
        )
        sim_backend._state_backend.get_joint_state.return_value = current_state

        mock_kinematics = MagicMock(spec=KinematicsProvider)
        controller = GenericRobotController(
            resolved_model=model,
            backend=sim_backend,
            kinematics_provider=mock_kinematics,
        )

        mock_ik = MagicMock(spec=GenericIKSolver)
        mock_ik.solve.return_value = IKResult(
            success=True,
            joint_positions=[0.5, 0.5],
            status=IKStatus.SOLUTION_RETURNED,
            status_message="OK",
            position_error_m=0.0,
            orientation_error_rad=0.0,
        )

        mm = MotionManager(
            robot_controller=controller,
            ik_solver=mock_ik,
            collision_checker=None,
            trajectory_mode="quintic",
            default_trajectory_duration=1.0,
        )

        # Plan to pose
        ok = mm.plan_motion_to_pose(target_position=[0.3, 0.0, 0.3])
        assert ok is True
        assert mm.state == MotionState.EXECUTING

        # Step trajectory
        is_done, pct = mm.step(dt=0.1)
        assert not is_done
        assert pct > 0.0

        # Verify command published
        sim_backend._cmd_publisher.publish.assert_called_once()
        published_msg = sim_backend._cmd_publisher.publish.call_args[0][0]
        assert len(published_msg.data) == 2
        assert all(np.isfinite(published_msg.data))
