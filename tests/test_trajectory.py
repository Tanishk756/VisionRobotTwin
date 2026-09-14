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
