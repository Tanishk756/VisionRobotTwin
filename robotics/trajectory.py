"""Trajectory Generation for Robotic Manipulators.

Supports:
1. Joint-Space Quintic Polynomial Interpolation (C2 continuous: zero endpoint velocity/acceleration).
2. Cartesian SE(3) Trajectories (quintic position interpolation + Quaternion SLERP).
3. Trajectory execution state tracking and progress metrics.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple, Union
import numpy as np

from utils.logger import get_logger

logger = get_logger("Robotics.Trajectory")


@dataclass(frozen=True)
class TrajectorySample:
    """Represents a sample along a joint-space trajectory."""
    time: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


@dataclass(frozen=True)
class CartesianTrajectorySample:
    """Represents a sample along a Cartesian SE(3) trajectory."""
    time: float
    position: np.ndarray
    orientation: np.ndarray  # [x, y, z, w]
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


def slerp_quaternion(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical Linear Interpolation (SLERP) between two unit quaternions [x, y, z, w].
    
    Args:
        q0: Start quaternion.
        q1: Goal quaternion.
        t: Interpolation factor in [0, 1].
        
    Returns:
        Interpolated unit quaternion [x, y, z, w].
    """
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)

    dot = np.dot(q0, q1)

    # Take the shortest path on the 4D hypersphere
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    # If quaternions are extremely close, use linear interpolation to avoid division by zero
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * t

    q_perp = q1 - q0 * dot
    q_perp = q_perp / np.linalg.norm(q_perp)

    result = q0 * np.cos(theta) + q_perp * np.sin(theta)
    return result / np.linalg.norm(result)


class JointQuinticTrajectory:
    """Smooth joint-space quintic polynomial trajectory with zero endpoint derivatives."""

    def __init__(
        self,
        q_start: Union[np.ndarray, List[float]],
        q_goal: Union[np.ndarray, List[float]],
        duration: float = 2.0,
    ):
        """Initializes quintic trajectory from q_start to q_goal over duration T."""
        if duration <= 0:
            raise ValueError(f"Trajectory duration must be positive, got {duration}")

        self.q_start = np.asarray(q_start, dtype=np.float64).flatten()
        self.q_goal = np.asarray(q_goal, dtype=np.float64).flatten()
        self.duration = float(duration)
        self.dof = len(self.q_start)

        if len(self.q_goal) != self.dof:
            raise ValueError(f"Dimension mismatch: start DoF {self.dof} != goal DoF {len(self.q_goal)}")

        # Quintic coefficients: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # Boundary conditions:
        # q(0) = q_0, q(T) = q_1
        # dq(0) = 0, dq(T) = 0
        # ddq(0) = 0, ddq(T) = 0
        T = self.duration
        self.a0 = self.q_start
        self.a1 = np.zeros(self.dof)
        self.a2 = np.zeros(self.dof)
        self.a3 = 10.0 * (self.q_goal - self.q_start) / (T ** 3)
        self.a4 = -15.0 * (self.q_goal - self.q_start) / (T ** 4)
        self.a5 = 6.0 * (self.q_goal - self.q_start) / (T ** 5)

    def evaluate(self, t: float) -> TrajectorySample:
        """Evaluates position, velocity, and acceleration at time t."""
        t_clamped = np.clip(t, 0.0, self.duration)

        pos = (
            self.a0
            + self.a1 * t_clamped
            + self.a2 * (t_clamped ** 2)
            + self.a3 * (t_clamped ** 3)
            + self.a4 * (t_clamped ** 4)
            + self.a5 * (t_clamped ** 5)
        )

        vel = (
            self.a1
            + 2.0 * self.a2 * t_clamped
            + 3.0 * self.a3 * (t_clamped ** 2)
            + 4.0 * self.a4 * (t_clamped ** 3)
            + 5.0 * self.a5 * (t_clamped ** 4)
        )

        acc = (
            2.0 * self.a2
            + 6.0 * self.a3 * t_clamped
            + 12.0 * self.a4 * (t_clamped ** 2)
            + 20.0 * self.a5 * (t_clamped ** 3)
        )

        # If t >= duration, velocities and accelerations are zero
        if t >= self.duration:
            vel = np.zeros(self.dof)
            acc = np.zeros(self.dof)
        elif t <= 0.0:
            vel = np.zeros(self.dof)
            acc = np.zeros(self.dof)

        return TrajectorySample(time=float(t), position=pos, velocity=vel, acceleration=acc)


class PiecewiseJointTrajectory:
    """Multi-segment quintic trajectory through a sequence of waypoints."""

    def __init__(self, waypoints: Sequence[np.ndarray], duration: float = 2.0):
        if len(waypoints) < 2:
            raise ValueError("Piecewise trajectory requires at least 2 waypoints")
        self.waypoints = [np.asarray(w, dtype=np.float64).flatten() for w in waypoints]
        self.duration = float(duration)
        self.num_segments = len(self.waypoints) - 1

        seg_lengths = [
            float(np.linalg.norm(self.waypoints[i + 1] - self.waypoints[i]))
            for i in range(self.num_segments)
        ]
        total_len = sum(seg_lengths)
        if total_len < 1e-6:
            seg_durations = [self.duration / self.num_segments] * self.num_segments
        else:
            seg_durations = [max(0.01, self.duration * (l / total_len)) for l in seg_lengths]
            scale = self.duration / sum(seg_durations)
            seg_durations = [d * scale for d in seg_durations]

        self.segments: List[JointQuinticTrajectory] = []
        self.segment_times = [0.0]
        curr_t = 0.0
        for i in range(self.num_segments):
            seg_traj = JointQuinticTrajectory(
                self.waypoints[i], self.waypoints[i + 1], duration=seg_durations[i]
            )
            self.segments.append(seg_traj)
            curr_t += seg_durations[i]
            self.segment_times.append(curr_t)

    def evaluate(self, t: float) -> TrajectorySample:
        """Evaluates piecewise trajectory sample at time t."""
        t_clamped = np.clip(t, 0.0, self.duration)
        for i in range(self.num_segments):
            if t_clamped <= self.segment_times[i + 1] or i == self.num_segments - 1:
                t_local = t_clamped - self.segment_times[i]
                return self.segments[i].evaluate(t_local)
        return self.segments[-1].evaluate(self.segments[-1].duration)


class CartesianSE3Trajectory:
    """Cartesian SE(3) trajectory using quintic position and quaternion SLERP."""

    def __init__(
        self,
        p_start: Union[np.ndarray, List[float]],
        q_start: Union[np.ndarray, List[float]],
        p_goal: Union[np.ndarray, List[float]],
        q_goal: Union[np.ndarray, List[float]],
        duration: float = 2.0,
    ):
        if duration <= 0:
            raise ValueError(f"Trajectory duration must be positive, got {duration}")

        self.p_start = np.asarray(p_start, dtype=np.float64).flatten()
        self.q_start = np.asarray(q_start, dtype=np.float64).flatten()
        self.p_goal = np.asarray(p_goal, dtype=np.float64).flatten()
        self.q_goal = np.asarray(q_goal, dtype=np.float64).flatten()
        self.duration = float(duration)

        self.pos_traj = JointQuinticTrajectory(self.p_start, self.p_goal, duration=self.duration)

    def evaluate(self, t: float) -> CartesianTrajectorySample:
        """Evaluates Cartesian position, quaternion orientation, and spatial velocities."""
        t_clamped = np.clip(t, 0.0, self.duration)
        alpha = t_clamped / self.duration

        pos_sample = self.pos_traj.evaluate(t)
        orn_interp = slerp_quaternion(self.q_start, self.q_goal, alpha)

        # Angular velocity approximation
        ang_vel = np.zeros(3)

        return CartesianTrajectorySample(
            time=float(t),
            position=pos_sample.position,
            orientation=orn_interp,
            linear_velocity=pos_sample.velocity,
            angular_velocity=ang_vel,
        )


class TrajectoryExecutor:
    """Manages trajectory playback state, progression, and timing."""

    def __init__(self, trajectory: Union[JointQuinticTrajectory, CartesianSE3Trajectory]):
        self.trajectory = trajectory
        self.current_time = 0.0
        self.is_complete = False
        self.is_aborted = False

    def reset(self) -> None:
        """Resets executor playback time to beginning."""
        self.current_time = 0.0
        self.is_complete = False
        self.is_aborted = False

    def abort(self) -> None:
        """Aborts active trajectory playback."""
        self.is_aborted = True

    @property
    def progress_pct(self) -> float:
        """Returns playback progress percentage in [0.0, 100.0]."""
        if self.trajectory.duration <= 0:
            return 100.0
        return float(np.clip((self.current_time / self.trajectory.duration) * 100.0, 0.0, 100.0))

    def step(self, dt: float) -> Tuple[Union[TrajectorySample, CartesianTrajectorySample], bool, float]:
        """Advances playback by dt and returns current trajectory sample.
        
        Returns:
            (sample, is_complete, progress_pct)
        """
        if self.is_aborted:
            sample = self.trajectory.evaluate(self.current_time)
            return sample, True, self.progress_pct

        self.current_time += dt
        if self.current_time >= self.trajectory.duration:
            self.current_time = self.trajectory.duration
            self.is_complete = True

        sample = self.trajectory.evaluate(self.current_time)
        return sample, self.is_complete, self.progress_pct
