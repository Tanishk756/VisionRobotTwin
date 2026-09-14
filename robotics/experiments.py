"""Task-Space Experiment Suite for Multi-Robot Kinematic & Control Benchmarking.

Provides:
1. Deterministic task-space trajectory generators (Line, Circle, Figure-Eight, Waypoint Box, SE(3) Sweep).
2. Shared SE(3) feasibility preflight across manipulators.
3. Multi-robot & multi-controller trial execution in PyBullet DIRECT physics.
4. Comprehensive trajectory tracking, orientation, joint motion, manipulability, safety, and completion metrics.
5. Dedicated obstacle reaching and motion planning experiment.
"""

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum
import math
import platform
import subprocess
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R

from config.settings import AppConfig, RobotConfig
from robotics.coordinate_transform import (
    compute_angular_distance,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)
from robotics.differential_ik import ResolvedRateController
from robotics.inverse_kinematics import GenericIKSolver
from robotics.kinematics import (
    compute_fk_at_configuration,
    compute_jacobian,
    compute_manipulability,
)
from robotics.robot_controller import GenericRobotController
from robotics.robot_registry import get_robot_registry
from robotics.trajectory import slerp_quaternion
from utils.logger import get_logger

logger = get_logger("Robotics.Experiments")


# =============================================================================
# 1. TASK-SPACE TRAJECTORY DEFINITIONS & GENERATORS
# =============================================================================

@dataclass
class TaskspaceTrajectory:
    """Represents a continuous, deterministic task-space SE(3) reference trajectory."""
    name: str
    description: str
    duration_s: float
    physics_hz: int
    timestamps: np.ndarray          # Shape: (N,)
    positions: np.ndarray           # Shape: (N, 3)
    orientations: np.ndarray        # Shape: (N, 4) in [x, y, z, w]
    linear_velocities: np.ndarray   # Shape: (N, 3)
    angular_velocities: np.ndarray  # Shape: (N, 3)
    is_se3_sweep: bool = False

    @property
    def num_samples(self) -> int:
        return len(self.timestamps)

    @property
    def dt(self) -> float:
        return 1.0 / self.physics_hz if self.physics_hz > 0 else 1.0 / 240.0


def _quintic_time_scaling(t: float, T: float) -> Tuple[float, float, float]:
    """Computes normalized quintic polynomial time scaling s(t), s_dot(t), s_ddot(t)."""
    if T <= 0.0:
        return 1.0, 0.0, 0.0
    tau = np.clip(t / T, 0.0, 1.0)
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5
    s_dot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / T
    s_ddot = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (T * T)
    return float(s), float(s_dot), float(s_ddot)


def generate_line_trajectory(
    center_pos: Sequence[float] = (0.48, 0.0, 0.35),
    length_m: float = 0.10,
    orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    duration_s: float = 2.0,
    physics_hz: int = 240,
) -> TaskspaceTrajectory:
    """Generates a smooth 10 cm horizontal straight-line translation along Y axis with fixed orientation."""
    num_samples = int(round(duration_s * physics_hz)) + 1
    timestamps = np.linspace(0.0, duration_s, num_samples, dtype=np.float64)
    
    start_pos = np.array(center_pos, dtype=np.float64) + np.array([0.0, -length_m / 2.0, 0.0])
    end_pos = np.array(center_pos, dtype=np.float64) + np.array([0.0, length_m / 2.0, 0.0])
    fixed_orn = np.array(orientation, dtype=np.float64)
    fixed_orn = fixed_orn / np.linalg.norm(fixed_orn)

    positions = np.zeros((num_samples, 3), dtype=np.float64)
    orientations = np.zeros((num_samples, 4), dtype=np.float64)
    lin_vels = np.zeros((num_samples, 3), dtype=np.float64)
    ang_vels = np.zeros((num_samples, 3), dtype=np.float64)

    delta_pos = end_pos - start_pos
    for i, t in enumerate(timestamps):
        s, s_dot, _ = _quintic_time_scaling(t, duration_s)
        positions[i] = start_pos + s * delta_pos
        orientations[i] = fixed_orn
        lin_vels[i] = s_dot * delta_pos
        ang_vels[i] = np.zeros(3)

    return TaskspaceTrajectory(
        name="LINE",
        description="10 cm straight horizontal translation along Y axis with fixed orientation.",
        duration_s=duration_s,
        physics_hz=physics_hz,
        timestamps=timestamps,
        positions=positions,
        orientations=orientations,
        linear_velocities=lin_vels,
        angular_velocities=ang_vels,
        is_se3_sweep=False,
    )


def generate_circle_trajectory(
    center_pos: Sequence[float] = (0.48, 0.0, 0.35),
    radius_m: float = 0.05,
    orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    duration_s: float = 3.0,
    physics_hz: int = 240,
) -> TaskspaceTrajectory:
    """Generates a smooth horizontal circle trajectory in the XY plane with fixed orientation."""
    num_samples = int(round(duration_s * physics_hz)) + 1
    timestamps = np.linspace(0.0, duration_s, num_samples, dtype=np.float64)
    
    center = np.array(center_pos, dtype=np.float64)
    fixed_orn = np.array(orientation, dtype=np.float64)
    fixed_orn = fixed_orn / np.linalg.norm(fixed_orn)

    positions = np.zeros((num_samples, 3), dtype=np.float64)
    orientations = np.zeros((num_samples, 4), dtype=np.float64)
    lin_vels = np.zeros((num_samples, 3), dtype=np.float64)
    ang_vels = np.zeros((num_samples, 3), dtype=np.float64)

    for i, t in enumerate(timestamps):
        s, s_dot, _ = _quintic_time_scaling(t, duration_s)
        angle = 2.0 * np.pi * s
        angle_dot = 2.0 * np.pi * s_dot

        positions[i, 0] = center[0] + radius_m * np.cos(angle)
        positions[i, 1] = center[1] + radius_m * np.sin(angle)
        positions[i, 2] = center[2]

        orientations[i] = fixed_orn

        lin_vels[i, 0] = -radius_m * np.sin(angle) * angle_dot
        lin_vels[i, 1] = radius_m * np.cos(angle) * angle_dot
        lin_vels[i, 2] = 0.0

        ang_vels[i] = np.zeros(3)

    return TaskspaceTrajectory(
        name="CIRCLE",
        description="5 cm radius planar circle in XY plane with fixed orientation.",
        duration_s=duration_s,
        physics_hz=physics_hz,
        timestamps=timestamps,
        positions=positions,
        orientations=orientations,
        linear_velocities=lin_vels,
        angular_velocities=ang_vels,
        is_se3_sweep=False,
    )


SETTLE_TOLERANCE_MM: float = 5.0
SUCCESS_POSITION_TOLERANCE_MM: float = 10.0
SUCCESS_ORIENTATION_TOLERANCE_DEG: float = 10.0
PREFLIGHT_POSITION_RESIDUAL_THRESHOLD_MM: float = 25.0
PREFLIGHT_ORIENTATION_RESIDUAL_THRESHOLD_DEG: float = 10.0
PLANNING_EXECUTION_POSITION_TOLERANCE_MM: float = 25.0


def generate_figure_eight_trajectory(
    center_pos: Sequence[float] = (0.48, 0.0, 0.35),
    amplitude_x_m: float = 0.04,
    amplitude_y_m: float = 0.03,
    orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    duration_s: float = 4.0,
    physics_hz: int = 240,
) -> TaskspaceTrajectory:
    """Generates a smooth Cartesian lemniscate (figure-eight) trajectory in the XY plane.

    X peak amplitude: amplitude_x_m (default: 0.04 m / 4 cm)
    Y peak amplitude: amplitude_y_m (default: 0.03 m / 3 cm)
    """
    num_samples = int(round(duration_s * physics_hz)) + 1
    timestamps = np.linspace(0.0, duration_s, num_samples, dtype=np.float64)
    
    center = np.array(center_pos, dtype=np.float64)
    fixed_orn = np.array(orientation, dtype=np.float64)
    fixed_orn = fixed_orn / np.linalg.norm(fixed_orn)

    positions = np.zeros((num_samples, 3), dtype=np.float64)
    orientations = np.zeros((num_samples, 4), dtype=np.float64)
    lin_vels = np.zeros((num_samples, 3), dtype=np.float64)
    ang_vels = np.zeros((num_samples, 3), dtype=np.float64)

    for i, t in enumerate(timestamps):
        s, s_dot, _ = _quintic_time_scaling(t, duration_s)
        phi = 2.0 * np.pi * s
        phi_dot = 2.0 * np.pi * s_dot

        positions[i, 0] = center[0] + amplitude_x_m * np.sin(phi)
        positions[i, 1] = center[1] + amplitude_y_m * np.sin(2.0 * phi)
        positions[i, 2] = center[2]

        orientations[i] = fixed_orn

        lin_vels[i, 0] = amplitude_x_m * np.cos(phi) * phi_dot
        lin_vels[i, 1] = amplitude_y_m * 2.0 * np.cos(2.0 * phi) * phi_dot
        lin_vels[i, 2] = 0.0

        ang_vels[i] = np.zeros(3)

    return TaskspaceTrajectory(
        name="FIGURE_EIGHT",
        description="Lemniscate figure-eight trajectory (X peak amplitude: 4 cm, Y peak amplitude: 3 cm).",
        duration_s=duration_s,
        physics_hz=physics_hz,
        timestamps=timestamps,
        positions=positions,
        orientations=orientations,
        linear_velocities=lin_vels,
        angular_velocities=ang_vels,
        is_se3_sweep=False,
    )


def generate_waypoint_box_trajectory(
    center_pos: Sequence[float] = (0.48, 0.0, 0.35),
    size_x_m: float = 0.06,
    size_y_m: float = 0.06,
    orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    duration_s: float = 4.0,
    physics_hz: int = 240,
) -> TaskspaceTrajectory:
    """Generates a smooth 4-corner closed box path in the XY plane."""
    num_samples = int(round(duration_s * physics_hz)) + 1
    timestamps = np.linspace(0.0, duration_s, num_samples, dtype=np.float64)
    
    center = np.array(center_pos, dtype=np.float64)
    fixed_orn = np.array(orientation, dtype=np.float64)
    fixed_orn = fixed_orn / np.linalg.norm(fixed_orn)

    hx = size_x_m / 2.0
    hy = size_y_m / 2.0
    corners = [
        center + np.array([-hx, -hy, 0.0]),
        center + np.array([ hx, -hy, 0.0]),
        center + np.array([ hx,  hy, 0.0]),
        center + np.array([-hx,  hy, 0.0]),
        center + np.array([-hx, -hy, 0.0]),
    ]

    positions = np.zeros((num_samples, 3), dtype=np.float64)
    orientations = np.zeros((num_samples, 4), dtype=np.float64)
    lin_vels = np.zeros((num_samples, 3), dtype=np.float64)
    ang_vels = np.zeros((num_samples, 3), dtype=np.float64)

    num_segments = len(corners) - 1
    seg_duration = duration_s / num_segments

    for i, t in enumerate(timestamps):
        seg_idx = min(int(t / seg_duration), num_segments - 1)
        seg_t = t - seg_idx * seg_duration
        s, s_dot, _ = _quintic_time_scaling(seg_t, seg_duration)

        p0 = corners[seg_idx]
        p1 = corners[seg_idx + 1]
        dp = p1 - p0

        positions[i] = p0 + s * dp
        orientations[i] = fixed_orn
        lin_vels[i] = s_dot * dp
        ang_vels[i] = np.zeros(3)

    return TaskspaceTrajectory(
        name="WAYPOINT_BOX",
        description="Smooth 4-corner closed box path (6 cm x 6 cm) in the XY plane.",
        duration_s=duration_s,
        physics_hz=physics_hz,
        timestamps=timestamps,
        positions=positions,
        orientations=orientations,
        linear_velocities=lin_vels,
        angular_velocities=ang_vels,
        is_se3_sweep=False,
    )


def generate_se3_orientation_sweep_trajectory(
    center_pos: Sequence[float] = (0.48, 0.0, 0.35),
    base_orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    max_roll_deg: float = 20.0,
    duration_s: float = 3.0,
    physics_hz: int = 240,
) -> TaskspaceTrajectory:
    """Generates an SE(3) trajectory combining 2 cm X harmonic translation with +/-20 deg X-axis roll rotation via quaternion SLERP."""
    num_samples = int(round(duration_s * physics_hz)) + 1
    timestamps = np.linspace(0.0, duration_s, num_samples, dtype=np.float64)
    
    center = np.array(center_pos, dtype=np.float64)
    r_base = R.from_quat(base_orientation)
    q_neutral = r_base.as_quat()
    q_pos = (r_base * R.from_euler("x", max_roll_deg, degrees=True)).as_quat()
    q_neg = (r_base * R.from_euler("x", -max_roll_deg, degrees=True)).as_quat()

    positions = np.zeros((num_samples, 3), dtype=np.float64)
    orientations = np.zeros((num_samples, 4), dtype=np.float64)
    lin_vels = np.zeros((num_samples, 3), dtype=np.float64)
    ang_vels = np.zeros((num_samples, 3), dtype=np.float64)

    # 3 phases: Neutral -> +Roll -> -Roll -> Neutral
    t1 = duration_s / 3.0
    t2 = 2.0 * duration_s / 3.0

    for i, t in enumerate(timestamps):
        s, s_dot, _ = _quintic_time_scaling(t, duration_s)
        # Gentle translation along X (2 cm harmonic amplitude)
        positions[i, 0] = center[0] + 0.02 * np.sin(2.0 * np.pi * s)
        positions[i, 1] = center[1]
        positions[i, 2] = center[2]
        lin_vels[i, 0] = 0.02 * 2.0 * np.pi * np.cos(2.0 * np.pi * s) * s_dot

        if t <= t1:
            phase_s, _, _ = _quintic_time_scaling(t, t1)
            orientations[i] = slerp_quaternion(q_neutral, q_pos, phase_s)
        elif t <= t2:
            phase_s, _, _ = _quintic_time_scaling(t - t1, t1)
            orientations[i] = slerp_quaternion(q_pos, q_neg, phase_s)
        else:
            phase_s, _, _ = _quintic_time_scaling(t - t2, duration_s - t2)
            orientations[i] = slerp_quaternion(q_neg, q_neutral, phase_s)

    # Compute numerical angular velocity
    for i in range(num_samples - 1):
        dt = timestamps[i + 1] - timestamps[i]
        if dt > 1e-6:
            r0 = R.from_quat(orientations[i])
            r1 = R.from_quat(orientations[i + 1])
            dr = r0.inv() * r1
            rotvec = dr.as_rotvec()
            ang_vels[i] = rotvec / dt
    if num_samples > 1:
        ang_vels[-1] = ang_vels[-2]

    return TaskspaceTrajectory(
        name="SE3_SWEEP",
        description=f"6-DoF trajectory with +/-{max_roll_deg} deg X-axis roll rotation via quaternion SLERP and 2 cm X harmonic translation.",
        duration_s=duration_s,
        physics_hz=physics_hz,
        timestamps=timestamps,
        positions=positions,
        orientations=orientations,
        linear_velocities=lin_vels,
        angular_velocities=ang_vels,
        is_se3_sweep=True,
    )


ALL_EXPERIMENT_NAMES: List[str] = [
    "line",
    "circle",
    "figure_eight",
    "waypoint_box",
    "se3_sweep",
]


@dataclass
class ExperimentDefinition:
    """Specification and default parameters for a task-space benchmark experiment."""
    name: str
    description: str
    generator_func: Any
    default_duration_s: float
    is_se3: bool = False

    def create_trajectory(self, duration_s: Optional[float] = None, physics_hz: int = 240) -> TaskspaceTrajectory:
        dur = duration_s if duration_s is not None else self.default_duration_s
        return self.generator_func(duration_s=dur, physics_hz=physics_hz)


EXPERIMENT_DEFINITIONS: Dict[str, ExperimentDefinition] = {
    "line": ExperimentDefinition(
        name="LINE",
        description="Straight 10 cm horizontal Cartesian translation with fixed orientation.",
        generator_func=generate_line_trajectory,
        default_duration_s=2.0,
        is_se3=False,
    ),
    "circle": ExperimentDefinition(
        name="CIRCLE",
        description="Continuous horizontal task-space circle (R = 5 cm) with fixed orientation.",
        generator_func=generate_circle_trajectory,
        default_duration_s=3.0,
        is_se3=False,
    ),
    "figure_eight": ExperimentDefinition(
        name="FIGURE_EIGHT",
        description="Lemniscate figure-eight trajectory (X peak amplitude: 4 cm, Y peak amplitude: 3 cm).",
        generator_func=generate_figure_eight_trajectory,
        default_duration_s=4.0,
        is_se3=False,
    ),
    "waypoint_box": ExperimentDefinition(
        name="WAYPOINT_BOX",
        description="Smooth 4-corner closed box path (6 cm x 6 cm) in the XY plane.",
        generator_func=generate_waypoint_box_trajectory,
        default_duration_s=4.0,
        is_se3=False,
    ),
    "se3_sweep": ExperimentDefinition(
        name="SE3_SWEEP",
        description="6-DoF trajectory with +/-20 deg X-axis roll rotation via quaternion SLERP and 2 cm X harmonic translation.",
        generator_func=generate_se3_orientation_sweep_trajectory,
        default_duration_s=3.0,
        is_se3=True,
    ),
}


def get_experiment_metadata(
    exp_name: str,
    duration_override_s: Optional[float] = None,
    physics_hz: int = 240,
    preflight_sample_stride: int = 1,
) -> Dict[str, Any]:
    """Single source of truth for experiment geometry, duration, and sample counts."""
    name_key = exp_name.lower()
    if name_key not in EXPERIMENT_DEFINITIONS:
        raise ValueError(f"Unknown experiment '{exp_name}'")
    exp_def = EXPERIMENT_DEFINITIONS[name_key]
    eff_dur = duration_override_s if duration_override_s is not None else exp_def.default_duration_s
    overridden = (duration_override_s is not None) and (duration_override_s != exp_def.default_duration_s)
    traj = exp_def.create_trajectory(duration_s=eff_dur, physics_hz=physics_hz)
    stride = max(1, preflight_sample_stride)
    sampled_indices = list(range(0, traj.num_samples, stride))
    if (traj.num_samples - 1) not in sampled_indices:
        sampled_indices.append(traj.num_samples - 1)

    if name_key == "line":
        geom_params = {"length_m": 0.10, "axis": "Y", "path_type": "Straight Cartesian Line"}
        orn_prof = "Fixed downward [1, 0, 0, 0]"
        rot_axis = "None (Fixed)"
    elif name_key == "circle":
        geom_params = {"radius_m": 0.05, "plane": "XY", "path_type": "Continuous Planar Circle"}
        orn_prof = "Fixed downward [1, 0, 0, 0]"
        rot_axis = "None (Fixed)"
    elif name_key == "figure_eight":
        geom_params = {"amplitude_x_m": 0.04, "amplitude_y_m": 0.03, "plane": "XY", "path_type": "Lemniscate (Figure-Eight)"}
        orn_prof = "Fixed downward [1, 0, 0, 0]"
        rot_axis = "None (Fixed)"
    elif name_key == "waypoint_box":
        geom_params = {"size_x_m": 0.06, "size_y_m": 0.06, "plane": "XY", "corners": 4, "path_type": "4-Corner Rectangular Route"}
        orn_prof = "Fixed downward [1, 0, 0, 0]"
        rot_axis = "None (Fixed)"
    elif name_key == "se3_sweep":
        geom_params = {"harmonic_translation_axis": "X", "harmonic_amplitude_m": 0.02, "rotation_axis": "X", "max_roll_deg": 20.0, "path_type": "6-DoF Roll SLERP + Harmonic Translation"}
        orn_prof = "Quaternion SLERP roll sweep (+/-20 deg around X-axis)"
        rot_axis = "X (Roll)"
    else:
        geom_params = {}
        orn_prof = "Custom"
        rot_axis = "N/A"

    return {
        "name": exp_def.name,
        "description": exp_def.description,
        "center_position": [0.48, 0.0, 0.35],
        "geometry_parameters": geom_params,
        "default_duration_s": exp_def.default_duration_s,
        "effective_duration_s": eff_dur,
        "duration_overridden": overridden,
        "orientation_profile": orn_prof,
        "rotation_axis": rot_axis,
        "physics_hz": physics_hz,
        "sample_count": traj.num_samples,
        "preflight_sample_stride": stride,
        "preflight_checked_samples": len(sampled_indices),
        "is_se3": exp_def.is_se3,
    }


def get_standard_experiment_trajectories(
    duration_scale: float = 1.0,
    physics_hz: int = 240,
) -> Dict[str, TaskspaceTrajectory]:
    """Returns the standardized suite of 5 task-space experiment trajectories."""
    return {
        "line": generate_line_trajectory(duration_s=2.0 * duration_scale, physics_hz=physics_hz),
        "circle": generate_circle_trajectory(duration_s=3.0 * duration_scale, physics_hz=physics_hz),
        "figure_eight": generate_figure_eight_trajectory(duration_s=4.0 * duration_scale, physics_hz=physics_hz),
        "waypoint_box": generate_waypoint_box_trajectory(duration_s=4.0 * duration_scale, physics_hz=physics_hz),
        "se3_sweep": generate_se3_orientation_sweep_trajectory(duration_s=3.0 * duration_scale, physics_hz=physics_hz),
    }


# =============================================================================
# 2. SHARED FEASIBILITY PREFLIGHT
# =============================================================================

@dataclass
class PreflightFeasibilityResult:
    """Outcome of dense shared SE(3) feasibility preflight verification."""
    feasible: bool
    status: str
    robots_tested: List[str]
    total_samples: int
    robot_max_position_residuals_mm: Dict[str, float]
    robot_max_orientation_residuals_deg: Dict[str, float]
    robot_feasibility: Dict[str, bool]
    failure_diagnostics: List[str] = field(default_factory=list)
    sample_stride: int = 1
    checked_samples: int = 0
    trajectory_total_samples: int = 0

    @property
    def shared_feasible(self) -> bool:
        return self.feasible

    @property
    def panda_feasible(self) -> bool:
        return self.robot_feasibility.get("panda", False)

    @property
    def kuka_feasible(self) -> bool:
        return self.robot_feasibility.get("kuka_iiwa", False)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["shared_feasible"] = self.shared_feasible
        d["panda_feasible"] = self.panda_feasible
        d["kuka_feasible"] = self.kuka_feasible
        d["max_position_residual_m_panda"] = self.robot_max_position_residuals_mm.get("panda", 0.0) / 1000.0
        d["max_position_residual_m_kuka"] = self.robot_max_position_residuals_mm.get("kuka_iiwa", 0.0) / 1000.0
        d["max_orientation_residual_deg_panda"] = self.robot_max_orientation_residuals_deg.get("panda", 0.0)
        d["max_orientation_residual_deg_kuka"] = self.robot_max_orientation_residuals_deg.get("kuka_iiwa", 0.0)
        d["trajectory_total_samples"] = self.trajectory_total_samples if self.trajectory_total_samples > 0 else self.total_samples
        d["preflight_sample_stride"] = self.sample_stride
        d["preflight_checked_samples"] = self.checked_samples if self.checked_samples > 0 else self.total_samples
        return d


def check_shared_feasibility(
    trajectory: Union[TaskspaceTrajectory, ExperimentDefinition, str],
    robots: Sequence[str] = ("panda", "kuka_iiwa"),
    position_tolerance_mm: float = PREFLIGHT_POSITION_RESIDUAL_THRESHOLD_MM,
    orientation_tolerance_deg: float = PREFLIGHT_ORIENTATION_RESIDUAL_THRESHOLD_DEG,
    sample_stride: int = 1,
    duration_override_s: Optional[float] = None,
    physics_hz: int = 240,
) -> PreflightFeasibilityResult:
    """Preflight verification ensuring both manipulators can feasibly solve all trajectory waypoints.

    Args:
        trajectory: The task-space reference trajectory (or definition / name) to validate.
        robots: List of robot model identifiers.
        position_tolerance_mm: Maximum permissible FK IK position residual in mm.
        orientation_tolerance_deg: Maximum permissible FK IK orientation residual in degrees.
        sample_stride: Step stride for sample evaluation to ensure thorough checks (default: 1 for publication).
        duration_override_s: Optional duration override in seconds.
        physics_hz: Simulation clock frequency.

    Returns:
        PreflightFeasibilityResult summarizing common feasibility across all robots.
    """
    if isinstance(trajectory, str):
        if trajectory.lower() in EXPERIMENT_DEFINITIONS:
            traj = EXPERIMENT_DEFINITIONS[trajectory.lower()].create_trajectory(duration_s=duration_override_s, physics_hz=physics_hz)
        else:
            raise ValueError(f"Unknown experiment '{trajectory}'")
    elif isinstance(trajectory, ExperimentDefinition):
        traj = trajectory.create_trajectory(duration_s=duration_override_s, physics_hz=physics_hz)
    else:
        traj = trajectory

    registry = get_robot_registry()
    robot_max_pos = {}
    robot_max_orn = {}
    robot_feasible = {}
    failure_notes = []

    stride = max(1, sample_stride)
    sampled_indices = list(range(0, traj.num_samples, stride))
    if (traj.num_samples - 1) not in sampled_indices:
        sampled_indices.append(traj.num_samples - 1)

    for rname in robots:
        cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        try:
            spec = registry.get_robot_spec(rname)
            rid = p.loadURDF(
                spec.urdf_path,
                basePosition=spec.base_position,
                baseOrientation=spec.base_orientation,
                useFixedBase=spec.fixed_base,
                physicsClientId=cid,
            )
            ctrl = GenericRobotController(cid, rid, spec)
            lows, highs, ranges, rests = ctrl.get_joint_limits()
            solver = GenericIKSolver(
                physics_client_id=cid,
                robot_id=rid,
                arm_joint_indices=ctrl.arm_joint_indices,
                lower_limits=lows,
                upper_limits=highs,
                joint_ranges=ranges,
                rest_poses=rests,
                end_effector_link_index=ctrl.ee_link_index,
                max_reach_m=spec.spherical_reach_m,
                min_reach_m=spec.min_reach_m,
                default_ee_orientation=spec.default_ee_orientation,
                max_residual_position_m=position_tolerance_mm / 1000.0,
                max_residual_orientation_rad=math.radians(orientation_tolerance_deg),
            )

            max_pos_mm = 0.0
            max_orn_deg = 0.0
            is_robot_ok = True

            for s_idx in sampled_indices:
                p_tgt = traj.positions[s_idx]
                o_tgt = traj.orientations[s_idx]
                res = solver.solve(p_tgt, o_tgt)
                pos_res_mm = (res.residual_position_m * 1000.0) if res.residual_position_m is not None else float("inf")
                orn_res_deg = math.degrees(res.residual_orientation_rad) if res.residual_orientation_rad is not None else float("inf")

                if pos_res_mm != float("inf"):
                    max_pos_mm = max(max_pos_mm, pos_res_mm)
                if orn_res_deg != float("inf"):
                    max_orn_deg = max(max_orn_deg, orn_res_deg)

                if not res.success or pos_res_mm > position_tolerance_mm or orn_res_deg > orientation_tolerance_deg:
                    is_robot_ok = False
                    failure_notes.append(
                        f"Robot '{rname}' failed sample #{s_idx} (t={traj.timestamps[s_idx]:.3f}s): "
                        f"pos_res={pos_res_mm:.2f}mm (limit {position_tolerance_mm}mm), "
                        f"orn_res={orn_res_deg:.2f}deg (limit {orientation_tolerance_deg}deg)"
                    )
                    break

            robot_max_pos[rname] = float(max_pos_mm)
            robot_max_orn[rname] = float(max_orn_deg)
            robot_feasible[rname] = is_robot_ok
        finally:
            p.disconnect(cid)

    overall_feasible = all(robot_feasible.values())
    status = "SHARED_FEASIBILITY_PASSED" if overall_feasible else "SHARED_FEASIBILITY_FAILED"

    return PreflightFeasibilityResult(
        feasible=overall_feasible,
        status=status,
        robots_tested=list(robots),
        total_samples=traj.num_samples,
        robot_max_position_residuals_mm=robot_max_pos,
        robot_max_orientation_residuals_deg=robot_max_orn,
        robot_feasibility=robot_feasible,
        failure_diagnostics=failure_notes,
        sample_stride=stride,
        checked_samples=len(sampled_indices),
        trajectory_total_samples=traj.num_samples,
    )


# =============================================================================
# 3. HIGH-FIDELITY METRICS ENGINE
# =============================================================================

@dataclass
class ExperimentMetrics:
    """Comprehensive performance and safety metrics for a task-space experiment trial."""
    # Position errors (mm)
    mean_position_error_mm: float
    rmse_position_error_mm: float
    median_position_error_mm: float
    p95_position_error_mm: float
    max_position_error_mm: float
    final_position_error_mm: float
    settled_final_position_error_mm: float

    # Orientation errors (deg)
    mean_orientation_error_deg: float
    rmse_orientation_error_deg: float
    p95_orientation_error_deg: float
    max_orientation_error_deg: float
    final_orientation_error_deg: float

    # Joint Motion
    total_joint_travel_rad: float
    max_joint_travel_rad: float
    rms_joint_velocity_rad_s: float
    peak_joint_velocity_rad_s: float
    rms_joint_accel_rad_s2: float
    peak_joint_accel_rad_s2: float

    # Kinematics & Manipulability
    mean_manipulability: float
    min_manipulability: float
    mean_sigma_min: float
    min_sigma_min: float
    max_condition_number: float
    singularity_warning_count: int
    singularity_time_pct: float

    # Safety & Completion
    self_collision_events: int
    env_collision_events: int
    joint_limit_events: int
    velocity_saturation_events: int
    settled_within_tolerance: bool
    time_to_final_goal_tolerance_s: float
    success: bool
    failure_reason: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def compute_experiment_metrics(
    time_arr: np.ndarray,
    des_pos: np.ndarray,
    act_pos: np.ndarray,
    des_orn: np.ndarray,
    act_orn: np.ndarray,
    joint_pos_arr: np.ndarray,
    joint_vel_arr: np.ndarray,
    manip_arr: np.ndarray,
    sigma_min_arr: np.ndarray,
    cond_arr: np.ndarray,
    near_sing_arr: np.ndarray,
    self_col_arr: np.ndarray,
    env_col_arr: np.ndarray,
    limit_viol_arr: np.ndarray,
    vel_sat_arr: np.ndarray,
    settle_tol_mm: float = SETTLE_TOLERANCE_MM,
    success_position_tolerance_mm: float = SUCCESS_POSITION_TOLERANCE_MM,
    success_orientation_tolerance_deg: float = SUCCESS_ORIENTATION_TOLERANCE_DEG,
    is_se3: bool = False,
) -> ExperimentMetrics:
    """Computes rigorous mathematical metrics from recorded trial timeseries."""
    N = len(time_arr)
    if N == 0:
        raise ValueError("Cannot compute metrics from empty timeseries")

    # 1. Position Errors (mm)
    pos_diffs = act_pos - des_pos
    pos_err_mm = np.linalg.norm(pos_diffs, axis=1) * 1000.0

    mean_pos_mm = float(np.mean(pos_err_mm))
    rmse_pos_mm = float(np.sqrt(np.mean(pos_err_mm ** 2)))
    median_pos_mm = float(np.median(pos_err_mm))
    p95_pos_mm = float(np.percentile(pos_err_mm, 95))
    max_pos_mm = float(np.max(pos_err_mm))
    final_pos_mm = float(pos_err_mm[-1])

    # 2. Orientation Errors (deg)
    orn_err_deg = np.zeros(N, dtype=np.float64)
    for i in range(N):
        orn_err_deg[i] = math.degrees(compute_angular_distance(des_orn[i], act_orn[i]))

    mean_orn_deg = float(np.mean(orn_err_deg))
    rmse_orn_deg = float(np.sqrt(np.mean(orn_err_deg ** 2)))
    p95_orn_deg = float(np.percentile(orn_err_deg, 95))
    max_orn_deg = float(np.max(orn_err_deg))
    final_orn_deg = float(orn_err_deg[-1])

    # 3. Joint Motion Metrics
    if N > 1:
        d_joints = np.abs(np.diff(joint_pos_arr, axis=0))
        total_joint_travel_rad = float(np.sum(d_joints))
        max_joint_travel_rad = float(np.max(np.sum(d_joints, axis=0)))
    else:
        total_joint_travel_rad = 0.0
        max_joint_travel_rad = 0.0

    rms_j_vel = float(np.sqrt(np.mean(joint_vel_arr ** 2)))
    peak_j_vel = float(np.max(np.abs(joint_vel_arr)))

    # Accelerations from measured velocities
    if N > 1:
        dt_arr = np.diff(time_arr)
        dt_arr = np.where(dt_arr <= 1e-6, 1.0 / 240.0, dt_arr)
        j_acc = np.diff(joint_vel_arr, axis=0) / dt_arr[:, None]
        rms_j_acc = float(np.sqrt(np.mean(j_acc ** 2)))
        peak_j_acc = float(np.max(np.abs(j_acc)))
    else:
        rms_j_acc = 0.0
        peak_j_acc = 0.0

    # 4. Kinematics & Manipulability
    mean_manip = float(np.mean(manip_arr))
    min_manip = float(np.min(manip_arr))
    mean_sigma = float(np.mean(sigma_min_arr))
    min_sigma = float(np.min(sigma_min_arr))
    max_cond = float(np.max(cond_arr))
    sing_count = int(np.sum(near_sing_arr))
    sing_pct = float(100.0 * sing_count / N)

    # 5. Safety
    self_col_count = int(np.sum(self_col_arr))
    env_col_count = int(np.sum(env_col_arr))
    limit_viol_count = int(np.sum(limit_viol_arr))
    vel_sat_count = int(np.sum(vel_sat_arr))

    # 6. Settling & Completion
    final_goal_pos = des_pos[-1]
    err_to_final_goal_mm = np.linalg.norm(act_pos - final_goal_pos, axis=1) * 1000.0
    settled_final_pos_mm = float(err_to_final_goal_mm[-1])

    # Find earliest time entering and staying within settle tolerance
    time_to_tol_s = float(time_arr[-1])
    settled = False
    for k in range(N):
        if np.all(err_to_final_goal_mm[k:] <= settle_tol_mm):
            time_to_tol_s = float(time_arr[k])
            settled = True
            break

    # Success criteria
    has_collision = (self_col_count > 0) or (env_col_count > 0)
    pos_success = settled_final_pos_mm <= success_position_tolerance_mm
    orn_success = True
    if is_se3:
        orn_success = final_orn_deg <= success_orientation_tolerance_deg

    success = (not has_collision) and pos_success and orn_success
    failure_reason = ""
    if has_collision:
        failure_reason = f"Collision detected (self: {self_col_count}, env: {env_col_count})"
    elif not pos_success:
        failure_reason = f"Final position error {settled_final_pos_mm:.2f}mm exceeded bound {success_position_tolerance_mm:.2f}mm"
    elif not orn_success:
        failure_reason = f"Final orientation error {final_orn_deg:.2f}deg exceeded bound {success_orientation_tolerance_deg:.2f}deg"

    return ExperimentMetrics(
        mean_position_error_mm=mean_pos_mm,
        rmse_position_error_mm=rmse_pos_mm,
        median_position_error_mm=median_pos_mm,
        p95_position_error_mm=p95_pos_mm,
        max_position_error_mm=max_pos_mm,
        final_position_error_mm=final_pos_mm,
        settled_final_position_error_mm=settled_final_pos_mm,
        mean_orientation_error_deg=mean_orn_deg,
        rmse_orientation_error_deg=rmse_orn_deg,
        p95_orientation_error_deg=p95_orn_deg,
        max_orientation_error_deg=max_orn_deg,
        final_orientation_error_deg=final_orn_deg,
        total_joint_travel_rad=total_joint_travel_rad,
        max_joint_travel_rad=max_joint_travel_rad,
        rms_joint_velocity_rad_s=rms_j_vel,
        peak_joint_velocity_rad_s=peak_j_vel,
        rms_joint_accel_rad_s2=rms_j_acc,
        peak_joint_accel_rad_s2=peak_j_acc,
        mean_manipulability=mean_manip,
        min_manipulability=min_manip,
        mean_sigma_min=mean_sigma,
        min_sigma_min=min_sigma,
        max_condition_number=max_cond,
        singularity_warning_count=sing_count,
        singularity_time_pct=sing_pct,
        self_collision_events=self_col_count,
        env_collision_events=env_col_count,
        joint_limit_events=limit_viol_count,
        velocity_saturation_events=vel_sat_count,
        settled_within_tolerance=settled,
        time_to_final_goal_tolerance_s=time_to_tol_s,
        success=success,
        failure_reason=failure_reason,
    )


# =============================================================================
# 4. TRIAL EXECUTION ENGINE
# =============================================================================

@dataclass
class ExperimentTrialResult:
    """Complete results for one experiment trial including time series and metrics."""
    experiment_name: str
    robot_name: str
    controller_type: str
    trial_index: int
    duration_s: float
    physics_hz: int
    dt: float
    metrics: ExperimentMetrics
    timeseries: Dict[str, np.ndarray] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "experiment_name": self.experiment_name,
            "robot_name": self.robot_name,
            "controller_type": self.controller_type,
            "trial_index": self.trial_index,
            "duration_s": self.duration_s,
            "physics_hz": self.physics_hz,
            "dt": self.dt,
            "metrics": self.metrics.to_dict(),
            "metadata": self.metadata,
        }

    def to_timeseries_records(self) -> List[Dict[str, Any]]:
        """Converts timeseries arrays to list of row dictionaries for tabular CSV export."""
        records: List[Dict[str, Any]] = []
        if "time_s" not in self.timeseries:
            return records
        n = len(self.timeseries["time_s"])
        for i in range(n):
            rec: Dict[str, Any] = {
                "experiment": self.experiment_name,
                "robot": self.robot_name,
                "controller": self.controller_type,
                "trial": self.trial_index,
                "time_s": float(self.timeseries["time_s"][i]),
                "desired_x": float(self.timeseries["desired_x"][i]),
                "desired_y": float(self.timeseries["desired_y"][i]),
                "desired_z": float(self.timeseries["desired_z"][i]),
                "actual_x": float(self.timeseries["actual_x"][i]),
                "actual_y": float(self.timeseries["actual_y"][i]),
                "actual_z": float(self.timeseries["actual_z"][i]),
                "position_error_mm": float(self.timeseries["position_error_mm"][i]),
                "desired_qx": float(self.timeseries["desired_qx"][i]),
                "desired_qy": float(self.timeseries["desired_qy"][i]),
                "desired_qz": float(self.timeseries["desired_qz"][i]),
                "desired_qw": float(self.timeseries["desired_qw"][i]),
                "actual_qx": float(self.timeseries["actual_qx"][i]),
                "actual_qy": float(self.timeseries["actual_qy"][i]),
                "actual_qz": float(self.timeseries["actual_qz"][i]),
                "actual_qw": float(self.timeseries["actual_qw"][i]),
                "orientation_error_deg": float(self.timeseries.get("orientation_error_deg", np.zeros(n))[i]),
                "manipulability": float(self.timeseries["manipulability"][i]),
                "sigma_min": float(self.timeseries["sigma_min"][i]),
                "condition_number": float(self.timeseries["condition_number"][i]),
            }
            if "joint_positions" in self.timeseries:
                j_pos = self.timeseries["joint_positions"][i]
                for j_idx in range(len(j_pos)):
                    rec[f"joint_{j_idx+1}_position"] = float(j_pos[j_idx])
            if "joint_velocities" in self.timeseries:
                j_vel = self.timeseries["joint_velocities"][i]
                for j_idx in range(len(j_vel)):
                    rec[f"joint_{j_idx+1}_velocity"] = float(j_vel[j_idx])
            rec["collision_state"] = int(bool(self.timeseries["self_collision"][i] or self.timeseries["env_collision"][i]))
            rec["singularity_state"] = int(bool(self.timeseries["condition_number"][i] > 100.0 or self.timeseries["sigma_min"][i] < 0.01))
            records.append(rec)
        return records


def execute_experiment_trial(
    experiment_def: Union[str, ExperimentDefinition, TaskspaceTrajectory],
    robot_name: str,
    controller_type: str,
    trajectory: Optional[TaskspaceTrajectory] = None,
    trial_index: int = 0,
    trajectory_duration: Optional[float] = None,
    settle_duration_s: float = 0.5,
    settle_tolerance_mm: float = SETTLE_TOLERANCE_MM,
    success_position_tolerance_mm: float = SUCCESS_POSITION_TOLERANCE_MM,
    success_orientation_tolerance_deg: float = SUCCESS_ORIENTATION_TOLERANCE_DEG,
    physics_hz: int = 240,
    gui: bool = False,
    seed: int = 42,
) -> Tuple[ExperimentMetrics, List[Dict[str, Any]]]:
    """Executes a single deterministic experiment trial in PyBullet DIRECT or GUI mode."""
    if isinstance(experiment_def, TaskspaceTrajectory):
        traj = experiment_def
        exp_name = traj.name.lower()
    elif isinstance(experiment_def, ExperimentDefinition):
        exp_name = experiment_def.name.lower()
        traj = trajectory or experiment_def.create_trajectory(duration_s=trajectory_duration, physics_hz=physics_hz)
    elif isinstance(experiment_def, str):
        exp_name = experiment_def.lower()
        if exp_name in EXPERIMENT_DEFINITIONS:
            traj = trajectory or EXPERIMENT_DEFINITIONS[exp_name].create_trajectory(duration_s=trajectory_duration, physics_hz=physics_hz)
        else:
            raise ValueError(f"Unknown experiment name '{experiment_def}'. Available: {ALL_EXPERIMENT_NAMES}")
    else:
        raise TypeError(f"Invalid experiment_def type: {type(experiment_def)}")

    np.random.seed(seed + trial_index)
    dt = 1.0 / physics_hz

    cid = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(dt, physicsClientId=cid)
    p.setGravity(0, 0, -9.81, physicsClientId=cid)

    # 1. Environment & Robot Setup
    plane_id = p.loadURDF("plane.urdf", physicsClientId=cid)
    table_id = p.loadURDF(
        "table/table.urdf",
        basePosition=[0.5, 0.0, -0.62],
        baseOrientation=[0, 0, 0, 1],
        useFixedBase=True,
        physicsClientId=cid,
    )

    registry = get_robot_registry()
    spec = registry.get_robot_spec(robot_name)
    rid = p.loadURDF(
        spec.urdf_path,
        basePosition=spec.base_position,
        baseOrientation=spec.base_orientation,
        useFixedBase=spec.fixed_base,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=cid,
    )

    ctrl = GenericRobotController(cid, rid, spec)
    lows, highs, ranges, rests = ctrl.get_joint_limits()
    ik_solver = GenericIKSolver(
        physics_client_id=cid,
        robot_id=rid,
        arm_joint_indices=ctrl.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=ctrl.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )

    rr_controller = ResolvedRateController(
        physics_client_id=cid,
        robot_controller=ctrl,
        enable_nullspace=True,
    )

    from robotics.collision import CollisionChecker
    allowed_mount = [(rid, -1, table_id, -1), (rid, 0, table_id, -1)]
    col_checker = CollisionChecker(
        physics_client_id=cid,
        robot_id=rid,
        table_id=table_id,
        allowed_link_pairs=allowed_mount,
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )

    try:
        # 2. Initialization & Settling to Start Pose
        start_p = traj.positions[0]
        start_o = traj.orientations[0]
        init_ik = ik_solver.solve(start_p, start_o)
        if not init_ik.success:
            raise RuntimeError(f"Could not solve start pose for {robot_name} on {exp_name}")

        ctrl.set_arm_joint_positions(init_ik.joint_positions, enforce_velocity_limits=False)
        for j_idx, q_val in zip(ctrl.arm_joint_indices, init_ik.joint_positions):
            p.resetJointState(rid, j_idx, float(q_val), targetVelocity=0.0, physicsClientId=cid)

        # Warm-up physics stepping (10 steps) to settle
        for _ in range(10):
            p.stepSimulation(physicsClientId=cid)

        # 3. Trajectory & Settle Scheduling
        traj_samples = traj.num_samples
        settle_samples = int(round(settle_duration_s * physics_hz))
        total_samples = traj_samples + settle_samples

        time_records = np.zeros(total_samples, dtype=np.float64)
        des_pos_rec = np.zeros((total_samples, 3), dtype=np.float64)
        act_pos_rec = np.zeros((total_samples, 3), dtype=np.float64)
        des_orn_rec = np.zeros((total_samples, 4), dtype=np.float64)
        act_orn_rec = np.zeros((total_samples, 4), dtype=np.float64)
        joint_pos_rec = np.zeros((total_samples, len(ctrl.arm_joint_indices)), dtype=np.float64)
        joint_vel_rec = np.zeros((total_samples, len(ctrl.arm_joint_indices)), dtype=np.float64)
        manip_rec = np.zeros(total_samples, dtype=np.float64)
        sigma_min_rec = np.zeros(total_samples, dtype=np.float64)
        cond_rec = np.zeros(total_samples, dtype=np.float64)
        near_sing_rec = np.zeros(total_samples, dtype=bool)
        self_col_rec = np.zeros(total_samples, dtype=bool)
        env_col_rec = np.zeros(total_samples, dtype=bool)
        limit_viol_rec = np.zeros(total_samples, dtype=bool)
        vel_sat_rec = np.zeros(total_samples, dtype=bool)

        for step_i in range(total_samples):
            if step_i < traj_samples:
                t_val = traj.timestamps[step_i]
                target_p = traj.positions[step_i]
                target_o = traj.orientations[step_i]
            else:
                t_val = traj.timestamps[-1] + (step_i - traj_samples + 1) * dt
                target_p = traj.positions[-1]
                target_o = traj.orientations[-1]

            time_records[step_i] = t_val
            des_pos_rec[step_i] = target_p
            des_orn_rec[step_i] = target_o

            # Controller Dispatch
            if controller_type == "resolved-rate":
                q_dot, m_metrics = rr_controller.compute_step(
                    target_position=target_p,
                    target_orientation=target_o,
                    dt=dt,
                )
                ctrl.set_arm_joint_velocities(q_dot)
            else:  # IK Position Control
                ik_sol = ik_solver.solve(target_p, target_o)
                ctrl.set_arm_joint_positions(
                    ik_sol.joint_positions,
                    dt=dt,
                    enforce_velocity_limits=True,
                )

            # Physics Step
            p.stepSimulation(physicsClientId=cid)

            # State Measurement
            curr_pos, curr_orn = ctrl.get_end_effector_pose()
            curr_q = ctrl.get_current_joint_positions()
            curr_qd = ctrl.get_current_joint_velocities()

            act_pos_rec[step_i] = curr_pos
            act_orn_rec[step_i] = curr_orn
            joint_pos_rec[step_i] = curr_q
            joint_vel_rec[step_i] = curr_qd

            # Kinematic Diagnostics
            _, _, J = compute_jacobian(cid, rid, ctrl.ee_link_index, ctrl.arm_joint_indices, curr_q)
            m_eval = compute_manipulability(J)
            manip_rec[step_i] = m_eval.manipulability
            sigma_min_rec[step_i] = m_eval.sigma_min
            cond_rec[step_i] = m_eval.condition_number
            near_sing_rec[step_i] = m_eval.near_singularity

            # Safety Queries
            col_res = col_checker.check_collision()
            self_col_rec[step_i] = col_res.self_collision
            env_col_rec[step_i] = col_res.env_collision

            # Limits check
            lows_arr = np.array(lows, dtype=np.float64)
            highs_arr = np.array(highs, dtype=np.float64)
            j_limits_violated = np.any(curr_q < (lows_arr - 0.01)) or np.any(curr_q > (highs_arr + 0.01))
            limit_viol_rec[step_i] = j_limits_violated
            max_vel = float(ctrl.spec.max_joint_velocity_radps)
            vel_sat_rec[step_i] = np.any(np.abs(curr_qd) >= (max_vel * 0.98))

        # 4. Metrics Computation
        metrics = compute_experiment_metrics(
            time_arr=time_records,
            des_pos=des_pos_rec,
            act_pos=act_pos_rec,
            des_orn=des_orn_rec,
            act_orn=act_orn_rec,
            joint_pos_arr=joint_pos_rec,
            joint_vel_arr=joint_vel_rec,
            manip_arr=manip_rec,
            sigma_min_arr=sigma_min_rec,
            cond_arr=cond_rec,
            near_sing_arr=near_sing_rec,
            self_col_arr=self_col_rec,
            env_col_arr=env_col_rec,
            limit_viol_arr=limit_viol_rec,
            vel_sat_arr=vel_sat_rec,
            settle_tol_mm=settle_tolerance_mm,
            success_position_tolerance_mm=success_position_tolerance_mm,
            success_orientation_tolerance_deg=success_orientation_tolerance_deg,
            is_se3=getattr(traj, "is_se3_sweep", False),
        )

        orn_err_deg_arr = np.array([
            math.degrees(compute_angular_distance(act_orn_rec[k], des_orn_rec[k]))
            for k in range(total_samples)
        ], dtype=np.float64)

        timeseries_dict = {
            "time_s": time_records,
            "desired_x": des_pos_rec[:, 0],
            "desired_y": des_pos_rec[:, 1],
            "desired_z": des_pos_rec[:, 2],
            "actual_x": act_pos_rec[:, 0],
            "actual_y": act_pos_rec[:, 1],
            "actual_z": act_pos_rec[:, 2],
            "position_error_mm": np.linalg.norm(act_pos_rec - des_pos_rec, axis=1) * 1000.0,
            "desired_qx": des_orn_rec[:, 0],
            "desired_qy": des_orn_rec[:, 1],
            "desired_qz": des_orn_rec[:, 2],
            "desired_qw": des_orn_rec[:, 3],
            "actual_qx": act_orn_rec[:, 0],
            "actual_qy": act_orn_rec[:, 1],
            "actual_qz": act_orn_rec[:, 2],
            "actual_qw": act_orn_rec[:, 3],
            "orientation_error_deg": orn_err_deg_arr,
            "manipulability": manip_rec,
            "sigma_min": sigma_min_rec,
            "condition_number": cond_rec,
            "joint_positions": joint_pos_rec,
            "joint_velocities": joint_vel_rec,
            "self_collision": self_col_rec,
            "env_collision": env_col_rec,
        }

        trial_res = ExperimentTrialResult(
            experiment_name=exp_name,
            robot_name=robot_name,
            controller_type=controller_type,
            trial_index=trial_index,
            duration_s=traj.duration_s,
            physics_hz=physics_hz,
            dt=dt,
            metrics=metrics,
            timeseries=timeseries_dict,
            metadata={
                "settle_duration_s": settle_duration_s,
                "settle_tolerance_mm": settle_tolerance_mm,
                "success_position_tolerance_mm": success_position_tolerance_mm,
                "success_orientation_tolerance_deg": success_orientation_tolerance_deg,
                "total_steps": total_samples,
                "is_se3_sweep": getattr(traj, "is_se3_sweep", False),
            },
        )
        return metrics, trial_res.to_timeseries_records()

    finally:
        p.disconnect(cid)


# =============================================================================
# 5. DEDICATED OBSTACLE REACH & PLANNING EXPERIMENT
# =============================================================================

@dataclass
class PlanningExperimentResult:
    """Results from the obstacle-reach and collision-aware motion planning experiment."""
    robot_name: str
    direct_path_free: bool
    rrt_planning_success: bool
    planning_time_ms: float
    raw_waypoints_count: int
    smoothed_waypoints_count: int
    raw_joint_path_length_rad: float
    smoothed_joint_path_length_rad: float
    min_collision_clearance_m: float
    execution_success: bool
    execution_final_pos_error_mm: float
    planning_execution_position_tolerance_mm: float = PLANNING_EXECUTION_POSITION_TOLERANCE_MM
    details: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def execute_obstacle_reach_experiment(
    robot_name: str = "panda",
    physics_hz: int = 240,
    gui: bool = False,
    seed: int = 42,
    planning_execution_position_tolerance_mm: float = PLANNING_EXECUTION_POSITION_TOLERANCE_MM,
) -> PlanningExperimentResult:
    """Evaluates collision-aware motion planning around a tall scene obstacle.

    Validates:
    - Direct joint-space path rejection due to collision.
    - Bidirectional RRT-Connect planner invocation and success.
    - Path shortcutting and waypoint reduction.
    - Execution without collision in PyBullet with final endpoint position error <= tolerance.
    """
    np.random.seed(seed)
    cid = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(1.0 / physics_hz, physicsClientId=cid)

    # 1. Environment & Obstacle Setup
    plane_id = p.loadURDF("plane.urdf", physicsClientId=cid)
    table_id = p.loadURDF("table/table.urdf", basePosition=[0.5, 0.0, -0.62], useFixedBase=True, physicsClientId=cid)

    obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.08], physicsClientId=cid)
    obs_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.08], rgbaColor=[0.8, 0.2, 0.2, 0.8], physicsClientId=cid)
    obs_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=obs_col,
        baseVisualShapeIndex=obs_vis,
        basePosition=[0.42, 0.0, 0.35],
        physicsClientId=cid,
    )

    registry = get_robot_registry()
    spec = registry.get_robot_spec(robot_name)
    rid = p.loadURDF(
        spec.urdf_path,
        basePosition=spec.base_position,
        baseOrientation=spec.base_orientation,
        useFixedBase=spec.fixed_base,
        physicsClientId=cid,
    )

    ctrl = GenericRobotController(cid, rid, spec)
    lows, highs, ranges, rests = ctrl.get_joint_limits()
    ik_solver = GenericIKSolver(
        physics_client_id=cid,
        robot_id=rid,
        arm_joint_indices=ctrl.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=ctrl.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )

    from robotics.collision import CollisionChecker
    from robotics.planning import is_joint_path_collision_free, RRTConnectPlanner, shortcut_path

    allowed_mount = [(rid, -1, table_id, -1), (rid, 0, table_id, -1)]
    col_checker = CollisionChecker(
        physics_client_id=cid,
        robot_id=rid,
        table_id=table_id,
        obstacle_ids=[obs_id],
        allowed_link_pairs=allowed_mount,
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )

    try:
        # Solve start and goal on opposite sides of obstacle
        start_target_pos = [0.42, -0.18, 0.35]
        goal_target_pos = [0.42, 0.18, 0.35]
        target_orn = list(spec.default_ee_orientation)

        ik_start = ik_solver.solve(start_target_pos, target_orn)
        ik_goal = ik_solver.solve(goal_target_pos, target_orn)
        if not ik_start.success or not ik_goal.success:
            raise RuntimeError(f"IK solve failed for planning start/goal on {robot_name}")

        q_start = list(ik_start.joint_positions)
        q_goal = list(ik_goal.joint_positions)

        ctrl.set_arm_joint_positions(q_start, enforce_velocity_limits=False)
        for j, val in zip(ctrl.arm_joint_indices, q_start):
            p.resetJointState(rid, j, float(val), targetVelocity=0.0, physicsClientId=cid)

        # 2. Direct Path Evaluation (straight-line joint interpolation)
        direct_is_free, _ = is_joint_path_collision_free(
            q_start, q_goal, col_checker, arm_joint_indices=ctrl.arm_joint_indices
        )

        # 3. RRT-Connect Planning
        planner = RRTConnectPlanner(
            lower_limits=lows,
            upper_limits=highs,
            collision_checker=col_checker,
            arm_joint_indices=ctrl.arm_joint_indices,
            step_size_rad=0.05,
            max_iterations=1000,
            random_seed=seed,
        )

        plan_res = planner.plan(q_start, q_goal)
        rrt_success = plan_res.success
        plan_time_ms = float(plan_res.planning_time_ms)

        raw_count = len(plan_res.path)
        raw_len = float(sum(np.linalg.norm(np.array(plan_res.path[i+1]) - np.array(plan_res.path[i])) for i in range(raw_count - 1))) if raw_count > 1 else 0.0

        # Shortcut path
        if rrt_success:
            smoothed_path = shortcut_path(plan_res.path, col_checker, ctrl.arm_joint_indices, max_attempts=30, random_seed=seed)
        else:
            smoothed_path = []

        smooth_count = len(smoothed_path)
        smooth_len = float(sum(np.linalg.norm(np.array(smoothed_path[i+1]) - np.array(smoothed_path[i])) for i in range(smooth_count - 1))) if smooth_count > 1 else 0.0

        # 4. Trajectory Execution & Verification
        min_clearance = float("inf")
        exec_success = False
        final_err_mm = float("inf")

        if rrt_success and smooth_count > 1:
            from robotics.trajectory import PiecewiseJointTrajectory
            traj = PiecewiseJointTrajectory(waypoints=smoothed_path, duration=3.0)
            dt = 1.0 / physics_hz
            steps = int(round(traj.duration * physics_hz))

            no_col = True
            for s in range(steps):
                sample = traj.evaluate(s * dt)
                ctrl.set_arm_joint_positions(sample.position, dt=dt, enforce_velocity_limits=False)
                p.stepSimulation(physicsClientId=cid)

                c_query = col_checker.check_collision()
                if c_query.in_collision:
                    no_col = False
                if c_query.min_env_clearance_m < min_clearance:
                    min_clearance = c_query.min_env_clearance_m

            # Settle to goal
            for _ in range(30):
                ctrl.set_arm_joint_positions(q_goal, dt=dt, enforce_velocity_limits=False)
                p.stepSimulation(physicsClientId=cid)

            ee_pos, _ = ctrl.get_end_effector_pose()
            final_err_mm = float(np.linalg.norm(np.array(ee_pos) - np.array(goal_target_pos)) * 1000.0)
            exec_success = no_col and (final_err_mm <= planning_execution_position_tolerance_mm)

        return PlanningExperimentResult(
            robot_name=robot_name,
            direct_path_free=direct_is_free,
            rrt_planning_success=rrt_success,
            planning_time_ms=plan_time_ms,
            raw_waypoints_count=raw_count,
            smoothed_waypoints_count=smooth_count,
            raw_joint_path_length_rad=raw_len,
            smoothed_joint_path_length_rad=smooth_len,
            min_collision_clearance_m=float(min_clearance if min_clearance != float("inf") else 0.0),
            execution_success=exec_success,
            execution_final_pos_error_mm=final_err_mm,
            planning_execution_position_tolerance_mm=planning_execution_position_tolerance_mm,
            details=f"Direct: {'FREE' if direct_is_free else 'BLOCKED'}, RRT: {'SUCCESS' if rrt_success else 'FAILED'}, Exec: {'SUCCESS' if exec_success else 'FAILED'}",
        )

    finally:
        p.disconnect(cid)


# =============================================================================
# 6. EXPERIMENT MANIFEST GENERATOR
# =============================================================================

@dataclass
class ExperimentManifest:
    """Standardized metadata manifest describing the experiment environment and parameters."""
    version: str
    git_commit_sha: str
    timestamp_utc: str
    environment: Dict[str, Any]
    execution: Dict[str, Any]
    tolerances: Dict[str, Any]
    trajectory_definitions: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def generate_experiment_manifest(
    robot_names: Sequence[str] = ("panda", "kuka_iiwa"),
    controller_types: Sequence[str] = ("ik", "resolved-rate"),
    deterministic_repeats: int = 1,
    random_seed: int = 42,
    physics_hz: int = 240,
    trajectory_duration: Optional[float] = None,
    preflight_sample_stride: int = 1,
    settle_tolerance_mm: float = SETTLE_TOLERANCE_MM,
    success_position_tolerance_mm: float = SUCCESS_POSITION_TOLERANCE_MM,
    success_orientation_tolerance_deg: float = SUCCESS_ORIENTATION_TOLERANCE_DEG,
    preflight_position_tolerance_mm: float = PREFLIGHT_POSITION_RESIDUAL_THRESHOLD_MM,
    preflight_orientation_tolerance_deg: float = PREFLIGHT_ORIENTATION_RESIDUAL_THRESHOLD_DEG,
    planning_execution_position_tolerance_mm: float = PLANNING_EXECUTION_POSITION_TOLERANCE_MM,
    git_commit_sha: Optional[str] = None,
) -> ExperimentManifest:
    """Builds a deterministic experiment manifest without machine-specific absolute paths or usernames."""
    if git_commit_sha:
        git_sha = str(git_commit_sha).strip()
    else:
        git_sha = "unknown"
        try:
            git_res = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if git_res.returncode == 0:
                git_sha = git_res.stdout.strip()
        except Exception:
            pass

    pybullet_pkg_ver = "unknown"
    try:
        import importlib.metadata
        pybullet_pkg_ver = importlib.metadata.version("pybullet")
    except Exception:
        pass

    pybullet_api_ver = "unknown"
    try:
        pybullet_api_ver = str(p.getAPIVersion())
    except Exception:
        pass

    opencv_ver = "unknown"
    try:
        import cv2
        opencv_ver = str(cv2.__version__)
    except Exception:
        pass

    traj_defs = {}
    for name in ALL_EXPERIMENT_NAMES:
        traj_defs[name] = get_experiment_metadata(
            name,
            duration_override_s=trajectory_duration,
            physics_hz=physics_hz,
            preflight_sample_stride=preflight_sample_stride,
        )

    return ExperimentManifest(
        version="1.2.0",
        git_commit_sha=git_sha,
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        environment={
            "python_version": platform.python_version(),
            "opencv_version": opencv_ver,
            "numpy_version": np.__version__,
            "pybullet_package_version": pybullet_pkg_ver,
            "pybullet_api_version": pybullet_api_ver,
            "os": platform.system(),
            "architecture": platform.machine(),
        },
        execution={
            "robots": list(robot_names),
            "controllers": list(controller_types),
            "physics_frequency_hz": physics_hz,
            "timestep_dt_s": 1.0 / physics_hz,
            "deterministic_repeats": deterministic_repeats,
            "random_seed": random_seed,
        },
        tolerances={
            "settle_tolerance_mm": settle_tolerance_mm,
            "success_position_tolerance_mm": success_position_tolerance_mm,
            "success_orientation_tolerance_deg": success_orientation_tolerance_deg,
            "preflight_position_residual_threshold_mm": preflight_position_tolerance_mm,
            "preflight_orientation_residual_threshold_deg": preflight_orientation_tolerance_deg,
            "planning_execution_position_tolerance_mm": planning_execution_position_tolerance_mm,
        },
        trajectory_definitions=traj_defs,
    )
