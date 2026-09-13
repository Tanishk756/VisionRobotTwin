"""Centralized Configuration for VisionRobotTwin.

Defines strongly-typed dataclasses for all system parameters, eliminating
magic numbers and providing a single source of truth for perception, kinematics,
control, simulation, and safety parameters.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple
import numpy as np


@dataclass
class CameraConfig:
    """Camera capture settings."""
    camera_index: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 30
    auto_exposure: bool = True
    buffer_size: int = 1
    synthetic_mode: bool = False  # Explicitly forced via --synthetic
    allow_synthetic_fallback: bool = False  # If False, raises error on camera open or mid-stream disconnect
    max_consecutive_read_failures: int = 30  # Number of failed read() attempts before disconnecting


@dataclass
class ArUcoConfig:
    """ArUco marker detection settings."""
    dictionary_name: str = "DICT_4X4_50"
    marker_size_m: float = 0.05  # 50 mm physical printed marker size
    target_marker_id: int = 0    # Manual tracking target marker
    pick_marker_id: int = 1      # Pick waypoint marker for autonomous mode
    place_marker_id: int = 2     # Place waypoint marker for autonomous mode
    anchor_marker_id: int = 10   # World / Robot base calibration anchor marker
    draw_axes_length_m: float = 0.035
    border_bits: int = 1


@dataclass
class CalibrationConfig:
    """Camera calibration settings."""
    calibration_file: Path = Path("calibration/camera_calibration.npz")
    default_fov_degrees: float = 65.0
    chessboard_pattern_size: Tuple[int, int] = (9, 6)  # Interior corners (columns, rows)
    square_size_m: float = 0.025  # 25 mm per square
    min_calibration_frames: int = 15


@dataclass
class FilterConfig:
    """Pose filtering and smoothing settings."""
    filter_type: str = "ema"  # 'ema' or 'one_euro'
    ema_alpha_position: float = 0.35  # Higher = more responsive, lower = smoother
    ema_alpha_orientation: float = 0.25
    one_euro_min_cutoff: float = 1.0
    one_euro_beta: float = 0.007
    one_euro_d_cutoff: float = 1.0


@dataclass
class TransformConfig:
    """SE(3) coordinate transformation parameters.
    
    Transforms the detected marker pose in camera optical frame C
    into robot base coordinate frame B:
      T_base_marker = T_base_camera @ T_camera_marker
    """
    transform_mode: str = "relative"  # 'relative' (intuitive teleoperation) or 'se3' (rigid transformation)
    is_calibrated_extrinsics: bool = False  # Flag denoting nominal vs calibrated extrinsics
    extrinsics_file: Path = Path("calibration/extrinsics.json")

    # Nominal Camera placed in front of robot looking slightly downward:
    # Camera frame: X right, Y down, Z forward
    # Robot base frame: X forward, Y left, Z up
    camera_position_in_robot_base: Tuple[float, float, float] = (0.70, 0.0, 0.40)
    # Pitch camera ~15 degrees down looking toward robot center
    camera_euler_rpy_rad: Tuple[float, float, float] = (np.pi, 0.26, 0.0)

    # Configured anchor location in robot base frame:
    anchor_position_in_robot_base: Tuple[float, float, float] = (0.50, 0.0, 0.0)
    anchor_euler_rpy_rad: Tuple[float, float, float] = (np.pi, 0.0, 0.0)

    # Tool orientation offset to keep Franka gripper pointing downward [x, y, z, w]
    tool_orientation_offset: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass
class WorkspaceConfig:
    """Workspace bounding and scaling limits for the Franka Panda manipulator."""
    # Robot Base Cartesian limits (meters)
    x_min: float = 0.25
    x_max: float = 0.70
    y_min: float = -0.40
    y_max: float = 0.40
    z_min: float = 0.08
    z_max: float = 0.65

    # Center of physical camera interaction volume (meters in camera frame)
    cam_center_x: float = 0.0
    cam_center_y: float = 0.0
    cam_center_z: float = 0.45

    # Scale factor from physical marker motion to robot motion (for relative mode)
    scale_x: float = 1.2
    scale_y: float = 1.2
    scale_z: float = 1.2

    # Cartesian center to place mapped interaction volume in accessible workspace
    robot_center_x: float = 0.50
    robot_center_y: float = 0.00
    robot_center_z: float = 0.35

    # Velocity and step limits for safety (time-based slew rate limiter)
    max_cartesian_step_m: float = 0.035  # Absolute maximum delta guard per cycle
    max_cartesian_velocity_mps: float = 0.40  # Max Cartesian target speed (m/s)
    max_angular_velocity_radps: float = 1.57  # Max Angular target rate (rad/s)


@dataclass
class RobotConfig:
    """Franka Emika Panda kinematic and physical parameters."""
    urdf_path: str = "franka_panda/panda.urdf"
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    end_effector_link_index: int = 11  # Panda hand / grasptarget link
    
    # Rest / Home joint angles (rad)
    home_joint_positions: List[float] = field(
        default_factory=lambda: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04]
    )
    
    # Default end-effector orientation: gripper pointing straight down [x, y, z, w]
    default_ee_orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    # Controller gains and limits
    max_joint_velocity_radps: float = 2.0
    position_gain: float = 0.15
    velocity_gain: float = 1.0
    max_joint_force: float = 200.0


@dataclass
class SimulationConfig:
    """PyBullet physics simulation parameters."""
    time_step: float = 1.0 / 240.0
    target_physics_hz: float = 240.0
    max_substeps_per_frame: int = 20
    gui: bool = True
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)
    target_sphere_radius: float = 0.022
    target_sphere_color: Tuple[float, float, float, float] = (0.9, 0.15, 0.15, 0.85)
    ee_sphere_color: Tuple[float, float, float, float] = (0.15, 0.85, 0.25, 0.85)
    trajectory_history_len: int = 120
    show_workspace_box: bool = True
    camera_distance: float = 1.4
    camera_yaw: float = 45.0
    camera_pitch: float = -30.0
    camera_target_position: Tuple[float, float, float] = (0.4, 0.0, 0.2)


@dataclass
class StateMachineConfig:
    """State machine transitions and timeout parameters."""
    lost_tracking_search_timeout_s: float = 2.5 # Transition from HOLD to SEARCH if absent > 2.5s
    approach_height_offset_m: float = 0.14      # Clearance above object before pick/place
    pick_descent_height_m: float = 0.035        # Final grasp height
    waypoint_tolerance_m: float = 0.020         # Cartesian convergence arrival threshold
    grasp_action_delay_s: float = 0.6           # Pause time to attach/detach grasp
    grasp_distance_threshold_m: float = 0.055   # Maximum allowable distance for physical grasp constraint (5.5 cm)
    consecutive_detection_threshold: int = 5    # Required consecutive frames to freeze pick/place targets
    waypoint_timeout_s: float = 10.0            # Max duration allowed to reach waypoint before triggering ERROR
    max_step_count_per_phase: int = 500         # Secondary iteration guard (triggers ERROR if exceeded)
    auto_demo: bool = False                     # If True, allows un-gated fallback demo coordinates for tests


@dataclass
class AppConfig:
    """Master application configuration."""
    camera: CameraConfig = field(default_factory=CameraConfig)
    aruco: ArUcoConfig = field(default_factory=ArUcoConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    transform: TransformConfig = field(default_factory=TransformConfig)
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    state_machine: StateMachineConfig = field(default_factory=StateMachineConfig)
    
    # App-level operational flags
    mode: str = "manual"  # 'manual' or 'auto'
    control_mode: str = "6dof"  # '6dof' (position + orientation) or '3dof' (position only)
    debug: bool = False
    max_frames: int = 0  # 0 = infinite interactive, >0 = bounded for automated testing/CI
    log_file: Path = Path("logs/vision_robot_twin.log")
    screenshots_dir: Path = Path("screenshots")
    demo_data_dir: Path = Path("demo/data")


def get_default_config() -> AppConfig:
    """Factory helper to obtain a default configuration instance."""
    return AppConfig()
