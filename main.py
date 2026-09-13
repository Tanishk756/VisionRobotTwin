"""VisionRobotTwin: Real-Time Vision-Guided Robotic Manipulator Digital Twin.

Main entry point integrating real-time computer vision, ArUco 6-DoF pose estimation,
SE(3) coordinate transformations, workspace mapping, inverse kinematics, and PyBullet
digital twin control of a Franka Emika Panda manipulator.
"""

import sys
import time
import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple
import cv2
import numpy as np

import json
from visionrobottwin_version import __version__
from config.settings import AppConfig, get_default_config
from vision.calibration import load_or_create_calibration
from vision.camera import Camera
from vision.aruco_detector import ArUcoDetector
from vision.pose_estimator import PoseEstimator
from robotics.simulator import PyBulletSimulator
from robotics.workspace_mapper import WorkspaceMapper
from robotics.state_machine import RoboticStateMachine, RobotState
from utils.logger import setup_logger, get_logger
from utils.filters import PoseFilter
from utils.fps_counter import FPSCounter
from utils.telemetry import TelemetryOverlay, TelemetryData


def parse_arguments() -> argparse.Namespace:
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(
        description="VisionRobotTwin: Real-Time Vision-Guided Robotic Manipulator Digital Twin",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--version", action="version", version=f"VisionRobotTwin {__version__}")
    parser.add_argument("--calibration-status", action="store_true", help="Print camera intrinsics and extrinsics calibration status and exit")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--width", type=int, default=1280, help="Camera width resolution")
    parser.add_argument("--height", type=int, default=720, help="Camera height resolution")
    parser.add_argument("--mode", type=str, choices=["manual", "auto"], default="manual", help="Initial operational mode")
    parser.add_argument("--control-mode", type=str, choices=["6dof", "3dof"], default="6dof", help="Control mode: 6-DoF (pos+orn) or 3-DoF (pos only)")
    parser.add_argument("--transform-mode", type=str, choices=["relative", "se3"], default="relative", help="Transform pipeline mode")
    parser.add_argument("--synthetic", action="store_true", help="Explicitly force synthetic simulated video stream")
    parser.add_argument("--allow-synthetic-fallback", action="store_true", help="Allow fallback to synthetic stream if physical webcam is missing")
    parser.add_argument("--auto-demo", action="store_true", help="Enable offline demo coordinates in auto mode without waiting for markers")
    parser.add_argument("--headless", action="store_true", help="Run PyBullet and perception without GUI window")
    parser.add_argument("--max-frames", type=int, default=0, help="Maximum frames to run before clean exit (0 = continuous)")
    parser.add_argument("--debug", action="store_true", help="Enable verbose debug logging")
    parser.add_argument("--record", action="store_true", help="Record OpenCV HUD output video to demo/recordings/")
    parser.add_argument("--record-data", action="store_true", help="Record trajectory session data to CSV")
    return parser.parse_args()


def print_calibration_status() -> None:
    """Prints diagnostic report of intrinsics and extrinsics without opening hardware."""
    calib_file = Path("calibration/camera_calibration.npz")
    report_file = Path("calibration/camera_calibration_report.json")
    ext_file = Path("calibration/extrinsics.json")

    print("Camera Intrinsics:")
    if calib_file.exists():
        print("CALIBRATED")
        print(f"Intrinsics file:\n{calib_file}")
        if report_file.exists():
            try:
                with open(report_file, "r", encoding="utf-8") as f:
                    rep = json.load(f)
                rms = rep.get("reprojection_error_rms")
                mean_err = rep.get("reprojection_error_mean")
                if rms is not None:
                    print(f"RMS:\n{rms:.4f} px (mean: {mean_err:.4f} px)")
            except Exception:
                pass
    else:
        print("FALLBACK PINHOLE")
        print("Intrinsics file:\nNone (nominal fallback model)")

    print("\nExtrinsics:")
    if ext_file.exists():
        print("CALIBRATED")
        print(f"Extrinsics file:\n{ext_file}")
        print("Anchor validation:\navailable (run tools/validate_extrinsics.py)")
    else:
        print("NOMINAL")
        print("Extrinsics file:\nNone (nominal config model)")
        print("Anchor validation:\nunavailable (no extrinsics calibration file found)")


@dataclass
class FrameResult:
    """Structured output from a single frame processing cycle."""
    success: bool
    frame: Optional[np.ndarray] = None
    display_frame: Optional[np.ndarray] = None
    detections: list = None
    target_pose: Optional[object] = None
    pick_pose: Optional[object] = None
    place_pose: Optional[object] = None
    raw_pos_cam: Optional[Tuple[float, float, float]] = None
    filtered_pos_cam: Optional[Tuple[float, float, float]] = None
    commanded_position: Optional[np.ndarray] = None
    commanded_orientation: Optional[np.ndarray] = None
    ee_position: Optional[np.ndarray] = None
    ee_orientation: Optional[np.ndarray] = None
    tracking_error_m: Optional[float] = None
    ik_status: str = "IDLE"
    state_name: str = "HOME"
    mode: str = "MANUAL"
    control_mode: str = "6dof"
    transform_mode: str = "relative"
    was_clamped: bool = False
    physics_substeps: int = 1
    fps: float = 0.0


class VisionRobotTwinApp:
    """Master application controller orchestrating vision, kinematics, and simulation."""

    def __init__(
        self,
        config: AppConfig,
        headless_sim: bool = False,
        record_data: bool = False,
        record_video: bool = False,
    ):
        self.config = config
        self.headless = headless_sim or not config.simulation.gui
        self.logger = get_logger("VisionRobotTwin")

        # 1. Perception & Calibration
        self.calibration = load_or_create_calibration(
            self.config.calibration.calibration_file,
            width=self.config.camera.width,
            height=self.config.camera.height,
            default_fov=self.config.calibration.default_fov_degrees,
        )
        self.camera = Camera(self.config.camera, self.config.aruco)
        self.aruco_detector = ArUcoDetector(self.config.aruco)
        self.pose_estimator = PoseEstimator(self.calibration, self.config.aruco)

        # 2. Filtering & Workspace Mapping
        self.pose_filter = PoseFilter(
            filter_type=self.config.filter.filter_type,
            ema_alpha_pos=self.config.filter.ema_alpha_position,
            ema_alpha_rot=self.config.filter.ema_alpha_orientation,
            one_euro_min_cutoff=self.config.filter.one_euro_min_cutoff,
            one_euro_beta=self.config.filter.one_euro_beta,
        )
        # Check if calibrated extrinsics exist in SE(3) mode
        if self.config.transform.transform_mode == "se3" and getattr(self.config.transform, "extrinsics_file", None) is not None:
            if Path(self.config.transform.extrinsics_file).exists():
                self.config.transform.is_calibrated_extrinsics = True

        self.workspace_mapper = WorkspaceMapper(self.config.workspace, self.config.transform)

        intrinsics_status = (
            f"CALIBRATED ({self.config.calibration.calibration_file})"
            if self.calibration.is_calibrated
            else "FALLBACK PINHOLE"
        )
        extrinsics_status = (
            f"CALIBRATED ({self.config.transform.extrinsics_file})"
            if self.workspace_mapper.tf_config.is_calibrated_extrinsics
            else "NOMINAL"
        )

        self.logger.info("=" * 70)
        self.logger.info(" INITIALIZING VISION-ROBOT DIGITAL TWIN (v1.2-dev)")
        self.logger.info(f" Python Version : {sys.version.split()[0]} | OpenCV Version: {cv2.__version__}")
        self.logger.info(f" Mode           : {config.mode.upper()} | Control Mode: {config.control_mode.upper()} | Transform: {config.transform.transform_mode.upper()}")
        self.logger.info(f" Camera         : {'SYNTHETIC' if config.camera.synthetic_mode else f'Index {config.camera.camera_index}'}")
        self.logger.info(f" INTRINSICS     : {intrinsics_status}")
        self.logger.info(f" EXTRINSICS     : {extrinsics_status}")
        self.logger.info("=" * 70)

        # 3. Robotics & PyBullet Digital Twin
        self.simulator = PyBulletSimulator(self.config, headless=self.headless)
        self.state_machine = RoboticStateMachine(self.config.state_machine, self.config.workspace)
        self.state_machine.set_mode(self.config.mode)

        # 4. Telemetry & FPS Monitoring
        self.fps_counter = FPSCounter()
        self.sim_fps_counter = FPSCounter()
        self.telemetry_overlay = TelemetryOverlay()

        # 5. Session Logging & Video Recording
        self.record_data = record_data
        self.record_video = record_video
        self._csv_file = None
        self._csv_writer = None
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None

        if self.record_data:
            self._init_csv_recorder()

        self._is_running = True
        self._is_paused = False
        self._paused_joint_positions: Optional[list] = None
        self._frames_processed = 0
        self._last_frame_result: Optional[FrameResult] = None

        # Memory for 6-DoF HOLD tracking
        self._last_commanded_target_pos: np.ndarray = np.array([
            self.config.workspace.robot_center_x,
            self.config.workspace.robot_center_y,
            self.config.workspace.robot_center_z,
        ], dtype=np.float64)
        self._last_commanded_target_orn: np.ndarray = np.array(
            self.config.robot.default_ee_orientation,
            dtype=np.float64,
        )
        self._last_loop_time: Optional[float] = None

    def _init_csv_recorder(self) -> None:
        """Initializes trajectory data logger."""
        self.config.demo_data_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self.config.demo_data_dir / f"session_{timestamp}.csv"
        try:
            self._csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp", "state", "marker_id",
                "target_x", "target_y", "target_z",
                "ee_x", "ee_y", "ee_z",
                "error_m", "fps"
            ])
            self.logger.info(f"Recording session telemetry to {csv_path}")
        except Exception as e:
            self.logger.error(f"Failed to initialize CSV logger: {e}")

    def process_frame(
        self,
        frame: Optional[np.ndarray] = None,
        wall_dt: Optional[float] = None,
    ) -> FrameResult:
        """Executes a complete frame iteration: perception, filtering, mapping, IK, and physics stepping."""
        now = time.perf_counter()
        if wall_dt is None:
            effective_dt = (now - self._last_loop_time) if self._last_loop_time is not None else (1.0 / max(self.config.camera.fps, 1))
        else:
            effective_dt = wall_dt
        self._last_loop_time = now

        # --- PHASE 1: ACQUIRE FRAME ---
        active_id = self.config.aruco.target_marker_id if self.state_machine.mode == "MANUAL" else (
            self.config.aruco.pick_marker_id if self.state_machine.state in (RobotState.APPROACH, RobotState.PICK)
            else self.config.aruco.place_marker_id
        )

        if frame is None:
            success, raw_frame = self.camera.read(active_marker_id=active_id, state=self.state_machine.state_name)
            if not success or raw_frame is None:
                return FrameResult(
                    success=False,
                    state_name=self.state_machine.state_name,
                    mode=self.state_machine.mode,
                    control_mode=self.config.control_mode,
                    transform_mode=self.config.transform.transform_mode,
                )
        else:
            raw_frame = frame

        fps = self.fps_counter.update()
        display_frame = raw_frame.copy()

        # --- PHASE 2: ARUCO DETECTION & POSE ESTIMATION ---
        detections = self.aruco_detector.detect(raw_frame)
        self.aruco_detector.draw_detections(
            display_frame,
            detections,
            highlight_id=active_id if self.state_machine.mode == "MANUAL" else None,
        )

        target_pose = None
        pick_pose = None
        place_pose = None

        for det in detections:
            pose = self.pose_estimator.estimate_pose(det)
            if pose is not None:
                self.pose_estimator.draw_axes(display_frame, pose)
                if det.id == self.config.aruco.target_marker_id:
                    target_pose = pose
                elif det.id == self.config.aruco.pick_marker_id:
                    pick_pose = pose
                elif det.id == self.config.aruco.place_marker_id:
                    place_pose = pose

        # --- PHASE 3: FILTERING & WORKSPACE MAPPING ---
        raw_pos_cam = None
        filt_pos_cam = None
        target_robot_pos = None
        target_robot_orn = None
        was_clamped = False

        if target_pose is not None:
            raw_pos_cam = (target_pose.x, target_pose.y, target_pose.z)

            # Filter 6-DoF Pose (Position + SLERP Orientation)
            filt_pos, filt_quat = self.pose_filter.update(
                np.array(raw_pos_cam),
                target_pose.quaternion_xyzw,
                timestamp=now,
            )
            filt_pos_cam = (float(filt_pos[0]), float(filt_pos[1]), float(filt_pos[2]))

            # Map to Robot Base Coordinate Frame
            mapped_target = self.workspace_mapper.map_camera_to_robot(
                filt_pos,
                filt_quat if self.config.control_mode == "6dof" else None,
                timestamp=now,
            )
            if mapped_target.is_valid:
                target_robot_pos = mapped_target.position
                target_robot_orn = mapped_target.orientation
                was_clamped = mapped_target.is_clamped
                self._last_commanded_target_pos = target_robot_pos.copy()
                self._last_commanded_target_orn = target_robot_orn.copy()
        else:
            self.pose_filter.reset()

        # --- PHASE 4: STATE MACHINE & ROBOT CONTROL ---
        commanded_cartesian_target = None
        commanded_orientation_target = None
        ik_status = "IDLE"

        if not self._is_paused:
            if self.state_machine.mode == "MANUAL":
                cmd_target, is_tracking = self.state_machine.update_manual_mode(
                    marker_detected=(target_pose is not None),
                    target_pos_robot=target_robot_pos,
                )
                commanded_cartesian_target = cmd_target

                if is_tracking and target_robot_orn is not None:
                    commanded_orientation_target = target_robot_orn
                    self._last_commanded_target_orn = target_robot_orn.copy()
                elif self.state_machine.state == RobotState.HOLD:
                    commanded_orientation_target = self._last_commanded_target_orn.copy()
                else:
                    commanded_orientation_target = self._last_commanded_target_orn.copy()
            else:  # AUTO MODE
                current_ee_pos, _ = self.simulator.controller.get_end_effector_pose()

                p_pick_robot = None
                if pick_pose is not None:
                    mapped_pick = self.workspace_mapper.map_camera_to_robot(
                        np.array([pick_pose.x, pick_pose.y, pick_pose.z]),
                        enforce_slew_rate=False,
                    )
                    if mapped_pick.is_valid:
                        p_pick_robot = mapped_pick.position

                p_place_robot = None
                if place_pose is not None:
                    mapped_place = self.workspace_mapper.map_camera_to_robot(
                        np.array([place_pose.x, place_pose.y, place_pose.z]),
                        enforce_slew_rate=False,
                    )
                    if mapped_place.is_valid:
                        p_place_robot = mapped_place.position

                cmd_target, action_status = self.state_machine.update_auto_mode(
                    current_ee_pos=current_ee_pos,
                    marker_1_pos=p_pick_robot,
                    marker_2_pos=p_place_robot,
                    gripper_attach_fn=lambda: self.simulator.gripper.attach_object(self.simulator.pick_cube_id),
                    gripper_detach_fn=lambda: self.simulator.gripper.detach_object(),
                    on_targets_stabilized_fn=lambda p_pick, p_place: (
                        self.simulator.set_pick_object_position(p_pick),
                        self.simulator.set_place_target_position(p_place),
                    ),
                )
                commanded_cartesian_target = cmd_target
                commanded_orientation_target = self.config.robot.default_ee_orientation

            # --- PHASE 5: INVERSE KINEMATICS & MOTOR COMMAND ---
            if commanded_cartesian_target is not None:
                self.simulator.set_target_visual_position(commanded_cartesian_target)

                # Solve IK (3-DoF or 6-DoF depending on control_mode)
                ik_orn = commanded_orientation_target if self.config.control_mode == "6dof" else None
                ik_res = self.simulator.ik_solver.solve(
                    target_position=commanded_cartesian_target,
                    target_orientation=ik_orn,
                )
                ik_status = ik_res.status_message
                if ik_res.success:
                    self.simulator.controller.set_arm_joint_positions(ik_res.joint_positions)
        else:
            if self._paused_joint_positions is not None:
                self.simulator.controller.set_arm_joint_positions(self._paused_joint_positions)
            ik_status = "HOLD (PAUSED)"
            ee_frozen_pos, _ = self.simulator.controller.get_end_effector_pose()
            commanded_cartesian_target = ee_frozen_pos
            commanded_orientation_target = self._last_commanded_target_orn.copy()

        # --- PHASE 6: SIMULATION PHYSICS STEPPING ---
        substeps = self.simulator.step(wall_dt=effective_dt)
        self.sim_fps_counter.update()

        ee_pos, ee_orn = self.simulator.controller.get_end_effector_pose()
        self.simulator.update_trajectory_visualization(ee_pos)
        pos_error = float(np.linalg.norm(ee_pos - commanded_cartesian_target)) if commanded_cartesian_target is not None else None

        result = FrameResult(
            success=True,
            frame=raw_frame,
            display_frame=display_frame,
            detections=detections,
            target_pose=target_pose,
            pick_pose=pick_pose,
            place_pose=place_pose,
            raw_pos_cam=raw_pos_cam,
            filtered_pos_cam=filt_pos_cam,
            commanded_position=commanded_cartesian_target,
            commanded_orientation=commanded_orientation_target,
            ee_position=ee_pos,
            ee_orientation=ee_orn,
            tracking_error_m=pos_error,
            ik_status=ik_status,
            state_name=self.state_machine.state_name,
            mode=self.state_machine.mode,
            control_mode=self.config.control_mode,
            transform_mode=self.config.transform.transform_mode,
            was_clamped=was_clamped,
            physics_substeps=substeps,
            fps=fps,
        )
        self._last_frame_result = result
        return result

    def run(self) -> None:
        """Main real-time perception-control loop."""
        self.logger.info("Starting real-time execution loop. Press [Q] to quit.")
        last_time = time.perf_counter()

        try:
            while self._is_running:
                now = time.perf_counter()
                loop_dt = now - last_time
                last_time = now
                self._frames_processed += 1

                # Check max frames bound
                if self.config.max_frames > 0 and self._frames_processed > self.config.max_frames:
                    self.logger.info(f"Reached max frames limit ({self.config.max_frames}). Exiting cleanly.")
                    break

                res = self.process_frame(wall_dt=loop_dt)
                if not res.success or res.frame is None:
                    self.logger.warning("Frame read failed.")
                    time.sleep(0.01)
                    continue

                display_frame = res.display_frame if res.display_frame is not None else res.frame.copy()

                # Telemetry HUD Rendering
                active_id = self.config.aruco.target_marker_id if res.mode == "MANUAL" else (
                    self.config.aruco.pick_marker_id if res.state_name in ("APPROACH", "PICK")
                    else self.config.aruco.place_marker_id
                )
                telem_data = TelemetryData(
                    mode=res.mode,
                    state=res.state_name if not self._is_paused else "PAUSED",
                    marker_id=res.target_pose.marker_id if res.target_pose else (active_id if res.mode == "AUTO" else None),
                    tracking_active=(res.target_pose is not None and res.state_name == "TRACK"),
                    raw_pos_cam=res.raw_pos_cam,
                    filtered_pos_cam=res.filtered_pos_cam,
                    marker_distance_m=res.target_pose.distance_m if res.target_pose else None,
                    robot_target_pos=tuple(res.commanded_position) if res.commanded_position is not None else None,
                    robot_ee_pos=(float(res.ee_position[0]), float(res.ee_position[1]), float(res.ee_position[2])) if res.ee_position is not None else None,
                    tracking_error_m=res.tracking_error_m,
                    ik_status=res.ik_status,
                    fps=res.fps,
                    sim_fps=self.sim_fps_counter.fps,
                    physics_target_hz=self.config.simulation.target_physics_hz,
                    physics_substeps=res.physics_substeps,
                    physics_actual_step_rate=res.physics_substeps / max(loop_dt, 1e-4),
                    lost_tracking_time_s=self.state_machine.lost_tracking_duration_s,
                    is_calibrated=self.calibration.is_calibrated,
                    is_calibrated_extrinsics=self.workspace_mapper.tf_config.is_calibrated_extrinsics,
                    transform_mode=self.workspace_mapper.tf_config.transform_mode,
                    workspace_clamped=res.was_clamped,
                    debug_mode=self.config.debug,
                )

                self.telemetry_overlay.render(display_frame, telem_data)

                # Video Recording
                if self.record_video:
                    if self._video_writer is None:
                        from tools.record_demo import create_video_writer
                        demo_dir = Path("demo/recordings")
                        demo_dir.mkdir(parents=True, exist_ok=True)
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        out_mp4 = demo_dir / f"session_{timestamp}.mp4"
                        fh, fw = display_frame.shape[:2]
                        self._video_writer, self._video_path = create_video_writer(
                            out_mp4, fw, fh, fps=float(self.config.camera.fps or 30.0)
                        )
                    # Draw recording badge
                    cv2.circle(display_frame, (display_frame.shape[1] - 30, 25), 8, (0, 0, 255), -1)
                    cv2.putText(display_frame, "REC", (display_frame.shape[1] - 65, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
                    self._video_writer.write(display_frame)

                # CSV Telemetry Recording
                if self._csv_writer and res.commanded_position is not None and res.ee_position is not None:
                    self._csv_writer.writerow([
                        f"{time.time():.4f}",
                        res.state_name,
                        res.target_pose.marker_id if res.target_pose else -1,
                        f"{res.commanded_position[0]:.4f}",
                        f"{res.commanded_position[1]:.4f}",
                        f"{res.commanded_position[2]:.4f}",
                        f"{res.ee_position[0]:.4f}",
                        f"{res.ee_position[1]:.4f}",
                        f"{res.ee_position[2]:.4f}",
                        f"{res.tracking_error_m:.4f}" if res.tracking_error_m else 0.0,
                        f"{res.fps:.1f}",
                    ])

                # Only show GUI window if not headless
                if not self.headless:
                    cv2.imshow("VisionRobotTwin - Perception HUD", display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key != 255:
                        self._handle_keypress(key, display_frame)

        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received.")
        finally:
            self.cleanup()

    def _handle_keypress(self, key: int, last_display_frame: Optional[np.ndarray] = None) -> None:
        """Handles interactive keyboard commands."""
        char = chr(key).lower() if 0 <= key <= 127 else ""

        if key in (ord("q"), 27):  # Q or ESC
            self.logger.info("Quit command triggered.")
            self._is_running = False

        elif char == "h":  # Home
            self.logger.info("Homing robot manipulator.")
            self.pose_filter.reset()
            self.workspace_mapper.reset()
            self.simulator.controller.reset_to_home()
            self.state_machine.transition_to(RobotState.HOME, "User pressed Home")

        elif key == ord(" "):  # Space: Hold / Resume
            self._is_paused = not self._is_paused
            if self._is_paused:
                self._paused_joint_positions = self.simulator.controller.get_current_joint_positions()
            else:
                self._paused_joint_positions = None
            self.logger.info(f"{'Paused (HOLD)' if self._is_paused else 'Resumed tracking'}.")

        elif char == "m":  # Manual Mode
            self.pose_filter.reset()
            self.workspace_mapper.reset()
            self.state_machine.set_mode("MANUAL")

        elif char == "a":  # Auto Mode
            self.pose_filter.reset()
            self.workspace_mapper.reset()
            self.state_machine.set_mode("AUTO")

        elif char == "r":  # Reset
            self.logger.info("Resetting simulation and tracking filters.")
            self.simulator.controller.reset_to_home()
            self.simulator.gripper.detach_object()
            self.pose_filter.reset()
            self.workspace_mapper.reset()
            self.state_machine.reset()

        elif char == "t":  # Toggle Trajectory
            enabled = self.simulator.toggle_trajectory()
            self.logger.info(f"Trajectory visualization {'enabled' if enabled else 'disabled'}.")

        elif char == "s":  # Screenshot
            self._capture_screenshots(last_display_frame, self._last_frame_result)

        elif char == "d":  # Debug Toggle
            self.config.debug = not self.config.debug
            self.logger.info(f"Debug telemetry {'enabled' if self.config.debug else 'disabled'}.")

        elif char == "c":  # Calibration Info
            self.logger.info(
                f"Camera Intrinsics: {'Calibrated' if self.calibration.is_calibrated else 'Fallback pinhole'} | "
                f"Extrinsics: {'Calibrated' if self.workspace_mapper.tf_config.is_calibrated_extrinsics else 'Nominal'}"
            )

    def _capture_screenshots(
        self,
        last_display_frame: Optional[np.ndarray] = None,
        last_result: Optional[FrameResult] = None,
    ) -> None:
        """Captures screenshots of camera HUD and PyBullet, and creates adjacent JSON metadata."""
        self.config.screenshots_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if last_display_frame is not None:
            hud_path = self.config.screenshots_dir / f"hud_{timestamp}.png"
            cv2.imwrite(str(hud_path), last_display_frame)
            self.logger.info(f"Saved Camera HUD screenshot: {hud_path}")

        bullet_path = self.config.screenshots_dir / f"sim_{timestamp}.png"
        self.simulator.capture_screenshot(bullet_path)
        self.logger.info(f"Saved Simulation screenshot: {bullet_path}")

        # Adjacent JSON metadata
        meta = {
            "timestamp": datetime.now().isoformat(),
            "version": __version__,
            "mode": self.state_machine.mode,
            "control_mode": self.config.control_mode,
            "transform_mode": self.config.transform.transform_mode,
            "state": self.state_machine.state_name,
            "marker_id": last_result.target_pose.marker_id if (last_result and last_result.target_pose) else None,
            "camera_pose": {
                "raw_pos_m": [float(v) for v in last_result.raw_pos_cam] if (last_result and last_result.raw_pos_cam) else None,
                "filtered_pos_m": [float(v) for v in last_result.filtered_pos_cam] if (last_result and last_result.filtered_pos_cam) else None,
            },
            "robot_target_pos": [float(v) for v in last_result.commanded_position] if (last_result and last_result.commanded_position is not None) else None,
            "robot_target_orn": [float(v) for v in last_result.commanded_orientation] if (last_result and last_result.commanded_orientation is not None) else None,
            "ee_pos": [float(v) for v in last_result.ee_position] if (last_result and last_result.ee_position is not None) else None,
            "ee_orn": [float(v) for v in last_result.ee_orientation] if (last_result and last_result.ee_orientation is not None) else None,
            "ik_status": last_result.ik_status if last_result else "UNKNOWN",
            "calibration_status": "CALIBRATED" if self.calibration.is_calibrated else "FALLBACK PINHOLE",
            "extrinsics_status": "CALIBRATED" if self.workspace_mapper.tf_config.is_calibrated_extrinsics else "NOMINAL",
        }
        json_path = self.config.screenshots_dir / f"session_{timestamp}.json"
        try:
            with open(json_path, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
            self.logger.info(f"Saved Screenshot metadata: {json_path}")
        except Exception as e:
            self.logger.warning(f"Failed to save screenshot metadata: {e}")

    def cleanup(self) -> None:
        """Releases all hardware, windows, files, and physics server resources cleanly."""
        self.logger.info("Cleaning up application resources...")
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
        if self._video_writer:
            try:
                self._video_writer.release()
                self.logger.info(f"Closed video recording: {self._video_path}")
            except Exception:
                pass
        if self.simulator:
            self.simulator.close()
        if self._csv_file:
            try:
                self._csv_file.close()
                self.logger.info("Closed session CSV log.")
            except Exception:
                pass
        self.logger.info("VisionRobotTwin terminated cleanly.")


def main() -> int:
    """Application main entry point."""
    args = parse_arguments()

    if args.calibration_status:
        print_calibration_status()
        return 0

    logger = setup_logger(debug=args.debug)

    config = get_default_config()
    config.camera.camera_index = args.camera
    config.camera.width = args.width
    config.camera.height = args.height
    config.camera.synthetic_mode = args.synthetic
    config.camera.allow_synthetic_fallback = args.allow_synthetic_fallback
    config.state_machine.auto_demo = args.auto_demo
    config.mode = args.mode
    config.control_mode = args.control_mode
    config.transform.transform_mode = args.transform_mode
    config.debug = args.debug
    config.max_frames = args.max_frames
    if args.headless:
        config.simulation.gui = False

    try:
        app = VisionRobotTwinApp(
            config=config,
            headless_sim=args.headless,
            record_data=args.record_data,
            record_video=args.record,
        )
        app.run()
        return 0
    except Exception as e:
        logger.critical(f"Fatal application error: {e}", exc_info=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
