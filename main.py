"""VisionRobotTwin: Real-Time Vision-Guided Robotic Manipulator Digital Twin.

Main entry point integrating real-time computer vision, ArUco 6-DoF pose estimation,
SE(3) coordinate transformations, workspace mapping, inverse kinematics, and PyBullet
digital twin control of a Franka Emika Panda manipulator.
"""

import sys
import time
import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Optional
import cv2
import numpy as np

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
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--width", type=int, default=1280, help="Camera width resolution")
    parser.add_argument("--height", type=int, default=720, help="Camera height resolution")
    parser.add_argument("--mode", type=str, choices=["manual", "auto"], default="manual", help="Initial operational mode")
    parser.add_argument("--synthetic", action="store_true", help="Force synthetic simulated video stream")
    parser.add_argument("--headless", action="store_true", help="Run PyBullet without GUI window")
    parser.add_argument("--debug", action="store_true", help="Enable verbose debug logging")
    parser.add_argument("--record-data", action="store_true", help="Record trajectory session data to CSV")
    return parser.parse_args()


class VisionRobotTwinApp:
    """Master application controller orchestrating vision, kinematics, and simulation."""

    def __init__(self, config: AppConfig, headless_sim: bool = False, record_data: bool = False):
        self.config = config
        self.logger = get_logger("VisionRobotTwin")

        self.logger.info("=" * 70)
        self.logger.info(" INITIALIZING VISION-ROBOT DIGITAL TWIN")
        self.logger.info(f" Python Version: {sys.version.split()[0]} | OpenCV Version: {cv2.__version__}")
        self.logger.info(f" Mode: {config.mode.upper()} | Camera Index: {config.camera.camera_index}")
        self.logger.info("=" * 70)

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
        self.workspace_mapper = WorkspaceMapper(self.config.workspace)

        # 3. Robotics & PyBullet Digital Twin
        self.simulator = PyBulletSimulator(self.config, headless=headless_sim)
        self.state_machine = RoboticStateMachine(self.config.state_machine, self.config.workspace)
        self.state_machine.set_mode(self.config.mode)

        # 4. Telemetry & FPS Monitoring
        self.fps_counter = FPSCounter()
        self.sim_fps_counter = FPSCounter()
        self.telemetry_overlay = TelemetryOverlay()

        # 5. Session CSV Logging
        self.record_data = record_data
        self._csv_file = None
        self._csv_writer = None
        if self.record_data:
            self._init_csv_recorder()

        self._is_running = True
        self._is_paused = False

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

    def run(self) -> None:
        """Main real-time perception-control loop."""
        self.logger.info("Starting real-time execution loop. Press [Q] to quit.")

        try:
            while self._is_running:
                loop_start = time.perf_counter()

                # --- PHASE 1: CAMERA PERCEPTION ---
                active_id = self.config.aruco.target_marker_id if self.state_machine.mode == "MANUAL" else (
                    self.config.aruco.pick_marker_id if self.state_machine.state in (RobotState.APPROACH, RobotState.PICK)
                    else self.config.aruco.place_marker_id
                )
                success, frame = self.camera.read(active_marker_id=active_id, state=self.state_machine.state_name)
                if not success or frame is None:
                    self.logger.warning("Frame read failed.")
                    time.sleep(0.01)
                    continue

                fps = self.fps_counter.update()
                display_frame = frame.copy()

                # ArUco Detection
                detections = self.aruco_detector.detect(frame)
                self.aruco_detector.draw_detections(
                    display_frame,
                    detections,
                    highlight_id=active_id if self.state_machine.mode == "MANUAL" else None,
                )

                # Find relevant markers
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

                # --- PHASE 2: POSE FILTERING & WORKSPACE MAPPING ---
                raw_pos_cam = None
                filt_pos_cam = None
                marker_dist = None
                target_robot_pos = None
                was_clamped = False

                if target_pose is not None:
                    raw_pos_cam = (target_pose.x, target_pose.y, target_pose.z)
                    marker_dist = target_pose.distance_m

                    # Filter 6-DoF Pose
                    filt_pos, _ = self.pose_filter.update(
                        np.array(raw_pos_cam),
                        target_pose.quaternion_xyzw,
                        timestamp=loop_start,
                    )
                    filt_pos_cam = (float(filt_pos[0]), float(filt_pos[1]), float(filt_pos[2]))

                    # Map to Robot Base Cartesian Space
                    mapped_target = self.workspace_mapper.map_camera_to_robot(filt_pos)
                    if mapped_target.is_valid:
                        target_robot_pos = mapped_target.position
                        was_clamped = mapped_target.is_clamped
                else:
                    self.pose_filter.reset()

                # --- PHASE 3: STATE MACHINE & ROBOT CONTROL ---
                commanded_cartesian_target = None
                ik_status = "IDLE"

                if not self._is_paused:
                    if self.state_machine.mode == "MANUAL":
                        cmd_target, is_tracking = self.state_machine.update_manual_mode(
                            marker_detected=(target_pose is not None),
                            target_pos_robot=target_robot_pos,
                        )
                        commanded_cartesian_target = cmd_target
                    else:  # AUTO MODE
                        current_ee_pos, _ = self.simulator.controller.get_end_effector_pose()
                        
                        # Map pick and place markers if visually detected
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
                        )
                        commanded_cartesian_target = cmd_target

                    # --- PHASE 4: INVERSE KINEMATICS & MOTOR COMMAND ---
                    if commanded_cartesian_target is not None:
                        # Update visual target sphere in PyBullet
                        self.simulator.set_target_visual_position(commanded_cartesian_target)

                        # Solve IK
                        ik_res = self.simulator.ik_solver.solve(commanded_cartesian_target)
                        if ik_res.success:
                            ik_status = "OK"
                            self.simulator.controller.set_arm_joint_positions(ik_res.joint_positions)
                        else:
                            ik_status = "REJECTED"

                # Step simulation physics
                self.simulator.step()
                sim_fps = self.sim_fps_counter.update()

                # End effector forward kinematics & error telemetry
                ee_pos, _ = self.simulator.controller.get_end_effector_pose()
                self.simulator.update_trajectory_visualization(ee_pos)
                pos_error = float(np.linalg.norm(ee_pos - commanded_cartesian_target)) if commanded_cartesian_target is not None else None

                # --- PHASE 5: CSV TELEMETRY LOGGING ---
                if self._csv_writer and commanded_cartesian_target is not None:
                    self._csv_writer.writerow([
                        f"{time.time():.4f}",
                        self.state_machine.state_name,
                        target_pose.marker_id if target_pose else -1,
                        f"{commanded_cartesian_target[0]:.4f}",
                        f"{commanded_cartesian_target[1]:.4f}",
                        f"{commanded_cartesian_target[2]:.4f}",
                        f"{ee_pos[0]:.4f}",
                        f"{ee_pos[1]:.4f}",
                        f"{ee_pos[2]:.4f}",
                        f"{pos_error:.4f}" if pos_error else 0.0,
                        f"{fps:.1f}",
                    ])

                # --- PHASE 6: RENDER TELEMETRY HUD & DISPLAY ---
                telem_data = TelemetryData(
                    mode=self.state_machine.mode,
                    state=self.state_machine.state_name if not self._is_paused else "PAUSED",
                    marker_id=target_pose.marker_id if target_pose else (active_id if self.state_machine.mode == "AUTO" else None),
                    tracking_active=(target_pose is not None and self.state_machine.state == RobotState.TRACK),
                    raw_pos_cam=raw_pos_cam,
                    filtered_pos_cam=filt_pos_cam,
                    marker_distance_m=marker_dist,
                    robot_target_pos=tuple(commanded_cartesian_target) if commanded_cartesian_target is not None else None,
                    robot_ee_pos=(float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])),
                    tracking_error_m=pos_error,
                    ik_status=ik_status,
                    fps=fps,
                    sim_fps=sim_fps,
                    lost_tracking_time_s=self.state_machine.lost_tracking_duration_s,
                    is_calibrated=self.calibration.is_calibrated,
                    workspace_clamped=was_clamped,
                    debug_mode=self.config.debug,
                )

                self.telemetry_overlay.render(display_frame, telem_data)
                cv2.imshow("VisionRobotTwin - Perception HUD", display_frame)

                # --- PHASE 7: KEYBOARD CONTROLS ---
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
            self.simulator.controller.reset_to_home()
            self.state_machine.transition_to(RobotState.HOME, "User pressed Home")

        elif key == ord(" "):  # Space: Hold / Resume
            self._is_paused = not self._is_paused
            self.logger.info(f"{'Paused (HOLD)' if self._is_paused else 'Resumed tracking'}.")

        elif char == "m":  # Manual Mode
            self.state_machine.set_mode("MANUAL")

        elif char == "a":  # Auto Mode
            self.state_machine.set_mode("AUTO")

        elif char == "r":  # Reset
            self.logger.info("Resetting simulation and tracking filters.")
            self.simulator.controller.reset_to_home()
            self.simulator.gripper.detach_object()
            self.pose_filter.reset()
            self.workspace_mapper.reset()
            self.state_machine.transition_to(RobotState.SEARCH, "User reset")

        elif char == "t":  # Toggle Trajectory
            enabled = self.simulator.toggle_trajectory()
            self.logger.info(f"Trajectory visualization {'enabled' if enabled else 'disabled'}.")

        elif char == "s":  # Screenshot
            self._capture_screenshots(last_display_frame)

        elif char == "d":  # Debug Toggle
            self.config.debug = not self.config.debug
            self.logger.info(f"Debug telemetry {'enabled' if self.config.debug else 'disabled'}.")

        elif char == "c":  # Calibration Info
            self.logger.info(
                f"Camera Calibration Status: {'Calibrated' if self.calibration.is_calibrated else 'Uncalibrated default'}\n"
                f"fx={self.calibration.fx:.1f}, fy={self.calibration.fy:.1f}, "
                f"cx={self.calibration.cx:.1f}, cy={self.calibration.cy:.1f}"
            )

    def _capture_screenshots(self, last_display_frame: Optional[np.ndarray] = None) -> None:
        """Captures screenshots of both camera HUD and PyBullet simulator."""
        self.config.screenshots_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Save Camera HUD Frame
        if last_display_frame is not None:
            hud_path = self.config.screenshots_dir / f"hud_{timestamp}.png"
            cv2.imwrite(str(hud_path), last_display_frame)
            self.logger.info(f"Saved Camera HUD screenshot: {hud_path}")

        # Capture PyBullet screenshot
        bullet_path = self.config.screenshots_dir / f"sim_{timestamp}.png"
        self.simulator.capture_screenshot(bullet_path)
        self.logger.info(f"Saved Simulation screenshot: {bullet_path}")

    def cleanup(self) -> None:
        """Releases all hardware, windows, files, and physics server resources cleanly."""
        self.logger.info("Cleaning up application resources...")
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
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

    # Configure Logger
    logger = setup_logger(debug=args.debug)

    # Build Configuration
    config = get_default_config()
    config.camera.camera_index = args.camera
    config.camera.width = args.width
    config.camera.height = args.height
    config.camera.synthetic_mode = args.synthetic
    config.mode = args.mode
    config.debug = args.debug

    try:
        app = VisionRobotTwinApp(
            config=config,
            headless_sim=args.headless,
            record_data=args.record_data,
        )
        app.run()
        return 0
    except Exception as e:
        logger.critical(f"Fatal application error: {e}", exc_info=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
