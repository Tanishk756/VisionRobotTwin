"""Generates a 15-second high-fidelity demonstration animated GIF (demo/demo.gif).

Combines side-by-side:
- Left View: Real-Time Camera Perception HUD with ArUco marker tracking & telemetry.
- Right View: PyBullet 3D Franka Emika Panda Digital Twin Simulation with trajectory lines.
"""

import sys
import time
from pathlib import Path
import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from config.settings import get_default_config
from main import VisionRobotTwinApp
from robotics.state_machine import RobotState
from utils.telemetry import TelemetryData
from utils.logger import setup_logger

logger = setup_logger("DemoGenerator")


def generate_demo_gif(
    output_path: Path = Path("demo/demo.gif"),
    duration_s: float = 15.0,
    fps: int = 15,
) -> None:
    """Records a 15-second simulation session and saves it as an optimized animated GIF."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    total_frames = int(duration_s * fps)
    frame_interval_s = 1.0 / fps

    print(f"Generating {duration_s}s demo GIF at {fps} FPS ({total_frames} frames)...")

    # Configure application in synthetic mode with headless simulation
    config = get_default_config()
    config.camera.synthetic_mode = True
    config.camera.width = 640
    config.camera.height = 480
    config.simulation.gui = False

    app = VisionRobotTwinApp(config, headless_sim=True)

    gif_frames = []
    start_sim_time = time.time()

    for frame_idx in range(total_frames):
        sim_time = frame_idx * frame_interval_s

        # First 8 seconds: Manual Tracking Mode
        # Remaining 7 seconds: Autonomous Pick and Place Mode
        if sim_time < 8.0:
            if app.state_machine.mode != "MANUAL":
                app.state_machine.set_mode("MANUAL")
            active_marker_id = 0
            curr_state_str = "MANUAL"
        else:
            if app.state_machine.mode != "AUTO":
                app.state_machine.set_mode("AUTO")
            active_marker_id = 1 if app.state_machine.state in (RobotState.APPROACH, RobotState.PICK) else 2
            curr_state_str = app.state_machine.state_name

        # 1. Camera Frame Perception
        success, frame = app.camera.read(active_marker_id=active_marker_id, state=curr_state_str)
        if not success or frame is None:
            continue

        display_frame = frame.copy()
        detections = app.aruco_detector.detect(frame)
        app.aruco_detector.draw_detections(display_frame, detections, highlight_id=active_marker_id)

        target_pose = None
        for det in detections:
            p = app.pose_estimator.estimate_pose(det)
            if p is not None:
                app.pose_estimator.draw_axes(display_frame, p)
                if det.id == active_marker_id:
                    target_pose = p

        # 2. Kinematics & Control
        raw_pos_cam = None
        filt_pos_cam = None
        target_robot_pos = None
        was_clamped = False

        if target_pose is not None:
            raw_pos_cam = (target_pose.x, target_pose.y, target_pose.z)
            filt_pos, _ = app.pose_filter.update(np.array(raw_pos_cam), target_pose.quaternion_xyzw)
            filt_pos_cam = (float(filt_pos[0]), float(filt_pos[1]), float(filt_pos[2]))
            mapped = app.workspace_mapper.map_camera_to_robot(filt_pos)
            if mapped.is_valid:
                target_robot_pos = mapped.position
                was_clamped = mapped.is_clamped

        commanded_target = None
        ik_status = "OK"

        if app.state_machine.mode == "MANUAL":
            cmd_tgt, is_tracking = app.state_machine.update_manual_mode(
                marker_detected=(target_pose is not None),
                target_pos_robot=target_robot_pos,
            )
            commanded_target = cmd_tgt
        else:
            ee_p, _ = app.simulator.controller.get_end_effector_pose()
            cmd_tgt, _ = app.state_machine.update_auto_mode(
                current_ee_pos=ee_p,
                gripper_attach_fn=lambda: app.simulator.gripper.attach_object(app.simulator.pick_cube_id),
                gripper_detach_fn=lambda: app.simulator.gripper.detach_object(),
            )
            commanded_target = cmd_tgt

        if commanded_target is not None:
            app.simulator.set_target_visual_position(commanded_target)
            ik_res = app.simulator.ik_solver.solve(commanded_target)
            if ik_res.success:
                app.simulator.controller.set_arm_joint_positions(ik_res.joint_positions)

        # Step physics
        for _ in range(8):  # 8 physics steps per video frame -> 120 Hz physics
            app.simulator.step()

        ee_pos, _ = app.simulator.controller.get_end_effector_pose()
        app.simulator.update_trajectory_visualization(ee_pos)
        pos_err = float(np.linalg.norm(ee_pos - commanded_target)) if commanded_target is not None else 0.0

        # Render Left View: OpenCV HUD
        telem = TelemetryData(
            mode=app.state_machine.mode,
            state=app.state_machine.state_name,
            marker_id=active_marker_id if target_pose else None,
            tracking_active=(target_pose is not None and app.state_machine.state == RobotState.TRACK),
            raw_pos_cam=raw_pos_cam,
            filtered_pos_cam=filt_pos_cam,
            marker_distance_m=target_pose.distance_m if target_pose else None,
            robot_target_pos=tuple(commanded_target) if commanded_target is not None else None,
            robot_ee_pos=(float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])),
            tracking_error_m=pos_err,
            ik_status=ik_status,
            fps=float(fps),
            sim_fps=240.0,
            lost_tracking_time_s=app.state_machine.lost_tracking_duration_s,
            is_calibrated=False,
            workspace_clamped=was_clamped,
        )
        app.telemetry_overlay.render(display_frame, telem)

        # Render Right View: PyBullet Simulator View
        import pybullet as p_bullet
        w_sim, h_sim, rgb_img, _, _ = p_bullet.getCameraImage(
            width=640,
            height=480,
            physicsClientId=app.simulator.client_id,
        )
        sim_rgb = np.asarray(rgb_img, dtype=np.uint8).reshape((h_sim, w_sim, 4))[:, :, :3]
        sim_bgr = cv2.cvtColor(sim_rgb, cv2.COLOR_RGB2BGR)

        # Add title banner to simulation view
        cv2.rectangle(sim_bgr, (15, 10), (w_sim - 15, 52), (20, 20, 24), -1)
        cv2.rectangle(sim_bgr, (15, 10), (w_sim - 15, 52), (55, 65, 81), 1)
        cv2.putText(
            sim_bgr,
            "PYBULLET 3D DIGITAL TWIN (FRANKA PANDA)",
            (30, 38),
            cv2.FONT_HERSHEY_DUPLEX,
            0.55,
            (46, 204, 113),
            1,
            cv2.LINE_AA,
        )

        # Composite side-by-side view (Left: 640x480, Right: 640x480 -> Total: 1280x480)
        composite_bgr = np.hstack((display_frame, sim_bgr))

        # Downscale slightly for web/GitHub optimization (960x360)
        target_w, target_h = 960, 360
        resized = cv2.resize(composite_bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
        rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        # Convert to PIL Image
        pil_img = Image.fromarray(rgb_frame)
        gif_frames.append(pil_img)

        if (frame_idx + 1) % 30 == 0:
            print(f"Processed {frame_idx + 1}/{total_frames} frames ({(frame_idx+1)/fps:.1f}s / {duration_s:.1f}s)...")

    app.cleanup()

    print(f"Saving animated GIF to {output_path}...")
    if gif_frames:
        frame_duration_ms = int(1000.0 / fps)
        gif_frames[0].save(
            str(output_path),
            save_all=True,
            append_images=gif_frames[1:],
            duration=frame_duration_ms,
            loop=0,
            optimize=True,
        )
        file_size_mb = output_path.stat().st_size / (1024 * 1024)
        print(f"[SUCCESS] Demo GIF generated successfully -> {output_path} ({file_size_mb:.2f} MB)")
    else:
        print("[ERROR] No frames captured for GIF.")


if __name__ == "__main__":
    generate_demo_gif()
