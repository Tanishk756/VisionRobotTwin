"""PyBullet Physics Simulator Environment for Franka Emika Panda.

Encapsulates PyBullet physics client lifecycle, URDF asset loading, visual targets,
object position synchronization with perception, bounded trajectory debug line management,
and screenshot capture.
"""

from collections import deque
from pathlib import Path
from typing import Deque, List, Optional, Tuple
import pybullet as p
import pybullet_data
import numpy as np

from config.settings import AppConfig, SimulationConfig, WorkspaceConfig
from robotics.robot_controller import PandaRobotController
from robotics.inverse_kinematics import PandaIKSolver
from robotics.gripper import VirtualGripper
from utils.logger import get_logger

logger = get_logger("Robotics.Simulator")


class PyBulletSimulator:
    """Manages the full PyBullet simulation environment, physics stepping, and digital twin."""

    def __init__(self, config: AppConfig, headless: bool = False):
        self.config = config
        self.sim_config = config.simulation
        self.ws_config = config.workspace
        self.headless = headless

        self.client_id: int = -1
        self.plane_id: int = -1
        self.table_id: int = -1
        self.robot_id: int = -1
        self.target_sphere_id: int = -1
        self.pick_cube_id: int = -1
        self.place_cube_id: int = -1

        self.controller: Optional[PandaRobotController] = None
        self.ik_solver: Optional[PandaIKSolver] = None
        self.gripper: Optional[VirtualGripper] = None

        # Trajectory visualization debug lines (Strictly bounded lifecycle)
        self.show_trajectory: bool = True
        self._trajectory_points: Deque[np.ndarray] = deque(maxlen=self.sim_config.trajectory_history_len)
        self._trajectory_line_ids: Deque[int] = deque()

        self._initialize_pybullet()
        self._load_environment()
        self._setup_robot_and_kinematics()
        self._draw_workspace_bounds()

    def _initialize_pybullet(self) -> None:
        """Connects to PyBullet physics server."""
        mode = p.DIRECT if self.headless or not self.sim_config.gui else p.GUI
        self.client_id = p.connect(mode)
        if self.client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet physics server.")

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(*self.sim_config.gravity, physicsClientId=self.client_id)
        p.setTimeStep(self.sim_config.time_step, physicsClientId=self.client_id)

        if not self.headless and self.sim_config.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
            p.resetDebugVisualizerCamera(
                cameraDistance=self.sim_config.camera_distance,
                cameraYaw=self.sim_config.camera_yaw,
                cameraPitch=self.sim_config.camera_pitch,
                cameraTargetPosition=self.sim_config.camera_target_position,
                physicsClientId=self.client_id,
            )

        logger.info(f"PyBullet initialized in {'DIRECT' if self.headless else 'GUI'} mode (Client ID: {self.client_id})")

    def _load_environment(self) -> None:
        """Loads ground plane, table, target markers, and manipulable cube objects."""
        # Ground plane
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        # Table surface under robot workspace
        self.table_id = p.loadURDF(
            "table/table.urdf",
            basePosition=[0.5, 0.0, -0.62],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True,
            physicsClientId=self.client_id,
        )

        # Visual Target Sphere
        sphere_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.001, physicsClientId=self.client_id)
        sphere_vis = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=self.sim_config.target_sphere_radius,
            rgbaColor=list(self.sim_config.target_sphere_color),
            physicsClientId=self.client_id,
        )
        self.target_sphere_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=sphere_col,
            baseVisualShapeIndex=sphere_vis,
            basePosition=[self.ws_config.robot_center_x, self.ws_config.robot_center_y, self.ws_config.robot_center_z],
            physicsClientId=self.client_id,
        )

        # Pick object (Red Cube)
        cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], physicsClientId=self.client_id)
        cube_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.02, 0.02, 0.02],
            rgbaColor=[0.9, 0.2, 0.2, 1.0],
            physicsClientId=self.client_id,
        )
        self.pick_cube_id = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=cube_col,
            baseVisualShapeIndex=cube_vis,
            basePosition=[0.45, -0.20, 0.02],
            physicsClientId=self.client_id,
        )

        # Place target visual mat (Blue Pad)
        pad_vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=0.05,
            length=0.005,
            rgbaColor=[0.2, 0.5, 0.9, 0.8],
            physicsClientId=self.client_id,
        )
        self.place_cube_id = p.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=pad_vis,
            basePosition=[0.45, 0.20, 0.003],
            physicsClientId=self.client_id,
        )

    def _setup_robot_and_kinematics(self) -> None:
        """Loads Franka Panda URDF and configures controller, IK solver, and gripper."""
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
        self.robot_id = p.loadURDF(
            self.config.robot.urdf_path,
            basePosition=self.config.robot.base_position,
            baseOrientation=self.config.robot.base_orientation,
            useFixedBase=True,
            flags=flags,
            physicsClientId=self.client_id,
        )

        self.controller = PandaRobotController(
            physics_client_id=self.client_id,
            robot_id=self.robot_id,
            config=self.config.robot,
        )

        lows, highs, ranges, rests = self.controller.get_joint_limits()
        self.ik_solver = PandaIKSolver(
            physics_client_id=self.client_id,
            robot_id=self.robot_id,
            robot_config=self.config.robot,
            arm_joint_indices=self.controller.arm_joint_indices,
            lower_limits=lows,
            upper_limits=highs,
            joint_ranges=ranges,
            rest_poses=rests,
            end_effector_link_index=self.controller.ee_link_index,
        )

        self.gripper = VirtualGripper(
            physics_client_id=self.client_id,
            robot_id=self.robot_id,
            finger_joint_indices=self.controller.finger_joint_indices,
            ee_link_index=self.controller.ee_link_index,
            max_grasp_distance_m=self.config.state_machine.grasp_distance_threshold_m,
        )

    def _draw_workspace_bounds(self) -> None:
        """Renders subtle 3D bounding box indicating safe workspace limits."""
        if not self.sim_config.show_workspace_box or self.headless:
            return

        ws = self.ws_config
        corners = [
            [ws.x_min, ws.y_min, ws.z_min],
            [ws.x_max, ws.y_min, ws.z_min],
            [ws.x_max, ws.y_max, ws.z_min],
            [ws.x_min, ws.y_max, ws.z_min],
            [ws.x_min, ws.y_min, ws.z_max],
            [ws.x_max, ws.y_min, ws.z_max],
            [ws.x_max, ws.y_max, ws.z_max],
            [ws.x_min, ws.y_max, ws.z_max],
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        color = [0.4, 0.4, 0.6]
        for start_idx, end_idx in edges:
            p.addUserDebugLine(
                corners[start_idx],
                corners[end_idx],
                lineColorRGB=color,
                lineWidth=1.0,
                physicsClientId=self.client_id,
            )

    def set_target_visual_position(self, target_pos: np.ndarray) -> None:
        """Updates the 3D position of the visual target sphere."""
        p.resetBasePositionAndOrientation(
            self.target_sphere_id,
            posObj=list(target_pos),
            ornObj=[0, 0, 0, 1],
            physicsClientId=self.client_id,
        )

    def set_pick_object_position(self, position: np.ndarray) -> None:
        """Synchronizes pick cube position with vision target."""
        pos = [float(position[0]), float(position[1]), max(0.02, float(position[2]))]
        p.resetBasePositionAndOrientation(
            self.pick_cube_id,
            posObj=pos,
            ornObj=[0, 0, 0, 1],
            physicsClientId=self.client_id,
        )
        logger.info(f"Synchronized Pick Cube to {pos}")

    def set_place_target_position(self, position: np.ndarray) -> None:
        """Synchronizes place pad position with vision target."""
        pos = [float(position[0]), float(position[1]), 0.003]
        p.resetBasePositionAndOrientation(
            self.place_cube_id,
            posObj=pos,
            ornObj=[0, 0, 0, 1],
            physicsClientId=self.client_id,
        )
        logger.info(f"Synchronized Place Pad to {pos}")

    def update_trajectory_visualization(self, current_ee_pos: np.ndarray) -> None:
        """Appends current end-effector position and draws smooth 3D trajectory line with bounded memory."""
        if not self.show_trajectory or self.headless:
            return

        if len(self._trajectory_points) > 0:
            prev_pt = self._trajectory_points[-1]
            dist = np.linalg.norm(current_ee_pos - prev_pt)
            if dist > 0.005:  # Only add point if moved more than 5mm
                line_id = p.addUserDebugLine(
                    list(prev_pt),
                    list(current_ee_pos),
                    lineColorRGB=[0.1, 0.8, 0.4],
                    lineWidth=2.0,
                    lifeTime=0,
                    physicsClientId=self.client_id,
                )
                self._trajectory_line_ids.append(line_id)
                self._trajectory_points.append(current_ee_pos.copy())

                # If queue exceeds capacity, remove and free oldest debug line in PyBullet
                while len(self._trajectory_line_ids) > self.sim_config.trajectory_history_len:
                    old_id = self._trajectory_line_ids.popleft()
                    try:
                        p.removeUserDebugItem(old_id, physicsClientId=self.client_id)
                    except Exception:
                        pass
        else:
            self._trajectory_points.append(current_ee_pos.copy())

    def clear_trajectory(self) -> None:
        """Clears all drawn trajectory lines and frees debug items."""
        while self._trajectory_line_ids:
            line_id = self._trajectory_line_ids.popleft()
            try:
                p.removeUserDebugItem(line_id, physicsClientId=self.client_id)
            except Exception:
                pass
        self._trajectory_points.clear()

    def toggle_trajectory(self) -> bool:
        """Toggles trajectory visualization on/off."""
        self.show_trajectory = not self.show_trajectory
        if not self.show_trajectory:
            self.clear_trajectory()
        return self.show_trajectory

    def step(self) -> None:
        """Advances the physics simulation by one time step."""
        p.stepSimulation(physicsClientId=self.client_id)

    def capture_screenshot(self, output_path: Path) -> bool:
        """Renders OpenGL view and saves screenshot image."""
        try:
            path = Path(output_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            w, h, rgb_img, _, _ = p.getCameraImage(
                width=1280,
                height=720,
                physicsClientId=self.client_id,
            )
            rgb_arr = np.asarray(rgb_img, dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
            import cv2
            bgr_arr = cv2.cvtColor(rgb_arr, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(path), bgr_arr)
            logger.info(f"Saved PyBullet screenshot to {path}")
            return True
        except Exception as e:
            logger.error(f"Failed to capture PyBullet screenshot: {e}")
            return False

    def close(self) -> None:
        """Disconnects PyBullet physics client safely and frees all resources."""
        self.clear_trajectory()
        if self.client_id >= 0:
            try:
                p.disconnect(physicsClientId=self.client_id)
                logger.info("PyBullet physics client disconnected.")
            except Exception as e:
                logger.warning(f"Error disconnecting PyBullet: {e}")
            self.client_id = -1
