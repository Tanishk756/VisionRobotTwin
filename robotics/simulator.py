"""PyBullet Physics Simulator Environment for Multi-Robot Digital Twins.

Encapsulates PyBullet physics client lifecycle, URDF asset loading for supported
manipulators (Franka Emika Panda, KUKA LBR iiwa), visual targets,
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
from robotics.robot_model import RobotModelSpec
from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController, PandaRobotController
from robotics.inverse_kinematics import GenericIKSolver, PandaIKSolver
from robotics.gripper import VirtualGripper
from utils.simulation_clock import SimulationClock
from utils.logger import get_logger

logger = get_logger("Robotics.Simulator")


class PyBulletSimulator:
    """Manages the full PyBullet simulation environment, physics stepping, and digital twin."""

    def __init__(self, config: AppConfig, headless: bool = False):
        self.config = config
        self.sim_config = config.simulation
        self.ws_config = config.workspace
        self.headless = headless

        self.sim_clock = SimulationClock(
            target_physics_hz=self.sim_config.target_physics_hz,
            simulation_time_step=self.sim_config.time_step,
            max_substeps_per_iteration=self.sim_config.max_substeps_per_frame,
        )

        self.client_id: int = -1
        self.plane_id: int = -1
        self.table_id: int = -1
        self.robot_id: int = -1
        self.target_sphere_id: int = -1
        self.pick_cube_id: int = -1
        self.place_cube_id: int = -1
        self.obstacle_ids: List[int] = []

        self.robot_spec: Optional[RobotModelSpec] = None
        self.controller: Optional[GenericRobotController] = None
        self.ik_solver: Optional[GenericIKSolver] = None
        self.gripper: Optional[VirtualGripper] = None
        self.collision_checker: Optional[object] = None

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

        # Visual Target Sphere (pure visual marker, no collision body)
        sphere_vis = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=self.sim_config.target_sphere_radius,
            rgbaColor=list(self.sim_config.target_sphere_color),
            physicsClientId=self.client_id,
        )
        self.target_sphere_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
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

        # Load obstacles if configured scene requires them
        if getattr(self.config, "scene_type", "default") == "obstacles":
            # Obstacle 1: Box obstacle in central-right workspace
            col_obs1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.12], physicsClientId=self.client_id)
            vis_obs1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.12], rgbaColor=[0.85, 0.25, 0.25, 1.0], physicsClientId=self.client_id)
            obs1_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=col_obs1,
                baseVisualShapeIndex=vis_obs1,
                basePosition=[0.55, 0.0, 0.12],
                physicsClientId=self.client_id,
            )
            self.obstacle_ids.append(obs1_id)

            # Obstacle 2: Cylinder pillar in left workspace
            col_obs2 = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.04, length=0.20, physicsClientId=self.client_id)
            vis_obs2 = p.createVisualShape(p.GEOM_CYLINDER, radius=0.04, length=0.20, rgbaColor=[0.25, 0.75, 0.35, 1.0], physicsClientId=self.client_id)
            obs2_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=col_obs2,
                baseVisualShapeIndex=vis_obs2,
                basePosition=[0.38, -0.28, 0.10],
                physicsClientId=self.client_id,
            )
            self.obstacle_ids.append(obs2_id)
            logger.info(f"Loaded {len(self.obstacle_ids)} obstacle objects into simulation scene.")

    def _setup_robot_and_kinematics(self) -> None:
        """Loads Robot URDF and configures generic controller, IK solver, and gripper."""
        registry = get_robot_registry()
        robot_name = getattr(self.config, "robot_name", "panda")
        self.robot_spec = registry.get_robot_spec(robot_name)

        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.robot_id = p.loadURDF(
            self.robot_spec.urdf_path,
            basePosition=self.robot_spec.base_position,
            baseOrientation=self.robot_spec.base_orientation,
            useFixedBase=self.robot_spec.fixed_base,
            flags=flags,
            physicsClientId=self.client_id,
        )

        self.controller = GenericRobotController(
            physics_client_id=self.client_id,
            robot_id=self.robot_id,
            spec=self.robot_spec,
            config=self.config.robot,
        )

        self.kinematics_provider = self.controller.kinematics_provider

        lows, highs, ranges, rests = self.controller.get_joint_limits()
        self.ik_solver = GenericIKSolver(
            physics_client_id=self.client_id,
            robot_id=self.robot_id,
            arm_joint_indices=self.controller.arm_joint_indices,
            lower_limits=lows,
            upper_limits=highs,
            joint_ranges=ranges,
            rest_poses=rests,
            end_effector_link_index=self.controller.ee_link_index,
            max_reach_m=self.robot_spec.spherical_reach_m,
            min_reach_m=self.robot_spec.min_reach_m,
            default_ee_orientation=self.robot_spec.default_ee_orientation,
            kinematics_provider=self.kinematics_provider,
        )

        from robotics.collision import CollisionChecker
        allowed_mount_pairs = []
        if self.table_id is not None:
            allowed_mount_pairs.extend([
                (self.robot_id, -1, self.table_id, -1),
                (self.robot_id, 0, self.table_id, -1),
            ])

        self.collision_checker = CollisionChecker(
            physics_client_id=self.client_id,
            robot_id=self.robot_id,
            table_id=self.table_id,
            obstacle_ids=self.obstacle_ids,
            allowed_link_pairs=allowed_mount_pairs,
            allowed_self_link_pairs=self.robot_spec.allowed_self_collision_pairs,
        )

        from robotics.differential_ik import ResolvedRateController
        self.resolved_rate_controller = ResolvedRateController(
            physics_client_id=self.client_id,
            robot_controller=self.controller,
            enable_nullspace=True,
            kinematics_provider=self.kinematics_provider,
        )

        from robotics.motion_manager import MotionManager
        self.motion_manager = MotionManager(
            robot_controller=self.controller,
            ik_solver=self.ik_solver,
            collision_checker=self.collision_checker,
            trajectory_mode=getattr(self.config, "trajectory_mode", "quintic"),
            scene_type=getattr(self.config, "scene_type", "default"),
        )

        if self.robot_spec.capabilities.has_gripper:
            self.gripper = VirtualGripper(
                physics_client_id=self.client_id,
                robot_id=self.robot_id,
                finger_joint_indices=self.controller.finger_joint_indices,
                ee_link_index=self.controller.ee_link_index,
                max_grasp_distance_m=self.config.state_machine.grasp_distance_threshold_m,
            )
        else:
            self.gripper = None

    def get_current_manipulability(self):
        """Computes live geometric Jacobian and manipulability metrics for the active robot."""
        from robotics.kinematics import compute_manipulability
        curr_q = self.controller.get_current_joint_positions()
        _, _, J = self.kinematics_provider.compute_jacobian(curr_q)
        metrics = compute_manipulability(J)
        return metrics, J

    def get_current_collision_state(self):
        """Queries active collision and self-collision status."""
        if self.collision_checker is not None:
            return self.collision_checker.check_collision()
        return None

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

        new_pt = np.asarray(current_ee_pos, dtype=np.float64)
        if len(self._trajectory_points) > 0:
            last_pt = self._trajectory_points[-1]
            if np.linalg.norm(new_pt - last_pt) < 0.005:
                return  # Skip negligible movement

            line_id = p.addUserDebugLine(
                list(last_pt),
                list(new_pt),
                lineColorRGB=[0.1, 0.9, 0.3],
                lineWidth=2.0,
                physicsClientId=self.client_id,
            )
            self._trajectory_line_ids.append(line_id)

            # Evict old debug lines when exceeding history capacity
            if len(self._trajectory_line_ids) > self.sim_config.trajectory_history_len:
                old_id = self._trajectory_line_ids.popleft()
                p.removeUserDebugItem(old_id, physicsClientId=self.client_id)

        self._trajectory_points.append(new_pt)

    def step(self, wall_dt: Optional[float] = None) -> int:
        """Steps physics simulation according to fixed-timestep clock accumulator.

        Args:
            wall_dt: Elapsed wall-clock time in seconds. If None, performs 1 step.

        Returns:
            Number of physics substeps actually computed.
        """
        if wall_dt is None:
            p.stepSimulation(physicsClientId=self.client_id)
            return 1

        return self.sim_clock.step(
            wall_dt=wall_dt,
            step_fn=lambda: p.stepSimulation(physicsClientId=self.client_id),
        )

    def render_rgb(self, width: int = 640, height: int = 480) -> np.ndarray:
        """Captures synthetic RGB frame from PyBullet simulation camera.

        Returns:
            RGB image array (H, W, 3) as uint8.
        """
        view_mat = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.sim_config.camera_target_position,
            distance=self.sim_config.camera_distance,
            yaw=self.sim_config.camera_yaw,
            pitch=self.sim_config.camera_pitch,
            roll=0.0,
            upAxisIndex=2,
            physicsClientId=self.client_id,
        )
        proj_mat = p.computeProjectionMatrixFOV(
            fov=50.0,
            aspect=float(width) / float(height),
            nearVal=0.1,
            farVal=5.0,
            physicsClientId=self.client_id,
        )
        _, _, rgb_img, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self.client_id,
        )
        rgb_arr = np.array(rgb_img, dtype=np.uint8).reshape((height, width, 4))
        return rgb_arr[:, :, :3]  # Return RGB channels

    def save_screenshot(self, output_path: Path) -> bool:
        """Saves current PyBullet viewport image to disk."""
        import cv2

        try:
            rgb = self.render_rgb(width=1280, height=720)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(output_path), bgr)
            logger.info(f"Saved PyBullet screenshot to {output_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to capture PyBullet screenshot: {e}")
            return False

    def close(self) -> None:
        """Disconnects physics client and cleans up simulator resources."""
        if self.client_id >= 0:
            try:
                p.disconnect(physicsClientId=self.client_id)
                logger.info("PyBullet physics client disconnected.")
            except Exception as e:
                logger.warning(f"Exception during PyBullet disconnect: {e}")
            finally:
                self.client_id = -1
