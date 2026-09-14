"""Generic Robot Manipulator Controller.

Programmatically discovers URDF joint metadata, manages joint position and velocity control
with torque and velocity limits, queries forward kinematics link states, and supports
multi-manipulator platforms (Franka Emika Panda, KUKA LBR iiwa, etc.).
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union
import pybullet as p
import numpy as np

from config.settings import RobotConfig
from robotics.robot_model import RobotModelSpec, RobotCapabilities
from robotics.robot_registry import get_robot_registry, create_robot_adapter
from robotics.adapters.base import RobotAdapter
from utils.logger import get_logger

logger = get_logger("Robotics.RobotController")


@dataclass
class JointInfo:
    """Metadata for an individual robot joint extracted from URDF."""
    index: int
    name: str
    joint_type: int
    lower_limit: float
    upper_limit: float
    max_force: float
    max_velocity: float
    link_name: str


class GenericRobotController:
    """Robot-agnostic controller for multi-DoF robotic manipulators in PyBullet."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        spec: Optional[RobotModelSpec] = None,
        adapter: Optional[RobotAdapter] = None,
        config: Optional[RobotConfig] = None,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id

        if spec is None:
            if adapter is not None:
                self.spec = adapter.spec
            else:
                self.spec = get_robot_registry().get_robot_spec("panda")
        else:
            self.spec = spec

        self.adapter = adapter or create_robot_adapter(self.spec.robot_id, self.spec)
        self.config = config or RobotConfig()

        self.joints: Dict[int, JointInfo] = {}
        self.arm_joint_indices: List[int] = []
        self.finger_joint_indices: List[int] = []
        self.ee_link_index: int = 0

        self._inspect_urdf()
        self.reset_to_home()

    @property
    def capabilities(self) -> RobotCapabilities:
        """Returns the hardware capability flags for the active robot."""
        return self.spec.capabilities

    def _inspect_urdf(self) -> None:
        """Inspects and parses loaded URDF joint metadata programmatically."""
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        logger.info(f"Loaded {self.spec.display_name} URDF with {num_joints} total joints/links.")

        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            lower = float(info[8])
            upper = float(info[9])
            max_force = float(info[10])
            max_vel = float(info[11])
            link_name = info[12].decode("utf-8")

            # Apply adapter-specific limit corrections (for default zero-limit URDF definitions)
            lower, upper = self.adapter.fix_joint_limits(joint_name, lower, upper)

            j_info = JointInfo(
                index=i,
                name=joint_name,
                joint_type=joint_type,
                lower_limit=lower,
                upper_limit=upper,
                max_force=max_force if max_force > 0 else self.spec.max_joint_force,
                max_velocity=max_vel if max_vel > 0 else self.spec.max_joint_velocity_radps,
                link_name=link_name,
            )
            self.joints[i] = j_info

            # Classify joint types
            if joint_type == p.JOINT_REVOLUTE:
                self.arm_joint_indices.append(i)
            elif joint_type == p.JOINT_PRISMATIC:
                self.finger_joint_indices.append(i)

        # Identify end-effector link index
        default_ee = self.arm_joint_indices[-1] if self.arm_joint_indices else 0
        self.ee_link_index = self.adapter.identify_ee_link_index(self.joints, default_ee)

        logger.info(f"Identified {len(self.arm_joint_indices)} controllable arm joints: {self.arm_joint_indices}")
        if self.finger_joint_indices:
            logger.info(f"Identified {len(self.finger_joint_indices)} gripper finger joints: {self.finger_joint_indices}")
        else:
            logger.info(f"No native gripper joints detected for {self.spec.display_name}")
        logger.info(f"Using End-Effector Link Index: {self.ee_link_index} ({self.joints.get(self.ee_link_index, JointInfo(0,'',0,0,0,0,0,'')).link_name})")

    def get_joint_limits(self) -> Tuple[List[float], List[float], List[float], List[float]]:
        """Returns (lower_limits, upper_limits, joint_ranges, rest_poses) for arm joints."""
        lows = [self.joints[idx].lower_limit for idx in self.arm_joint_indices]
        highs = [self.joints[idx].upper_limit for idx in self.arm_joint_indices]
        ranges = [highs[i] - lows[i] for i in range(len(lows))]
        rests = list(self.spec.home_joint_positions[: len(self.arm_joint_indices)])
        return lows, highs, ranges, rests

    def reset_to_home(self) -> None:
        """Instantly resets joints to the safe HOME configuration."""
        for idx, joint_idx in enumerate(self.arm_joint_indices):
            if idx < len(self.spec.home_joint_positions):
                target_angle = self.spec.home_joint_positions[idx]
            else:
                target_angle = 0.0
            p.resetJointState(self.robot_id, joint_idx, target_angle, targetVelocity=0.0, physicsClientId=self.client_id)

        for joint_idx in self.finger_joint_indices:
            p.resetJointState(self.robot_id, joint_idx, 0.04, targetVelocity=0.0, physicsClientId=self.client_id)

    def set_arm_joint_positions(
        self,
        target_joint_positions: Sequence[float],
        dt: Optional[float] = None,
        enforce_velocity_limits: bool = True,
    ) -> List[float]:
        """Applies position control to arm joints with torque and velocity constraints.

        Args:
            target_joint_positions: Desired target angles per controllable arm joint (rad).
            dt: Optional elapsed delta time for explicit target rate limiting.
            enforce_velocity_limits: If True and dt is provided, clamps commanded angle
                delta per joint to [ -max_vel * dt, +max_vel * dt ].

        Returns:
            The actual commanded target positions (after rate limiting if applied).
        """
        num_targets = min(len(target_joint_positions), len(self.arm_joint_indices))
        active_indices = self.arm_joint_indices[:num_targets]
        raw_targets = [float(target_joint_positions[i]) for i in range(num_targets)]

        max_vels = [float(self.joints[idx].max_velocity) for idx in active_indices]

        if enforce_velocity_limits and dt is not None and dt > 0.0:
            current_positions = np.array(self.get_current_joint_positions()[:num_targets], dtype=np.float64)
            raw_targets_arr = np.array(raw_targets, dtype=np.float64)
            delta_q = raw_targets_arr - current_positions
            max_deltas = np.array(max_vels, dtype=np.float64) * dt

            # Compute proportional coordinated scaling factor:
            # scale = min(1.0, min(max_delta_i / abs(delta_q_i)))
            scale = 1.0
            nonzero_mask = np.abs(delta_q) > 1e-9
            if np.any(nonzero_mask):
                ratios = max_deltas[nonzero_mask] / np.abs(delta_q[nonzero_mask])
                scale = float(min(1.0, np.min(ratios)))

            active_targets = (current_positions + scale * delta_q).tolist()
        else:
            active_targets = raw_targets

        forces = [self.spec.max_joint_force] * num_targets

        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=active_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=active_targets,
            forces=forces,
            physicsClientId=self.client_id,
        )
        return active_targets

    def set_arm_joint_velocities(
        self,
        target_joint_velocities: Sequence[float],
        max_force: Optional[float] = None,
    ) -> List[float]:
        """Applies velocity control to arm joints with torque constraints.

        Args:
            target_joint_velocities: Desired joint angular rates per arm joint (rad/s).
            max_force: Optional torque limit override.

        Returns:
            The commanded joint velocities (after joint limit clamping).
        """
        num_targets = min(len(target_joint_velocities), len(self.arm_joint_indices))
        active_indices = self.arm_joint_indices[:num_targets]
        clamped_velocities = []
        for i, idx in enumerate(active_indices):
            max_vel = self.joints[idx].max_velocity
            v = float(np.clip(target_joint_velocities[i], -max_vel, max_vel))
            clamped_velocities.append(v)

        force_val = float(max_force if max_force is not None else self.spec.max_joint_force)
        forces = [force_val] * num_targets

        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=active_indices,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=clamped_velocities,
            forces=forces,
            physicsClientId=self.client_id,
        )
        return clamped_velocities

    def get_current_joint_positions(self) -> List[float]:
        """Returns current positions of controllable arm joints (rad)."""
        states = p.getJointStates(self.robot_id, self.arm_joint_indices, physicsClientId=self.client_id)
        return [float(state[0]) for state in states]

    def get_current_joint_velocities(self) -> List[float]:
        """Returns current velocities of controllable arm joints (rad/s)."""
        states = p.getJointStates(self.robot_id, self.arm_joint_indices, physicsClientId=self.client_id)
        return [float(state[1]) for state in states]

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns forward kinematics (position [x,y,z], quaternion [x,y,z,w]) of the end effector."""
        link_state = p.getLinkState(self.robot_id, self.ee_link_index, physicsClientId=self.client_id)
        pos = np.array(link_state[0], dtype=np.float64)  # World position [x, y, z]
        orn = np.array(link_state[1], dtype=np.float64)  # World orientation [x, y, z, w]
        return pos, orn

    def compute_cartesian_error(self, target_position: np.ndarray) -> float:
        """Computes Euclidean distance error between current EE and target."""
        ee_pos, _ = self.get_end_effector_pose()
        target = np.asarray(target_position, dtype=np.float64)
        return float(np.linalg.norm(ee_pos - target))


class PandaRobotController(GenericRobotController):
    """Backward-compatible Franka Emika Panda Robot Controller."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        config: Optional[RobotConfig] = None,
    ):
        spec = get_robot_registry().get_robot_spec("panda")
        super().__init__(
            physics_client_id=physics_client_id,
            robot_id=robot_id,
            spec=spec,
            config=config,
        )
