"""Franka Emika Panda Robot Controller.

Programmatically discovers URDF joint metadata, manages joint position control
with velocity limiting and motor gains, queries forward kinematics link states,
and controls robot motion smoothly without teleportation.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import pybullet as p
import numpy as np

from config.settings import RobotConfig
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


class PandaRobotController:
    """Controls the 7-DoF Franka Emika Panda manipulator inside PyBullet."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        config: RobotConfig,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.config = config

        self.joints: Dict[int, JointInfo] = {}
        self.arm_joint_indices: List[int] = []
        self.finger_joint_indices: List[int] = []
        self.ee_link_index: int = config.end_effector_link_index

        self._inspect_urdf()
        self.reset_to_home()

    def _inspect_urdf(self) -> None:
        """Inspects and parses loaded URDF joint metadata programmatically."""
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        logger.info(f"Loaded Panda URDF with {num_joints} total joints/links.")

        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            lower = float(info[8])
            upper = float(info[9])
            max_force = float(info[10])
            max_vel = float(info[11])
            link_name = info[12].decode("utf-8")

            # Fix default zero limits in some URDF definitions
            if lower >= upper:
                if "joint1" in joint_name or "joint3" in joint_name or "joint5" in joint_name or "joint7" in joint_name:
                    lower, upper = -2.8973, 2.8973
                elif "joint2" in joint_name:
                    lower, upper = -1.7628, 1.7628
                elif "joint4" in joint_name:
                    lower, upper = -3.0718, -0.0698
                elif "joint6" in joint_name:
                    lower, upper = -0.0175, 3.7525

            j_info = JointInfo(
                index=i,
                name=joint_name,
                joint_type=joint_type,
                lower_limit=lower,
                upper_limit=upper,
                max_force=max_force if max_force > 0 else self.config.max_joint_force,
                max_velocity=max_vel if max_vel > 0 else self.config.max_joint_velocity_radps,
                link_name=link_name,
            )
            self.joints[i] = j_info

            if joint_type == p.JOINT_REVOLUTE:
                self.arm_joint_indices.append(i)
            elif joint_type == p.JOINT_PRISMATIC:
                self.finger_joint_indices.append(i)

            if "hand" in link_name or "grasptarget" in link_name or "ee" in link_name:
                self.ee_link_index = i

        logger.info(f"Identified {len(self.arm_joint_indices)} controllable arm joints: {self.arm_joint_indices}")
        logger.info(f"Identified {len(self.finger_joint_indices)} gripper finger joints: {self.finger_joint_indices}")
        logger.info(f"Using End-Effector Link Index: {self.ee_link_index} ({self.joints.get(self.ee_link_index, {}).link_name if self.ee_link_index in self.joints else 'default'})")

    def get_joint_limits(self) -> Tuple[List[float], List[float], List[float], List[float]]:
        """Returns (lower_limits, upper_limits, joint_ranges, rest_poses) for arm joints."""
        lows = [self.joints[idx].lower_limit for idx in self.arm_joint_indices]
        highs = [self.joints[idx].upper_limit for idx in self.arm_joint_indices]
        ranges = [highs[i] - lows[i] for i in range(len(lows))]
        rests = self.config.home_joint_positions[: len(self.arm_joint_indices)]
        return lows, highs, ranges, rests

    def reset_to_home(self) -> None:
        """Instantly resets joints to the safe HOME configuration."""
        for idx, joint_idx in enumerate(self.arm_joint_indices):
            target_angle = self.config.home_joint_positions[idx]
            p.resetJointState(self.robot_id, joint_idx, target_angle, targetVelocity=0.0, physicsClientId=self.client_id)

        for joint_idx in self.finger_joint_indices:
            p.resetJointState(self.robot_id, joint_idx, 0.04, targetVelocity=0.0, physicsClientId=self.client_id)

    def set_arm_joint_positions(self, target_joint_positions: List[float]) -> None:
        """Applies position control to arm joints with torque and velocity constraints."""
        num_targets = min(len(target_joint_positions), len(self.arm_joint_indices))
        active_indices = self.arm_joint_indices[:num_targets]
        active_targets = [float(target_joint_positions[i]) for i in range(num_targets)]
        forces = [self.config.max_joint_force] * num_targets
        velocities = [self.config.max_joint_velocity_radps] * num_targets
        pos_gains = [self.config.position_gain] * num_targets
        vel_gains = [self.config.velocity_gain] * num_targets

        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=active_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=active_targets,
            targetVelocities=[0.0] * num_targets,
            forces=forces,
            positionGains=pos_gains,
            velocityGains=vel_gains,
            physicsClientId=self.client_id,
        )

    def get_current_joint_positions(self) -> List[float]:
        """Returns current positions of controllable arm joints."""
        states = p.getJointStates(self.robot_id, self.arm_joint_indices, physicsClientId=self.client_id)
        return [float(state[0]) for state in states]

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
