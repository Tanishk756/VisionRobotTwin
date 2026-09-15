"""Robotics package for kinematics, transforms, control, simulation, and state management."""

from robotics.coordinate_transform import (
    SE3Transform,
    create_homogeneous_matrix,
    invert_homogeneous_matrix,
    compose_transforms,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)
from robotics.workspace_mapper import WorkspaceMapper, MappedTarget
from robotics.kinematics_provider import KinematicsProvider, PyBulletKinematicsProvider
from robotics.inverse_kinematics import GenericIKSolver, PandaIKSolver, IKResult, IKStatus
from robotics.robot_model import RobotModelSpec, RobotCapabilities
from robotics.robot_registry import RobotRegistry, get_robot_registry, list_available_robots, create_robot_adapter
from robotics.robot_controller import GenericRobotController, PandaRobotController, JointInfo
from robotics.gripper import VirtualGripper
from robotics.simulator import PyBulletSimulator
from robotics.state_machine import RoboticStateMachine, RobotState

__all__ = [
    "SE3Transform",
    "create_homogeneous_matrix",
    "invert_homogeneous_matrix",
    "compose_transforms",
    "euler_to_rotation_matrix",
    "rotation_matrix_to_euler",
    "quaternion_to_rotation_matrix",
    "rotation_matrix_to_quaternion",
    "WorkspaceMapper",
    "MappedTarget",
    "KinematicsProvider",
    "PyBulletKinematicsProvider",
    "GenericIKSolver",
    "PandaIKSolver",
    "IKResult",
    "IKStatus",
    "RobotModelSpec",
    "RobotCapabilities",
    "RobotRegistry",
    "get_robot_registry",
    "list_available_robots",
    "create_robot_adapter",
    "GenericRobotController",
    "PandaRobotController",
    "JointInfo",
    "VirtualGripper",
    "PyBulletSimulator",
    "RoboticStateMachine",
    "RobotState",
]
