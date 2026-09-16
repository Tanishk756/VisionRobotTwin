"""Generic Robot Manipulator Controller.

Manages joint position and velocity control with torque and velocity limits,
queries forward kinematics link states, and supports multi-manipulator platforms
(Franka Emika Panda, KUKA LBR iiwa, etc.) via backend-agnostic ResolvedRobotModel metadata.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple, Union, Any
import threading
import numpy as np

from config.settings import RobotConfig
from robotics.backends.base import RobotBackend, TimestampedJointState
from robotics.robot_model import (
    RobotModelSpec,
    RobotCapabilities,
    JointRole,
    JointMotionType,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)
from robotics.robot_registry import get_robot_registry, create_robot_adapter
from robotics.adapters.base import RobotAdapter
from robotics.kinematics_provider import KinematicsProvider
from utils.logger import get_logger

logger = get_logger("Robotics.RobotController")


@dataclass
class JointInfo:
    """Metadata for an individual robot joint extracted from URDF (backward compatibility model)."""
    index: int
    name: str
    joint_type: int
    lower_limit: float
    upper_limit: float
    max_force: float
    max_velocity: float
    link_name: str


class GenericRobotController:
    """Robot-agnostic controller for multi-DoF robotic manipulators."""

    def __init__(
        self,
        physics_client_id: Optional[int] = None,
        robot_id: Optional[int] = None,
        spec: Optional[RobotModelSpec] = None,
        adapter: Optional[RobotAdapter] = None,
        config: Optional[RobotConfig] = None,
        backend: Optional[RobotBackend] = None,
        kinematics_provider: Optional[KinematicsProvider] = None,
        model_query_lock: Optional[threading.RLock] = None,
        resolved_model: Optional[ResolvedRobotModel] = None,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id

        if spec is None:
            if resolved_model is not None:
                try:
                    self.spec = get_robot_registry().get_robot_spec(resolved_model.robot_id)
                except (KeyError, ValueError):
                    self.spec = RobotModelSpec(
                        robot_id=resolved_model.robot_id,
                        display_name=resolved_model.display_name,
                        urdf_path="",
                        capabilities=resolved_model.capabilities,
                        home_joint_positions=list(resolved_model.home_joint_positions),
                    )
            elif adapter is not None:
                self.spec = adapter.spec
            else:
                self.spec = get_robot_registry().get_robot_spec("panda")
        else:
            self.spec = spec

        if adapter is not None:
            self.adapter = adapter
        else:
            try:
                self.adapter = create_robot_adapter(self.spec.robot_id, self.spec)
            except (KeyError, ValueError):
                self.adapter = None
        self.config = config or RobotConfig()

        if resolved_model is not None:
            self.model: ResolvedRobotModel = resolved_model
        else:
            if physics_client_id is None or robot_id is None:
                raise ValueError("Either resolved_model or both physics_client_id and robot_id must be supplied.")
            from robotics.pybullet_model import PyBulletRobotModelResolver
            self.model = PyBulletRobotModelResolver.resolve(
                physics_client_id=physics_client_id,
                robot_body_id=robot_id,
                spec=self.spec,
                adapter=self.adapter,
            )

        # Populate backward-compatible self.joints mapping
        self.joints: Dict[int, JointInfo] = {}
        for j in self.model.all_joints:
            native_idx = j.native_index if j.native_index is not None else j.model_index
            jtype = (
                j.native_joint_type
                if j.native_joint_type is not None
                else (
                    0
                    if j.motion_type == JointMotionType.REVOLUTE
                    else (1 if j.motion_type == JointMotionType.PRISMATIC else 4)
                )
            )
            self.joints[native_idx] = JointInfo(
                index=native_idx,
                name=j.name,
                joint_type=jtype,
                lower_limit=j.lower_limit,
                upper_limit=j.upper_limit,
                max_force=j.max_force,
                max_velocity=j.max_velocity,
                link_name=j.link_name,
            )

        if kinematics_provider is None:
            if self.client_id is not None and self.robot_id is not None:
                from robotics.kinematics_provider import PyBulletKinematicsProvider
                self.kinematics_provider: KinematicsProvider = PyBulletKinematicsProvider(
                    physics_client_id=self.client_id,
                    robot_body_id=self.robot_id,
                    arm_joint_indices=self.arm_joint_indices,
                    end_effector_link_index=self.ee_link_index,
                    query_lock=model_query_lock,
                )
            else:
                raise ValueError("kinematics_provider must be supplied when constructing without PyBullet client.")
        else:
            self.kinematics_provider = kinematics_provider

        if backend is None:
            if self.client_id is not None and self.robot_id is not None:
                from robotics.backends.pybullet_backend import PyBulletRobotBackend
                self.backend: RobotBackend = PyBulletRobotBackend(
                    physics_client_id=self.client_id,
                    robot_body_id=self.robot_id,
                    arm_joint_indices=self.arm_joint_indices,
                    joint_names=list(self.model.arm_joint_names),
                    default_joint_force=self.spec.max_joint_force,
                )
            else:
                raise ValueError("backend must be supplied when constructing without PyBullet client.")
        else:
            self.backend = backend

        if not self.backend.is_connected():
            self.backend.connect()

        self._validate_initial_backend_state()

    @property
    def capabilities(self) -> RobotCapabilities:
        """Returns the hardware capability flags for the active robot."""
        return self.model.capabilities

    @property
    def arm_joint_indices(self) -> List[int]:
        """Controllable arm joint indices. Returns native indices if available, else canonical indices."""
        if self.model is not None:
            try:
                return list(self.model.require_arm_native_indices())
            except ValueError:
                return list(range(self.model.dof))
        return []

    @property
    def finger_joint_indices(self) -> List[int]:
        """Gripper joint indices. Returns native indices if available, else canonical indices."""
        if self.model is not None:
            try:
                return list(self.model.require_gripper_native_indices())
            except ValueError:
                return list(range(len(self.model.gripper_joints)))
        return []

    @property
    def ee_link_index(self) -> int:
        """End-effector link index."""
        if self.model is not None and self.model.ee_link_native_index is not None:
            return self.model.ee_link_native_index
        return self.model.dof - 1 if self.model and self.model.dof > 0 else 0

    def _validate_backend_state(self, state: Optional[TimestampedJointState]) -> None:
        """Validates that joint state telemetry matches resolved model degrees of freedom and joint names."""
        if state is None or not isinstance(state, TimestampedJointState):
            return
        if len(state.positions) != self.model.dof:
            raise ValueError(
                f"Backend joint state DoF mismatch: expected {self.model.dof} joints, got {len(state.positions)}"
            )
        if state.velocities is not None and len(state.velocities) != self.model.dof:
            raise ValueError(
                f"Backend joint state velocities mismatch: expected {self.model.dof} velocities, got {len(state.velocities)}"
            )
        if state.joint_names and tuple(state.joint_names) != self.model.arm_joint_names:
            raise ValueError(
                f"Backend joint names mismatch: expected {self.model.arm_joint_names}, got {state.joint_names}"
            )

    def _validate_initial_backend_state(self) -> None:
        try:
            state = self.backend.get_joint_state()
            self._validate_backend_state(state)
        except Exception as e:
            if isinstance(e, ValueError):
                raise

    def get_joint_limits(self) -> Tuple[List[float], List[float], List[float], List[float]]:
        """Returns (lower_limits, upper_limits, joint_ranges, rest_poses) for arm joints."""
        lows = list(self.model.arm_lower_limits)
        highs = list(self.model.arm_upper_limits)
        ranges = list(self.model.arm_joint_ranges)
        rests = list(self.model.home_joint_positions[: self.model.dof])
        return lows, highs, ranges, rests

    def reset_to_home(self) -> None:
        """Instantly resets joints to safe HOME configuration (Simulation Only / Deprecated)."""
        if self.client_id is not None and self.robot_id is not None:
            from robotics.pybullet_model import teleport_robot_to_home
            teleport_robot_to_home(self.client_id, self.robot_id, self.model)
        else:
            raise RuntimeError(
                "reset_to_home() is a legacy simulation teleport operation and is unavailable "
                "for backend-agnostic robot controllers without a PyBullet physics client."
            )

    def set_arm_joint_positions(
        self,
        target_joint_positions: Sequence[float],
        dt: Optional[float] = None,
        enforce_velocity_limits: bool = True,
    ) -> List[float]:
        """Applies position control to arm joints with velocity constraints.

        Args:
            target_joint_positions: Desired target angles per controllable arm joint (rad).
            dt: Optional elapsed delta time for explicit target rate limiting.
            enforce_velocity_limits: If True and dt is provided, clamps commanded angle
                delta per joint to [ -max_vel * dt, +max_vel * dt ].

        Returns:
            The actual commanded target positions (after rate limiting if applied).
        """
        num_targets = min(len(target_joint_positions), self.model.dof)
        raw_targets = [float(target_joint_positions[i]) for i in range(num_targets)]
        max_vels = [float(self.model.arm_max_velocities[i]) for i in range(num_targets)]

        if enforce_velocity_limits and dt is not None and dt > 0.0:
            current_positions = np.array(self.get_current_joint_positions()[:num_targets], dtype=np.float64)
            raw_targets_arr = np.array(raw_targets, dtype=np.float64)
            delta_q = raw_targets_arr - current_positions
            max_deltas = np.array(max_vels, dtype=np.float64) * dt

            # Compute proportional coordinated scaling factor:
            scale = 1.0
            nonzero_mask = np.abs(delta_q) > 1e-9
            if np.any(nonzero_mask):
                ratios = max_deltas[nonzero_mask] / np.abs(delta_q[nonzero_mask])
                scale = float(min(1.0, np.min(ratios)))

            active_targets = (current_positions + scale * delta_q).tolist()
        else:
            active_targets = raw_targets

        self.backend.command_joint_positions(active_targets)
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
        if max_force is not None and not self.backend.transport_capabilities.effort_limit_override:
            from robotics.backends.base import UnsupportedBackendOperationError
            raise UnsupportedBackendOperationError(
                f"The connected backend '{self.backend.__class__.__name__}' does not support torque/effort limit overrides."
            )

        num_targets = min(len(target_joint_velocities), self.model.dof)
        clamped_velocities = []
        for i in range(num_targets):
            max_vel = self.model.arm_max_velocities[i]
            v = float(np.clip(target_joint_velocities[i], -max_vel, max_vel))
            clamped_velocities.append(v)

        self.backend.command_joint_velocities(clamped_velocities, effort_limit=max_force)
        return clamped_velocities


    def get_current_joint_positions(self) -> List[float]:
        """Returns current positions of controllable arm joints (rad)."""
        joint_state = self.backend.get_joint_state()
        self._validate_backend_state(joint_state)
        return list(joint_state.positions)

    def get_current_joint_velocities(self) -> List[float]:
        """Returns current velocities of controllable arm joints (rad/s)."""
        joint_state = self.backend.get_joint_state()
        self._validate_backend_state(joint_state)
        if joint_state.velocities is None:
            from robotics.backends.base import BackendStateFieldUnavailableError
            raise BackendStateFieldUnavailableError("Joint velocities are not available from backend telemetry.")
        return list(joint_state.velocities)

    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns forward kinematics (position [x,y,z], quaternion [x,y,z,w]) of the end effector via KinematicsProvider."""
        current_q = self.get_current_joint_positions()
        return self.kinematics_provider.compute_fk(current_q)

    def compute_cartesian_error(self, target_position: np.ndarray) -> float:
        """Computes Euclidean distance error between current EE and target."""
        ee_pos, _ = self.get_end_effector_pose()
        target = np.asarray(target_position, dtype=np.float64)
        return float(np.linalg.norm(ee_pos - target))


class PandaRobotController(GenericRobotController):
    """Backward-compatible Franka Emika Panda Robot Controller."""

    def __init__(
        self,
        physics_client_id: Optional[int] = None,
        robot_id: Optional[int] = None,
        config: Optional[RobotConfig] = None,
        robot_config: Optional[RobotConfig] = None,
        kinematics_provider: Optional[KinematicsProvider] = None,
        backend: Optional[RobotBackend] = None,
        resolved_model: Optional[ResolvedRobotModel] = None,
    ):
        spec = get_robot_registry().get_robot_spec("panda")
        super().__init__(
            physics_client_id=physics_client_id,
            robot_id=robot_id,
            spec=spec,
            config=config or robot_config,
            backend=backend,
            kinematics_provider=kinematics_provider,
            resolved_model=resolved_model,
        )
