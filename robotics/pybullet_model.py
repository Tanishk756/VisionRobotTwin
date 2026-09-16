"""PyBullet Robot Model Resolver and Simulation State Teleport Utilities.

Inspects loaded URDF models in PyBullet physics clients to generate immutable
ResolvedRobotModel metadata structures, and provides simulation-only state initialization.
"""

from typing import Optional, List, Dict, Any, Tuple
try:
    import pybullet as p
except ImportError:
    p = None  # type: ignore

from robotics.robot_model import (
    RobotModelSpec,
    RobotCapabilities,
    JointRole,
    JointMotionType,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)
from robotics.adapters.base import RobotAdapter
from robotics.robot_registry import create_robot_adapter
from utils.logger import get_logger

logger = get_logger("Robotics.PyBulletModel")


class PyBulletRobotModelResolver:
    """Inspects a PyBullet physics client body and produces an immutable ResolvedRobotModel."""

    @staticmethod
    def resolve(
        physics_client_id: int,
        robot_body_id: int,
        spec: RobotModelSpec,
        adapter: Optional[RobotAdapter] = None,
    ) -> ResolvedRobotModel:
        """Inspects and parses loaded URDF joint metadata from PyBullet programmatically.

        Args:
            physics_client_id: Active PyBullet client ID.
            robot_body_id: MultiBody ID for the robot.
            spec: Static configuration specifications for the robot.
            adapter: Optional robot adapter for limit and EE fixes.

        Returns:
            ResolvedRobotModel containing resolved joints, limits, and EE identification.
        """
        active_adapter = adapter or create_robot_adapter(spec.robot_id, spec)
        num_joints = p.getNumJoints(robot_body_id, physicsClientId=physics_client_id)
        logger.info(f"Resolving {spec.display_name} URDF with {num_joints} total joints/links.")

        raw_joint_dicts: List[Dict[str, Any]] = []
        for i in range(num_joints):
            info = p.getJointInfo(robot_body_id, i, physicsClientId=physics_client_id)
            joint_name = info[1].decode("utf-8")
            native_type = info[2]
            lower = float(info[8])
            upper = float(info[9])
            max_force = float(info[10])
            max_vel = float(info[11])
            link_name = info[12].decode("utf-8")

            # Apply adapter-specific limit corrections
            lower, upper = active_adapter.fix_joint_limits(joint_name, lower, upper)

            # Fallback force and velocity
            eff_force = max_force if max_force > 0 else spec.max_joint_force
            eff_vel = max_vel if max_vel > 0 else spec.max_joint_velocity_radps

            # Map PyBullet joint type to generic JointMotionType
            if native_type == p.JOINT_REVOLUTE:
                motion_type = JointMotionType.REVOLUTE
            elif native_type == p.JOINT_PRISMATIC:
                motion_type = JointMotionType.PRISMATIC
            elif native_type == p.JOINT_FIXED:
                motion_type = JointMotionType.FIXED
            else:
                motion_type = JointMotionType.OTHER

            # Substring matching on movable joints (Ruling D)
            is_movable = (native_type != p.JOINT_FIXED)
            name_lower = joint_name.lower()
            if is_movable and any(pat.lower() in name_lower for pat in spec.gripper_joint_name_patterns):
                role = JointRole.GRIPPER
            elif is_movable and any(pat.lower() in name_lower for pat in spec.arm_joint_name_patterns):
                role = JointRole.ARM
            else:
                role = JointRole.OTHER

            raw_joint_dicts.append({
                "native_index": i,
                "name": joint_name,
                "role": role,
                "motion_type": motion_type,
                "lower": lower,
                "upper": upper,
                "max_force": eff_force,
                "max_velocity": eff_vel,
                "link_name": link_name,
                "native_joint_type": native_type,
            })

        # Identify EE link index using adapter policy
        class _JointInfoProxy:
            def __init__(self, **kwargs: Any):
                self.__dict__.update(kwargs)

        joint_map_for_adapter = {
            d["native_index"]: _JointInfoProxy(
                index=d["native_index"],
                name=d["name"],
                link_name=d["link_name"],
            )
            for d in raw_joint_dicts
        }

        arm_raw = [d for d in raw_joint_dicts if d["role"] == JointRole.ARM]
        default_ee = arm_raw[-1]["native_index"] if arm_raw else 0
        ee_link_native_idx = active_adapter.identify_ee_link_index(joint_map_for_adapter, default_ee)
        ee_link_name = joint_map_for_adapter[ee_link_native_idx].link_name if ee_link_native_idx in joint_map_for_adapter else spec.end_effector_link_name

        # Construct final ResolvedJointMetadata objects with contiguous canonical indices
        arm_counter = 0
        grip_counter = 0
        other_counter = 0
        all_resolved: List[ResolvedJointMetadata] = []
        arm_resolved: List[ResolvedJointMetadata] = []
        grip_resolved: List[ResolvedJointMetadata] = []

        for model_idx, d in enumerate(raw_joint_dicts):
            if d["role"] == JointRole.ARM:
                canon_idx = arm_counter
                arm_counter += 1
            elif d["role"] == JointRole.GRIPPER:
                canon_idx = grip_counter
                grip_counter += 1
            else:
                canon_idx = other_counter
                other_counter += 1

            meta = ResolvedJointMetadata(
                model_index=model_idx,
                canonical_index=canon_idx,
                name=d["name"],
                role=d["role"],
                motion_type=d["motion_type"],
                lower_limit=d["lower"],
                upper_limit=d["upper"],
                max_force=d["max_force"],
                max_velocity=d["max_velocity"],
                link_name=d["link_name"],
                native_index=d["native_index"],
                native_joint_type=d["native_joint_type"],
            )
            all_resolved.append(meta)
            if meta.role == JointRole.ARM:
                arm_resolved.append(meta)
            elif meta.role == JointRole.GRIPPER:
                grip_resolved.append(meta)

        resolved_model = ResolvedRobotModel(
            robot_id=spec.robot_id,
            display_name=spec.display_name,
            all_joints=tuple(all_resolved),
            arm_joints=tuple(arm_resolved),
            gripper_joints=tuple(grip_resolved),
            ee_link_name=ee_link_name,
            ee_link_native_index=ee_link_native_idx,
            home_joint_positions=tuple(spec.home_joint_positions),
            capabilities=spec.capabilities,
        )

        logger.info(
            f"Resolved {resolved_model.display_name}: {resolved_model.dof} arm DoF, "
            f"{len(resolved_model.gripper_joints)} gripper joints, EE link '{resolved_model.ee_link_name}' "
            f"(native idx {resolved_model.ee_link_native_index})."
        )
        return resolved_model


def teleport_robot_to_home(
    physics_client_id: int,
    robot_body_id: int,
    resolved_model: ResolvedRobotModel,
) -> None:
    """Sets joint state teleport in PyBullet simulation to the configured home configuration.

    Args:
        physics_client_id: Active PyBullet client ID.
        robot_body_id: MultiBody ID for the robot.
        resolved_model: Resolved robot model with native indices and home positions.
    """
    for idx, jinfo in enumerate(resolved_model.arm_joints):
        if jinfo.native_index is not None:
            target_angle = (
                resolved_model.home_joint_positions[idx]
                if idx < len(resolved_model.home_joint_positions)
                else 0.0
            )
            p.resetJointState(
                robot_body_id,
                jinfo.native_index,
                target_angle,
                targetVelocity=0.0,
                physicsClientId=physics_client_id,
            )

    for jinfo in resolved_model.gripper_joints:
        if jinfo.native_index is not None:
            p.resetJointState(
                robot_body_id,
                jinfo.native_index,
                0.04,
                targetVelocity=0.0,
                physicsClientId=physics_client_id,
            )
