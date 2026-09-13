"""Virtual Gripper and Grasp Constraint Abstraction.

Manages both the visual prismatic finger joints of the Franka Panda hand
and reliable dynamic physics constraints (p.createConstraint / p.removeConstraint)
for robust virtual object pick-and-place manipulation.
"""

from typing import List, Optional
import pybullet as p
import numpy as np

from utils.logger import get_logger

logger = get_logger("Robotics.Gripper")


class VirtualGripper:
    """Controls gripper finger motion and attaches/releases physics grasp constraints."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        finger_joint_indices: List[int],
        ee_link_index: int,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.finger_indices = finger_joint_indices
        self.ee_link_index = ee_link_index

        self._active_constraint_id: Optional[int] = None
        self._grasped_body_id: Optional[int] = None
        self._is_closed: bool = False

    @property
    def is_grasping(self) -> bool:
        """Returns True if a body is currently attached via grasp constraint."""
        return self._active_constraint_id is not None

    def open(self, target_width: float = 0.04) -> None:
        """Opens the gripper fingers."""
        self._is_closed = False
        for joint_idx in self.finger_indices:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_width,
                force=20.0,
                physicsClientId=self.client_id,
            )

    def close(self, target_width: float = 0.0) -> None:
        """Closes the gripper fingers."""
        self._is_closed = True
        for joint_idx in self.finger_indices:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_width,
                force=40.0,
                physicsClientId=self.client_id,
            )

    def attach_object(self, object_body_id: int) -> bool:
        """Creates a rigid kinematic constraint between the end-effector and target object."""
        if self._active_constraint_id is not None:
            self.detach_object()

        try:
            # Query relative transform between EE and object
            ee_state = p.getLinkState(self.robot_id, self.ee_link_index, physicsClientId=self.client_id)
            ee_pos = ee_state[0]
            ee_orn = ee_state[1]

            obj_pos, obj_orn = p.getBasePositionAndOrientation(object_body_id, physicsClientId=self.client_id)

            # Invert EE transform to get local offset
            inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
            parent_to_child_pos, parent_to_child_orn = p.multiplyTransforms(
                inv_ee_pos, inv_ee_orn, obj_pos, obj_orn
            )

            constraint_id = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=self.ee_link_index,
                childBodyUniqueId=object_body_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=parent_to_child_pos,
                childFramePosition=[0, 0, 0],
                parentFrameOrientation=parent_to_child_orn,
                physicsClientId=self.client_id,
            )

            self._active_constraint_id = constraint_id
            self._grasped_body_id = object_body_id
            self.close()
            logger.info(f"Attached grasp constraint (ID: {constraint_id}) to body {object_body_id}.")
            return True

        except Exception as e:
            logger.error(f"Failed to attach grasp constraint: {e}")
            return False

    def detach_object(self) -> None:
        """Removes active grasp constraint and releases the object."""
        if self._active_constraint_id is not None:
            try:
                p.removeConstraint(self._active_constraint_id, physicsClientId=self.client_id)
                logger.info(f"Released grasp constraint {self._active_constraint_id} (body {self._grasped_body_id}).")
            except Exception as e:
                logger.warning(f"Error removing grasp constraint: {e}")
            self._active_constraint_id = None
            self._grasped_body_id = None

        self.open()
