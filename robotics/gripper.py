"""Virtual Gripper and Physical Distance-Gated Grasp Constraint Manager.

Manages finger joint kinematics and ensures dynamic rigid grasp constraints
can ONLY attach when the end-effector tool center point is physically adjacent
to the target manipulable object, eliminating remote/magical attachments.
"""

from dataclasses import dataclass
from typing import List, Optional
try:
    import pybullet as p
except ImportError:
    p = None  # type: ignore
import numpy as np

from utils.logger import get_logger

logger = get_logger("Robotics.Gripper")


@dataclass
class GraspResult:
    """Structured output for grasp attempt validation."""
    success: bool
    distance_m: float
    reason: str
    constraint_id: Optional[int] = None


class VirtualGripper:
    """Controls gripper finger motion and attaches distance-validated grasp constraints."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        finger_joint_indices: List[int],
        ee_link_index: int,
        max_grasp_distance_m: float = 0.055,
    ):
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.finger_indices = finger_joint_indices
        self.ee_link_index = ee_link_index
        self.max_grasp_distance_m = max_grasp_distance_m

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

    def attach_object(self, object_body_id: int) -> GraspResult:
        """Attaches a rigid kinematic constraint only if physically within grasp threshold.

        Args:
            object_body_id: PyBullet body ID of the object to grasp.

        Returns:
            GraspResult with distance, success flag, and reason.
        """
        if object_body_id < 0:
            return GraspResult(success=False, distance_m=float("inf"), reason="INVALID_OBJECT_ID")

        if self._active_constraint_id is not None:
            self.detach_object()

        try:
            # Query EE state and Object state
            ee_state = p.getLinkState(self.robot_id, self.ee_link_index, physicsClientId=self.client_id)
            ee_pos = np.array(ee_state[0], dtype=np.float64)
            ee_orn = ee_state[1]

            obj_pos_raw, obj_orn = p.getBasePositionAndOrientation(object_body_id, physicsClientId=self.client_id)
            obj_pos = np.array(obj_pos_raw, dtype=np.float64)

            # Compute actual Euclidean distance
            distance = float(np.linalg.norm(ee_pos - obj_pos))

            # Strictly enforce grasp distance threshold
            if distance > self.max_grasp_distance_m:
                logger.warning(
                    f"Grasp rejected: End-effector distance to object ({distance:.4f}m) "
                    f"exceeds maximum threshold ({self.max_grasp_distance_m:.4f}m)."
                )
                return GraspResult(
                    success=False,
                    distance_m=distance,
                    reason=f"DISTANCE_EXCEEDED ({distance*100:.1f}cm > {self.max_grasp_distance_m*100:.1f}cm)",
                )

            # Compute local offset transform
            inv_ee_pos, inv_ee_orn = p.invertTransform(list(ee_pos), ee_orn)
            parent_to_child_pos, parent_to_child_orn = p.multiplyTransforms(
                inv_ee_pos, inv_ee_orn, list(obj_pos), obj_orn
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
            logger.info(f"Grasp attached: constraint {constraint_id} on object {object_body_id} (dist: {distance*100:.2f}cm).")
            return GraspResult(
                success=True,
                distance_m=distance,
                reason="ATTACHED_SUCCESS",
                constraint_id=constraint_id,
            )

        except Exception as e:
            logger.error(f"Grasp attachment failed with exception: {e}")
            return GraspResult(success=False, distance_m=float("inf"), reason=f"EXCEPTION: {e}")

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
