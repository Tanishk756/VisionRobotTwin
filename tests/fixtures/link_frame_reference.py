"""Convert historical COM goldens to the corrected URDF link-frame convention.

The original capture is retained unchanged. This conversion uses the URDF's
local inertial transform, independent of the production FK implementation.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def link_pose_from_legacy(golden, suffix):
    com_pos = np.array(golden[f"fk_pos_{suffix}"])
    com_rot = Rotation.from_quat(golden[f"fk_orn_{suffix}"])
    # KUKA link 7 has a 20 mm local inertial translation; Panda grasp target
    # has an identity inertial transform.
    offset = [0, 0, 0.02] if golden["robot_id"] == "kuka_iiwa" else [0, 0, 0]
    return com_pos - com_rot.apply(offset), com_rot.as_quat()


def legacy_com_position(golden, link_pos, link_orn):
    offset = [0, 0, 0.02] if golden["robot_id"] == "kuka_iiwa" else [0, 0, 0]
    return np.array(link_pos) + Rotation.from_quat(link_orn).apply(offset)
