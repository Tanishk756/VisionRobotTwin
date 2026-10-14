"""Six-axis xArm profiles using the models bundled with pybullet_data.

The gripper profile controls the native linkage, but deliberately exposes the
arm flange as its reference frame. Automatic grasping awaits TCP validation.
"""
from robotics.adapters.base import RobotAdapter
from robotics.robot_model import RobotCapabilities, RobotModelSpec


class XArm6Adapter(RobotAdapter):
    with_gripper = False

    def __init__(self, spec=None):
        if spec is None:
            grip = self.with_gripper
            spec = RobotModelSpec(
                robot_id="xarm6_gripper" if grip else "xarm6",
                display_name="xArm6 with gripper (flange control)" if grip else "xArm6",
                urdf_path="xarm/xarm6_with_gripper.urdf" if grip else "xarm/xarm6_robot.urdf",
                home_joint_positions=[0.0, -0.6, -1.2, 0.0, 1.8, 0.0],
                arm_joint_name_patterns=["joint"],
                gripper_joint_name_patterns=["drive_joint", "finger_joint", "knuckle_joint"] if grip else [],
                end_effector_link_name="link6",
                capabilities=RobotCapabilities(has_gripper=grip, supports_pick_place=False, max_payload_kg=0.0),
                max_joint_force=50.0,
                max_joint_velocity_radps=1.0,
                position_gain=0.3,
                spherical_reach_m=0.85,
                min_reach_m=0.05,
                gripper_open_position=0.0,
                # Leave a small gap between the opposing finger meshes.
                gripper_closed_position=0.75,
                use_urdf_inertia=False,
                # The parallel linkage meshes overlap at their mechanical pivots.
                # Only these linkage contacts are exempt; arm/finger and external
                # obstacle contacts remain checked.
                allowed_self_collision_pairs=[(8, 10), (9, 10), (11, 13), (12, 13)] if grip else [],
            )
        super().__init__(spec)

    def fix_joint_limits(self, joint_name, lower, upper):
        if lower >= upper and joint_name in {f"joint{i}" for i in range(1, 7)}:
            raise ValueError(f"Invalid xArm URDF limits for {joint_name}")
        return lower, upper

    def identify_ee_link_index(self, joint_info_map, default_index=6):
        for index, info in joint_info_map.items():
            if info.link_name == self.spec.end_effector_link_name:
                return index
        raise ValueError("xArm flange link is missing")


class XArm6GripperAdapter(XArm6Adapter):
    with_gripper = True
