"""Tests verifying robot model metadata parity against pre-A4 goldens."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.adapters.panda import PandaAdapter
from robotics.adapters.kuka_iiwa import KukaIiwaAdapter
from tests.fixtures.model_golden import PANDA_GOLDEN_METADATA, KUKA_GOLDEN_METADATA


@pytest.fixture
def pybullet_sim():
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
    yield cid
    p.disconnect(physicsClientId=cid)


def _inspect_raw_pybullet(cid, robot_id, spec, adapter):
    bid = p.loadURDF(spec.urdf_path, spec.base_position, spec.base_orientation, useFixedBase=spec.fixed_base, physicsClientId=cid)
    num_joints = p.getNumJoints(bid, physicsClientId=cid)
    all_joints = []
    arm_indices = []
    gripper_indices = []
    
    for i in range(num_joints):
        info = p.getJointInfo(bid, i, physicsClientId=cid)
        jname = info[1].decode("utf-8")
        jtype = info[2]
        lower, upper = float(info[8]), float(info[9])
        lower, upper = adapter.fix_joint_limits(jname, lower, upper)
        force = float(info[10]) if float(info[10]) > 0 else spec.max_joint_force
        vel = float(info[11]) if float(info[11]) > 0 else spec.max_joint_velocity_radps
        link_name = info[12].decode("utf-8")
        
        # Substring pattern match on movable joints (Ruling D)
        is_movable = (jtype != p.JOINT_FIXED)
        name_lower = jname.lower()
        if is_movable and any(pat.lower() in name_lower for pat in spec.gripper_joint_name_patterns):
            role = "GRIPPER"
            gripper_indices.append(i)
        elif is_movable and any(pat.lower() in name_lower for pat in spec.arm_joint_name_patterns):
            role = "ARM"
            arm_indices.append(i)
        else:
            role = "OTHER"
            
        all_joints.append({
            "native_index": i,
            "name": jname,
            "native_type": jtype,
            "lower": lower,
            "upper": upper,
            "force": force,
            "vel": vel,
            "link_name": link_name,
            "role": role,
        })
        
    class DummyJInfo:
        def __init__(self, **kw):
            self.__dict__.update(kw)
    jmap = {j["native_index"]: DummyJInfo(**j) for j in all_joints}
    default_ee = arm_indices[-1] if arm_indices else 0
    ee_idx = adapter.identify_ee_link_index(jmap, default_ee)
    ee_link_name = jmap[ee_idx].link_name
    
    return {
        "robot_id": spec.robot_id,
        "display_name": spec.display_name,
        "num_total_joints": num_joints,
        "arm_joint_indices": arm_indices,
        "arm_joint_names": [jmap[idx].name for idx in arm_indices],
        "gripper_joint_indices": gripper_indices,
        "gripper_joint_names": [jmap[idx].name for idx in gripper_indices],
        "ee_link_index": ee_idx,
        "ee_link_name": ee_link_name,
        "home_joint_positions": list(spec.home_joint_positions),
        "arm_lower_limits": [jmap[idx].lower for idx in arm_indices],
        "arm_upper_limits": [jmap[idx].upper for idx in arm_indices],
        "arm_max_forces": [jmap[idx].force for idx in arm_indices],
        "arm_max_velocities": [jmap[idx].vel for idx in arm_indices],
        "all_joints": all_joints,
    }


def test_panda_model_golden_parity(pybullet_sim):
    """Asserts that Panda raw PyBullet extraction strictly matches pre-A4 golden baseline."""
    cid = pybullet_sim
    spec = get_robot_registry().get_robot_spec("panda")
    adapter = PandaAdapter(spec)
    
    actual = _inspect_raw_pybullet(cid, "panda", spec, adapter)
    expected = PANDA_GOLDEN_METADATA
    
    assert actual["robot_id"] == expected["robot_id"]
    assert actual["display_name"] == expected["display_name"]
    assert actual["num_total_joints"] == expected["num_total_joints"]
    assert actual["arm_joint_indices"] == expected["arm_joint_indices"]
    assert actual["arm_joint_names"] == expected["arm_joint_names"]
    assert actual["gripper_joint_indices"] == expected["gripper_joint_indices"]
    assert actual["gripper_joint_names"] == expected["gripper_joint_names"]
    assert actual["ee_link_index"] == expected["ee_link_index"]
    assert actual["ee_link_name"] == expected["ee_link_name"]
    
    np.testing.assert_allclose(actual["home_joint_positions"], expected["home_joint_positions"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_lower_limits"], expected["arm_lower_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_upper_limits"], expected["arm_upper_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_max_forces"], expected["arm_max_forces"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_max_velocities"], expected["arm_max_velocities"], atol=1e-12, rtol=0)
    
    for act_j, exp_j in zip(actual["all_joints"], expected["all_joints"]):
        assert act_j["native_index"] == exp_j["native_index"]
        assert act_j["name"] == exp_j["name"]
        assert act_j["native_type"] == exp_j["native_type"]
        assert act_j["link_name"] == exp_j["link_name"]
        assert act_j["role"] == exp_j["role"]
        np.testing.assert_allclose(act_j["lower"], exp_j["lower"], atol=1e-12, rtol=0)
        np.testing.assert_allclose(act_j["upper"], exp_j["upper"], atol=1e-12, rtol=0)
        np.testing.assert_allclose(act_j["force"], exp_j["force"], atol=1e-12, rtol=0)
        np.testing.assert_allclose(act_j["vel"], exp_j["vel"], atol=1e-12, rtol=0)


def test_kuka_model_golden_parity(pybullet_sim):
    """Asserts that KUKA raw PyBullet extraction strictly matches pre-A4 golden baseline."""
    cid = pybullet_sim
    spec = get_robot_registry().get_robot_spec("kuka_iiwa")
    adapter = KukaIiwaAdapter(spec)
    
    actual = _inspect_raw_pybullet(cid, "kuka_iiwa", spec, adapter)
    expected = KUKA_GOLDEN_METADATA
    
    assert actual["robot_id"] == expected["robot_id"]
    assert actual["display_name"] == expected["display_name"]
    assert actual["num_total_joints"] == expected["num_total_joints"]
    assert actual["arm_joint_indices"] == expected["arm_joint_indices"]
    assert actual["arm_joint_names"] == expected["arm_joint_names"]
    assert actual["gripper_joint_indices"] == expected["gripper_joint_indices"]
    assert actual["gripper_joint_names"] == expected["gripper_joint_names"]
    assert actual["ee_link_index"] == expected["ee_link_index"]
    assert actual["ee_link_name"] == expected["ee_link_name"]
    
    np.testing.assert_allclose(actual["home_joint_positions"], expected["home_joint_positions"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_lower_limits"], expected["arm_lower_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_upper_limits"], expected["arm_upper_limits"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_max_forces"], expected["arm_max_forces"], atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual["arm_max_velocities"], expected["arm_max_velocities"], atol=1e-12, rtol=0)
    
    for act_j, exp_j in zip(actual["all_joints"], expected["all_joints"]):
        assert act_j["native_index"] == exp_j["native_index"]
        assert act_j["name"] == exp_j["name"]
        assert act_j["native_type"] == exp_j["native_type"]
        assert act_j["link_name"] == exp_j["link_name"]
        assert act_j["role"] == exp_j["role"]
        np.testing.assert_allclose(act_j["lower"], exp_j["lower"], atol=1e-12, rtol=0)
        np.testing.assert_allclose(act_j["upper"], exp_j["upper"], atol=1e-12, rtol=0)
        np.testing.assert_allclose(act_j["force"], exp_j["force"], atol=1e-12, rtol=0)
        np.testing.assert_allclose(act_j["vel"], exp_j["vel"], atol=1e-12, rtol=0)
