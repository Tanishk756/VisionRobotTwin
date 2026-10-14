"""Clearance policy and physically stepped failure-injection regressions."""
from unittest.mock import patch

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from robotics.collision_provider import CollisionResult, PyBulletCollisionProvider, ClearanceCollisionProvider
from robotics.planning import RRTConnectPlanner, is_joint_path_collision_free, shortcut_path
from robotics.robustness import EXPECTED_FAULTS, PROFILES, RobustnessConfig, run_robustness_trial, summarize_robustness


@pytest.fixture
def box_scene():
    client = p.connect(p.DIRECT)
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05] * 3, physicsClientId=client)
    robot = p.createMultiBody(0, shape, basePosition=[0, 0, 0], physicsClientId=client)
    obstacle = p.createMultiBody(0, shape, basePosition=[0.115, 0, 0], physicsClientId=client)
    yield client, robot, obstacle
    p.disconnect(client)


def test_near_miss_is_rejected_without_claiming_contact(box_scene):
    client, robot, obstacle = box_scene
    legacy = PyBulletCollisionProvider(client, robot, obstacle_ids=[obstacle])
    strict = PyBulletCollisionProvider(client, robot, obstacle_ids=[obstacle], environment_clearance_m=0.02)
    assert not legacy.check_collision().in_collision
    result = strict.check_collision()
    assert result.in_collision and result.clearance_violation
    assert not result.env_collision and not result.self_collision
    assert result.min_env_clearance_m == pytest.approx(0.015)
    # Mount exemptions apply equally to contact and margin queries.
    exempt = PyBulletCollisionProvider(client, robot, obstacle_ids=[obstacle],
        environment_clearance_m=0.02, allowed_body_pairs=[(robot, obstacle)])
    assert not exempt.check_collision().in_collision


@pytest.mark.parametrize("value", [-1, float("nan"), float("inf")])
def test_invalid_clearance_rejected(box_scene, value):
    client, robot, _ = box_scene
    with pytest.raises(ValueError):
        PyBulletCollisionProvider(client, robot, environment_clearance_m=value)


def test_candidate_installation_failure_restores_positions_and_velocities():
    client = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        body = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True, physicsClientId=client)
        original = p.resetJointState
        for index in range(7):
            original(body, index, 0.1 * index, targetVelocity=0.02, physicsClientId=client)
        before = p.getJointStates(body, range(7), physicsClientId=client)
        calls = [0]

        def fail_once(*args, **kwargs):
            calls[0] += 1
            if calls[0] == 3:
                raise RuntimeError("candidate installation failed")
            return original(*args, **kwargs)

        checker = PyBulletCollisionProvider(client, body, arm_joint_indices=range(7))
        with patch("pybullet.resetJointState", side_effect=fail_once):
            with pytest.raises(RuntimeError, match="installation"):
                checker.check_collision([0.5] * 7)
        after = p.getJointStates(body, range(7), physicsClientId=client)
        np.testing.assert_allclose([state[:2] for state in after], [state[:2] for state in before], atol=1e-12)
    finally:
        p.disconnect(client)


class NarrowPassage:
    def check_collision(self, q):
        # No physical contact; the middle violates a chosen clearance policy.
        violation = 0.4 <= q[0] <= 0.6
        return CollisionResult(in_collision=violation, clearance_violation=violation,
                               min_env_clearance_m=0.005 if violation else 0.05)


def test_direct_rrt_and_shortcut_share_clearance_rejection():
    checker = NarrowPassage()
    assert not is_joint_path_collision_free([0], [1], checker)[0]
    planner = RRTConnectPlanner([0], [1], checker, random_seed=1, max_iterations=10)
    assert not planner.plan([0], [1]).success
    assert not planner.plan([-0.1], [1]).success
    # A shortcut cannot erase a safety-relevant detour in a 2D scene.
    class Detour:
        def check_collision(self, q):
            return CollisionResult(in_collision=0.4 <= q[0] <= 0.6 and q[1] < 0.5)
    path = [np.array(q) for q in ([0, 0], [0, 1], [1, 1], [1, 0])]
    smoothed = shortcut_path(path, Detour(), max_attempts=30, random_seed=1)
    assert len(smoothed) > 2
    assert all(is_joint_path_collision_free(a, b, Detour())[0] for a, b in zip(smoothed, smoothed[1:]))


def test_planning_headroom_does_not_change_physical_contact_flags(box_scene):
    client, robot, obstacle = box_scene
    provider = PyBulletCollisionProvider(client, robot, obstacle_ids=[obstacle], environment_clearance_m=0.01)
    planning = ClearanceCollisionProvider(provider, environment_clearance_m=0.02)
    assert not provider.check_collision().in_collision
    assert planning.check_collision().clearance_violation
    assert not planning.check_collision().env_collision


def test_seed_one_panda_plan_keeps_requested_execution_clearance():
    from robotics.experiments import execute_obstacle_reach_experiment
    result = execute_obstacle_reach_experiment("panda", seed=1, environment_clearance_m=0.01)
    assert result.rrt_planning_success and result.execution_success
    assert result.min_env_clearance_m >= 0.01
    assert result.planning_clearance_buffer_m == 0.005


@pytest.mark.parametrize("profile", PROFILES)
@pytest.mark.parametrize("controller", ["ik", "resolved-rate"])
def test_robustness_profiles_have_explicit_expected_outcomes(profile, controller):
    result = run_robustness_trial("panda", controller, profile, seed=0)
    assert result.outcome_passed, result.to_dict()
    if profile in EXPECTED_FAULTS:
        assert result.tracking_success is None
        assert result.observed_fault == EXPECTED_FAULTS[profile]
        assert result.post_fault_command_blocked
        assert result.final_measured_velocity_rad_s < 0.05
    else:
        assert result.tracking_success and result.observed_fault is None
    if profile == "detection_dropout":
        assert result.dropout_frames > 0


def test_seed_repeatability_and_separate_failure_statistics():
    first = run_robustness_trial(seed=3)
    again = run_robustness_trial(seed=3)
    changed = run_robustness_trial(seed=4)
    assert first.to_dict() == again.to_dict()
    assert first.final_error_mm != changed.final_error_mm
    stopped = run_robustness_trial(profile="command_starvation")
    summaries = summarize_robustness([first, again, stopped])
    assert summaries["panda/resolved-rate/command_starvation"]["tracking_trials"] == 0
    assert summaries["panda/resolved-rate/nominal"]["tracking_trials"] == 2


@pytest.mark.parametrize("kwargs", [{"duration_s": 0}, {"physics_hz": 0}, {"start_jitter_rad": float("nan")},
                                     {"environment_clearance_m": -1}, {"telemetry_delay_s": 0.01}])
def test_robustness_configuration_rejects_invalid_values(kwargs):
    with pytest.raises(ValueError):
        RobustnessConfig(**kwargs)
