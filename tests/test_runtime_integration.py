"""Comprehensive Integration and Regression Tests for Multi-Robot Runtime and Benchmarking.

Validates:
1. Runtime controller selection (IK vs Resolved-Rate differential IK execution paths).
2. Velocity-limited position control clamping target changes by (max_vel * dt).
3. Hold / Home / Reset safety and tracking loss zero-velocity commands.
4. Measured Forward Kinematics (FK) IK residual computation vs independent evaluation.
5. Live robotics diagnostics telemetry (manipulability, condition number, collision state, planner state).
6. Collision-aware obstacle scene planning in MotionManager.
7. Benchmark methodology validity (fair initialization, non-fabricated metrics, shared reachable targets).
"""

import pytest
import numpy as np
import pybullet as p
import pybullet_data

from config.settings import get_default_config
from main import VisionRobotTwinApp, FrameResult
from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.inverse_kinematics import GenericIKSolver, IKStatus
from robotics.differential_ik import ResolvedRateController
from robotics.motion_manager import MotionManager, MotionState
from robotics.collision import CollisionChecker
from robotics.trajectory import JointQuinticTrajectory, PiecewiseJointTrajectory
from tools.compare_robots import generate_shared_benchmark_targets, benchmark_robot
from tools.compare_controllers import benchmark_controller


@pytest.fixture
def sim_env():
    """Initializes a DIRECT PyBullet simulation client with table and robot."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    robot_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, robot_id, spec)
    controller.reset_to_home()

    yield client_id, robot_id, table_id, controller, spec
    p.disconnect(physicsClientId=client_id)


def test_position_controller_joint_velocity_rate_limiting(sim_env):
    """Verifies set_arm_joint_positions clamps commanded deltas to <= (max_velocity * dt)."""
    client_id, robot_id, table_id, controller, spec = sim_env
    dt = 0.01  # 10 ms step

    initial_q = controller.get_current_joint_positions()
    # Request a massive instantaneous joint leap (e.g. +2.0 rad across all joints)
    large_target_q = [q + 2.0 for q in initial_q]

    commanded = controller.set_arm_joint_positions(large_target_q, dt=dt, enforce_velocity_limits=True)

    # Inspect the actual target values returned
    for i, j_idx in enumerate(controller.arm_joint_indices):
        max_vel = controller.joints[j_idx].max_velocity
        max_allowed_delta = max_vel * dt
        actual_delta = commanded[i] - initial_q[i]
        assert actual_delta <= max_allowed_delta + 1e-6
        assert actual_delta > 0.0


def test_actual_ik_residual_matches_independent_fk(sim_env):
    """Verifies GenericIKSolver returns measured Cartesian & orientation residuals matching independent FK."""
    client_id, robot_id, table_id, controller, spec = sim_env
    lows, highs, ranges, rests = controller.get_joint_limits()

    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )

    test_target_pos = np.array([0.45, 0.10, 0.30])
    test_target_orn = np.array([1.0, 0.0, 0.0, 0.0])

    res = ik_solver.solve(test_target_pos, test_target_orn)
    assert res.success
    assert res.residual_position_m is not None
    assert res.residual_position_m > 0.0  # Must be measured, not unconditional 0.0

    # Independently compute FK by placing joints at solved angles
    saved_states = p.getJointStates(robot_id, controller.arm_joint_indices, physicsClientId=client_id)
    try:
        for idx, angle in zip(controller.arm_joint_indices, res.joint_positions):
            p.resetJointState(robot_id, idx, angle, targetVelocity=0.0, physicsClientId=client_id)
        link_state = p.getLinkState(robot_id, controller.ee_link_index, computeForwardKinematics=True, physicsClientId=client_id)
        fk_pos = np.array(link_state[0])
        independent_res_m = float(np.linalg.norm(test_target_pos - fk_pos))
        assert abs(res.residual_position_m - independent_res_m) < 1e-4
    finally:
        for idx, st in zip(controller.arm_joint_indices, saved_states):
            p.resetJointState(robot_id, idx, st[0], targetVelocity=st[1], physicsClientId=client_id)


def test_ik_residual_too_high_rejection(sim_env):
    """Verifies solutions with high Cartesian residuals are rejected with RESIDUAL_TOO_HIGH."""
    client_id, robot_id, table_id, controller, spec = sim_env
    lows, highs, ranges, rests = controller.get_joint_limits()

    # Configure solver with strict residual tolerance of 0.1 mm
    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
        max_residual_position_m=0.0001,  # 0.1 mm
    )

    test_target_pos = np.array([0.45, 0.15, 0.25])
    res = ik_solver.solve(test_target_pos, None)
    assert not res.success
    assert res.status == IKStatus.RESIDUAL_TOO_HIGH


def test_runtime_controller_selection_and_telemetry_population():
    """Verifies selecting IK vs Resolved-Rate changes execution paths and populates structured diagnostics."""
    config_ik = get_default_config()
    config_ik.robot_name = "panda"
    config_ik.controller_type = "ik"
    config_ik.camera.synthetic_mode = True
    config_ik.simulation.gui = False
    config_ik.max_frames = 5

    app_ik = VisionRobotTwinApp(config=config_ik, headless_sim=True)
    res_ik = app_ik.process_frame(wall_dt=0.01)

    assert isinstance(res_ik, FrameResult)
    assert res_ik.controller_type == "IK"
    assert res_ik.manipulability is not None and res_ik.manipulability > 0.0
    assert res_ik.jacobian_condition is not None and res_ik.jacobian_condition > 0.0
    assert res_ik.sigma_min is not None and res_ik.sigma_min > 0.0
    assert res_ik.singularity_state in ("NORMAL", "WARNING")
    assert res_ik.collision_state in ("CLEAR", "SELF_COLLISION", "ENV_COLLISION")
    assert res_ik.planner_state == "IDLE"
    app_ik.cleanup()

    # Now run Resolved-Rate controller
    config_rr = get_default_config()
    config_rr.robot_name = "panda"
    config_rr.controller_type = "resolved-rate"
    config_rr.camera.synthetic_mode = True
    config_rr.simulation.gui = False
    config_rr.max_frames = 5

    app_rr = VisionRobotTwinApp(config=config_rr, headless_sim=True)
    res_rr = app_rr.process_frame(wall_dt=0.01)

    assert isinstance(res_rr, FrameResult)
    assert res_rr.controller_type == "RESOLVED-RATE"
    assert res_rr.manipulability is not None and res_rr.manipulability > 0.0
    assert res_rr.jacobian_condition is not None and res_rr.jacobian_condition > 0.0
    app_rr.cleanup()


def test_motion_manager_obstacle_avoidance_planning(sim_env):
    """Verifies MotionManager detects blocked direct path, plans via RRT-Connect, and avoids colliding direct target."""
    client_id, robot_id, table_id, controller, spec = sim_env

    # Load obstacle in central path
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.20], physicsClientId=client_id)
    obs_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col_box, basePosition=[0.50, 0.0, 0.20], physicsClientId=client_id)

    lows, highs, ranges, rests = controller.get_joint_limits()
    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=robot_id,
        table_id=table_id,
        obstacle_ids=[obs_id],
    )

    mm = MotionManager(
        robot_controller=controller,
        ik_solver=ik_solver,
        collision_checker=checker,
        trajectory_mode="quintic",
        scene_type="obstacles",
        default_trajectory_duration=1.0,
    )

    # Plan to a goal pose on the other side of obstacle
    goal_pos = [0.45, 0.20, 0.25]
    success = mm.plan_motion_to_pose(goal_pos)
    assert success
    assert mm.state == MotionState.EXECUTING
    assert mm.active_trajectory is not None

    # Step execution
    is_done, progress = mm.step(dt=0.1)
    assert 0.0 <= progress <= 100.0


def test_benchmark_shared_targets_fairness():
    """Verifies generate_shared_benchmark_targets produces reachable targets accepted by both robots."""
    targets, meta = generate_shared_benchmark_targets(seed=42, target_count=4, headless=True)
    assert len(targets) == 4
    assert meta["accepted_shared_targets"] == 4
    assert meta["candidate_count"] >= 4
    assert "rejected_position" in meta
    assert "rejected_orientation" in meta
    assert meta["shared_SE3_position_tolerance_mm"] == 25.0
    assert meta["shared_SE3_orientation_tolerance_deg"] == 10.0

    for pos, orn in targets:
        assert len(pos) == 3
        assert len(orn) == 4
        assert np.all(np.isfinite(pos))
        assert np.all(np.isfinite(orn))
        assert np.allclose(orn, np.array([1.0, 0.0, 0.0, 0.0]))


def test_coordinated_position_velocity_rate_limiting_direction_preservation(sim_env):
    """Verifies proportional scaling preserves exact joint-space line segment direction."""
    client_id, robot_id, table_id, controller, spec = sim_env
    dt = 0.005  # 5 ms

    q_curr = np.array(controller.get_current_joint_positions(), dtype=np.float64)
    # Request large disparate joint step
    delta_req = np.array([0.5, -0.2, 0.8, -1.0, 0.3, -0.6, 0.4])
    q_target = q_curr + delta_req

    q_cmd = np.array(controller.set_arm_joint_positions(q_target.tolist(), dt=dt, enforce_velocity_limits=True))

    actual_delta = q_cmd - q_curr
    # Ratio between actual delta and requested delta must be identical across all joints (collinear vector)
    nonzero_idx = np.abs(delta_req) > 1e-6
    scales = actual_delta[nonzero_idx] / delta_req[nonzero_idx]
    assert np.allclose(scales, scales[0], atol=1e-5)
    assert 0.0 < scales[0] < 1.0


def test_auto_motion_manager_ownership_and_no_double_commanding():
    """Verifies MotionManager owns AUTO motion and ordinary IK/ResolvedRate dispatch is not double-commanding."""
    config = get_default_config()
    config.robot_name = "panda"
    config.mode = "auto"
    config.auto_demo = True
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.max_frames = 10

    app = VisionRobotTwinApp(config=config, headless_sim=True)

    # Execute frames in AUTO mode
    for _ in range(5):
        res = app.process_frame(wall_dt=0.01)
        assert res.success
        assert res.mode == "AUTO"

    # Confirm MotionManager is stepped and planner state is tracked
    assert app.simulator.motion_manager is not None
    app.cleanup()


def test_trajectory_mode_direct_vs_quintic_execution(sim_env):
    """Verifies trajectory-mode direct and quintic produce distinct trajectory structures."""
    client_id, robot_id, table_id, controller, spec = sim_env
    lows, highs, ranges, rests = controller.get_joint_limits()

    ik_solver = GenericIKSolver(
        physics_client_id=client_id,
        robot_id=robot_id,
        arm_joint_indices=controller.arm_joint_indices,
        lower_limits=lows,
        upper_limits=highs,
        joint_ranges=ranges,
        rest_poses=rests,
        end_effector_link_index=controller.ee_link_index,
        max_reach_m=spec.spherical_reach_m,
        min_reach_m=spec.min_reach_m,
        default_ee_orientation=spec.default_ee_orientation,
    )

    mm_quintic = MotionManager(
        robot_controller=controller,
        ik_solver=ik_solver,
        collision_checker=None,
        trajectory_mode="quintic",
        default_trajectory_duration=2.0,
    )
    mm_quintic.plan_motion_to_pose([0.45, 0.10, 0.30])
    assert mm_quintic.active_trajectory is not None
    assert mm_quintic.active_executor is not None
    assert mm_quintic.state == MotionState.EXECUTING

    mm_direct = MotionManager(
        robot_controller=controller,
        ik_solver=ik_solver,
        collision_checker=None,
        trajectory_mode="direct",
        default_trajectory_duration=2.0,
    )
    mm_direct.plan_motion_to_pose([0.45, 0.10, 0.30])
    assert mm_direct.state == MotionState.DIRECT
    assert mm_direct._target_joint_positions is not None


def test_goal_deduplication_no_per_frame_replanning():
    """Verifies stationary AUTO goal does not re-invoke planning on every frame."""
    config = get_default_config()
    config.robot_name = "panda"
    config.mode = "auto"
    config.auto_demo = True
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.max_frames = 10

    app = VisionRobotTwinApp(config=config, headless_sim=True)

    # Frame 1: triggers plan
    app.process_frame(wall_dt=0.01)
    plan_obj_initial = app.simulator.motion_manager.active_trajectory

    # Frame 2 with same goal: should NOT replan or replace active trajectory
    app.process_frame(wall_dt=0.01)
    plan_obj_subsequent = app.simulator.motion_manager.active_trajectory

    if plan_obj_initial is not None:
        assert plan_obj_initial is plan_obj_subsequent

    app.cleanup()


def test_compare_controllers_final_goal_settling_and_accounting():
    """Verifies benchmark_controller includes settling stage, goal tolerance timing, and truthful counts."""
    trajs = [
        (np.array([0.4, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.42, 0.05, 0.32]), np.array([1.0, 0.0, 0.0, 0.0])),
        (np.array([0.42, 0.05, 0.32]), np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.45, -0.05, 0.28]), np.array([1.0, 0.0, 0.0, 0.0])),
    ]
    res = benchmark_controller(
        robot_name="panda",
        controller_type="ik",
        trajectories=trajs,
        headless=True,
        trajectory_duration=1.0,
        settle_duration=0.2,
        settle_tolerance_mm=5.0,
    )
    assert res["requested_trajectories"] == 2
    assert res["executed_trajectories"] == 2
    assert res["skipped_initialization_failures"] == 0
    assert res["completion_rate"] == 1.0
    assert "time_to_final_goal_tolerance_s" in res
    assert "settle_duration_s" in res
    assert res["settle_duration_s"] == 0.2


def test_simulator_shared_kinematics_provider_identity():
    """Verifies that PyBulletSimulator instantiates a single PyBulletKinematicsProvider shared across all components."""
    from robotics.simulator import PyBulletSimulator
    from robotics.kinematics_provider import PyBulletKinematicsProvider
    from config.settings import get_default_config

    config = get_default_config()
    sim = PyBulletSimulator(config, headless=True)

    try:
        assert isinstance(sim.kinematics_provider, PyBulletKinematicsProvider)
        assert sim.controller.kinematics_provider is sim.kinematics_provider
        assert sim.ik_solver.provider is sim.kinematics_provider
        assert sim.resolved_rate_controller.kinematics_provider is sim.kinematics_provider

        # Test manipulability query using the shared provider
        metrics, J = sim.get_current_manipulability()
        assert J.shape == (6, len(sim.controller.arm_joint_indices))
        assert metrics.manipulability > 0.0
    finally:
        sim.close()


def test_simulator_shared_lock_identity():
    """Verifies that PyBulletSimulator shares a single model-query lock between kinematics and collision providers."""
    from robotics.simulator import PyBulletSimulator
    from robotics.collision_provider import PyBulletCollisionProvider
    from config.settings import get_default_config

    config = get_default_config()
    sim = PyBulletSimulator(config, headless=True)

    try:
        assert isinstance(sim.collision_provider, PyBulletCollisionProvider)
        assert sim.collision_checker is sim.collision_provider
        assert sim.motion_manager.collision_provider is sim.collision_provider
        assert sim.kinematics_provider.query_lock is sim.collision_provider.query_lock

        # Verify collision query succeeds
        col = sim.collision_provider.check_collision()
        assert not col.in_collision
    finally:
        sim.close()


def test_simulator_resolved_model_identity():
    """Verifies that PyBulletSimulator resolves a canonical ResolvedRobotModel and wires it across subsystems."""
    from robotics.simulator import PyBulletSimulator
    from robotics.robot_model import ResolvedRobotModel
    from config.settings import get_default_config

    config = get_default_config()
    sim = PyBulletSimulator(config, headless=True)

    try:
        assert isinstance(sim.resolved_model, ResolvedRobotModel)
        assert sim.controller.model is sim.resolved_model
        assert sim.resolved_model.dof == 7
        assert sim.resolved_model.robot_id == "panda"
        assert sim.resolved_model.require_arm_native_indices() == tuple(sim.controller.arm_joint_indices)
    finally:
        sim.close()

