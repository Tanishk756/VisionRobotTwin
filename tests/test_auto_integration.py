"""Full Synthetic End-to-End Perception-Gated Autonomous Pick-and-Place Integration Test.

Tests the full closed-loop pipeline without teleportation:
Synthetic Image -> ArUco Detection -> PnP Pose Estimation -> Workspace Mapping
-> Consecutive Target Acquisition -> Target Freeze & Cube Sync -> APPROACH -> PICK
-> Physical PyBullet Distance-Gated Constraint Attachment -> LIFT -> MOVE_TO_PLACE
-> PLACE -> Constraint Detachment -> RETURN_HOME -> SEARCH.
"""

import time
import numpy as np
import pybullet as p
import pytest

from config.settings import AppConfig
from main import VisionRobotTwinApp
from robotics.state_machine import RobotState


def test_full_synthetic_auto_pick_and_place_e2e():
    """Verify that complete autonomous pick-and-place sequence executes via perception and PyBullet physics."""
    config = AppConfig()
    config.camera.synthetic_mode = True
    config.simulation.gui = False
    config.mode = "auto"
    config.control_mode = "6dof"
    config.state_machine.auto_demo = False  # Strict perception gating from vision
    config.state_machine.consecutive_detection_threshold = 3  # Fast gating for test
    config.state_machine.grasp_action_delay_s = 0.05
    config.state_machine.waypoint_tolerance_m = 0.045
    config.state_machine.waypoint_timeout_s = 15.0

    app = VisionRobotTwinApp(config, headless_sim=True)

    try:
        # Initial cube position (on table)
        initial_cube_pos, _ = p.getBasePositionAndOrientation(app.simulator.pick_cube_id, physicsClientId=app.simulator.client_id)
        assert initial_cube_pos[2] < 0.05, f"Initial cube height {initial_cube_pos[2]} should be resting on table surface"

        # Step app in a bounded real-time control loop
        max_test_iterations = 1000
        reached_lift_with_cube = False
        reached_place_release = False
        reached_search_after_home = False
        fsm_states_visited = set()

        for iteration in range(max_test_iterations):
            res = app.process_frame(wall_dt=1.0 / 30.0)
            assert res.success, "Frame processing failed in auto loop"

            current_state = app.state_machine.state
            fsm_states_visited.add(current_state)

            # Check cube tracking during LIFT/MOVE_TO_PLACE
            cube_pos, _ = p.getBasePositionAndOrientation(app.simulator.pick_cube_id, physicsClientId=app.simulator.client_id)

            if current_state in (RobotState.LIFT, RobotState.MOVE_TO_PLACE):
                if cube_pos[2] > 0.10 and app.simulator.gripper.is_grasping:
                    reached_lift_with_cube = True

            if current_state == RobotState.RETURN_HOME and not app.simulator.gripper.is_grasping:
                reached_place_release = True

            if reached_place_release and current_state == RobotState.SEARCH:
                reached_search_after_home = True
                break

            # If error reached, fail test immediately
            assert current_state != RobotState.ERROR, f"FSM entered ERROR state at iteration {iteration}"

        # Assert full sequence completion through SEARCH
        assert RobotState.APPROACH in fsm_states_visited, "Did not visit APPROACH state"
        assert RobotState.PICK in fsm_states_visited, "Did not visit PICK state"
        assert RobotState.LIFT in fsm_states_visited, "Did not visit LIFT state"
        assert RobotState.MOVE_TO_PLACE in fsm_states_visited, "Did not visit MOVE_TO_PLACE state"
        assert RobotState.PLACE in fsm_states_visited, "Did not visit PLACE state"
        assert RobotState.RETURN_HOME in fsm_states_visited, "Did not visit RETURN_HOME state"

        assert reached_lift_with_cube, "Cube was not successfully attached and lifted into the air"
        assert reached_place_release, "Cube was not successfully placed and released"
        assert reached_search_after_home, "FSM did not transition back to SEARCH after completing RETURN_HOME"
        assert not app.simulator.gripper.is_grasping, "Gripper must not be grasping after cycle completion"

        # Check final cube position is resting near place target pad
        final_cube_pos, _ = p.getBasePositionAndOrientation(app.simulator.pick_cube_id, physicsClientId=app.simulator.client_id)
        place_pad_pos, _ = p.getBasePositionAndOrientation(app.simulator.place_cube_id, physicsClientId=app.simulator.client_id)

        xy_dist = np.linalg.norm(np.array(final_cube_pos[:2]) - np.array(place_pad_pos[:2]))
        assert xy_dist < 0.08, f"Final cube placement position ({final_cube_pos[:2]}) is too far from place pad ({place_pad_pos[:2]}) (distance: {xy_dist*100:.1f} cm)"
        assert -0.05 < final_cube_pos[2] < 0.06, f"Final cube height {final_cube_pos[2]} is not near support surface"

        # Check robot EE reached home position within tolerance
        ee_pos, _ = app.simulator.controller.get_end_effector_pose()
        home_pos = np.array([
            app.config.workspace.robot_center_x,
            app.config.workspace.robot_center_y,
            app.config.workspace.robot_center_z,
        ])
        ee_home_dist = float(np.linalg.norm(ee_pos - home_pos))
        assert ee_home_dist < app.config.state_machine.waypoint_tolerance_m, (
            f"Robot EE position ({ee_pos}) did not return within tolerance ({app.config.state_machine.waypoint_tolerance_m} m) of home ({home_pos}), distance: {ee_home_dist:.4f} m"
        )

    finally:
        app.cleanup()
