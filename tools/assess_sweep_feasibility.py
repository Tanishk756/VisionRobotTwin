"""Post-hoc endpoint geometry diagnostic, never a controller/oracle input."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
import pybullet as p
from robotics.research.contracts import ScenarioConfig
from robotics.research.scenarios import ResearchScenario
from robotics.research.plant_twin import PlantTwinHarness
from tools.run_plant_twin_experiment import safe_json


def assess_endpoint(scenario, robot):
    resolved = replace(scenario, config=replace(scenario.config, robot=robot))
    with PlantTwinHarness(resolved.config, resolved) as harness:
        plant = harness._plant  # explicit evaluator-only access
        target = np.array(plant.robot_spec.home_joint_positions)
        target[0] += scenario.target_delta_first_joint_rad
        initial = plant.collision_provider.check_collision()
        p.resetBasePositionAndOrientation(harness._plant_obstacle,
                                          harness._obstacle_position(resolved.config.duration_s),
                                          (0, 0, 0, 1), physicsClientId=plant.client_id)
        endpoint = plant.collision_provider.check_collision(target)
        original = list(plant.collision_provider.obstacle_ids)
        try:
            plant.collision_provider.obstacle_ids[:] = [body for body in original if body != harness._plant_obstacle]
            static = plant.collision_provider.check_collision(target)
        finally:
            plant.collision_provider.obstacle_ids[:] = original
        limits = plant.resolved_model
        within_limits = bool(np.all(target >= limits.arm_lower_limits) and np.all(target <= limits.arm_upper_limits))
        return {"robot": robot, "scenario": scenario.scenario_id, "target_joint_positions": list(target),
                "target_within_joint_limits": within_limits, "initial_collision": initial.in_collision,
                "target_static_geometry_free": not static.in_collision and within_limits,
                "target_final_geometry_free": not endpoint.in_collision and within_limits,
                "static_target_min_environment_clearance_m": static.min_env_clearance_m,
                "final_target_min_environment_clearance_m": endpoint.min_env_clearance_m,
                "static_target_colliding_links": static.colliding_links,
                "final_target_colliding_links": endpoint.colliding_links}


def audit(folder):
    folder = Path(folder)
    output = folder / "endpoint-feasibility.json"
    if output.exists():
        raise ValueError("Endpoint diagnostic already exists; preserve the original")
    manifest = json.loads((folder / "manifest.json").read_text())
    scenarios = [ResearchScenario(**{**snapshot, "config": ScenarioConfig(**snapshot["config"])})
                 for snapshot in manifest["scenarios"]]
    rows = [assess_endpoint(scene, robot) for robot in manifest["robots"] for scene in scenarios]
    result = {"purpose": "Post-hoc endpoint geometry only; not a dynamic trajectory feasibility proof or controller input",
              "source_files_sha256": {"tools/assess_sweep_feasibility.py": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
              "sweep_manifest_sha256": hashlib.sha256((folder / "manifest.json").read_bytes()).hexdigest(),
              "cases": rows}
    output.write_text(json.dumps(safe_json(result), indent=2, allow_nan=False), encoding="utf-8")
    return result


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep-dir", required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.sweep_dir)
    print(json.dumps(safe_json(result), indent=2))
