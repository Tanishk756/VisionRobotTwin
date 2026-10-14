"""Versioned development scenarios; future plant trajectories are evaluator-only."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import re

import numpy as np

from robotics.research.contracts import ScenarioConfig, nonnegative, vector


@dataclass(frozen=True)
class ResearchScenario:
    scenario_id: str
    split: str
    config: ScenarioConfig
    frame: str
    waypoints: tuple
    occlusions: tuple = ()
    fixtures: tuple = ()
    target_delta_first_joint_rad: float = 0.35
    schema_version: int = 1

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("Unsupported scenario schema version")
        if not isinstance(self.scenario_id, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_-]*", self.scenario_id) or self.split not in ("development", "calibration", "test"):
            raise ValueError("Scenario ID and valid data split are required")
        if self.frame not in ("world", "nominal_home_ee"):
            raise ValueError("Unsupported scenario frame")
        points = tuple(vector("waypoint [t,x,y,z]", row, 4) for row in self.waypoints)
        if not points or points[0][0] != 0 or any(b[0] <= a[0] for a, b in zip(points, points[1:])):
            raise ValueError("Waypoints must start at zero with increasing times")
        intervals = tuple(vector("occlusion [start,end]", row, 2) for row in self.occlusions)
        if any(a < 0 or b <= a or b > self.config.duration_s for a, b in intervals):
            raise ValueError("Invalid occlusion interval")
        fixtures = tuple(vector("fixture [x,y,z,radius]", row, 4) for row in self.fixtures)
        if any(row[3] <= 0 for row in fixtures):
            raise ValueError("Fixture radius must be positive")
        vector("target offset", [self.target_delta_first_joint_rad])
        object.__setattr__(self, "waypoints", points)
        object.__setattr__(self, "occlusions", intervals)
        object.__setattr__(self, "fixtures", fixtures)

    def position(self, time_s, anchor=(0, 0, 0)):
        nonnegative("time_s", time_s)
        points = np.array(self.waypoints)
        local = np.array([np.interp(time_s, points[:, 0], points[:, axis]) for axis in (1, 2, 3)])
        return local + np.array(anchor)

    def occluded(self, time_s):
        return any(start <= time_s < end for start, end in self.occlusions)


def load_scenario(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    allowed = {"schema_version", "scenario_id", "split", "config", "frame", "waypoints",
               "occlusions", "fixtures", "target_delta_first_joint_rad"}
    if not isinstance(data, dict) or set(data) - allowed:
        raise ValueError("Unknown scenario fields")
    required = {"schema_version", "scenario_id", "split", "config", "frame", "waypoints"}
    if not required <= set(data):
        raise ValueError("Missing required scenario fields")
    return ResearchScenario(**{**data, "config": ScenarioConfig(**data["config"])})


def scenario_snapshot(scenario):
    """Resolved, self-contained definition, including all configuration defaults."""
    return asdict(scenario)
