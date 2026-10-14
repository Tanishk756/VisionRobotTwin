"""Known blocked/free endpoints and evaluator-only artifact preservation."""
from robotics.research.contracts import ScenarioConfig
from robotics.research.scenarios import ResearchScenario
from tools.assess_sweep_feasibility import assess_endpoint


def test_known_static_endpoint_contact_is_not_counted_as_feasible():
    scene = ResearchScenario("blocked-goal", "development", ScenarioConfig(), "nominal_home_ee",
                             ((0, 0, 0.4, 0),), fixtures=((0, 0, 0, 0.05),),
                             target_delta_first_joint_rad=0)
    result = assess_endpoint(scene, "panda")
    assert result["target_within_joint_limits"]
    assert not result["target_static_geometry_free"]
    assert not result["target_final_geometry_free"]


def test_free_home_endpoint_has_no_geometry_guarantee_beyond_endpoint():
    scene = ResearchScenario("free-goal", "development", ScenarioConfig(), "world",
                             ((0, 0.7, 0.7, 0.7),), target_delta_first_joint_rad=0)
    result = assess_endpoint(scene, "panda")
    assert result["target_static_geometry_free"]
    assert result["target_final_geometry_free"]
