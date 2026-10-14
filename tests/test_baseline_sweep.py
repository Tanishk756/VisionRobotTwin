"""Pair completeness, error retention, seed grouping, and sweep enumeration."""
from dataclasses import replace
import json
from pathlib import Path
import pytest

from robotics.research.scenarios import load_scenario
from robotics.research.sweep_analysis import analyze
from tools import sweep_research_baselines as module

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "config/research_scenarios/crossing-v1.json"


def row(cell, seed, contact, safe, intervention, method="fixed-horizon"):
    return dict(cell=cell, robot="panda", scenario="crossing", seed=seed, method=method,
                horizon_s=0.1, margin_m=0.01, status="ok", contact_episodes=contact,
                task_completed=safe, collision_free_task_completed=safe,
                execution_faults=[], initial_in_collision=False,
                intervention_fraction=intervention, final_joint_error_rad=0.1)


def test_expected_grid_and_nominal_not_duplicated(tmp_path):
    scene = load_scenario(SCENE)
    tasks = module.build_tasks([scene], ["panda"], [0, 1], [0.1, 0.2], [0.01], tmp_path)
    assert len(tasks) == 10
    assert sum(task[2] == "none" for task in tasks) == 2
    assert len({task[0] for task in tasks}) == 10
    for cell in {task[3] for task in tasks}:
        assert {task[1].config.seed for task in tasks if task[3] == cell} == {0, 1}


@pytest.mark.parametrize("changes", [dict(seeds=[0, 0]), dict(horizons=[0]), dict(margins=[0.5]),
                                      dict(robots=["unknown"]), dict(seeds=[-1]), dict(margins=[float("nan")])])
def test_invalid_grid_rejected(tmp_path, changes):
    args = dict(scenarios=[load_scenario(SCENE)], robots=["panda"], seeds=[0], horizons=[0.1], margins=[0.01], output=tmp_path)
    args.update(changes)
    with pytest.raises(ValueError): module.build_tasks(**args)


def test_tuning_cannot_use_calibration_or_test_split(tmp_path):
    scene = replace(load_scenario(SCENE), split="test")
    with pytest.raises(ValueError, match="development"):
        module.build_tasks([scene], ["panda"], [0], [0.1], [0.01], tmp_path)


def test_pairing_and_pareto_keep_both_safety_and_progress():
    rows = [row("nominal", seed, 1, False, 0, "none") for seed in (0, 1)]
    rows += [row("good", 0, 0, True, 0.5), row("good", 1, 1, False, 0.5)]
    rows += [row("worse", 0, 0, False, 0.5), row("worse", 1, 1, False, 0.5)]
    result = analyze(rows)
    delta = result["paired_vs_nominal"]["good"]
    assert delta["seed_clusters"] == 2
    assert delta["collision_fraction_delta_vs_nominal"]["mean"] == -0.5
    assert delta["safe_completion_fraction_delta_vs_nominal"]["mean"] == 0.5
    assert set(result["exploratory_pareto_cells"]) == {"good", "nominal"}
    assert analyze(rows) == result


def test_errors_remain_in_denominator_and_exclude_cell_from_selection():
    rows = [row("nominal", seed, 0, True, 0, "none") for seed in (0, 1)]
    rows += [row("broken", 0, 0, True, 0),
             dict(cell="broken", robot="panda", scenario="crossing", seed=1, method="fixed-horizon",
                  horizon_s=0.1, margin_m=0.01, status="error", error="failure")]
    result = analyze(rows)
    assert result["groups"]["broken"]["failure_adjusted_safe_completion_fraction"] == 0.5
    assert result["groups"]["broken"]["errors"] == 1
    assert "broken" not in result["paired_vs_nominal"]
    assert "broken" not in result["exploratory_pareto_cells"]


def test_duplicate_cases_cannot_inflate_evidence():
    sample = row("a", 0, 0, True, 0)
    with pytest.raises(ValueError, match="Duplicate"): analyze([sample, sample])


def test_missing_case_in_every_cell_is_detected_from_declared_grid():
    sample = row("nominal", 0, 0, True, 0, "none")
    report = analyze([sample], expected_keys={("panda", "crossing", 0), ("panda", "crossing", 1)})
    assert not report["groups"]["nominal"]["complete_pair_grid"]
    assert report["groups"]["nominal"]["failure_adjusted_safe_completion_fraction"] == 0.5
    assert not report["exploratory_pareto_cells"]


def test_contact_onset_policy_state_is_not_contact_sample_count(tmp_path):
    (tmp_path / "decisions.json").write_text(json.dumps([
        dict(time_s=0, state="TRACK"), dict(time_s=0.1, state="HOLD"), dict(time_s=0.3, state="FAULT")]))
    frames = [dict(time_s=t, in_collision=c) for t, c in [(0, False), (0.1, True), (0.2, True), (0.3, False), (0.4, True)]]
    (tmp_path / "trace.jsonl").write_text("\n".join(json.dumps(frame) for frame in frames))
    counts = module.onset_states(tmp_path)
    assert counts["HOLD"] == counts["FAULT"] == 1
    assert sum(counts.values()) == 2


def test_sequential_sweep_journals_failures_and_preserves_output(tmp_path, monkeypatch):
    def execute(task):
        data = module.task_metadata(task)
        if task[2] == "motion-aware":
            return {**data, "status": "error", "error": "injected"}
        return {**row(task[3], task[1].config.seed, 0, True, 0, task[2]), **data}
    monkeypatch.setattr(module, "execute_case", execute)
    index, analysis = module.sweep([SCENE], ["panda"], [0], [0.1], [0.01], tmp_path)
    assert index["expected_trials"] == 3
    assert len(index["errors"]) == 1
    assert len((tmp_path / "progress.jsonl").read_text().splitlines()) == 3
    assert (tmp_path / "manifest.json").is_file()
    with pytest.raises(ValueError, match="empty"):
        module.sweep([SCENE], ["panda"], [0], [0.1], [0.01], tmp_path)
