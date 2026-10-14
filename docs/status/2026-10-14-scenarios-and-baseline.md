# Versioned scenarios and first engineering safety comparison

Implemented the next research milestone:

- Version-1 JSON development cases: crossing, sudden approach, occluded crossing,
  and narrow passage with known static fixtures.
- Observation-only candidate collision checks against the whole arm, self,
  table and fixtures, preserving twin state and transport randomness.
- A fixed-horizon transported-stop comparator with HOLD hysteresis, distinct
  clear-observation recovery, and latched query/execution faults.
- A paired comparison CLI with resolved scenario manifests, decisions, failures,
  completion/contact metrics and decision-time diagnostics.

The comparator holds the moving obstacle at its last observed pose. It is an
engineering reference, not the new uncertainty-aware algorithm or a reproduction
of a published method. Static fixtures are a shared known map. Future plant
trajectories and occlusion schedules remain outside controller inputs.

## Validation

The comparison CLI completed all 24 trials with no runner errors. Nominal control
had 11 contact episodes and 4 target completions; fixed-horizon control had 9
contact episodes and 1 target completion. Both had only 1 collision-free target
completion. All occlusion cases latched telemetry timeout. These are one-seed
development observations, with failures preserved, not statistical evidence of
superiority or continuous collision safety.

Regression: **519 passed, 7 optional skips**. The focused new scenario/baseline
suite passed all 21 tests; documentation links and 24 consistent source manifests
were also verified.

[Saved results and provenance](../experiments/2026-10-14-fixed-horizon-baseline/REPORT.md).
[Interfaces and reproduction](../research/BASELINE_SCENARIOS.md).

## Next research work

Calibrate the obstacle estimator on disjoint calibration scenarios, introduce
motion-aware deterministic prediction as a stronger comparator, sweep existing
baseline parameters on development data, and reproduce a suitable published
method. Add identified motor/stopping behavior and evaluator convergence checks.
Freeze held-out scenarios only after these development steps. Novelty review G0
remains open; no new safety contribution or hardware validity is established.
