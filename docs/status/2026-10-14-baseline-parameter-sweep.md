# Paired baseline sweep and endpoint diagnosis

Implemented a development-only multi-seed sweep, independent-process execution,
per-trial journaling, complete-pair/error checks, seed-cluster paired diagnostics,
per-family/scenario breakdowns and exploratory Pareto analysis. Nominal runs once
per pair. An evaluator-only endpoint audit detects unsafe exact targets without
changing controller information or retroactively deleting cases.

## Recorded result

324/324 trials completed with no runner errors: three families, four scenarios,
three seeds, two horizons (100/200 ms), two margins (10/30 mm), two stop methods,
plus nominal reference runs. All child trial manifests share one source snapshot.

Nominal: 33 contact trials and 3 collision-free completions out of 36. Every stop
cell: 27 contact trials out of 36. At 10 mm, each stop cell has 3 safe completions;
at 30 mm, none. More clearance/horizon does not solve the observed dynamic cases
and generally increases intervention. No best held-out setting is selected.

The KUKA and xArm narrow-passage exact goals intersect static fixtures. In the
post-hoc exact-endpoint-free subset, every method has 27 contact trials out of 30.
The full-grid contact-count benefit therefore comes from refusing these flagged
targets, not improved dynamic avoidance. Exact-target collision does not prove
that the whole goal tolerance region or every possible trajectory is infeasible.
Original scenes and numerical results are retained.

## Validation

Full regression: **564 passed, 7 optional skips**. Fourteen sweep/analysis tests
and two endpoint geometry tests are included. The parallel pilot and full
324-trial CLI exited 0; no policy query/estimation failures appeared in decision
logs. Documentation targets and the shared trial source snapshot were verified.

[Results and figures](../experiments/2026-10-14-baseline-sweep-v1/REPORT.md).
[Protocol and reproduction](../research/BASELINE_SWEEP.md).

## Next

Create a versioned scenario successor with endpoint/tolerance preflight and
trajectory feasibility investigation. Identify stop response and queued-command
behavior. Reproduce a compatible published steering/planning comparator. The
novel safety method, reliable uncertainty calibration and hardware evidence are
still pending. These findings improve the research design, not an acceptance claim.
