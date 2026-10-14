# Deterministic motion-aware comparator milestone

Added constant-velocity obstacle-mean prediction to the existing stop/recovery
policy. The whole-arm candidate query samples the predicted obstacle path and
restores twin robot/obstacle state, including on errors. Estimates must match
current controller time and latest delivered capture time. No future scenario
trajectory or plant truth enters prediction; covariance is deliberately unused.

The CLI supports `motion-aware` in individual runs and paired comparisons.
Common horizon and margin settings are exposed for subsequent fair sweeps.
Decision logs retain the actual policy estimate. Original two-method defaults
are preserved to reproduce earlier experiments.

## Validation

- 548 full regression tests passed, 7 optional skips.
- Thirteen new motion-aware tests are included in that run.
- The three-method comparison CLI completed all 36 cases with no runner errors.
- All retained manifests share one source snapshot.

## Development result

Nominal control: 11 contact episodes / 4 target completions / 1 collision-free
completion across 12 cases. Frozen baseline: 9 / 1 / 1. Motion-aware baseline:
9 / 1 / 1. No aggregate improvement is established by adding deterministic
motion prediction under these defaults. Failures and watchdog stops are retained.
The set uses one seed and is not a held-out statistical evaluation.

[Results and provenance](../experiments/2026-10-14-motion-aware-baseline/REPORT.md).
[Reproduction and assumptions](../research/MOTION_AWARE_BASELINE.md).

Next: systematic development horizon/margin sweeps with additional paired seeds,
stopping/avoidance analysis and published-baseline reproduction. Reliable
uncertainty calibration and candidate-method novelty remain open.
