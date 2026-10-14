# First obstacle uncertainty calibration diagnostic

Added separate fitting and validation scenario definitions, a sensor-only
calibration runner, empirical covariance inflation, horizon/age/scenario coverage
breakdowns and seed-cluster bootstrap intervals. The runner prevents overlapping
scenario IDs or seed sets, preserves output directories, and fits scaling without
validation scores. Future truth is evaluator-only. An exportable diagnostic
figure accompanies the recorded results.

## Result

Forty 3-second trials completed: twenty fitting and twenty validation. At nominal
95% confidence, validation region coverage was 87.6%, 85.8%, 83.8% and 80.6% at
0, 50, 100 and 200 ms. RMSE rose from 59.0 to 132.9 mm. All fitting inflation
factors remained 1.0; the simple fitting trajectories did not reveal the shifted
validation motion/delay error. Undercoverage remains visible.

The result does not establish reliable calibration, collision risk, image-level
sensing accuracy, or physical safety. No fitted scaling is applied to safety
constraints. The current validation cases now inform development; any revised
fitting method needs new untouched validation cases before an independent claim.

## Validation

- 535 regression tests passed, 7 optional skips.
- 16 focused calibration tests passed.
- Calibration release CLI exited 0 and generated the report, manifest and SVG/PNG.
- [Recorded diagnostic](../experiments/2026-10-14-obstacle-calibration-v1/REPORT.md).
- [Protocol and reproduction](../research/UNCERTAINTY_CALIBRATION.md).

Next: a deterministic obstacle-motion comparator without probabilistic safety
claims, alongside improved error modeling and representative fitting trajectories.
Published-baseline reproduction, method novelty, held-out controller evaluation
and hardware validation remain pending.
