# Deterministic motion-aware baseline: development comparison

36/36 trials completed with zero runner errors. Four development scenarios, three robot families, seed 0 and three methods. No initially colliding trial occurred.

| Method | Trials | Contact episodes | Final target completions | Collision-free completions | Watchdog-fault trials |
|---|---:|---:|---:|---:|---:|
| none | 12 | 11 | 4 | 1 | 3 |
| fixed-horizon | 12 | 9 | 1 | 1 | 3 |
| motion-aware | 12 | 9 | 1 | 1 | 3 |

## Findings and limits

The deterministic motion-aware comparator does not improve aggregate contact counts or collision-free task completion over the frozen-obstacle comparator in this one-seed set. Its intervention timing differs, but all crossing, sudden-approach and occluded-crossing cases still contact obstacles. Both comparators avoid the KUKA/xArm narrow-passage contacts at the cost of incomplete tasks. Panda narrow-passage is the only collision-free completion for each method. All occluded runs latch telemetry timeout.

These are development diagnostics, not statistical superiority results. Defaults have not been tuned at matched empirical risk. Constant-velocity means and a stop-only policy can fail when an obstacle approaches a held arm; the experiment does not isolate filtering, observation-age compensation and extrapolation effects. Uncertainty covariance remains unused following its failed calibration diagnostic.

## Validation and reproduction

Full regression: 548 passed, 7 optional skips. Thirteen new motion-aware checks cover numeric estimate timing, future whole-arm contact, per-sample obstacle positions, failed-query restoration, hidden plant-state independence, unused covariance, estimator/reset/fault handling, policy estimate logging and shared comparison parameters.

Run `python tools/compare_research_baselines.py --modes none fixed-horizon motion-aware --output-dir experiments/motion-aware-reproduction` from the repository root. Output must be empty. [Method and interfaces](../../research/MOTION_AWARE_BASELINE.md).

All 36 retained manifests have a single source snapshot and record dirty source based on commit 65ae625. They embed resolved scenarios, seeds, controller parameters and source hashes. Raw traces/events/estimates/predictions/decisions remain locally under ignored `experiments/motion-aware-development/`; summaries retain raw-trace hashes. Use `decisions.json` for the actual motion-aware policy estimate, which may differ from the separate logging estimator.

Next: development parameter sweeps and additional paired seeds, stronger stopping/avoidance behavior, and a suitable published comparator. Neither formal safety, a new research contribution nor hardware validity is established.
