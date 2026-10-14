# Obstacle uncertainty: first calibration diagnostic

The initial experiment measures whether the constant-velocity estimator's
position ellipsoids achieve their nominal pointwise coverage. It does not
establish calibrated collision probabilities or a safe uncertainty-aware filter.

## Reproduce

```powershell
python tools/calibrate_obstacle_estimator.py --output-dir experiments/calibration-example --plot
```

The output directory must be empty. Plot export uses the existing development
dependency Matplotlib. Without `--plot`, only NumPy/SciPy and project modules
are required. Defaults run 20 fitting and 20 validation trials, each 3 seconds.

## Protocol

Definitions are separate from robot-controller development scenes, under
`config/research_calibration/`. All are marked `calibration`: the validation
subset is a calibration-development diagnostic, not the final paper test set.

Fitting uses linear motion and linear motion with occlusion, seeds 0–9.
Validation uses different turning/delay/occlusion trajectories, seeds 100–109.
Scenario IDs and seed sets must be disjoint. A shared seed is used across the
two scenarios in each subset; these trials are not statistically independent.

This fast experiment simulates only timestamped position sensing. It uses the
research event queue and the same Gaussian-noise, dropout and jitter distributions
as the plant/twin harness, without an arm, images, or collision dynamics. It does
not claim identical random draws to a full robot trial. Ground-truth trajectory
positions are used to generate measurements and score forecasts, never as
estimator inputs. All scored predictions use observations delivered by the
current time. Future truth is accessed only by the evaluator.

At the fixed sensing cadence, score horizons 0, 50, 100 and 200 ms, including
occlusion periods. Do not silently discard startup, stale or high-error forecasts.
Times before the first observation are counted as unavailable; forecasts beyond
the trial end are excluded explicitly. Per-trial counters record capture loss,
reordering and forecast availability. The estimator uses the configured sensor
noise standard deviation, process spectral density 0.1 and initial velocity
standard deviation 0.5 m/s. This assumes the noise setting is known; unknown
sensor calibration error is outside the first experiment.

## Scoring and empirical scaling

For position error e and predicted position covariance P, score eᵀP⁻¹e.
Compare it to the chi-square(3) 95% threshold to assess a nominal Gaussian
position ellipsoid. This reference threshold is a modeling assumption, not
evidence that the errors are Gaussian.

For each horizon, fit covariance inflation as the larger of 1 and the fitting
frames' empirical 95th-percentile score divided by that threshold. The same
frozen inflation is then evaluated on validation data. The fitting rule cannot
shrink covariance, and it does not access validation errors. Correlated frames
mean this is not split-conformal calibration or a finite-sample coverage guarantee.

Report coverage for each trial, average across trials, and resample **seeds**
1,000 times for descriptive 95% bootstrap intervals. Scenarios and frames sharing
a seed remain grouped. Intervals quantify noise/delay-seed variation conditional
on these fixed trajectories; they do not cover an arbitrary population of scenes.
Additional breakdowns report observation-age bins and scenario coverage.

## First result and decision

On these fitting cases, all conservative inflation factors remained 1.0. On
the shifted validation cases, nominal 95% regions achieved approximately 87.6%,
85.8%, 83.8% and 80.6% coverage at 0/50/100/200 ms. Position RMSE rose from
59.0 mm to 132.9 mm across those horizons. The saved report retains all values,
intervals, availability counters and per-scenario/age breakdowns.

The estimator **is not demonstrated calibrated under this shift**. The validation
trajectories contain piecewise velocity changes and combined delay/noise/occlusion
stress; the experiment does not isolate a causal effect of one factor. These
artificial changes can violate constant-velocity/Gaussian assumptions. Do not
generalize the exact coverage figures to physical sensing or all deployment scenes.

Keep the failure visible. Do not refit scaling on the same validation data and
then call its improved coverage held out. If revising process noise, estimator
structure or fitting motion families after this diagnostic, treat the existing
validation set as development data and create a new untouched validation set.

No calibrated scaling is wired into safety constraints. Next, add the planned
deterministic motion-aware comparator without probabilistic safety claims, while
designing richer fitting/error models and new calibration-validation cases.

[Recorded experiment and figure](../experiments/2026-10-14-obstacle-calibration-v1/REPORT.md).
