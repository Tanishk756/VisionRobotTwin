# Deterministic obstacle-motion comparator

This engineering comparator extends the fixed-horizon stop baseline with an
estimated obstacle trajectory. It is not a new research contribution, a published
method reproduction, or a probabilistic safety controller.

## Reproduce

```powershell
python tools/compare_research_baselines.py --modes none fixed-horizon motion-aware --output-dir experiments/motion-aware-comparison
```

Defaults run 36 paired development cases: four scenarios, three robot families,
seed 0 and three methods. The output directory must be empty. Existing commands
without `--modes` retain the original two-method comparison.

Single case:

```powershell
python tools/run_plant_twin_experiment.py --scenario config/research_scenarios/crossing-v1.json --robot panda --baseline motion-aware --output-dir experiments/motion-example
```

The comparison CLI now also exposes common `--horizon` and `--margin` settings.
Use separate output directories for parameter sweeps. These defaults have not
been tuned on independent development data or selected at matched empirical risk.

## Method and information boundary

At each control decision, a constant-velocity estimator updates from the latest
delivered observation and predicts obstacle position/velocity means at the
current controller time. For a sampled arm state at future time tau, the obstacle
position is mean(now) + (tau-now) * estimated_velocity. No scenario trajectory,
future observation, sampled transport delay or plant truth enters this query.

The predictor therefore compensates observation age and extrapolates future
obstacle motion. Its current mean is filtered rather than the raw observed
position, so differences from the frozen baseline include filtering, age
compensation and motion forecasting. This is not an ablation isolating just one
of those effects. Such ablations remain future research work.

Only numerical position and velocity means are used for collision geometry.
Covariance is deliberately ignored following the
[failed nominal-coverage diagnostic](UNCERTAINTY_CALIBRATION.md). Gaussian
confidence regions are not converted into collision-risk or safety constraints.

The candidate query includes commands in flight and a hypothetical new command.
It checks whole-arm/self/map geometry while temporarily moving only the twin's
tracked sphere. Robot joints/velocities and sphere pose/velocity are restored,
including on query errors. Estimate reference time and capture age must match
the current delivered observation. Static fixtures remain the shared known map.

## Policy and comparison fairness

The motion-aware and frozen baselines share the same horizon, sample interval,
clearance margin, transported zero-velocity stop, HOLD dwell, recovery observation
count, extra recovery clearance, and latched execution faults. Both use the same
nominal proportional controller, velocity cap, decision cadence and exogenous
scenario/sensing seeds. Command timing remains nominal in prediction, and stops
experience ordinary command transport delay except for the shared local watchdog.

The policy's estimator consumes observations visible at decision times. A separate
logging estimator may see intervening deliveries in the simulation loop.
`decisions.json` therefore records the **actual policy estimate** used by the
motion-aware query; use that field when auditing its predictions. Estimator/query
failures latch policy FAULT rather than silently resuming. An explicit policy
reset also resets estimator history, without resetting the execution supervisor.

## Limits and next experiments

Constant-velocity extrapolation can fail at abrupt turns, occlusion and model
mismatch. Nominal arm velocity tracking omits identified dynamics and stopping
distance. Temporal/configuration sampling is finite. A predicted collision can
remain unavoidable under a stop-only policy, especially when an obstacle moves
into a stationary arm. No formal collision freedom is claimed.

One-seed results are development diagnostics; compare task progress alongside
contact episodes and retain all watchdog faults and incomplete tasks. Decision
latency diagnostics do not establish end-to-end real-time performance.

Next: paired horizon/margin sweeps on development data, more independent seeds,
motion/age/filtering ablations, stopping-model identification and a compatible
published comparator. A defensible novel method still requires the open G0 review.

[Recorded development results](../experiments/2026-10-14-motion-aware-baseline/REPORT.md).
