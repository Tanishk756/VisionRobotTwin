# Paired development parameter sweep

This stage checks whether the observed baseline limitations persist across
horizon/margin settings and sensing seeds. It is development evaluation, not
selection of a best test-set controller or evidence of a novel method.

## Fixed protocol

Before running the release sweep, fix:

- Three robot families: Panda, KUKA iiwa and bare xArm6.
- Four existing development scenarios: crossing, sudden approach, occluded
  crossing and narrow passage. No new favorable scenes replace failures.
- Three sensing/transport seeds: 0, 1 and 2, paired across every cell.
- Prediction horizons: 100 and 200 ms.
- Environment margins: 10 and 30 mm, with the same 5 mm recovery headroom.
- Both frozen-obstacle and deterministic motion-aware stop policies.
- Nominal control once per robot/scenario/seed pair, rather than once per
  parameter cell. Nominal control does not use horizon or margin.

This produces 36 nominal runs plus eight parameter/method cells of 36 runs:
**324 trials**. Each original scene lasts two simulated seconds. The sweep
does not modify sensing, controller gains, target, recovery or watchdog settings.

## Reproduce

```powershell
python tools/sweep_research_baselines.py --workers 3 --plot --output-dir experiments/baseline-sweep
```

Use an empty output directory. `--workers 1` runs sequentially. Each concurrent
worker owns independent physics clients, RNG streams and trial directories.
Fresh worker processes inherit BLAS/OpenMP thread limits of one to avoid nested
thread oversubscription. Wall-clock timing under shared load is diagnostic;
measure end-to-end real-time performance separately on controlled hardware.

The CLI accepts explicit scenario paths, robot IDs, seeds, horizons and margins.
It rejects duplicate dimensions, invalid ranges, unknown robots, and any scene
not marked `development`. Calibration and final test scenes cannot be used in
this tuning runner. This checks declared split labels; it does not detect a
manually relabelled duplicate of a held-out trajectory.

## Outputs and failure handling

- A manifest is written before execution, with resolved scenes, dimensions,
  environment, source hashes, workers and expected trial count.
- `progress.jsonl` is flushed after every result, including errors.
- `results.json` retains every successful summary and every error. Raw traces,
  decisions and per-trial source manifests remain in individual directories.
- `analysis.json` contains cell summaries, per-robot/scenario breakdowns,
  paired differences, descriptive intervals and an exploratory Pareto set.
- `REPORT.md` contains a readable result table.
- With `--plot`, SVG and PNG figures show contact and safe completion versus
  clearance margin. These are development diagnostics, not publication claims.

Errors are not silently converted into collision-free trials or removed from
successful-completion denominators. Their collision outcomes remain unknown.
Observed contact fraction reports its completed-trial denominator. A cell must
contain every declared pair with no errors before entering paired or Pareto
analysis. Incomplete nominal reference data suppress paired comparisons.

## Metrics and statistical interpretation

Measure the fraction of trials with contact, contact episodes, final target
completion, collision-free completion, intervention fraction, final joint error,
watchdog faults and initially unsafe states. Retain the existing target criterion
of final maximum joint error <= 0.02 rad; this is not a Cartesian grasping metric.

Paired effects subtract the nominal outcome for the same robot/scenario/seed.
Average across the fixed twelve robot/scenario combinations within each seed,
then bootstrap seeds together across those combinations. Do not bootstrap
individual physics frames or treat every parameter run as an independent sample.
The 1,000 seed-cluster resamples use RNG seed 419. With only three independent
seed clusters, intervals are weak descriptive diagnostics, not hypothesis tests.
They quantify sensing-seed variation conditional on the fixed scenes, not broad
generalization across unseen tasks, obstacle trajectories or robot populations.

Exploratory Pareto cells minimize observed contact fraction and intervention
while maximizing failure-adjusted collision-free completion. Equal cells remain
ties. Pareto membership is computed on observed development data and is neither
an uncertainty-aware ranking nor a guarantee of transfer. No setting is promoted
to a final method solely because it looks best on this sweep.

## Failure analysis

Align contact onsets with the latest controller decision, counting policy states
WAIT, TRACK, HOLD or FAULT at each onset. Consecutive contacting physics samples
form one episode. A contact beginning during policy HOLD/FAULT does **not** prove
that the physical plant was stationary: old commands may remain in flight,
software stops are delayed, and motor response is not instantaneous. Raw traces
and stopping-system identification are needed before causal or unavoidability
claims. These onset labels guide follow-up investigation rather than certify it.

Run an independent evaluator-only endpoint check:

```powershell
python tools/assess_sweep_feasibility.py --sweep-dir experiments/baseline-sweep
```

This writes `endpoint-feasibility.json` once, preserving the original audit. It
checks the exact target configuration against joint limits, self/table/fixture
geometry, and the moving obstacle's final position. It does not prove existence
of a collision-free dynamic path or infeasibility of the entire 0.02 rad target
tolerance region. The audit is post hoc and cannot be fed into the controller.
Original v1 cases and denominators must remain intact; any endpoint-free subset
is an explicitly labelled diagnostic, not a retroactively improved primary test.

The first release found static fixture contact at the KUKA and xArm narrow-passage
exact targets. This is a benchmark-design flaw to address in a versioned successor,
with goal/tolerance checks and trajectory feasibility investigation before tuning.
Do not infer that refusal to approach those exact targets demonstrates an
algorithmic failure or that avoiding their contacts demonstrates better dynamic
obstacle avoidance.

## Remaining research work

The estimator's uncertainty is still uncalibrated under the measured shift.
Neither stop policy replans or steers around an approaching obstacle. Prediction
uses nominal velocities/transport and finite samples; it does not model all
future controller refreshes, validated braking or uncertainty tubes. The 200 ms
horizon can extend beyond a command's 150 ms lifetime, so prediction includes
expiry rather than silently assuming continued command validity.

Use this evidence to decide whether tuning changes the tradeoff and which failure
mechanism needs a stronger algorithm. Next requirements include a compatible
published comparator, stopping-model identification, representative uncertainty
calibration, controlled ablations and genuinely held-out evaluation. G0 novelty
review remains open.
