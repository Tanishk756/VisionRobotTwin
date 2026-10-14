# Research proposal: delayed-observation whole-arm command filtering

Status: candidate study, 2026-10-14; G0 open. See [gap audit](RELATED_WORK.md).

## Problem and scope

Control a simulated articulated arm around a moving obstacle using delayed,
noisy pose observations and joint telemetry. A separate nominal twin supports
prediction. The simulated plant can have different mass and actuation response.
The first task is joint-velocity reaching; vision-level image experiments and
physical validation follow later. Pose-level corruption is not image evidence.

At controller time t, observation i has capture time c_i and delivery time d_i.
Its age is t-c_i, not t-d_i. Robot state is (q, qdot); commands are joint
velocities with creation time, sequence ID, and an application event. The
controller knows its own submitted commands, but not hidden plant state or
future obstacle motion. The evaluator alone reads plant truth.

Consider an obstacle belief propagated to future application time t+a and then
along a horizon s. A candidate geometric constraint is

    distance(robot_link(q_pred(t+a+s)), obstacle_mean(t+a+s))
      >= clearance + obstacle_error(t+a+s) + robot_error(t+a+s).

This is a design sketch, not a derived safety condition. Distance linearization,
interpolation, correlations, braking, command history, initial feasibility,
and validity of each error bound require analysis. Covariance is not itself a
hard error bound or collision-probability guarantee. If assumptions fail, report
empirical behavior and invalidate any conditional guarantee.

## Candidate contribution and alternatives

Investigate uncertainty propagation that explicitly accounts for observation
age and commands in flight, coupled with independently calibrated twin tracking
residuals for all arm links. Determine whether this improves the Pareto tradeoff
relative to strong delay-aware and uncertainty-aware comparators.

Potential contributions are a differentiated filter, an independently evaluated
benchmark, and explanatory evidence. Existing uncertainty margins, delay
prediction, and digital-twin supervision are acknowledged prior art. If the
method adds no technical distinction, revise the question or assess a benchmark
paper on its own merits; do not relabel integration as novelty.

## Falsifiable hypotheses

| ID | Hypothesis | Primary test / falsifier |
|---|---|---|
| H1 | More useful progress at comparable empirical safety | Paired held-out Pareto curves; no advantage over tuned comparators falsifies |
| H2 | Observation-age handling helps under delay/dropout | Same filter with age compensation removed; equivalent or worse results falsify |
| H3 | Residual adaptation helps under unseen plant mismatch | Same filter with fixed residual model; no benefit falsifies |
| H4 | Computation fits the selected controller deadline | End-to-end P99 latency and deadline misses; budget violations falsify |

No numerical success target is selected from test results. Pilot data determine
sample sizes and practical effect sizes before the final protocol is frozen.

## Initial evaluation

Use three robot families: Panda, KUKA iiwa, and xArm6. Development, estimator
calibration, and final evaluation scenarios are disjoint. Compare fixed-horizon
stopping, tuned fixed margins, deterministic prediction, a compatible published
method, and the candidate method. Hold nominal control and sensing streams fixed.

Measure contact episodes, clearance violations, completion/time, intervention
duration, tracking error, uncertainty coverage, latency, and failures. Include
incomplete trials in denominators and report unavoidable/initially unsafe cases.
Pair seeds across methods and compute scenario-aware confidence intervals.
Analyze ablations of age handling, residual modeling, braking, and recovery.

## Assumptions and current limits

Initially use a shared simulated clock, world-frame observations, sphere
obstacles, fixed-rate physics, and velocity commands. Event delivery occurs at
the first physics boundary at/after its scheduled time; record both times.
Constant-velocity obstacle estimation is a baseline assumption, not ground truth.
Dropout must leave missing information rather than reveal truth. Plant mass and
motor gains are configurable; no friction/calibration/braking identification is
claimed yet. Python object encapsulation is an API boundary, not a security wall.

For the research plant, do not run the interactive truth-based collision guard:
that would give it oracle state and mask method failures. Evaluate contacts
independently. Run only in DIRECT simulation; this harness is not a hardware API.

## Next deliverables

Implement typed observations/commands, deterministic transport, independent
plant/twin clients, observation-only twin updates, moving-obstacle recording,
and isolation/replay tests. These are infrastructure, not a novel safety method.
Then add the estimator, calibrated uncertainty, initial engineering comparator,
and literature-supported baseline before developing the proposed filter.
