# VisionRobotTwin research roadmap

Status: proposed research plan, 2026-10-14. No novelty or publication claim is
established by this document. Tasks below are pending unless explicitly marked
as existing infrastructure. Venue acceptance depends on contribution, evidence,
writing, and peer review; completing a feature checklist cannot guarantee it.

Implementation update: [proposal](PROPOSAL.md), [gap audit](RELATED_WORK.md),
[claim register](CLAIMS.md), and the [first plant/twin testbed](TESTBED.md)
are available. G0 remains open; the testbed is an initial infrastructure slice,
not completion of all WP1/WP2 tasks or a novel predictive safety method.
The testbed now also includes command expiry, latched telemetry/command
watchdogs, queue cancellation on reset, and a nominal command-history/FK
prediction API. Dynamic rollout validation and robust prediction remain pending.
Versioned development scenarios, observation-only whole-arm candidate queries,
and the first fixed-horizon comparator are implemented; see
[the baseline guide](BASELINE_SCENARIOS.md). Published-method reproduction,
uncertainty calibration and held-out evaluation remain pending.
The first [uncertainty calibration diagnostic](UNCERTAINTY_CALIBRATION.md) now
measures fitting/validation coverage and exposes undercoverage under motion/delay
shift. Reliable calibrated uncertainty is not established and remains an open task.

## 1. Research objective and intended contribution

Study vision-guided arm motion when observations are delayed, noisy, missing,
and imperfectly aligned with the robot model. The primary question is:

> Can explicitly accounting for observation age and plant/twin mismatch improve
> task progress at a comparable measured collision risk, relative to fixed
> margins and deterministic predictive safety filters?

Candidate method: a predictive command filter that propagates obstacle state
uncertainty from the observation timestamp to the current time, includes robot
tracking/model error and stopping behavior, and chooses a minimally modified
feasible command. This is a candidate contribution to investigate, not an
established research gap. Delayed-state safety, uncertainty-aware CBFs, and
predictive safety filtering must be checked before choosing a formulation.

Potential paper contributions, conditional on results:

1. A precisely specified observation-age and mismatch-aware safety filter,
   with a new technical mechanism distinguished from the closest literature.
2. An independently evaluated manipulation benchmark with controlled sensing,
   transport, dynamics, and calibration mismatch.
3. Evidence explaining the safety/progress tradeoff across robot families,
   uncertainty regimes, and failure cases, with reproducible artifacts.

The platform and benchmark support the method; avoid presenting ordinary
software integration as algorithmic novelty. If the method is not novel, assess
whether a benchmark contribution is independently strong enough, or revise the
research question before expanding implementation.

## 2. Scope and existing foundation

Existing infrastructure: Panda, KUKA iiwa, xArm6, xArm6 with gripper; canonical
kinematics; IK and resolved-rate control; collision providers; runtime sampled
collision guard; transport fault injection; seeded trials and result manifests.
See [current validation](../status/2026-10-14-xarm6-and-execution-monitor.md).

Primary task: vision-guided reaching/tracking around moving obstacles. Begin
with velocity commands and geometric obstacles. Add position-controller
compatibility after the main filter works. Keep grasp/contact modeling outside
the first method paper. xArm6 variants are one family; the three families are
Panda, KUKA iiwa, and xArm6. Use the bare xArm for the primary comparison.

Hardware is unavailable. Complete simulation methods and evaluation now.
Physical sensing, stopping, grasping, payload, and deployment safety remain
unvalidated. Avoid claiming a physical digital twin has been validated when
the physical-side system is itself simulated.

## 3. Research decision gates

| Gate | Evidence required | Decision |
|---|---|---|
| G0: research gap | Literature matrix and explicit difference from closest methods | Proceed, narrow, or change the method |
| G1: trustworthy testbed | Independent plant/twin, no truth leakage, deterministic replay, checked evaluator | Start comparative algorithm experiments |
| G2: method feasibility | Stable command filtering, feasible fallback, measured timing, explicit assumptions | Freeze the initial formulation or redesign |
| G3: pilot benefit | Paired pilot results against strong tuned baselines, failures investigated | Freeze hypotheses and evaluation protocol |
| G4: main evidence | Held-out results, uncertainty estimates, ablations, source provenance | Decide which claims are supported |
| G5: submission readiness | Claims mapped to evidence, related-work audit, reproducible release, external review | Select venue and prepare submission |

Failure at a gate is useful evidence. Do not continue toward a preferred claim
by changing metrics, excluding failures, or weakening baselines after seeing
test results.

## 4. Work packages and detailed tasks

### WP0 — Literature, problem definition, and hypotheses

- [ ] Create a literature matrix: problem, uncertainty type, observation delay,
  model mismatch, plant dynamics, intervention mechanism, assumptions,
  theoretical guarantees, baselines, code availability, and experiments.
- [ ] Cover predictive safety filters, delayed/sample-data CBFs, robust and
  chance-constrained control, uncertainty calibration, and digital-twin
  verification. Read full methods rather than relying on abstracts.
- [ ] Inspect the three to five closest methods deeply and record precisely
  what our proposed method adds. Search beyond the starter papers below.
- [ ] Write a two-page proposal: problem, candidate technical contribution,
  mathematical variables, hypotheses, baselines, evaluation, and limitations.
- [ ] Define assumptions: sensor clock synchronization, bounded/unbounded
  uncertainty, obstacle acceleration, actuation delay, stopping capability,
  estimator validity, and initial-state feasibility.
- [ ] Define the claims that would remain meaningful if hardware stays absent.

Suggested hypotheses to refine before main experiments:

- H1: at matched empirical safety levels, the method improves completion or
  completion time compared with fixed-margin and deterministic prediction.
- H2: compensating observation age improves performance under held-out delay
  and dropout patterns compared with the same filter without compensation.
- H3: modeling twin mismatch improves robustness under unseen dynamics or
  calibration shifts compared with an otherwise identical matched-model filter.
- H4: filtering fits a declared control budget on specified compute hardware.

Deliverables: `RELATED_WORK.md`, `PROPOSAL.md`, initial `CLAIMS.md`.
Acceptance: G0 review identifies a defensible technical distinction and an
experiment capable of disproving each hypothesis. A new acronym is insufficient.

### WP1 — Independent plant and twin

- [ ] Refactor the simulator lifecycle so two independent PyBullet clients can
  run simultaneously, with explicit client IDs on every physics operation.
- [ ] Give the plant its own motor dynamics, gravity, masses/inertia, friction,
  payload settings, geometry, and state. Give the twin separate nominal values.
- [ ] Implement interfaces for commands, observations, time, and evaluation.
  Controllers receive observations; only the evaluator accesses plant truth.
- [ ] Implement an event queue for commands and observations with independent
  sensing, computation, transport, and actuation timing.
- [ ] Use a deterministic simulation clock for experiments and wall-clock
  timing separately for computational performance measurements.
- [ ] Add configurable fixed delay, jitter, packet loss, stale frames,
  reordering, actuator lag, saturation, and calibration transform errors.
- [ ] Make obstacle motion and randomization reproducible from scenario seeds.
- [ ] Audit collision queries and initialization to prevent plant-state,
  obstacle-future, or evaluator-truth leakage into the controller.
- [ ] Record plant and twin configurations independently in the manifest.

Acceptance: identical seeds replay the same simulated events within declared
tolerances; perturbing plant truth without delivering an observation cannot
immediately change the controller's estimate. Twin queries must not mutate
plant state. Multiple-client isolation tests pass. G1 requires these checks.

### WP2 — Perception, timestamps, and uncertainty

- [ ] Define an observation schema containing capture time, delivery time,
  frame ID, coordinate frame, pose, validity, and uncertainty representation.
- [ ] Implement a pose/velocity estimator with prediction to the current time.
  Start with a tractable constant-velocity filter and document its limits.
- [ ] Reject or explicitly handle out-of-order frames and invalid timestamps.
- [ ] Model dropout by missing observations, not by silently substituting truth.
- [ ] Propagate pose, calibration, and motion uncertainty consistently between
  coordinate frames; distinguish covariance from conservative error bounds.
- [ ] Implement structured pose-level corruption for fast algorithm experiments.
- [ ] Add a separate rendered-image path through the actual detector/pose
  estimator, including marker occlusion and image degradation. Clearly label
  pose-level and image-level evidence in reports.
- [ ] Evaluate estimator error, interval coverage, covariance calibration where
  applicable, and behavior under unseen noise and obstacle accelerations.
- [ ] Define an out-of-distribution/invalid-estimate fallback and log its use.

Acceptance: age grows correctly during dropout; uncertainty changes according
to the model; intervals/ellipsoids have measured coverage on held-out data.
An uncalibrated covariance must not be presented as a collision-risk guarantee.

### WP3 — Scenario suite and independent evaluator

- [ ] Create versioned scenario files: robot, task, obstacle geometry/motion,
  observation schedule, uncertainty, plant/twin mismatch, duration, and seed.
- [ ] Include static clutter, crossing obstacles, head-on approaches, narrow
  passages, temporary occlusion, delayed obstacle updates, and combined faults.
- [ ] Include benign cases that should not trigger unnecessary interventions.
- [ ] Include adverse cases: sudden motion, insufficient stopping distance,
  initially unsafe states, unreachable goals, and optimizer infeasibility.
  Report feasibility categories rather than hiding impossible cases.
- [ ] Run the evaluator against plant truth independently of the filter's
  distance model, using finer temporal checks or swept checks where available.
- [ ] Quantify evaluator discretization sensitivity; fine sampling still does
  not imply continuous collision freedom.
- [ ] Define contact events, clearance violations, and sustained contacts so
  consecutive physics samples are not miscounted as independent incidents.
- [ ] Record raw observations, delivered commands, estimated states, plant
  states, interventions, constraint residuals, and terminal outcomes.
- [ ] Implement replay and plots generated from recorded data.

Acceptance: known collision and near-miss cases are correctly classified;
event counting and replay are tested. Evaluation continues after a controller
failure so terminal outcomes and stopping behavior remain measurable.

### WP4 — Baseline implementations and fairness

- [ ] B0: existing sampled fixed-horizon collision guard.
- [ ] B1: fixed-clearance controller, including a sweep of margins rather than
  one deliberately poor setting.
- [ ] B2: deterministic predictive filter with no uncertainty propagation.
- [ ] B3: suitable published CBF/predictive filter, implemented from available
  code or carefully reproduced equations, with deviations disclosed.
- [ ] Optional uncontrolled nominal controller only in simulation, to measure
  scenario difficulty; it is not a sufficient primary comparator.
- [ ] Use the same nominal controller, sensing stream, tasks, robot limits,
  compute resources, and available information where comparisons permit.
- [ ] Tune all methods using development scenarios and comparable budgets.
- [ ] Document when a method requires different assumptions or extra inputs.
- [ ] Sweep risk/margin settings and compare Pareto curves; compare progress
  at matched safety rather than rewarding faster unsafe motion.
- [ ] Validate baseline behavior on simple analytic or author-provided cases.

Acceptance: baselines solve ordinary cases; tuning is frozen before held-out
evaluation. An external reader can reproduce settings and understand deviations.

### WP5 — Candidate predictive command filter

- [ ] Choose a formalism after WP0: robust predictive filtering, sampled-data
  CBF constraints, or another justified approach. Do not implement every option.
- [ ] Propagate observations from capture time through delivery/command delay
  to the future times at which commands affect the plant.
- [ ] Define how obstacle uncertainty, robot tracking error, and calibration
  uncertainty enter separation constraints without counting the same error twice.
- [ ] Include measured/identified acceleration and braking behavior rather
  than assuming a zero-velocity command stops motion instantly.
- [ ] Optimize deviation from nominal motion subject to joint position,
  velocity, acceleration, and collision constraints appropriate to the model.
- [ ] Check all relevant robot links, self-collision, and swept motion; avoid
  restricting the method to end-effector clearance while claiming arm safety.
- [ ] Implement mismatch estimation or bounded residual adaptation with
  independent calibration data and conservative behavior when bounds fail.
- [ ] Handle solver deadlines, numerical failures, infeasibility, stale state,
  and invalid uncertainty with a defined fallback and an explicit fault reason.
- [ ] Specify soft versus hard constraints. If safety constraints can be
  relaxed, log every relaxation and count its consequences in evaluation.
- [ ] Establish which properties can be proven and under which assumptions.
  If no guarantee is proved, describe results as empirical collision avoidance.
- [ ] Test whether fallback states are actually recoverable under the assumed
  plant dynamics; a stop request alone does not establish recursive feasibility.
- [ ] Profile end-to-end latency and constraint-check overhead.

Acceptance: G2 requires numerical validation, explicit assumptions, correct
fallback behavior, and timing measurements. Analytical claims require a
separate derivation and review, including discretization and delay assumptions.

### WP6 — Recovery and task integration

- [ ] Define explicit TRACK, SLOW, HOLD, REPLAN, RESUME, and FAULT transitions.
- [ ] Separate transient uncertainty holds from latched execution faults.
  Never silently clear an active collision or failed-stop fault.
- [ ] Use hysteresis/dwell criteria to prevent switching chatter.
- [ ] Replan around persistent obstacles with a computation deadline and
  collision-check the transition back to tracking.
- [ ] Log why motion resumed and what observations justified recovery.
- [ ] Evaluate filter-only first; add recovery to every compatible baseline
  or run a separate comparison so gains are not misattributed to the filter.

Acceptance: temporary occlusion can recover when justified; persistent unsafe
conditions remain stopped; repeated pause/resume does not produce command jumps.

### WP7 — Pilot, frozen protocol, and main experiments

- [ ] Run pilots to measure variability, scenario feasibility, runtime cost,
  and failure modes; use these data for sample-size and compute-budget planning.
- [ ] Freeze method settings, tuning budget, primary endpoints, exclusion
  rules, scenario split, and analysis before main evaluation.
- [ ] Use disjoint development, uncertainty-calibration, and test scenarios.
  Keep test seeds and obstacle patterns out of method tuning.
- [ ] Evaluate Panda, KUKA iiwa, and bare xArm6. Gripper xArm is supplementary.
- [ ] Start with one-factor delay/noise/mismatch sweeps, then selected combined
  stress tests. Avoid an unaffordable full Cartesian product.
- [ ] Use paired scenario seeds across methods. Seeds must cover independent
  motion/noise realizations, not merely repeat identical deterministic runs.
- [ ] Report confidence intervals, effect sizes, and per-family breakdowns.
  Account for clustering by scenario rather than treating physics frames as
  independent trials. Report failed/timed-out runs in denominators.
- [ ] Analyze collision rate, clearance violations, completion rate/time,
  tracking error, intervention time, command distortion, and recovery success.
- [ ] For completion time, handle incomplete trials explicitly; also report
  success-conditioned time separately to expose survivorship effects.
- [ ] Report median/P95/P99 computation time and deadline misses on specified
  hardware, distinguishing simulated time from wall time.
- [ ] Report zero observed collisions with an uncertainty bound, not as proof
  of zero collision probability. State the experimental independence assumptions.
- [ ] Perform timestep/evaluator-resolution sensitivity checks and retain
  failure videos and raw logs.

Suggested pilot budget: 10–20 independent seeds per selected scenario cell,
explicitly provisional. Use observed variability and intended effect size to
choose the main budget. No arbitrary trial count establishes publishability.

Acceptance: G3 freezes the protocol; G4 requires an auditable result set, paired
analysis, visible failures, and claims that survive the primary comparisons.

### WP8 — Ablations and generalization

- [ ] Remove observation-age compensation, keeping other components identical.
- [ ] Remove mismatch adaptation while retaining the same sensing uncertainty.
- [ ] Replace adaptive uncertainty with tuned constant margins.
- [ ] Remove stopping-aware prediction, with the limitation stated explicitly.
- [ ] Separate filter-only gains from replanning/recovery gains.
- [ ] Test held-out obstacle speeds/accelerations, delay distributions,
  calibration errors, and dynamics shifts.
- [ ] Evaluate sensitivity to prediction horizon, sampling rate, confidence
  settings, and bounded-error violations.
- [ ] If practical, validate a small subset in a second simulator to expose
  engine-specific effects. Do not make this a prerequisite for initial pilots.

Acceptance: identify which components cause improvements and where they fail.
Do not claim cross-robot generalization from aggregate results alone.

### WP9 — Hardware validation when access becomes available

- [ ] Select one supported physical arm and document its command/stop interface.
- [ ] Calibrate camera intrinsics/extrinsics and measure timestamp/transport
  behavior, tracking error, and stopping response under the intended operating
  conditions. Obtain appropriate local lab authorization and supervision.
- [ ] Reuse representative reaching scenarios with safe obstacle arrangements;
  test controlled visual delay/dropout and model mismatch within the lab setup.
- [ ] Use independent measurements for clearance/pose where practical.
- [ ] Compare the main method with the strongest compatible baseline using
  repeated trials and disclose any retuning between simulation and hardware.
- [ ] State remaining limitations, including generalization beyond that arm,
  laboratory conditions, and certified safety.

Acceptance: physical claims are supported by physical evidence. If unavailable,
remove those claims and select a simulation-compatible paper framing.

### WP10 — Paper, artifacts, and external review

- [ ] Maintain `CLAIMS.md`: every claim maps to an equation, experiment,
  figure/table, assumptions, and limitation.
- [ ] Freeze a clean source revision and environment before final experiments.
  Record hashes, seeds, configurations, dependencies, and baseline versions.
- [ ] Package one-command reproduction of a small smoke suite and instructions
  for the full evaluation; disclose hardware, runtime, and storage requirements.
- [ ] Produce figures: architecture/information boundaries, uncertainty
  calibration, safety/progress curves, delay/mismatch sweeps, ablations,
  timing distribution, and representative failures.
- [ ] Write around the problem and contribution: introduction, related work,
  problem formulation, method, experiments, limitations, and conclusion.
- [ ] Prepare a short video showing representative behavior and failure cases.
- [ ] Obtain robotics/control feedback on the method and statistics feedback
  on the evaluation before final submission.
- [ ] Verify citations, licenses, anonymity requirements, current venue scope,
  deadlines, and author policies when choosing the actual submission.

Acceptance: G5 requires a clear contribution relative to closest work, strong
comparators, defensible analysis, reproducible results, and accurate claims.

## 5. Milestones, dependencies, and effort

Estimates are planning ranges in focused person-weeks, not calendar promises.
They assume one developer/researcher and exclude hardware procurement and review.

| Milestone | Work | Dependencies | Indicative effort | Reviewable output |
|---|---|---|---|---|
| M0 | WP0 | Existing platform | 1–2 weeks | Proposal and gap decision |
| M1 | WP1 | M0 scope | 2–3 weeks | Independent plant/twin and timing tests |
| M2 | WP2–WP4 initial versions | M1 interfaces | 3–5 weeks | Calibrated estimator, scenarios, verified baselines |
| M3 | WP5 | M0, M2 | 3–5 weeks | Filter formulation, implementation, pilot |
| M4 | WP6–WP8 | M3 passes G2/G3 | 3–5 weeks | Frozen evaluation and main results |
| M5 | WP10 | M4 supports claims | 2–3 weeks | Submission draft and reproducible release |
| Optional physical study | WP9 | Hardware access, mature method | 2–4+ weeks | Physical validation evidence |

Sequential simulation effort is approximately 14–23 focused person-weeks;
literature findings, baseline reproduction, or a failed hypothesis can extend
or redirect it. Begin figure/logging infrastructure early. Do not start main
experiments before the method, baseline tuning, and protocol are frozen.

## 6. First implementation backlog

Complete M0's proposal/gap review before committing to a particular safety
algorithm. The following testbed tasks remain useful across plausible methods:

1. Define `Observation`, `Command`, `ScenarioConfig`, and `TrialRecord` schemas.
2. Add plant/twin client ownership and prohibit controller access to truth.
3. Add deterministic command/observation queues and clock tests.
4. Add a moving-sphere crossing scenario with exact ground-truth recording.
5. Add delayed/noisy pose observations and an age-aware pose/velocity estimator.
6. Implement independent collision event evaluation and replay.
7. Run the current collision guard as B0 and expose its failure/success cases.
8. Review the initial dataset and select the closest published baseline.

Suggested new modules: `robotics/research/plant_twin.py`, `transport.py`,
`observations.py`, `estimation.py`, `scenarios.py`, `evaluation.py`, and later
`predictive_filter.py`. Keep research interfaces separate from the interactive
application until the behavior is stable, then integrate through backend APIs.
Module names are proposals, not files already implemented.

## 7. Risks and scope controls

| Risk | Response |
|---|---|
| Closest paper already solves the proposed problem | Reframe at G0; do not assert novelty from integration |
| Always-stop policy appears safest | Evaluate progress, matched-safety comparisons, and unnecessary holds |
| Simulation truth leaks into estimates | Separate interfaces and test information boundaries |
| Gaussian assumptions fail under occlusion | Check calibration and stress non-Gaussian/OOD cases |
| Stop command does not stop the plant in time | Identify stopping dynamics and report unavoidable cases |
| Strong baseline removes the apparent benefit | Report honestly; refine the mechanism or research question |
| Computation exceeds control deadlines | Include deadline misses; simplify or use justified approximations |
| Too many optional features delay evidence | Defer extra robots, RL/VLAs, dashboards, and grasping |
| No hardware access | Make simulation-only claims and discuss transfer limitations |

## 8. Starter literature and venue framing

These are starting points from the preliminary search, not an exhaustive review
or verified evidence that the proposed method is novel:

- [Digital Twin Enabled Runtime Verification for Autonomous Mobile Robots
  under Uncertainty](https://arxiv.org/abs/2412.09913).
- [Uncertainty Aware-Predictive Control Barrier Functions: Safer human–robot
  interaction through probabilistic motion forecasting](https://doi.org/10.1016/j.robot.2025.105291).
- [Online Control Barrier Function Construction for Safety-Critical Motion
  Control of Manipulators](https://ieeexplore.ieee.org/document/10510639/).
- [Multi-Robot Collision Avoidance under Uncertainty with Probabilistic Safety
  Barrier Certificates](https://arxiv.org/abs/1912.09957).
- [RA-L scope and journal information](https://www.ieee-ras.org/publications/ieee-robotics-and-automation-letters/).

For ICRA/IROS, aim for one focused method and compelling evidence. For a longer
Robotics and Autonomous Systems journal treatment, consider a deeper method,
broader validation, and analysis where the results justify that scope. RA-L is
another possible concise-method venue. These are strategic suggestions, not
acceptance forecasts or current submission instructions. Recheck the selected
venue's official requirements at submission time.
