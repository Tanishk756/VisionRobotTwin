# Next steps after the 2026-10-14 runtime fixes

Planning baseline: commit `8db9ea6`. This document prioritizes the remaining
work in [the paper roadmap](PAPER_ROADMAP.md); it does not replace that roadmap
or declare its open research gates complete.

## 1. Where we are stopping today

- Runtime fixes are committed and pushed. Local regression: **579 passed,
  7 optional checks skipped**. Windows Python 3.11/3.12 and ROS2 Humble CI passed
  with zero annotations for the baseline commit.
- Four simulation profiles are available: Panda, KUKA iiwa, xArm6 and xArm6
  with gripper. All eight obstacle/controller launch combinations passed.
- Two consecutive Panda offline pick/place cycles and longer resolved-rate
  tracking checks passed. The offline demo deliberately resets its pick object.
- Independent simulated plant/twin clients, timestamped observation delivery,
  command expiry/watchdogs, nominal prediction, versioned scenes, two stop
  comparators and a paired sweep runner exist.
- The development sweep completed **324 trials with no execution errors**.
  This is experiment infrastructure evidence, not an algorithmic success.
- The exact KUKA and xArm narrow-passage targets intersect static fixtures.
  In the endpoint-free post-hoc subgroup, every method still has **27 contact
  trials out of 30**. No dynamic-avoidance benefit has been demonstrated.
- Nominal 95% estimator regions achieved roughly **80.6–87.6% coverage** in
  the shifted validation diagnostic. Calibrated uncertainty is not established.
- Novelty review remains open. Physical hardware is unavailable.

Evidence: [runtime fixes](../status/2026-10-14-runtime-launch-fixes.md),
[baseline sweep](../experiments/2026-10-14-baseline-sweep-v1/REPORT.md),
[uncertainty diagnostic](UNCERTAINTY_CALIBRATION.md),
[claim register](CLAIMS.md).

## 2. Priority and dependency order

| Order | Work package | Main output | Required before |
|---|---|---|---|
| P0 | Scenario successor and preflight | Audited v2 development scenes | New comparative claims |
| P1 | Stop and command-pipeline identification | Measured reaction/braking envelope and contact replays | Choosing prediction horizons or fallback behavior |
| P2 | Research gap and published baseline specification | Explicit novelty decision and reproducible comparator spec | Committing to a candidate algorithm |
| P3 | Revised estimator and independent calibration | Measured forecast reliability and invalid-estimate policy | Uncertainty-dependent filtering claims |
| P4 | Candidate command filter and baseline reproduction | Implemented method, reference comparator and ablations | Pilot benefit gate |
| P5 | Fair pilot and frozen held-out evaluation | Reproducible comparison with uncertainty intervals | Paper-level empirical claims |
| P6 | Image-based validation and paper artifacts | End-to-end sensing evidence and reproducible manuscript | Submission decision |
| Deferred | Physical validation | Hardware sensing/dynamics/stop evidence | Physical transfer or deployment claims |

P2 can progress while P0/P1 are being investigated. Do not make P0 fixes depend
on favorable results from the proposed algorithm. This is a dependency plan,
not a requirement to work on all packages simultaneously.

## 3. P0 — Next implementation task: trustworthy scenario successor

**Purpose:** distinguish controller failures from flawed or deliberately
infeasible tasks before spending more time tuning.

1. Preserve all v1 scenes, results and denominators unchanged. Create explicitly
   versioned v2 files under `config/research_scenarios/`.
2. Extend evaluator-only preflight to check joint limits, initial contacts,
   self/table/fixture geometry, exact goals and the declared goal tolerance.
   Sampling the tolerance region is diagnostic, not a proof of feasibility.
3. Repair the flagged KUKA/xArm narrow-passage geometry or goals in v2. Record
   the exact change and why it was necessary; audit Panda and all other scenes
   with the same procedure.
4. For each nominally feasible scene/robot pair, seek and retain a reference
   trajectory that the independent plant can execute to a collision-free goal.
   Record timing, clearance and actuator assumptions. A geometric path alone
   does not establish dynamic feasibility.
5. Label cases as witnessed feasible, intentionally adverse/infeasible, or
   unresolved. Keep initially unsafe and unavoidable-contact stress cases in
   a separately declared evaluation category rather than mixing their outcomes
   into ordinary feasible-task completion rates.
6. Add benign static/no-obstacle controls and controlled dynamic cases whose
   difficulty can vary through obstacle speed, crossing time and clearance.
7. Add meaningful tests for known invalid endpoints, valid endpoints, initial
   contact classification, client isolation and deterministic replay.

**Deliverables:** v2 scene files, a preflight report per robot/scene, reference
trajectory evidence, regression tests and an updated feasibility section.

**Exit gate:** every primary development case has a recorded feasibility
classification and evidence; no unresolved case is silently called feasible.
The evaluator-only audit must not become privileged controller input.

## 4. P1 — Identify why HOLD still produces contact

**Purpose:** replace assumed stopping behavior with measured behavior.

1. Replay representative contact trials around the first contact: last capture,
   delivery, controller decision, command enqueue/arrival/expiry, halt request,
   measured joint velocity and true clearance.
2. Separate policy HOLD from actual physical stopping. Identify commands already
   in flight and whether older commands can arrive after a stop request.
3. Run controlled stop trials over initial joint velocities, robot families,
   actuation lag, transport delay and selected dynamics mismatch. Include
   stationary controls and contact-free stopping fixtures.
4. Measure reaction delay, time/distance to stop, tracking residuals and whole-arm
   swept clearance. Keep wall-clock compute time separate from simulated time.
5. Define an explicit measured stop criterion and observation interval. Test
   command expiry, cancellation, reset and halt ordering against it.
6. Quantify sensitivity to collision sampling frequency. Report remaining
   discretization limitations rather than claiming continuous safety.

**Deliverables:** stop-identification runner, timing diagrams/replay plots,
measured envelopes and focused queue/stop regressions.

**Exit gate:** prediction horizons and fallback behavior have a documented
relationship to observed reaction/braking behavior. If stopping cannot protect
the tested interaction, classify it or redesign the task/filter; simply making
the fixed margin larger is not sufficient evidence of a solution.

## 5. P2 — Close the research-gap decision and specify a strong baseline

1. Extend `RELATED_WORK.md` from initial screening to the closest delay-aware,
   predictive and uncertainty-aware control methods, reading original methods
   and assumptions. Verify current references when this work is performed.
2. Write a comparison matrix covering observation age, queued commands, braking,
   whole-arm geometry, uncertainty sources, mismatch and claimed guarantees.
3. Select a compatible published comparator and document equations, sensing
   access, backup policy, solver, tunable parameters and reproduction deviations.
4. Reproduce a small reference case before adapting that comparator to the arm
   testbed. Check source/license requirements before incorporating external code.
5. State the candidate contribution mathematically. Investigate the existing
   proposed question about allocating observation-age, in-flight-command and
   tracking uncertainty without double counting; do not assume it is novel.
6. Update `PROPOSAL.md` and `CLAIMS.md` with a proceed/narrow/pivot decision.

**Exit gate G0:** a defensible technical difference and a falsifiable experiment
exist. If the mechanism is already established, change the research direction
or evaluate whether the benchmark itself supports a distinct contribution.

## 6. P3 — Repair and validate forecast uncertainty

1. Treat the already inspected shifted validation cases as development data
   for any estimator redesign. Create new independent fit, calibration and
   final evaluation splits before using their outcomes.
2. Separate delay, jitter, dropout, acceleration changes and calibration error
   in controlled experiments, then test declared combinations.
3. Compare the current constant-velocity model with a justified alternative
   such as acceleration-aware dynamics or bounded residual inflation.
4. Measure forecast error and coverage by horizon, observation age, trajectory
   family and seed. Report forecast availability and stale/unavailable cases.
5. Group statistical resampling by independent trials/seeds; correlated frames
   are not independent calibration examples or independent trials.
6. Define the declared operating regime, a predeclared coverage acceptance
   rule and uncertainty intervals before examining the new evaluation set.
   Measured coverage in that regime is not a general collision-risk guarantee.
7. Specify conservative behavior for unavailable, stale or out-of-regime
   estimates, using the stopping evidence from P1. Log every fallback.

**Deliverables:** revised estimator, split manifests, calibration/validation
plots and documented fallback behavior.

**Exit gate:** reliability is quantified on independent data. If it is inadequate,
retain a deterministic/bounded formulation or revise the estimator; do not
present an inflated covariance alone as validated safety.

## 7. P4 — Build the candidate method only after its prerequisites

1. Specify state/input variables, timestamps, transport assumptions, obstacle
   motion model, robot limits, constraints, fallback and optimization objective.
2. Propagate delivered observations to the decision/actuation time, account
   for queued commands, and incorporate only justified mismatch/stop models.
3. Filter nominal velocity commands with minimal necessary modification. Select
   the formulation after P2; a steering-capable filter may be necessary because
   the current stop-only comparators cannot move around an approaching obstacle.
4. Cover whole-arm geometry and expose infeasibility, solver failure, stale state
   and compute-budget overruns. Do not hide them as successful stops.
5. Implement the published comparator with the same observation stream, nominal
   controller, command limits and allowed compute resources.
6. Add ablations removing age compensation, queued-command treatment, mismatch
   handling and uncertainty treatment individually where those components exist.
7. Test deterministic replay, no truth leakage, invalid estimates, solver failure,
   constraint residuals and fallback activation. Measure end-to-end latency and
   deadline misses on declared compute hardware.

**Exit gate G2:** stable execution and justified fallback behavior in development
cases. Formal guarantees may be claimed only if their assumptions and derivation
are actually established; regression tests are not a proof.

## 8. P5 — Fair experiments and a held-out decision

1. Start with a small paired pilot on audited v2 development scenes. Compare
   nominal control, both existing stop policies, the published comparator and
   the candidate method. Tune all relevant comparators fairly.
2. Plot contact risk versus safe completion, progress and intervention. Compare
   at matched empirical safety levels where supported; do not choose only one
   convenient baseline setting.
3. Preserve all trials, errors, infeasibility and timeouts. Distinguish contact
   trials from repeated contact samples and from contact episodes.
4. Use pilot variability and effect size to choose the main independent trial
   count. Three sensing seeds are insufficient for strong generalization claims;
   do not select the final count solely to obtain a preferred significance result.
5. Freeze methods, tuning budget, splits, success definitions, statistical plan
   and primary comparisons before running the final held-out evaluation.
6. Evaluate unseen obstacle trajectories, delay/dropout patterns and selected
   mismatch levels across Panda, KUKA and bare xArm6. Treat the gripper variant
   as an additional profile, not a fourth independent robot family.
7. Report uncertainty intervals, paired effects, tail latency/deadline misses,
   failed trials and representative counterexamples. Keep exploratory analyses
   explicitly separate from the preregistered primary comparison.

**Exit gates G3/G4:** a repeatable pilot benefit justifies main evaluation; the
held-out evidence determines the claims. If no benefit appears, investigate or
pivot instead of increasing feature count or rewriting the hypothesis afterward.

## 9. P6 — Connect sensing and prepare the paper/release

1. Add a rendered-image path through the real ArUco detector and pose estimator.
   Vary viewpoint, occlusion, blur, lighting and resolution with recorded seeds.
2. Compare image-derived observations with pose-level corruption experiments.
   Label their evidence separately; synthetic Gaussian pose noise alone is not
   end-to-end visual validation.
3. Produce architecture/method figures, primary tradeoff plots, ablations,
   failure replays and a reproducibility table from saved artifacts.
4. Draft problem, related work, method, experimental protocol, results,
   limitations and conclusion around supported claims. Map each claim to data.
5. Provide pinned dependencies, scenario hashes, source commit, seeds, commands,
   raw-trace availability and a tested reproduction entry point.
6. Obtain independent technical review, then select the venue and verify its
   current official submission requirements and deadlines.

**Exit gate G5:** contribution, evidence, limitations and reproducibility survive
review. This plan cannot predict or guarantee ICRA/IROS/journal acceptance.

## 10. Hardware work stays deferred

When hardware becomes available, schedule a separate validation campaign for
camera intrinsics/extrinsics and clocks, measured motor/stop behavior, payload,
tracking mismatch, guarded low-speed reaching and eventual grasp validation.
Check the applicable robot deployment requirements at that time. Until then,
describe the evaluation as simulation with an independent simulated plant;
do not claim physical transfer or validated physical safety.

## 11. Next-session checklist and working practice

The next authorized implementation unit should be **P0: v2 scenario preflight**.

- [ ] Confirm the working tree and baseline CI before editing.
- [ ] Reproduce the two flagged v1 exact-target contacts without changing v1.
- [ ] Implement reusable evaluator-only preflight and known-case regressions.
- [ ] Create/audit v2 development scenes and record feasibility categories.
- [ ] Run a small replay/nominal smoke before a new large sweep.
- [ ] Update the evidence report and claim register with actual findings.
- [ ] Run relevant checks, commit and push the completed unit, and inspect CI.

At each milestone, preserve historical results, commit useful code/tests/docs,
keep generated raw logs outside Git, and record source/configuration provenance.
Do not run an expensive main sweep until its scene, baseline and split gates
are satisfied. Defer additional manipulator families, UI expansion and more
grasp features unless they address a specific experiment or usability failure.

Plan by deliverable rather than a promised publication date. Review after P0/P1
and again after the pilot: those findings may change the method and remaining
effort substantially. Finish each session with the validated commit, CI status,
remaining blockers and one explicit next implementation task.
