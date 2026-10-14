# Research foundation milestone

Implemented the first research infrastructure slice and documented the candidate
study. The broader [roadmap](../research/PAPER_ROADMAP.md) remains in progress.

## Completed

- Initial methods screening of five primary research sources with an explicit
  overlap/gap audit. G0 remains open rather than asserting unverified novelty.
- Candidate proposal, falsifiable hypotheses, assumptions and claim register.
- Immutable observation, command and scenario contracts.
- Monotonic event queues recording requested and actual delivery times.
- Independent plant/twin PyBullet clients, configurable plant mass/gain mismatch,
  noisy delayed sensing, dropout, moving obstacle and delayed velocity commands.
- Twin synchronization solely from delivered observations, with stale sequence
  rejection and independent sensing/command RNG streams.
- Baseline obstacle constant-velocity estimator with non-mutating prediction.
- Evaluator-only truth records, sampled contact episodes and terminal evaluation.
- CLI runner with trace/event/estimate export and source/configuration manifests.
- Twenty new regression checks and six short three-family/two-seed smoke runs.

Validation: **474 passed, 7 optional skips**. All six CLI experiments exited 0.
[Recorded smoke results](../experiments/2026-10-14-plant-twin-foundation/REPORT.md).

## Remaining

This is not a completed paper contribution or the full WP1/WP2 scope. G0 requires
further targeted review and a defensible technical distinction. G1 still needs
stronger scenario/evaluator validation and complete information/timing interfaces.
The twin is observation-synchronized kinematics, not a dynamic rollout model.
Uncertainty calibration, image-level sensing, command expiry/fault fallback,
braking identification, published baselines, candidate filter, ablations, main
held-out experiments and physical validation are pending.

Next implementation slice: command-history/expiry and a twin query interface,
versioned crossing-obstacle scenarios, estimator calibration experiments, and a
fixed-horizon engineering comparator operating only on delivered observations.
Resolve the published baseline and G0 method distinction before claiming a new
safety algorithm.
