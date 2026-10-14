# Independent plant/twin foundation: smoke results

2026-10-14. Six 0.5 s simulation trials, three robot families, two seeds.
All runner processes exited with code 0. These are benign infrastructure
checks using unfiltered proportional reaching, not safety-method evaluation.

| Robot / seed | State records | Delivered observations | Dropped observations | Sampled contact episodes | Minimum obstacle clearance (mm) |
|---|---:|---:|---:|---:|---:|
| kuka_iiwa-seed0 | 121 | 11 | 2 | 0 | 311.31 |
| kuka_iiwa-seed1 | 121 | 11 | 3 | 0 | 310.73 |
| panda-seed0 | 121 | 11 | 2 | 0 | 195.97 |
| panda-seed1 | 121 | 11 | 3 | 0 | 195.52 |
| xarm6-seed0 | 121 | 11 | 2 | 0 | 286.95 |
| xarm6-seed1 | 121 | 11 | 3 | 0 | 286.27 |

Defaults: 240 Hz physics, one capture per eight steps, 80 ms observation delay
with 20 ms jitter, 40 ms command delay with 10 ms jitter, 10% dropout, 1 mrad
joint noise, 3 mm obstacle noise, plant mass scale 1.2, plant velocity gain 0.8,
and twin velocity gain 1.0. Values are experimental settings, not identified
hardware parameters. Actual event delivery is quantized to physics ticks.

Each subdirectory retains the summary, exact raw-trace hash, source-file hashes,
environment, and configuration. Raw traces/events/estimates remain locally under
`experiments/plant-twin-m1/`. Reproduce with the command in
[the testbed guide](../../research/TESTBED.md), setting the corresponding robot,
seed, duration and a new empty output directory. Manifests record a dirty working
tree based on commit 17d434f; they do not claim the implementation was already in
that commit. Same-seed exact replay is covered by tests in the current environment.

Regression: 474 passed, 7 optional skips. Twenty new tests cover immutable
contracts, validation, delivery order/timing, separate physics clients, delayed
information, non-mutating twin queries, stale messages, estimator behavior,
truth collision detection, terminal obstacle state, seed replay, isolated RNG
streams, mass mismatch, output provenance and resource cleanup.

No calibrated uncertainty, continuous collision guarantee, controller performance
advantage, published baseline reproduction or hardware validation is established.
