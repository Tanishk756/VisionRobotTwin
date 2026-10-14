# Execution deadlines and nominal prediction: smoke results

Eight half-second simulation checks: six nominal three-family/two-seed trials,
one complete startup sensor dropout, and one 1 s command delay exceeding the
150 ms command watchdog. The experiment driver exited 0. Expected fault trials
continue to record terminal state rather than ending at the first fault.

| Trial | State records | Execution fault | Sampled contact episodes |
|---|---:|---|---:|
| kuka_iiwa-seed0 | 121 | None | 0 |
| kuka_iiwa-seed1 | 121 | None | 0 |
| missing-commands | 121 | command_watchdog_timeout at 0.150s | 0 |
| panda-seed0 | 121 | None | 0 |
| panda-seed1 | 121 | None | 0 |
| startup-dropout | 121 | startup_telemetry_timeout at 0.300s | 0 |
| xarm6-seed0 | 121 | None | 0 |
| xarm6-seed1 | 121 | None | 0 |

All six nominal trials completed without execution faults. Complete sensor loss
latched startup telemetry timeout at 0.300 s and canceled 11 queued commands.
Missing command deliveries latched command watchdog timeout at 0.150 s and
canceled 37 queued commands. Stop is a local software request outside the delayed
command queue, not a braking-distance or physical safety guarantee.

Validation: full suite 496 passed / 7 optional skips. After a final forecast-latch
correction and two additional tests, the focused research suite passed all 44
checks. Those tests cover expired/stale messages, watchdog boundaries, cancellation,
reset and failed halt, analytical motion integration, missing first deliveries,
forecast expiry gaps, limit reporting, and three-family truth isolation.

Each trial retains a manifest and summary. Raw traces, events, obstacle estimates,
and nominal prediction samples remain under local ignored
`experiments/plant-twin-watchdogs/`. Manifests record dirty source based on
91c7f36 with exact source-file hashes. These benign short trials validate the
infrastructure and selected fault behavior, not estimator calibration, a novel
controller's superiority, physical stopping, or continuous collision avoidance.

Reproduction and assumptions: [testbed guide](../../research/TESTBED.md).
