# ROS2 verification, robustness experiments, and clearance margins

Date: 14 October 2026. Base commit: `d09bfe4e13e7b2ead7d1314a67cfe408e109664b`.
This report records pre-commit validation. Experiment manifests retain the working-tree state at capture; subsequent commit-level checks are available in GitHub Actions. Application version remains `1.3.0-dev`.

## 1. ROS2 validation verified

GitHub Actions completed successfully for the previously pushed hardening commit:

- [ROS2 Humble Integration CI, run 38427452767](https://github.com/Tanishk756/VisionRobotTwin/actions/runs/38427452767):
  **417 regression tests passed**, followed by **4 controller-level E2E tests passed**.
- [Windows validation, run 38427452587](https://github.com/Tanishk756/VisionRobotTwin/actions/runs/38427452587): success.

The ROS2 log was inspected, rather than inferring success from an old status file.
This confirms the October 13 guard/watchdog hardening passed real ROS2 Humble
controller infrastructure with simulation hardware. It is not physical-arm evidence.

The controller-level watchdog test now relies on automatic monitor startup during
arming, without explicitly calling `start_watchdog_monitor()`. Both CI workflows
also run a two-seed robustness smoke and upload its result package. Those new
workflow/test edits are evaluated by the workflows on push.

## 2. Seeded robustness suite

New implementation: `robotics/robustness.py`.
CLI: `tools/run_robustness_experiments.py`.

The default experiment varies robot, controller, seed, initial joint configuration,
and target offset. Each selected seed is evaluated against seven profiles:

| Profile | Injection | Expected outcome |
|---|---|---|
| Nominal | Seeded joint/start and Cartesian goal perturbations | Reach final pose within 10 mm / 10 degrees |
| Pose noise | Gaussian position noise, 3 mm standard deviation; rotation-vector noise, 0.01 rad standard deviation | Reach final pose despite noisy targets |
| Detection dropout | Contiguous loss from 35% to 55% of trajectory duration | Freeze measured joint posture during loss; recover tracking |
| Stale telemetry | Receive age becomes 200 ms, exceeding 100 ms state timeout | Latch `state_stale`, stop and reject subsequent commands |
| Transport disconnect | Inject disconnected/unhealthy transport status | Latch `backend_unhealthy`, stop and reject subsequent commands |
| Dispatch failure | Backend returns false from command dispatch | Latch `command_dispatch_failed`, stop and reject subsequent commands |
| Command starvation | Stop sending movement commands | Watchdog latches `command_watchdog_timeout` and stops |

The main reference motion is a one-second quintic Cartesian translation from the
perturbed home FK pose, with fixed initial orientation and 0.5 seconds of settling.
Truth measurements bypass the injected telemetry; the controller does not.

The JSON summary separates **tracking success** from **expected-stop success**.
A fault profile passes only if the exact expected fault is latched, another command
is blocked without backend dispatch, halt is attempted, final measured joint speed
is below 0.05 rad/s, and there are no collision/clearance or joint-limit events.
Stopping short of the goal is not counted as tracking success.

Reports include means, standard deviations, P95/max errors, explicit success rates,
orientation errors, fault-detection delay, clearance events, and per-trial records.
The manifest records the actual seed list, configuration, dirty state, base HEAD,
and runtime-source hash before experiments start. Nonfinite/unexecuted planning
measurements serialize as JSON null. Any unexpected trial error or failed outcome
makes the CLI exit nonzero, while retaining the failure evidence.

## 3. Configurable collision clearance

`PyBulletCollisionProvider` and the compatibility `CollisionChecker` now accept
separate environment and self-clearance margins. Contact exemptions also apply
to distance queries. Results distinguish physical contact flags from
`clearance_violation`; either condition rejects a planned configuration.

The same policy reaches direct-path validation, RRT-Connect expansion, and path
shortcutting. `MotionManager` applies an engine-independent clearance wrapper
with an additional 5 mm of planning headroom for each positive margin and finer
0.02 rad collision sampling. Requested execution clearance remains separate from
the planning headroom. The HUD displays clearance violations explicitly.

Legacy runtime defaults remain zero to preserve previous demonstrations and
reference configurations. Enable a policy explicitly:

```powershell
python main.py --robot panda --synthetic --scene obstacles --clearance-mm 10
python main.py --robot kuka_iiwa --synthetic --scene obstacles --clearance-mm 10 --self-clearance-mm 5
```

These flags affect collision-aware planning. Manual IK/resolved-rate target
streaming is not newly protected by a dynamic collision-stop supervisor.
Execution-time monitoring/replanning across the main runtime remains the next phase.

The obstacle experiment checks actual clearance during trajectory playback **and
settling**, stops on violation, and records environment and self clearance separately.
Candidate-query restoration now also covers exceptions during temporary joint
installation, so a partial installation cannot leave the simulator changed.
Planner configuration, endpoint limits, and collision resolution receive validation.

## Measured evidence

- **435 passed, 7 ROS2-dependent skips**, Windows / Python 3.12.10, 66.29 seconds.
- **28 new regression cases**, including the near-miss policy, contact exemptions,
  state restoration on error, all seven robustness profiles with both controllers,
  repeatability, and the previously failing seeded Panda clearance case.
- **84/84 robustness outcomes passed** across two robots, two controllers, seven
  profiles, and three seeds: **36/36 tracking trials**, **48/48 expected fault stops**.
- **6/6 obstacle planning and execution cases passed** using 10 mm environment
  clearance, zero additional self-clearance, 5 mm planning headroom, and seeds 0–2.
- Minimum measured environment clearance across those runs: **15.46 mm**.
  Maximum: **19.22 mm**. The requested minimum was 10 mm.
- Worst noisy-profile final position error: **5.65 mm**. This is a pose-noise
  simulation result, not an optical camera accuracy measurement.
- Maximum observed fault-detection delay from injection: **95.83 ms** in simulation
  time. This does not establish physical braking latency or real-time scheduling.
- Three 60-frame synthetic smokes with `--clearance-mm 10` exited 0: Panda IK,
  Panda resolved-rate, KUKA IK. Bytecode compilation and whitespace checks passed.

The first unbuffered Panda/seed-1 obstacle execution dipped to **9.986 mm** and
was correctly rejected/stopped. That counterexample is preserved rather than
discarded. Additional planning headroom and finer sampling resolved all six
tested cases without lowering the requested 10 mm execution threshold.

See the [full result package](../experiments/2026-10-14-robustness/REPORT.md),
[source manifest](../experiments/2026-10-14-robustness/manifest.json), and
[counterexample](../experiments/2026-10-14-robustness/clearance-counterexample.json).
The tested runtime-source SHA-256 is
`97106910614964b44e67c5e7b91fbe9f9aff86a9c92808067d46f52f71c5c82d`.

Reproduce the full seeded run:

```powershell
python tools/run_robustness_experiments.py --planning
```

For a smaller run:

```powershell
python tools/run_robustness_experiments.py --robots panda --controllers resolved-rate --seeds 0 1
```

The runner is always headless. Output goes into a new `experiments/robustness_*`
directory. Explicit `--output-dir` paths must not already exist, to avoid mixing
results with an earlier run.

## Remaining scope

Noise is injected at pose level, not into camera pixels or detector internals.
Detection loss is one bounded burst; the distribution does not cover every
occlusion pattern. Transport faults retain an independent simulation halt path;
loss of physical stop authority is not modeled. Three seeds provide a regression
sample, not a statistical reliability guarantee. Collision sampling remains
discrete, and a 5 mm planning buffer is not a hardware safety margin certification.

Physical robot selection, read-only hardware commissioning, measured camera
calibration, dynamic-obstacle runtime monitoring/replanning, and contact-based grasping
remain outstanding. Remote workflows separately validate each pushed checkpoint. All guards remain
non-safety-rated software.

## Post-commit verification

Commit `0b144c2` passed [ROS2 Humble CI](https://github.com/Tanishk756/VisionRobotTwin/actions/runs/38522132839) with **445 regression tests and 4 controller-level tests**, plus the seeded robustness smoke. [Windows Python 3.11 and 3.12 validation](https://github.com/Tanishk756/VisionRobotTwin/actions/runs/38522132826) also passed. A subsequent workflow-maintenance change replaces deprecated Node.js 20 actions with official Node.js 24 releases; GitHub Actions records its own validation separately.
