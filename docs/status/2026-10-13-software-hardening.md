# VisionRobotTwin software hardening — 13 October 2026

## Scope and evidence

Audited the integrated `main` checkout at base commit
`7ea68db110adf503908d0a88df806831785f80a4`, application `1.3.0-dev`.
The changes described here are local, uncommitted changes. No remote CI, ROS2
middleware, camera hardware, or physical manipulator validation was performed.

The initial local test run reported 338 passes, 7 skips, and 9 setup errors caused
by access to an existing external pytest temporary directory. Validation used
new workspace-local temporary directories and disabled the inaccessible cache.
The unchanged software baseline has 347 passing Windows-compatible tests.

## Corrected defects

| Area | Defect | Resulting behavior |
|---|---|---|
| End-effector frames | FK measured the inertial centre of mass while IK targeted the URDF link origin. KUKA link 7 has a 20 mm offset. | FK composes the inertial transform in double precision and reports the world-frame link origin. The captured KUKA raw-IK residual falls from 19.89 mm to 0.163 mm. |
| Jacobian frame | Jacobians were left in base coordinates while FK and tracking targets were in world coordinates. | Both linear and angular Jacobians rotate into world coordinates. Finite differences cover translated and rotated bases on both robots. |
| Canonical joint order | Jacobian inputs and raw IK outputs assumed arm joints occupied the first columns in canonical order. | Explicit mapping through movable-joint indices supports reordered arm joints and retains other joints' measured configuration. |
| Motor configuration | The execution-backend refactor omitted configured position and velocity gains. | Generic controller construction forwards model/configuration gains to PyBullet. Explicit backend gains are validated; direct legacy backend callers retain Bullet defaults when gains are omitted. |
| Trajectory tracking | Resolved-rate experiments calculated reference velocities without using them. | Optional world-frame linear/angular feedforward is combined with pose feedback; experiment and controller-comparison runners supply it and use zero feedforward while settling. |
| Resolved-rate inputs | Invalid orientation targets could produce NaNs or index errors. | Invalid quaternions/feedforward produce zero motion; invalid cycle times/configuration are rejected. Measured quaternions are normalized. |
| Joint velocity scaling | Independent clipping could change the direction of the commanded joint vector. | Coordinated scaling respects the most restrictive configured/model joint velocity limit. |
| Command authorization | Telemetry and transport prerequisites could become invalid after arming. | Every dispatched command rechecks local connection, health, mode capability, joint identity/count, finite positions, and receive-time freshness. |
| Dispatch errors | A false return or transport exception did not reliably latch a fault or stop possible partial dispatch. | Dispatch failure records rejection, attempts a software halt, and latches a fault without advancing the accepted sequence. Halt failure is retained as a fault. |
| Fault recovery | Disarm/disconnect could replace the fault state with DISARMED. | Faults survive disarm, disconnect, and reconnect. Recovery requires explicit healthy reset and rearming. |
| Configured envelopes | Position margins were unused; step vectors could have the wrong length; NaN timeouts could bypass comparisons. | Margins are enforced, vector length must match DoF, empty envelopes and nonfinite configuration are rejected. |
| Watchdog lifecycle | A watchdog flag alone did not ensure monitoring during complete caller starvation; disconnect joined while holding the command lock. | Arming starts monitoring by default. Deterministic/manual polling can explicitly disable auto-start. Monitor shutdown joins outside the command lock and can survive fault reset. |
| Telemetry snapshots | Frozen dataclasses could retain mutable input lists; future receive timestamps appeared fresh. | Snapshots own immutable tuples, validate unique nonempty names, and reject future timestamps when evaluating freshness. |
| Physical command eligibility | Missing expected identity and unresolved limits could default to eligibility; old safety snapshots were accepted. | Identity and limit evidence are explicit requirements, all eight safety conditions are required by default, and safety snapshots must be fresh. Observation remains separate from command eligibility. |
| Physical state and limits | Disconnected cached state, future timestamps, empty robot limit maps, and untrusted sources could be accepted. | These cases fail closed. Timeouts are validated and state freshness is evaluated after acquisition. |
| Generic controller dispatch | Extra joint targets were silently truncated and rejected dispatches appeared successful. | Targets must be finite and match DoF exactly; failed backend dispatch raises an error. |
| Motion completion | DIRECT mode completed after one rate-limited update; trajectory time alone implied arrival. | Completion requires measured joint arrival. A configurable settling timeout marks stalled motion failed and attempts a halt. Failed motion stops dispatching subsequent commands. |
| Motion replacement | Old executor/goal/planning metadata could survive replanning. | Planning resets prior trajectory state and records explicit failure details. |
| Trajectory data | Rotating SE(3) samples reported zero angular velocity; invalid duration/time inputs propagated. | SLERP reports its spatial angular velocity, including zero outside the active interval. Invalid quaternions, endpoints, durations, and steps are rejected. |
| Calibration integrity | Redundant JSON translation/quaternion fields could disagree with the matrix. | Loading rejects malformed, nonfinite, or inconsistent redundant transforms. |
| Commissioning diagnostics | Secret-detection errors repeated the rejected note. | Errors report the credential finding without reproducing the note. |
| Experiment accounting | Joint-limit events did not fail a trial; fixed-orientation paths skipped orientation completion checks. | Success requires collision-free execution, no joint-limit events, and both final pose tolerances. Failed IK is never dispatched. |
| Experiment provenance | Manifests always claimed v1.2 and omitted working-tree modifications. | Manifests record the application version, actual HEAD, dirty state, runtime-source SHA-256, frame convention, and feedforward policy before trials start. |
| Reference preservation | Publishing new results targeted the historical v1.2 package. | Reference publishing uses the current application version. The original v1.2 artifacts remain unchanged. |

## Validation

- **407 passed, 7 skipped**, Python 3.12.10 / Windows, 44.85 seconds.
- **60 additional behavioral regression cases**, compared with the 347-test baseline.
- All three bounded 60-frame synthetic smokes exited **0**: Panda IK, Panda
  resolved-rate, KUKA IK.
- Source/test bytecode compilation and `git diff --check` passed.
- **60/60 tracking trials passed**: two robots, two controllers, five paths,
  three deterministic repeats, 240 Hz.
- **2/2 obstacle planning/execution runs passed**; both direct paths were blocked.
- Original COM goldens and v1.2 experiment artifacts are preserved. Parity tests
  explicitly convert historical COM poses into the corrected link convention.

Reproduce the local suite in a fresh temporary directory:

```powershell
.\.venv\Scripts\python.exe -m pytest -q -o addopts= -p no:cacheprovider --basetemp=.test-tmp-new
.\.venv\Scripts\python.exe tools/run_taskspace_experiments.py --all --repeats 3 --headless
```

`--basetemp` must name a disposable test directory: pytest may clear its contents.

## Measured tracking results

Position RMSE in mm; these are simulation measurements, not camera or arm accuracy.

| Path | Panda IK | Panda resolved-rate | KUKA IK | KUKA resolved-rate |
|---|---:|---:|---:|---:|
| Line | 0.90 | 0.30 | 2.38 | 0.62 |
| Circle | 1.03 | 0.43 | 2.37 | 0.57 |
| Figure eight | 0.91 | 0.31 | 2.51 | 0.50 |
| Waypoint box | 0.90 | 0.25 | 2.55 | 0.52 |
| SE(3) sweep | 0.94 | 0.19 | 2.42 | 0.48 |

The historical v1.2 benchmark had 45/60 completion successes and KUKA IK errors
around 20 mm. The current results use corrected frames, restored gains, and
resolved-rate feedforward; they are a changed configuration, not a claim that
the historical experiment was rerun unchanged. Three repeats test determinism,
not performance over different seeds, scenes, payloads, or noise distributions.

Obstacle endpoint errors: Panda **1.00 mm**, KUKA **1.42 mm**. Minimum observed
clearances remain small: approximately **2.64 mm** and **5.58 mm**, respectively.
These clearances are acceptable only under the configured simulation criteria.

See the [complete benchmark package](../experiments/2026-10-13-hardening/REPORT.md),
[manifest](../experiments/2026-10-13-hardening/manifest.json), and
[summary](../experiments/2026-10-13-hardening/summary.json).

The tested runtime-source SHA-256 is
`4109bb7533af670c3d8cf65f81e7011b6fff740c03daa722bc3255c5ec1b47c4`.
The source hash still matched the working runtime after validation.

## Remaining limitations and required evidence

| Limitation | Current disposition / next work |
|---|---|
| Physical manipulator integration | Target remains unresolved. Requires explicit target selection, vendor/control-interface review, read-only adapter development, and on-site commissioning. No physical commands were sent. |
| Metric camera accuracy | Requires intrinsic/extrinsic capture with the actual camera, marker dimensions, lighting, and mounting; calibration and live-benchmark tools are available. No physical camera measurements were invented. |
| ROS2 middleware/controller revalidation | Seven tests remain skipped locally because this is Windows without ROS2 Humble. The modified guard lifecycle must pass the existing Ubuntu/ROS2 CI before a release. Historical green CI is not evidence for this patch. |
| Functional safety | Guards/watchdogs remain non-safety-rated. Software tests cannot establish certified emergency-stop, STO, braking, or interlock behavior. |
| Grasp fidelity | Virtual grasping still uses proximity-gated constraint attachment. Contact/friction-based grasping, slip, payload, and tactile behavior remain separate simulation/physical validation work. |
| Planning coverage | RRT-Connect uses discrete joint-space collision samples and static scenes. Continuous collision guarantees, dynamic-obstacle replanning, conservative hardware margins, and uncertainty-aware execution are not established. |
| Perception limits | Monocular marker scale, calibration drift, occlusion, blur, and lighting remain physical sensing limits; software filtering is not a substitute for empirical characterization. |
| Generalization | The measured paths and two registered robots do not establish accuracy across arbitrary robots, workspaces, joint subsets, payloads, or base motion. |
| High-gain position steps | Configured gains now take effect. Runtime movement should use bounded updates/trajectories; instantaneous large setpoints can create transients or strike objects. Regression approach tests now use bounded updates. |
| Release state | Application remains `1.3.0-dev`; changes and evidence are local and uncommitted. No release, deployment, push, or new remote CI result is claimed. |

This report closes the defects demonstrated by the audit and regression cases.
It does not certify the absence of every possible defect or close hardware gates.
