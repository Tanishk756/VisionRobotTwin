# Versioned scenarios and fixed-horizon engineering comparator

This milestone implements a first comparator, not a new research safety method
or a reproduction of a published state-of-the-art controller.

## Reproduce

```powershell
python tools/compare_research_baselines.py --output-dir experiments/baseline-comparison
```

Defaults: four development scenarios, Panda/KUKA iiwa/xArm6, seed 0, nominal
control and fixed-horizon stopping. This is 24 trials. Output must be empty.
Use `--seeds 0 1 2` for more paired realizations. Do not treat these scenarios
as held-out evaluation after using their results for development.

Run an individual case:

```powershell
python tools/run_plant_twin_experiment.py --scenario config/research_scenarios/crossing-v1.json --robot panda --seed 0 --baseline fixed-horizon --output-dir experiments/crossing-example
```

`--baseline none` selects the paired nominal controller. `--horizon` and
`--margin` expose the engineering comparator parameters. CLI robot/seed/duration/
observation-delay overrides are reflected in the resolved manifest. Invalid
overrides, such as a duration shorter than an occlusion interval, are rejected.

## Scenario schema version 1

Definitions live in `config/research_scenarios/` and specify schema version,
ID, data split, configuration, motion frame, time/position waypoints, optional
occlusion intervals, static spherical fixtures, and first-joint target offset.
Unknown top-level keys and invalid versions, times, geometry and intervals are
rejected. IDs are restricted to safe lowercase identifier characters.

- `crossing-v1`: a sphere traverses the nominal home end-effector region.
- `sudden-approach-v1`: a sphere waits, approaches, pauses and retreats.
- `occluded-crossing-v1`: crossing motion with a 400 ms observation blackout.
- `narrow-passage-v1`: two static spheres flank the motion region; a separate
  tracked obstacle remains farther away.

Waypoints interpolate linearly and hold their last position after the last time.
`nominal_home_ee` means a translated origin at the nominal home end effector,
with **world-aligned axes**, not axes rotated with the end-effector orientation.
The anchor comes from nominal twin FK, never measured plant truth. Static fixtures
are a known map provided equally to plant and twin; the moving obstacle enters
the twin only through delivered observations. The controller is not given future
motion or occlusion schedules. Actual distance/feasibility differs by arm geometry,
so cross-robot outcomes require separate interpretation.

## Collision-query interface

`PlantTwinHarness.check_candidate_motion()` uses delivered telemetry, intention
history, and a hypothetical new command. It does not submit that command, consume
transport randomness, or query plant truth. It samples predicted whole-arm
configuration at the current time and along a nominal horizon; table, known
fixtures, self-collision and the last observed sphere are checked. Joint-limit
violations reject motion. Candidate queries restore twin joints/velocities and
leave plant state unchanged. Margin requests above 0.5 m are rejected because
that exceeds the guaranteed distance-query range used here.

The moving obstacle is frozen at its last observed position for this comparator.
Finite temporal/configuration sampling can miss between-sample contacts. This is
not a continuous collision certificate, braking-distance model, uncertainty tube,
or proof that holding is safe against an approaching obstacle.

## Baseline and recovery

Both methods use the same delayed-observation proportional joint controller,
target, velocity cap, sensing schedule and 30 Hz default decision cadence. The
fixed-horizon policy uses a 100 ms nominal horizon, 20 ms samples and a 20 mm
environment margin. It transmits zero velocity when the sampled candidate is
unsafe; the stop experiences the same command delay as ordinary commands.
Previously submitted commands remain in flight until replaced or canceled by
the separate execution supervisor.

The policy transitions WAIT → TRACK → HOLD. HOLD resumes only after three
distinct clear observation sequences, at least 100 ms holding, and an additional
5 mm clearance margin. Repeated decisions on one frame cannot satisfy recovery.
Query failures and execution faults enter a latched FAULT state. Resetting the
policy does not clear a supervisor fault, and automatic recovery never resets it.
All methods retain the same local telemetry/command watchdog supervisor.

These are engineering defaults, not optimally tuned baseline settings. A fair
research comparison still requires parameter sweeps and a published comparator.

## Outputs and interpretation

Each trial retains trace, event, estimate, prediction and decision files plus a
source/configuration manifest. `comparison.json` and `comparison.csv` index both
methods and retain failed trial records. Resolved scenario definitions, including
defaults and overrides, are embedded in manifests.

Metrics include sampled contact episodes, initially colliding state, minimum
environment clearance, maximum final joint error, task completion, collision-free
completion, first entry into the target tolerance, intervention fraction and
decision computation P95/deadline misses. Completion means final maximum joint
error <= 0.02 rad, not a Cartesian or physical manipulation criterion. Intervention
fraction includes startup WAIT and latched FAULT decisions. P95 computation covers
policy selection and decision serialization, not the entire perception/simulation
loop; it is a diagnostic rather than an end-to-end real-time claim.

One-seed development outcomes cannot establish statistical superiority. Contacts,
incomplete tasks, infeasible cases and watchdog faults must remain visible.
Uncertainty calibration, motion-aware prediction, braking identification, strong
published baselines, held-out tests and hardware evidence remain pending.
