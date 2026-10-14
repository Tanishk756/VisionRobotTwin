# xArm6 profiles and execution collision monitoring

Added two simulation configurations: `xarm6` and `xarm6_gripper`. Both have six
arm joints, a native flange reference, canonical joint mapping, IK and
resolved-rate control. Existing Panda and KUKA configurations remain available.

The bundled xArm URDFs contain invalid link2/link3 inertia tensors. These
profiles request geometry-derived inertia instead of loading those tensors.
The gripper profile explicitly exempts four overlapping linkage pivot pairs
from self-contact detection. Other self contacts and environment contacts remain
checked. Its six revolute linkage joints open at 0 and close at 0.75 rad, leaving
a small finger gap. G closes and O opens. Automatic pick/place is capability
gated because the xArm grasp TCP and physical grasp behavior are unvalidated.

## Execution monitor

The default simulator wraps the arm backend in `CollisionGuardBackend`.
Position commands check the measured-to-command segment at 0.01 rad joint-space
sampling. Velocity commands check a 0.1 s projected segment. Measured clearance
is checked before and after every runtime physics step, including manual,
AUTO and paused execution. Query failures, predicted collision, joint-limit
prediction and transport rejection latch faults and request a software halt.
A fault blocks further arm commands, holds gripper joints, and surfaces as an
ERROR state. It cannot be cleared by marker reacquisition or changing modes.
R requires a currently clear state; home motions are checked commands rather
than instantaneous teleports. Stopping failures propagate visibly.

This is finite sampled simulation supervision, without continuous geometric
coverage or a braking-distance guarantee. It adds query overhead and does not
replace physical safety systems. Gripper commands are monitored through actual
physics states rather than a linkage trajectory predictor. Direct external
PyBullet calls bypass backend command validation; runtime physics monitoring
still checks measured states on the next simulator step.

## Validation

- Full regression run: 449 passed, 7 optional tests skipped.
- Final focused run after reset, health reporting and HUD updates: 33 passed,
  covering execution faults, all manual/AUTO controller combinations, xArm
  kinematics/control/gripper behavior and existing runtime integration.
- Four 30-frame synthetic headless launches: both xArm profiles with both
  controllers, all exit code 0.
- Twelve seeded robustness trials: both profiles, both controllers, nominal,
  pose noise and detection dropout, seed 0, all passed. Final errors ranged
  from 0.016 to 5.089 mm. Requested environment clearance: 10 mm.
- Results: [report](../experiments/2026-10-14-xarm6/REPORT.md),
  [manifest](../experiments/2026-10-14-xarm6/manifest.json),
  [summary](../experiments/2026-10-14-xarm6/summary.json),
  [trials](../experiments/2026-10-14-xarm6/trials.csv).

The manifest records the dirty working tree and source hash used for those
trials. This small one-seed smoke package does not establish broad statistical
robustness, obstacle replanning success, hardware validity or a calibrated
payload rating. Existing Panda/KUKA reference packages remain historical
measurements. Both experiment CLIs now accept registered xArm profiles while
keeping their existing default Panda/KUKA selection.

## Launch

```powershell
python main.py --robot xarm6 --synthetic
python main.py --robot xarm6_gripper --synthetic --controller resolved-rate
```
