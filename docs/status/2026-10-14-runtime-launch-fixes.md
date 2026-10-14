# Runtime launch fixes (2026-10-14)

Local application logs exposed failures that the earlier green CI run missed.
The obstacle launch path was absent from its smoke checks.

## Fixed failures

- Obstacle startup passed `length` to a cylinder collision shape. PyBullet
  requires `height` for collision shapes and `length` for visual shapes.
- Closing the physics window could disconnect between frame operations and
  raise a fatal telemetry error. The application now exits cleanly when its
  simulation connection closes, while still propagating connected runtime
  failures. Simulator cleanup is idempotent. Failed initialization releases
  the physics client and camera.
- Resolved-rate boundary damping did not bound commands over the execution
  guard's lookahead. A longer KUKA run also reproduced a roughly 30 nanoradian
  numerical overshoot at a hard limit. Commands now respect the guard horizon,
  retain a 5 mrad interior margin, and remain velocity bounded. The guard itself
  still rejects invalid or colliding commands and retains latched faults.
- The offline auto demo tried to pick the already placed object at its original
  location on its second cycle. Its existing scene synchronization callback now
  runs at each demo cycle start. This explicit demonstration resets the cube;
  it does not represent a physical object returning by itself.
- Windows batch scripts now resolve their environment and application relative
  to the script directory, including when invoked from another directory.

## Local validation

- Full regression suite: **579 passed, 7 skipped**. Optional ROS2-dependent
  checks are exercised by the separate ROS2 CI workflow.
- Final focused launch/controller suite: **20 passed**, including real scene
  construction, failed construction cleanup, closing before/during a frame,
  propagation of connected errors, near-limit velocity bounds, and two
  complete physical pick/place cycles using a deterministic demo clock.
- Eight actual bounded CLI launches: four robot profiles, both controllers,
  obstacle scenes and synthetic camera. All exited successfully. The gripper
  profile launches also invoked `run.bat` from outside the repository.
- Longer resolved-rate tracking: all four profiles completed 360 frames at
  1/30 s per frame (12 s simulated) without an execution guard fault.
- Python compilation and whitespace checks passed.

CI now includes an obstacle launch matrix for all four profiles and both
controllers. Local raw logs remain ignored. This closes the reproduced
software failures; hardware validation remains pending.
