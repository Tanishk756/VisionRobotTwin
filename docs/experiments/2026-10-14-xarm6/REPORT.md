# Seeded simulation robustness report

Pose-level sensing and simulated transport faults; no physical camera or hardware evidence.

Completed: 12/12; expected outcomes passed: 12; errors: 0.

Tracking requires both final position and orientation tolerances. Expected fault stops are assessed separately from tracking success. Fault trials must latch the specified fault, block a subsequent command, attempt halt, and finish below 0.05 rad/s with no collision/clearance or joint-limit events.

| Robot/controller/profile | Outcomes | Tracking | Final error mean / P95 (mm) |
|---|---:|---:|---:|
| xarm6/ik/nominal | 1/1 | 1/1 | 0.38 / 0.38 |
| xarm6/ik/pose_noise | 1/1 | 1/1 | 4.40 / 4.40 |
| xarm6/ik/detection_dropout | 1/1 | 1/1 | 0.38 / 0.38 |
| xarm6/resolved-rate/nominal | 1/1 | 1/1 | 0.03 / 0.03 |
| xarm6/resolved-rate/pose_noise | 1/1 | 1/1 | 0.61 / 0.61 |
| xarm6/resolved-rate/detection_dropout | 1/1 | 1/1 | 0.47 / 0.47 |
| xarm6_gripper/ik/nominal | 1/1 | 1/1 | 0.38 / 0.38 |
| xarm6_gripper/ik/pose_noise | 1/1 | 1/1 | 5.09 / 5.09 |
| xarm6_gripper/ik/detection_dropout | 1/1 | 1/1 | 0.38 / 0.38 |
| xarm6_gripper/resolved-rate/nominal | 1/1 | 1/1 | 0.02 / 0.02 |
| xarm6_gripper/resolved-rate/pose_noise | 1/1 | 1/1 | 0.59 / 0.59 |
| xarm6_gripper/resolved-rate/detection_dropout | 1/1 | 1/1 | 0.47 / 0.47 |

## Clearance planning

Required environment clearance: 10 mm; self clearance: 0 mm. Positive margins receive an additional 5 mm planning buffer and 0.02 rad path sampling; execution is checked against the requested margins.

| Robot / seed | Plan | Execution | Minimum environment / self clearance (mm) |
|---|---|---|---:|

## Limits

Gaussian noise is injected into pose targets, not camera images. Detection loss is one contiguous burst. Start joints and goal offsets vary by seed. These profiles do not model all real delays, optical failures, payloads, or loss of physical stop authority.

The guard uses deterministic simulation-time polling; real background-thread and ROS2 behavior require the separate unit and middleware suites. Planning uses discrete collision samples, not a continuous collision guarantee. Unknown/unexecuted clearances serialize as null in JSON.

See [manifest.json](manifest.json), [summary.json](summary.json), and [trials.csv](trials.csv) for configuration, source provenance, failures, and individual measurements.
