# Project status and engineering evidence

Current checkpoint: [Robustness and clearance, 14 October 2026](2026-10-14-robustness-and-clearance.md).

## Validation reports

- [14 October: ROS2 verification, seeded robustness, and clearance margins](2026-10-14-robustness-and-clearance.md)
- [13 October: kinematics, controller, and execution hardening](2026-10-13-software-hardening.md)
- [16 September: v1.3 architecture and integration milestones](2026-09-16-v1.3-development-summary.md)

## Design specifications

Engineering designs are retained in `docs/design/`. They describe the original decisions; current code, tests, and dated validation reports take precedence for implementation status.

- [2026-09-13-multi-robot-control-design](../design/2026-09-13-multi-robot-control-design.md)
- [2026-09-15-v1.3-backend-architecture-design](../design/2026-09-15-v1.3-backend-architecture-design.md)
- [2026-09-15-v1.3-phase-a2-kinematics-provider-design](../design/2026-09-15-v1.3-phase-a2-kinematics-provider-design.md)
- [2026-09-15-v1.3-phase-a3-collision-provider-design](../design/2026-09-15-v1.3-phase-a3-collision-provider-design.md)
- [2026-09-15-v1.3-phase-a4-resolved-robot-model-design](../design/2026-09-15-v1.3-phase-a4-resolved-robot-model-design.md)
- [2026-09-15-v1.3-phase-b1-ros2-joint-state-design](../design/2026-09-15-v1.3-phase-b1-ros2-joint-state-design.md)
- [2026-09-16-v1.3-phase-b2-ros2-simulation-command-design](../design/2026-09-16-v1.3-phase-b2-ros2-simulation-command-design.md)
- [2026-09-16-v1.3-phase-b3-command-safety-design](../design/2026-09-16-v1.3-phase-b3-command-safety-design.md)
- [2026-09-16-v1.3-phase-b3_5-ros2-control-validation-design](../design/2026-09-16-v1.3-phase-b3_5-ros2-control-validation-design.md)
- [2026-09-16-v1.3-phase-b4-physical-integration-design](../design/2026-09-16-v1.3-phase-b4-physical-integration-design.md)

## Reproducible simulation evidence

- [Seeded robustness and clearance experiments](../experiments/2026-10-14-robustness/REPORT.md)
- [13 October controller hardening benchmarks](../experiments/2026-10-13-hardening/REPORT.md)
- [Historical v1.2 reference benchmarks](../experiments/v1.2_reference/REPORT.md)

Historical experiment data is retained with its original provenance. Local execution notes and tooling caches are excluded from Git.
