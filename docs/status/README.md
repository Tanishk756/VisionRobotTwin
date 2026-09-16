# VisionRobotTwin Documentation & Project Status Index

This directory contains canonical project status reports, documentation audit logs, and status indexes for the VisionRobotTwin robotics platform.

---

## 📌 Current Authoritative Status Documents

- **[End-of-Day Development Summary (2026-09-16)](2026-09-16-v1.3-development-summary.md)**:  
  The single canonical source of truth regarding the true development state of VisionRobotTwin v1.3 on branch `develop/v1.3.0`. Summarizes architecture boundaries (A1–A4), ROS2 integration (B1–B3.5), vendor-neutral physical framework (B4.1), test evidence, capability matrix, and physical hardware gate boundaries.

- **[Markdown Audit & Reconciliation Report (2026-09-16)](2026-09-16-markdown-audit.md)**:  
  Exhaustive audit and classification table of every Markdown file in the repository, resolving contradictions, fixing local link leaks, and documenting the source-of-truth hierarchy.

---

## 🏛️ Architecture Specifications & Implementation Plans

Historical and active phase specifications and plans are maintained under `docs/superpowers/`:

### Historical Specifications (`docs/superpowers/specs/`)
- **v1.2 Architecture**: [`2026-09-13-multi-robot-control-design.md`](../superpowers/specs/2026-09-13-multi-robot-control-design.md)
- **Phase A1 (Execution Backend)**: [`2026-09-15-v1.3-backend-architecture-design.md`](../superpowers/specs/2026-09-15-v1.3-backend-architecture-design.md)
- **Phase A2 (KinematicsProvider)**: [`2026-09-15-v1.3-phase-a2-kinematics-provider-design.md`](../superpowers/specs/2026-09-15-v1.3-phase-a2-kinematics-provider-design.md)
- **Phase A3 (CollisionProvider)**: [`2026-09-15-v1.3-phase-a3-collision-provider-design.md`](../superpowers/specs/2026-09-15-v1.3-phase-a3-collision-provider-design.md)
- **Phase A4 (Resolved Robot Model)**: [`2026-09-15-v1.3-phase-a4-resolved-robot-model-design.md`](../superpowers/specs/2026-09-15-v1.3-phase-a4-resolved-robot-model-design.md)
- **Phase B1 (Read-Only ROS2 JointState)**: [`2026-09-15-v1.3-phase-b1-ros2-joint-state-design.md`](../superpowers/specs/2026-09-15-v1.3-phase-b1-ros2-joint-state-design.md)
- **Phase B2 (ROS2 Simulation Command Transport)**: [`2026-09-16-v1.3-phase-b2-ros2-simulation-command-design.md`](../superpowers/specs/2026-09-16-v1.3-phase-b2-ros2-simulation-command-design.md)
- **Phase B3 (Software Command Safety Layer)**: [`2026-09-16-v1.3-phase-b3-command-safety-design.md`](../superpowers/specs/2026-09-16-v1.3-phase-b3-command-safety-design.md)
- **Phase B3.5 (Real ros2_control Validation)**: [`2026-09-16-v1.3-phase-b3_5-ros2-control-validation-design.md`](../superpowers/specs/2026-09-16-v1.3-phase-b3_5-ros2-control-validation-design.md)
- **Phase B4 (Controlled Physical Integration)**: [`2026-09-16-v1.3-phase-b4-physical-integration-design.md`](../superpowers/specs/2026-09-16-v1.3-phase-b4-physical-integration-design.md) *(Active)*

### Historical Implementation Plans (`docs/superpowers/plans/`)
- **v1.2 Implementation**: [`2026-09-13-multi-robot-control-implementation.md`](../superpowers/plans/2026-09-13-multi-robot-control-implementation.md)
- **Phase A1 (Execution Backend)**: [`2026-09-15-v1.3-phase-a1-execution-backend.md`](../superpowers/plans/2026-09-15-v1.3-phase-a1-execution-backend.md)
- **Phase A2 (KinematicsProvider)**: [`2026-09-15-v1.3-phase-a2-kinematics-provider.md`](../superpowers/plans/2026-09-15-v1.3-phase-a2-kinematics-provider.md)
- **Phase A3 (CollisionProvider)**: [`2026-09-15-v1.3-phase-a3-collision-provider.md`](../superpowers/plans/2026-09-15-v1.3-phase-a3-collision-provider.md)
- **Phase A4 (Resolved Robot Model)**: [`2026-09-15-v1.3-phase-a4-resolved-robot-model.md`](../superpowers/plans/2026-09-15-v1.3-phase-a4-resolved-robot-model.md)
- **Phase B1 (Read-Only ROS2 JointState)**: [`2026-09-15-v1.3-phase-b1-ros2-joint-state.md`](../superpowers/plans/2026-09-15-v1.3-phase-b1-ros2-joint-state.md)
- **Phase B2 (ROS2 Simulation Command Transport)**: [`2026-09-16-v1.3-phase-b2-ros2-simulation-command.md`](../superpowers/plans/2026-09-16-v1.3-phase-b2-ros2-simulation-command.md)
- **Phase B3 (Software Command Safety Layer)**: [`2026-09-16-v1.3-phase-b3-command-safety.md`](../superpowers/plans/2026-09-16-v1.3-phase-b3-command-safety.md)
- **Phase B3.5 (Real ros2_control Validation)**: [`2026-09-16-v1.3-phase-b3_5-ros2-control-validation.md`](../superpowers/plans/2026-09-16-v1.3-phase-b3_5-ros2-control-validation.md)
- **Phase B4 (Controlled Physical Integration)**: [`2026-09-16-v1.3-phase-b4-physical-integration.md`](../superpowers/plans/2026-09-16-v1.3-phase-b4-physical-integration.md) *(Active)*

---

## 🔒 Reference & Provenance Artifacts

- **[v1.2 Reference Research Report](../experiments/v1.2_reference/REPORT.md)**: Immutable reference benchmarks, metrics tables, and 3D trajectory plots for the Franka Panda and KUKA iiwa manipulators in PyBullet.

---

## 📖 Source-of-Truth Rules

1. **Current Code & Tests Override Planning Prose**: A historical plan describing future intent does not override code proving that a feature has been completed.
2. **Current Development Summary Governs Present State**: For all questions regarding current readiness, active gates, test baselines, and physical hardware status, refer directly to `docs/status/2026-09-16-v1.3-development-summary.md`.
3. **Historical Context Is Preserved**: Baselines and initial statements in historical specifications and plans represent authentic engineering history and must not be retroactively altered.
