# Multi-Robot Control and Planning Implementation Plan

**Plan ID**: `PLAN-2026-09-13-MULTI-ROBOT-IMPLEMENTATION`  
**Target Milestone**: VisionRobotTwin v1.2.0-dev  
**Strategy**: Test-Driven Development (TDD) across 6 structured milestone commits.

---

## Milestone Commit Breakdown

### Commit 1: "Refactor robot layer into generic multi-manipulator architecture"
- **Tasks**:
  1. Create `robotics/robot_model.py`: `RobotCapabilities`, `RobotModelSpec`, and workspace configuration schemas.
  2. Create `robotics/robot_registry.py`: `RobotRegistry` singleton with `register_robot()`, `get_robot_spec()`, `list_robot_ids()`.
  3. Create `robotics/adapters/base.py`, `robotics/adapters/panda.py`, and `robotics/adapters/__init__.py`.
  4. Create `robotics/robot_controller.py`: `GenericRobotController` managing discovery of arm/gripper joints and execution of joint motor commands.
  5. Add `tests/test_robot_registry.py` and `tests/test_multi_robot_controller.py`.
- **Validation**: All 68 existing tests + new registry & controller tests pass.

---

### Commit 2: "Add KUKA iiwa support and generic kinematics"
- **Tasks**:
  1. Create `robotics/adapters/kuka_iiwa.py`: Adapter for 7-DoF KUKA LBR iiwa (`kuka_iiwa/model.urdf`).
  2. Refactor `robotics/inverse_kinematics.py`: Implement `GenericIKSolver` supporting arbitrary $N$-DoF robots, joint limit bounds, and standardized `IKResult`.
  3. Update `robotics/simulator.py` to instantiate robots via `RobotRegistry`.
  4. Update `main.py` CLI: Add `--list-robots`, `--robot-info <name>`, and `--robot <name>`.
  5. Add `tests/test_multi_robot_ik.py`.
- **Validation**: Generic IK and FK verified on both Panda and KUKA iiwa.

---

### Commit 3: "Add Jacobian resolved-rate control and manipulability metrics"
- **Tasks**:
  1. Create `robotics/kinematics.py` & `robotics/differential_ik.py`:
     - Geometric Jacobian computation ($6 \times n$).
     - Yoshikawa manipulability index $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$.
     - Condition number $\kappa(\mathbf{J}) = \sigma_{\max} / \sigma_{\min}$.
     - Adaptive Damped Least-Squares (DLS) pseudoinverse solver.
     - Resolved-rate Cartesian velocity controller with null-space joint limit centering.
  2. Add `tests/test_jacobian.py`, `tests/test_manipulability.py`, and `tests/test_differential_ik.py`.
  3. Update `main.py` CLI: `--controller {ik, resolved-rate}`.
- **Validation**: Verification against numerical finite-difference FK and singularity safety.

---

### Commit 4: "Add trajectory generation and collision checking"
- **Tasks**:
  1. Create `robotics/trajectory.py`:
     - Joint-space quintic polynomial trajectory ($C^2$ smooth).
     - Cartesian SE(3) trajectory interpolation (linear/quintic position + quaternion SLERP).
     - Trajectory executor tracking waypoint progress and Cartesian error.
  2. Create `robotics/collision.py`:
     - Continuous/discrete collision query between robot links, ground table, and obstacles.
     - State-preserving collision simulation sandbox.
     - Allowed contact whitelist (e.g. gripper tips vs. grasp object).
  3. Add `tests/test_trajectory.py` and `tests/test_collision.py`.
- **Validation**: Trajectory boundary conditions and collision detection accuracy verified.

---

### Commit 5: "Add collision-aware joint-space motion planner"
- **Tasks**:
  1. Create `robotics/planning.py`:
     - Direct linear joint-space path validator.
     - Bidirectional RRT-Connect planner with deterministic random seed.
     - Randomized shortcut post-processing for path length reduction.
  2. Integrate collision-aware motion policy in `robotics/state_machine.py` and `robotics/simulator.py`.
  3. Add obstacle scene generator (`--scene obstacles`).
  4. Add `tests/test_planning.py`.
- **Validation**: RRT-Connect circumvents obstacles deterministically without state corruption.

---

### Commit 6: "Add multi-robot and controller benchmarking"
- **Tasks**:
  1. Create `tools/compare_robots.py`:
     - Benchmarks Panda vs. KUKA iiwa over shared reachable workspaces.
     - Measures IK solve times, Cartesian residuals, manipulability, and planning success.
     - Exports `benchmarks/robot_comparison_YYYYMMDD_HHMMSS/` (`summary.json`, `results.csv`, and charts).
  2. Create `tools/compare_controllers.py`:
     - Benchmarks IK position controller vs. Resolved-Rate Jacobian controller.
     - Measures tracking error, energy/joint travel, and singularity avoidance.
     - Exports `benchmarks/controller_comparison_YYYYMMDD_HHMMSS/`.
  3. Add `tests/test_robot_benchmark.py` and `tests/test_controller_benchmark.py`.
- **Validation**: Headless execution of both benchmark tools produces complete metrics.

---

### Commit 7: "Document v1.2 multi-robot robotics stack"
- **Tasks**:
  1. Update `README.md`: Document multi-robot support matrix, CLI commands, and kinematics pipeline.
  2. Update `ARCHITECTURE.md`: Add multi-robot design diagrams and differential IK mathematics.
  3. Update `VALIDATION.md`: Add cross-robot and controller benchmark matrices.
  4. Update `CHANGELOG.md`: Record all new unreleased v1.2.0 capabilities.
  5. Update `PORTFOLIO.md`: Add multi-manipulator research framework highlights.
- **Validation**: Full suite runs with 0 errors, clean git diff, and clean security scan.
