# VisionRobotTwin Task-Space Benchmark Experiment Report

> [!NOTE]
> **REFERENCE PYBULLET SIMULATION RESULTS**<br>
> Generated: 2026-09-14T11:09:13.744574+00:00 UTC<br>
> Software Version: 1.2.0-dev | Commit: `f316b08c80db165a05983cb6bc9b2fcadbeafc04`<br>
> Physics Engine: PyBullet package `3.2.7` (API: `202010061`) (`DIRECT` mode, fixed timestep dt = 0.004167s / 240 Hz)

---

## 1. Executive Summary & Objective

This benchmark suite provides a mathematically rigorous, reproducible experimental comparison of:
1. **Manipulators**: Franka Emika Panda (7-DoF) vs KUKA LBR iiwa (7-DoF).
2. **Controllers**: IK position control with coordinated joint velocity limits vs Resolved-Rate Jacobian differential velocity control with damping.
3. **Trajectories**: Identical deterministic Cartesian task-space trajectories executed under strictly matching physical simulation parameters.
4. **Motion Planning**: Obstacle-blocked Cartesian reachability comparing direct joint-space interpolation against RRT-Connect planning and shortcutting.

## 2. Environment & Simulation Parameters

| Parameter | Value | Details |
| :--- | :--- | :--- |
| **OS** | `Windows` | 10.0.26200 |
| **Python** | `3.12.10` | CPython |
| **NumPy** | `2.2.6` | Vectorized linear algebra |
| **PyBullet Package** | `3.2.7` | Physics client `DIRECT` |
| **PyBullet API** | `202010061` | Internal C API version |
| **Physics Frequency** | `240 Hz` | `dt = 0.004167s` |
| **Settle Tolerance** | `5.0 mm` | Settled final position threshold |
| **Success Position Tolerance** | `10.0 mm` | Trial completion position threshold |
| **Success Orientation Tolerance** | `10.0 deg` | Trial completion orientation threshold |
| **Planning Position Tolerance** | `25.0 mm` | Obstacle reach final endpoint threshold |
| **Deterministic Repeats** | `3` | Deterministic repeatability executions |
| **Random Seed** | `42` | Deterministic initialization seed |

## 3. Shared Feasibility Preflight

Before executing any trajectory trial, the entire desired task-space curve is sampled at dense intervals. Both Franka Emika Panda and KUKA LBR iiwa solvers verify that inverse kinematics solutions exist with Cartesian position residual $\le 25\text{ mm}$ and orientation error $\le 10^\circ$ across every checked waypoint.

| Trajectory | Shared Feasible | Panda Feasible | KUKA Feasible | Checked Samples | Total Samples | Stride | Max Pos Residual (mm) | Max Orn Residual (deg) | Policy |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| `line` | **PASS** | PASS | PASS | 481 | 481 | 1 | 20.08 mm | 0.08° | `EXACT_PATH` |
| `circle` | **PASS** | PASS | PASS | 721 | 721 | 1 | 20.28 mm | 0.08° | `EXACT_PATH` |
| `figure_eight` | **PASS** | PASS | PASS | 961 | 961 | 1 | 20.24 mm | 0.08° | `EXACT_PATH` |
| `waypoint_box` | **PASS** | PASS | PASS | 961 | 961 | 1 | 20.19 mm | 0.08° | `EXACT_PATH` |
| `se3_sweep` | **PASS** | PASS | PASS | 721 | 721 | 1 | 20.08 mm | 0.08° | `EXACT_PATH` |

Every generated trajectory sample was checked during preflight verification (`preflight_sample_stride = 1`).

## 4. Task Definitions

1. **LINE**: Straight 10 cm horizontal Cartesian translation with fixed orientation.
   - Center Position: `[0.48, 0.0, 0.35]`
   - Geometry: `{'length_m': 0.1, 'axis': 'Y', 'path_type': 'Straight Cartesian Line'}`
   - Duration: `2.0 s` (481 samples @ 240 Hz)
   - Orientation Profile: `Fixed downward [1, 0, 0, 0]` (Rotation Axis: `None (Fixed)`)

2. **CIRCLE**: Continuous horizontal task-space circle (R = 5 cm) with fixed orientation.
   - Center Position: `[0.48, 0.0, 0.35]`
   - Geometry: `{'radius_m': 0.05, 'plane': 'XY', 'path_type': 'Continuous Planar Circle'}`
   - Duration: `3.0 s` (721 samples @ 240 Hz)
   - Orientation Profile: `Fixed downward [1, 0, 0, 0]` (Rotation Axis: `None (Fixed)`)

3. **FIGURE_EIGHT**: Lemniscate figure-eight trajectory (X peak amplitude: 4 cm, Y peak amplitude: 3 cm).
   - Center Position: `[0.48, 0.0, 0.35]`
   - Geometry: `{'amplitude_x_m': 0.04, 'amplitude_y_m': 0.03, 'plane': 'XY', 'path_type': 'Lemniscate (Figure-Eight)'}`
   - Duration: `4.0 s` (961 samples @ 240 Hz)
   - Orientation Profile: `Fixed downward [1, 0, 0, 0]` (Rotation Axis: `None (Fixed)`)

4. **WAYPOINT_BOX**: Smooth 4-corner closed box path (6 cm x 6 cm) in the XY plane.
   - Center Position: `[0.48, 0.0, 0.35]`
   - Geometry: `{'size_x_m': 0.06, 'size_y_m': 0.06, 'plane': 'XY', 'corners': 4, 'path_type': '4-Corner Rectangular Route'}`
   - Duration: `4.0 s` (961 samples @ 240 Hz)
   - Orientation Profile: `Fixed downward [1, 0, 0, 0]` (Rotation Axis: `None (Fixed)`)

5. **SE3_SWEEP**: 6-DoF trajectory with +/-20 deg X-axis roll rotation via quaternion SLERP and 2 cm X harmonic translation.
   - Center Position: `[0.48, 0.0, 0.35]`
   - Geometry: `{'harmonic_translation_axis': 'X', 'harmonic_amplitude_m': 0.02, 'rotation_axis': 'X', 'max_roll_deg': 20.0, 'path_type': '6-DoF Roll SLERP + Harmonic Translation'}`
   - Duration: `3.0 s` (721 samples @ 240 Hz)
   - Orientation Profile: `Quaternion SLERP roll sweep (+/-20 deg around X-axis)` (Rotation Axis: `X (Roll)`)

## 5. Cross-Robot & Cross-Controller Summary Matrix

| Experiment | Metric | Panda (IK) | Panda (Resolved-Rate) | KUKA iiwa (IK) | KUKA iiwa (Resolved-Rate) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Line** | **RMSE Pos (mm)** | 2.39 | 12.27 | 20.44 | 13.06 |
| | **P95 Pos (mm)** | 3.96 | 20.78 | 20.50 | 20.82 |
| | **Mean Orn (deg)** | 0.00° | 0.01° | 0.07° | 0.05° |
| | **Joint Travel (rad)** | 0.46 rad | 0.44 rad | 1.68 rad | 0.64 rad |
| | **Min Manipulability** | 0.08 | 0.08 | 0.08 | 0.08 |
| | **Success Rate** | **100%** | **100%** | **0%** | **100%** |
| **Circle** | **RMSE Pos (mm)** | 27.46 | 22.38 | 20.99 | 22.67 |
| | **P95 Pos (mm)** | 53.21 | 35.96 | 23.40 | 35.91 |
| | **Mean Orn (deg)** | 0.02° | 0.01° | 0.07° | 0.02° |
| | **Joint Travel (rad)** | 1.55 rad | 1.77 rad | 3.13 rad | 2.00 rad |
| | **Min Manipulability** | 0.08 | 0.07 | 0.06 | 0.07 |
| | **Success Rate** | **100%** | **100%** | **0%** | **100%** |
| **Figure Eight** | **RMSE Pos (mm)** | 10.81 | 15.38 | 20.92 | 15.71 |
| | **P95 Pos (mm)** | 29.06 | 29.75 | 22.24 | 29.77 |
| | **Mean Orn (deg)** | 0.01° | 0.01° | 0.07° | 0.02° |
| | **Joint Travel (rad)** | 1.80 rad | 1.56 rad | 3.16 rad | 1.79 rad |
| | **Min Manipulability** | 0.07 | 0.08 | 0.07 | 0.07 |
| | **Success Rate** | **100%** | **100%** | **0%** | **100%** |
| **Waypoint Box** | **RMSE Pos (mm)** | 6.56 | 13.95 | 20.81 | 14.32 |
| | **P95 Pos (mm)** | 14.81 | 21.26 | 21.82 | 21.24 |
| | **Mean Orn (deg)** | 0.00° | 0.01° | 0.07° | 0.02° |
| | **Joint Travel (rad)** | 1.32 rad | 1.28 rad | 2.74 rad | 1.55 rad |
| | **Min Manipulability** | 0.08 | 0.08 | 0.07 | 0.07 |
| | **Success Rate** | **100%** | **100%** | **0%** | **100%** |
| **Se3 Sweep** | **RMSE Pos (mm)** | 10.31 | 5.95 | 20.63 | 7.19 |
| | **P95 Pos (mm)** | 22.88 | 13.48 | 21.20 | 14.21 |
| | **Mean Orn (deg)** | 8.23° | 6.64° | 0.98° | 6.72° |
| | **Joint Travel (rad)** | 1.86 rad | 2.78 rad | 5.15 rad | 3.42 rad |
| | **Min Manipulability** | 0.08 | 0.08 | 0.08 | 0.08 |
| | **Success Rate** | **100%** | **100%** | **0%** | **100%** |

## 6. Visual Evidence & Trajectory Comparisons

### Trajectory 3D Comparison Line
![Trajectory 3D Comparison Line](plots/trajectory_3d_comparison_line.png)

### Trajectory 3D Comparison Circle
![Trajectory 3D Comparison Circle](plots/trajectory_3d_comparison_circle.png)

### Trajectory 3D Comparison Figure Eight
![Trajectory 3D Comparison Figure Eight](plots/trajectory_3d_comparison_figure_eight.png)

### Trajectory 3D Comparison Waypoint Box
![Trajectory 3D Comparison Waypoint Box](plots/trajectory_3d_comparison_waypoint_box.png)

### Trajectory 3D Comparison Se3 Sweep
![Trajectory 3D Comparison Se3 Sweep](plots/trajectory_3d_comparison_se3_sweep.png)

### Telemetry Timeseries Line
![Telemetry Timeseries Line](plots/telemetry_timeseries_line.png)

### Telemetry Timeseries Circle
![Telemetry Timeseries Circle](plots/telemetry_timeseries_circle.png)

### Telemetry Timeseries Figure Eight
![Telemetry Timeseries Figure Eight](plots/telemetry_timeseries_figure_eight.png)

### Telemetry Timeseries Se3 Sweep
![Telemetry Timeseries Se3 Sweep](plots/telemetry_timeseries_se3_sweep.png)

### Summary Benchmarks Comparison
![Summary Benchmarks Comparison](plots/summary_benchmarks_comparison.png)

## 7. Obstacle-Blocked Motion Planning Experiment

A dedicated obstacle avoidance experiment tests the complete collision-aware planning stack when moving between points $[0.42, -0.18, 0.35]$ and $[0.42, 0.18, 0.35]$ separated by a rigid box obstacle at $[0.42, 0.0, 0.35]$.

Under this PyBullet configuration, planning PASS requires collision-free trajectory execution AND a final endpoint error $\le 25.0\text{ mm}$. This criterion validates high-level obstacle clearing and endpoint arrival, distinct from the 5.0 mm continuous settled tracking tolerance.

| Robot | Direct Path State | RRT-Connect Status | Plan Time (ms) | Raw Waypoints | Smoothed Waypoints | Raw Travel (rad) | Smoothed Travel (rad) | Min Clearance (m) | Execution | Final Error (mm) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **PANDA** | `BLOCKED` | `SUCCESS` | 1473.5 ms | 50 | 14 | 2.39 rad | 1.70 rad | 0.0027 m | **PASS** | 1.20 mm |
| **KUKA_IIWA** | `BLOCKED` | `SUCCESS` | 669.6 ms | 36 | 4 | 1.70 rad | 1.30 rad | 0.0056 m | **PASS** | 18.82 mm |

## 8. Failure Cases & Singularity Telemetry

- **Feasibility**: 5 shared-feasible trajectories verified, 0 failed feasibility preflight trajectories.
- **Trial Execution**: 60 executed trials (45 successful, 15 failed), 0 skipped trials due to preflight gating.
- **Collisions**: Recorded 0 self-collisions and 0 environment-collisions across all executed trials.
- **Singularity Warnings**: 0 total near-singularity warning steps (condition number $> 100$ or $\sigma_{\min} < 0.01$) observed during execution.
- **Joint Limit Events**: 0 joint-limit violations observed during execution.

## 9. Limitations & Conservative Interpretation

> [!WARNING]
> **Experimental Scope & Limitations:**
> - All results reported herein were gathered strictly inside **PyBullet physics simulation** under idealized rigid-body dynamics.
> - No claim is made regarding physical hardware performance, motor thermal limits, gear backlash, or physical friction non-linearities.
> - Kinematic and controller comparisons reflect the specific URDF models, joint limits, and damping parameters configured in this environment.

## 10. Reproduction Commands

To reproduce the complete benchmark run deterministically:
```bash
python tools/run_taskspace_experiments.py --all --repeats 3 --headless
```

To run individual trajectories:
```bash
python tools/run_taskspace_experiments.py --experiment line --headless
python tools/run_taskspace_experiments.py --experiment circle --headless
python tools/run_taskspace_experiments.py --robot panda --controller ik --all --headless
```
