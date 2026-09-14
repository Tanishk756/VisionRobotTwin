# VisionRobotTwin Task-Space Benchmark Experiment Suite

A mathematically rigorous, reproducible robotics research benchmarking suite comparing multi-manipulator kinematics, trajectory tracking, and motion planning inside PyBullet.

---

## 1. Overview & Research Objective

The experiment suite provides deterministic benchmarking comparing:
- **Robots**: Franka Emika Panda (7-DoF) vs KUKA LBR iiwa (7-DoF).
- **Controllers**: Inverse Kinematics (IK) Position Control with joint-rate limiting vs Resolved-Rate Jacobian differential velocity control with adaptive Levenberg-Marquardt damping.
- **Physics Engine**: PyBullet `DIRECT` simulation with fixed $\Delta t = 1/240\text{ s}$ ($240\text{ Hz}$) and zero wall-clock bias.
- **Trajectories**: Identical deterministic SE(3) Cartesian trajectories evaluated under strict shared preflight feasibility.
- **Motion Planning**: Obstacle-blocked Cartesian reachability comparing direct joint-space interpolation against RRT-Connect planning and shortcutting.

---

## 2. Benchmark Architecture

```
VisionRobotTwin/
├── robotics/
│   ├── experiments.py              # Experiment definitions, trajectory generators, preflight, metrics
│   ├── differential_ik.py          # Resolved-rate controller & manipulability
│   ├── planning.py                 # Direct path validation & RRT-Connect planner
│   ├── collision.py                # PyBullet self & environment collision queries
│   └── trajectory.py               # Quintic polynomials & SLERP
├── tools/
│   └── run_taskspace_experiments.py # CLI experiment runner & visualizer
├── tests/
│   └── test_experiment_suite.py    # 14 automated mathematical & execution tests
├── docs/experiments/v1.2_reference/ # Curated reference results & plots
│   ├── REPORT.md                   # Full markdown research report
│   ├── manifest.json               # Environment & execution manifest
│   ├── summary.json                # Aggregate summary statistics
│   ├── trial_results.csv           # Tabular trial metrics
│   └── plots/                      # 3D comparisons & telemetry figures
└── EXPERIMENTS.md                  # This architecture & reproduction guide
```

---

## 3. Experiment Matrix & Task Definitions

| Experiment | Description | Motion Profile | Target Feasibility |
| :--- | :--- | :--- | :--- |
| **LINE** | 10 cm horizontal Cartesian translation along Y axis. | Quintic time scaling ($T = 2.0\text{s}$) | Fixed down-facing orientation ($z = 0.35\text{m}$) |
| **CIRCLE** | Horizontal circular path ($R = 5\text{ cm}$) in XY plane. | Constant angular velocity ($T = 3.0\text{s}$) | Fixed orientation ($z = 0.35\text{m}$) |
| **FIGURE EIGHT** | Lemniscate of Gerono ($A_x = 4\text{ cm}, A_y = 3\text{ cm}$). | Direction reversals & inflection ($T = 4.0\text{s}$) | Fixed orientation ($z = 0.35\text{m}$) |
| **WAYPOINT BOX** | 4-corner closed rectangular path ($6\text{ cm} \times 6\text{ cm}$). | Multi-segment quintic blend ($T = 4.0\text{s}$) | Corner transitions ($z = 0.35\text{m}$) |
| **SE3 SWEEP** | Harmonic translation with $\pm 20^\circ$ X-axis roll sweep. | Quaternion SLERP ($T = 3.0\text{s}$) | 6-DoF full pose tracking ($z = 0.35\text{m}$) |
| **OBSTACLE REACH** | Reaching across a tall central barrier ($z = 0.35\text{m}$). | RRT-Connect + Shortcutting | Direct path blocked; RRT planned |

---

## 4. Preflight Feasibility Verification

Before any trial is executed, a dense preflight check runs IK across the entire trajectory for **both** Panda and KUKA iiwa. Both manipulators must satisfy:
- Cartesian position residual $\le 25.0\text{ mm}$
- Orientation residual $\le 10.0^\circ$
- Valid joint limit compliance

If a trajectory fails feasibility on either robot, it is rejected and marked `SHARED_FEASIBILITY_FAILED`.

---

## 5. Summary Reference Results (PyBullet Simulation)

> [!NOTE]
> Values represent mean across 3 deterministic repeatability executions ($240\text{ Hz}$ PyBullet `DIRECT` mode).

| Experiment | Metric | Panda (IK) | Panda (Resolved-Rate) | KUKA iiwa (IK) | KUKA iiwa (Resolved-Rate) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Line** | **RMSE Pos (mm)** | 2.39 mm | 12.27 mm | 20.44 mm | 13.06 mm |
| | **Joint Travel (rad)** | 0.46 rad | 0.44 rad | 1.68 rad | 0.64 rad |
| | **Min Manipulability** | 0.08 | 0.08 | 0.08 | 0.08 |
| | **Success Rate** | **100%** | **100%** | **0%\*** | **100%** |
| **Circle** | **RMSE Pos (mm)** | 27.46 mm | 22.38 mm | 20.99 mm | 22.67 mm |
| | **Joint Travel (rad)** | 1.55 rad | 1.77 rad | 3.13 rad | 2.00 rad |
| | **Min Manipulability** | 0.08 | 0.07 | 0.06 | 0.07 |
| | **Success Rate** | **100%** | **100%** | **0%\*** | **100%** |
| **Figure Eight** | **RMSE Pos (mm)** | 10.81 mm | 15.38 mm | 20.92 mm | 15.71 mm |
| | **Joint Travel (rad)** | 1.80 rad | 1.56 rad | 3.16 rad | 1.79 rad |
| | **Min Manipulability** | 0.07 | 0.08 | 0.07 | 0.07 |
| | **Success Rate** | **100%** | **100%** | **0%\*** | **100%** |
| **Waypoint Box** | **RMSE Pos (mm)** | 6.56 mm | 13.95 mm | 20.81 mm | 14.32 mm |
| | **Joint Travel (rad)** | 1.32 rad | 1.28 rad | 2.74 rad | 1.55 rad |
| | **Min Manipulability** | 0.08 | 0.08 | 0.07 | 0.07 |
| | **Success Rate** | **100%** | **100%** | **0%\*** | **100%** |
| **SE3 Sweep** | **RMSE Pos (mm)** | 10.31 mm | 5.95 mm | 20.63 mm | 7.19 mm |
| | **Mean Orn (deg)** | 8.23° | 6.64° | 0.98° | 6.72° |
| | **Joint Travel (rad)** | 1.86 rad | 2.78 rad | 5.15 rad | 3.42 rad |
| | **Success Rate** | **100%** | **100%** | **0%\*** | **100%** |

*\* Note: KUKA IK position control exhibits a numerical residual of ~20 mm in PyBullet's default IK solver, exceeding the strict 10.0 mm settling threshold. Resolved-Rate Jacobian control on KUKA cleanly converges to the trajectory across all tasks (100% success rate).*

---

## 6. Obstacle-Blocked Planning Results

| Robot | Direct Joint Path | RRT-Connect Status | Planning Time | Raw Wpts | Smoothed Wpts | Execution Clearance | Final Pos Error |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Franka Panda** | `BLOCKED` | `SUCCESS` | 1474 ms | 50 | 14 | 2.7 mm | 1.20 mm (PASS) |
| **KUKA LBR iiwa** | `BLOCKED` | `SUCCESS` | 670 ms | 36 | 4 | 5.6 mm | 18.82 mm (PASS) |

---

## 7. Reproduction Commands

To run the complete benchmark suite:
```bash
python tools/run_taskspace_experiments.py --all --repeats 3 --headless
```

To run individual trajectories:
```bash
python tools/run_taskspace_experiments.py --experiment line --headless
python tools/run_taskspace_experiments.py --experiment circle --headless
python tools/run_taskspace_experiments.py --robot panda --controller ik --all --headless
```

To run automated test verification:
```bash
pytest tests/test_experiment_suite.py -v
```
