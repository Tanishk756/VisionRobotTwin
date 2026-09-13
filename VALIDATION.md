# Systems Validation Matrix & Technical Audit Report

## VisionRobotTwin (v1.2 Development Cycle)

This document provides a strict, evidence-grounded validation report for all perception, kinematics, control, simulation, and hardware subsystems. In accordance with rigorous robotics engineering standards, **synthetic simulation results are explicitly distinguished from physical hardware validation**.

---

## 1. Subsystem Verification Matrix

| Subsystem / Feature | Verification Method | Status | Evidence & Notes |
| :--- | :--- | :---: | :--- |
| **Unit Kinematics & Transforms** | Automated pytest (`test_transforms.py`) | **`PASS`** | $SO(3)$ orthogonality, Euler/Quaternion roundtrip, $SE(3)$ analytical inversion, and composition verified. |
| **World-Anchor Extrinsics Math** | Automated pytest (`test_extrinsics_math.py`) | **`PASS`** | $\mathbf{T}_{\text{robot}\to\text{camera}} = \mathbf{T}_{\text{robot}\to\text{anchor}} \cdot \mathbf{T}_{\text{camera}\to\text{anchor}}^{-1}$ analytical derivation, identity, translation, 90° rotation, and validation residuals verified. |
| **Extrinsics JSON Schema & IO** | Automated pytest (`test_extrinsics_io.py`) | **`PASS`** | Save/load roundtrip, strict schema verification, and NaN/Inf rejection verified. |
| **Robust Pose Aggregation** | Automated pytest (`test_pose_aggregation.py`) | **`PASS`** | Multi-sample median pose aggregation, standard deviation computation, and outlier rejection ($>3\sigma$) verified. |
| **Calibration Quality Heuristics**| Automated pytest (`test_calibration_quality.py`)| **`PASS`** | Laplacian sharpness, bounding area ratio, spatial/scale diversity score, duplicate sample rejection, and report serialization verified. |
| **Benchmark Suite V2 Pipeline** | Automated pytest (`test_benchmark_v2.py` / `test_benchmark_pipeline.py`)| **`PASS`** | Stationary jitter mode, dynamic tracking mode, percentiles, CSV time-series, and anonymous manifest verified. |
| **Demo Recording & Metadata** | Automated pytest (`test_demo_and_snapshots.py`)| **`PASS`** | Snapshot adjacent JSON metadata capture (`screenshots/session_*.json`) and video writer initialization verified. |
| **Signal Filtering (EMA / 1-Euro)** | Automated pytest (`test_filters.py`) | **`PASS`** | Low-pass step response, noise variance reduction $> 60\%$, and quaternion SLERP smoothing verified. |
| **Workspace Safety & Slew Limiting**| Automated pytest (`test_workspace.py`) | **`PASS`** | Workspace bounding box clamping, NaN/Inf rejection, and time-based Cartesian velocity limiting verified. |
| **IK Solver & Limit Checking** | Automated pytest (`test_ik_and_robot.py`) | **`PASS`** | IK solutions returned, out-of-limits strictly rejected, and dynamic tracking verified against test tolerance. |
| **Distance-Gated Virtual Grasp** | Automated pytest (`test_gripper_physics.py`) | **`PASS`** | Physical proximity threshold ($5.5\text{ cm}$) strictly enforced; remote grasps rejected; constraints cleaned on release. |
| **State Machine & Perception Gating**| Automated pytest (`test_state_machine.py`) | **`PASS`** | Gated on consecutive observations of ID 1 & 2 with median pose aggregation; timeout triggers `ERROR`. |
| **Camera Hardware / Synthetic Modes**| Automated pytest (`test_camera.py`) | **`PASS`** | Physical camera initialization and mid-stream disconnect handling verified; synthetic fallback gated. |
| **Simulation Physics Scheduling** | Automated pytest (`test_simulation_clock.py`)| **`PASS`** | Multi-substep accumulator advances 1/240s physics steps between perception frames without real-time slowdown. |
| **Autonomous Pick-and-Place E2E**| Automated pytest (`test_auto_integration.py`)| **`PASS (Synthetic)`**| True perception $\to$ gating $\to$ waypoint $\to$ PyBullet physical constraint $\to$ transport $\to$ place $\to$ home. |
| **PyBullet Headless Digital Twin** | Automated test (`test_headless_integration.py`)| **`PASS`** | Bounded headless pipeline runs without desktop GUI or display dependencies. |
| **Physical Webcam Live Capture** | Live Hardware Detection | **`USER-CONFIRMED / PASS`** | Physical operator webcam smoke test completed on Marker 0 teleoperation (v1.1 baseline). |
| **Physical Camera Intrinsics** | Chessboard Calibration Tool | **`NOT TESTED`** | Uncalibrated fallback uses horizontal FOV pinhole model. Run `python tools/calibrate_camera.py` for physical metrics. |
| **World-Anchor Extrinsic Calibration** | Anchor Tool (`tools/calibrate_extrinsics.py`) | **`NOT TESTED`** | Verified on synthetic stream; physical camera extrinsic calibration pending live Marker ID 10 capture by user. |
| **Physical Stationary Benchmark** | Benchmark Tool (`tools/benchmark_live.py`) | **`NOT MEASURED`** | Synthetic stationary jitter and tracking verified. Physical optical jitter pending user hardware run. |
| **Physical Franka Manipulator Hardware** | Hardware Telemetry | **`NOT TESTED`** | Manipulator control executed inside validated PyBullet digital twin. |

---

## 2. Benchmark & Performance Status

```
+-------------------------------------------------------------------------------+
| PARAMETER                            | STATUS / VALUE                         |
+-------------------------------------------------------------------------------+
| Automated Unit & Integration Tests   | PASSING (68 / 68 tests passing)        |
| Continuous Integration (CI)          | Configured (Windows Python 3.11 & 3.12)|
| Physics Simulation Clock Rate        | 240 Hz Target (Fixed 1/240s timestep)  |
| Dynamic Tracking Acceptance Criterion| Error < 45 mm (Dynamic Test Threshold) |
| Basic Physical Webcam Smoke Test     | USER-CONFIRMED / PASS (Marker 0)       |
| Static Marker Optical Jitter         | NOT MEASURED (Physical camera req)     |
| Camera Intrinsic Calibration Status  | DEFAULT PINHOLE (Metric calib pending) |
| World-Anchor Extrinsics Status       | NOMINAL (Physical calibration pending) |
+-------------------------------------------------------------------------------+
```

---

## 3. Physical Hardware Calibration & Validation Workflow

To calibrate and validate the physical perception-control pipeline on real webcam hardware:

1. **Camera Intrinsic Calibration**:
   ```powershell
   python tools/calibrate_camera.py --camera 0 --cols 9 --rows 6 --square-size 0.025
   ```
   *Generates `calibration/camera_calibration.npz` and `calibration/camera_calibration_report.json`.*

2. **World-Anchor Extrinsic Calibration (Marker ID 10)**:
   Place printed Marker ID 10 ($50\text{ mm} \times 50\text{ mm}$) at a fixed known position relative to the virtual robot base (e.g. $X=0.50\text{ m}, Y=0.0\text{ m}, Z=0.0\text{ m}, \text{roll}=\pi$):
   ```powershell
   python tools/calibrate_extrinsics.py --camera 0 --marker-id 10 --samples 30
   ```
   *Saves calibrated $\mathbf{T}_{\text{robot}\to\text{camera}}$ to `calibration/extrinsics.json`.*

3. **Extrinsic Calibration Validation**:
   ```powershell
   python tools/validate_extrinsics.py --camera 0 --marker-id 10
   ```
   *Measures and reports anchor point residual translation error (mm) and orientation error (deg).*

4. **Stationary Jitter & Tracking Benchmark**:
   ```powershell
   # Standstill jitter measurement (place marker 0 completely still)
   python tools/benchmark_live.py --camera 0 --duration 10.0 --benchmark-mode stationary

   # Dynamic tracking benchmark
   python tools/benchmark_live.py --camera 0 --duration 15.0 --benchmark-mode tracking
   ```
   *Outputs structured session results to `benchmarks/YYYYMMDD_HHMMSS/`.*

5. **Demo Video Recording**:
   ```powershell
   python tools/record_demo.py --camera 0 --duration 15.0
   # Or directly:
   python main.py --camera 0 --record
   ```
