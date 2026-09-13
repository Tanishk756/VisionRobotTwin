# Changelog

All notable changes to the VisionRobotTwin project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Camera Calibration Quality Reports**: JSON report export (`calibration/camera_calibration_report.json`) and visual diagnostic residual plots (`calibration/calibration_diagnostics.png`).
- **Calibration Sample-Quality Heuristics**: Pre-solve gating based on Laplacian variance sharpness, bounding area ratio, duplicate view rejection, and spatial/scale diversity scoring.
- **World-Anchor Extrinsic Calibration**: Camera-to-Virtual-Robot Extrinsic Calibration via ArUco Marker ID 10 ($\mathbf{T}_{\text{robot}\to\text{camera}} = \mathbf{T}_{\text{robot}\to\text{anchor}} \cdot \mathbf{T}_{\text{camera}\to\text{anchor}}^{-1}$).
- **Dedicated World Anchor Marker**: Printable Marker ID 10 asset (`assets/markers/marker_10_world_anchor.png`) and updated marker sheet tooling.
- **Extrinsics Calibration Tool**: CLI utility (`tools/calibrate_extrinsics.py`) with robust median pose aggregation, outlier rejection, and standard deviation reporting.
- **Extrinsics Validation Tool**: CLI utility (`tools/validate_extrinsics.py`) measuring anchor point residual error in mm and degrees.
- **Runtime Extrinsics Integration**: Automatic loading of `calibration/extrinsics.json` in SE(3) mode and telemetry HUD indicator (`EXTRINSICS: CALIBRATED / NOMINAL`).
- **Calibration Status CLI Command**: Diagnostic status report via `python main.py --calibration-status` without opening hardware.
- **Physical Benchmark Suite V2**: Upgraded `tools/benchmark_live.py` supporting `--benchmark-mode {stationary, tracking, standard}`, session folders (`benchmarks/YYYYMMDD_HHMMSS/`), time-series `frames.csv`, diagnostic plots, and system manifest.
- **Physical Demo Video Recorder**: Video recording utility (`tools/record_demo.py` and `python main.py --record`) supporting MP4 and AVI fallbacks.
- **Snapshot Metadata Capture**: Adjacent JSON metadata export (`screenshots/session_YYYYMMDD_HHMMSS.json`) on keyboard snapshot capture (`S`).

## [1.1.0] - 2026-09-13

### Added
- **6-DoF Optical Pose Estimation**: Monocular OpenCV 4.8+ ArUco detection and Perspective-n-Point (PnP / IPPE) 6-DoF pose solver.
- **Franka Panda Digital Twin**: PyBullet simulation environment with 7-DoF Franka Emika Panda manipulator and Franka Hand gripper.
- **Kinematic Control Modes**: Support for 6-DoF (Cartesian position + orientation) and 3-DoF (position-only) manipulation.
- **Transformation Pipelines**: Modular transformation support for intuitive Relative teleoperation mapping and analytical $SE(3)$ homogeneous frame composition ($\mathbf{T}_{B \to M} = \mathbf{T}_{B \to C} \cdot \mathbf{T}_{C \to M}$).
- **Signal Filtering**: Adaptive 1 Euro filtering for translational positions and Spherical Linear Interpolation (SLERP) for orientation quaternions.
- **Perception-Gated Autonomy**: Multi-waypoint autonomous pick-and-place finite state machine with consecutive detection gating and sample median aggregation.
- **Distance-Gated Virtual Grasping**: Physical proximity validation ($< 5.5\text{ cm}$) preventing remote or unverified object attachments.
- **Safety & Limits Enforcement**: Strict joint-limit checking with `IKStatus.OUT_OF_LIMITS` rejection, Cartesian workspace boundary clamping, time-based slew-rate velocity limiting, and NaN/Inf sanitization.
- **Tracking-Loss Hold & Recovery**: Full-pose frozen joint hold on operator pause (`SPACE`) and temporary visual occlusion recovery.
- **Fixed-Step Physics Scheduler**: Multi-substep accumulator decoupling camera frame rate from 240 Hz PyBullet simulation timestep.
- **Unified Processing Pipeline**: Shared `VisionRobotTwinApp.process_frame()` execution path across interactive runtime, headless CI validation, and benchmarking.
- **Camera Calibration & Diagnostics**: Interactive chessboard calibration utility reporting OpenCV RMS reprojection error (px) and mean point error.
- **Benchmarking Tools**: Standalone benchmark suite measuring frame rate, tracking error, jitter, detection stability, and physics stepping rate.
- **Automated CI & Testing**: 49 automated unit and integration tests executing across Windows Python 3.11 and 3.12 via GitHub Actions.

### Validation & Verification
- **Automated Test Suite**: 49 / 49 unit and integration tests passing (`pytest -v`).
- **Continuous Integration (CI)**: Multi-Python GitHub Actions workflow verified on Windows 3.11 & 3.12.
- **Synthetic End-to-End Autonomous Pipeline**: Verified from perception gating through pick, transport, release, and return to search state in PyBullet.
- **Basic Physical Webcam Smoke Test**: User-confirmed manual-tracking smoke test (Marker 0 teleoperation).
- **Physical Calibrated Benchmarks**: Not yet measured (hardware-dependent; tools provided in `tools/`).
- **Physical Franka Hardware**: Not tested (software digital twin implementation).
