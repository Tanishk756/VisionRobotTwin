# VisionRobotTwin v1.1.0 — Engineering-Hardened Release

**VisionRobotTwin v1.1.0** is the first formal public release of a real-time vision-guided robotic manipulation digital twin connecting OpenCV-based ArUco pose estimation to a simulated 7-DoF Franka Emika Panda in PyBullet.

---

## 🌟 Highlights

- **Real-Time 6-DoF Optical Tracking**: Monocular Perspective-n-Point (PnP / IPPE) pose estimation from standard webcam video.
- **7-DoF Franka Panda Digital Twin**: PyBullet physics simulation with joint position control, forward kinematics, and trajectory visualization.
- **Damped Least-Squares (DLS) Inverse Kinematics**: Numerical IK solver with nullspace posture projection and strict joint limit verification.
- **Perception-Gated Autonomy**: Finite State Machine (FSM) executing multi-waypoint pick-and-place manipulation with sample median stabilization.
- **Distance-Gated Virtual Grasping**: Physics-grounded attachment requiring spatial proximity ($< 5.5\text{ cm}$) to target object.
- **Fixed-Step Physics Scheduler**: Multi-substep accumulator decoupling camera loop timing from 240 Hz PyBullet simulation physics.
- **Comprehensive Quality Assurance**: 49 automated unit and integration tests passing in CI across Windows Python 3.11 and 3.12.

---

## 👁️ Computer Vision Subsystem

- **ArUco Detection**: OpenCV 4.8+ `cv2.aruco.ArucoDetector` with `DICT_4X4_50` fiducials.
- **6-DoF PnP Pose Estimation**: Infinitesimal Plane-based Pose Estimation (IPPE) solving rotation matrix $\mathbf{R} \in SO(3)$ and translation $\mathbf{t} \in \mathbb{R}^3$.
- **Camera Calibration**: Interactive chessboard calibration tool calculating OpenCV RMS reprojection error (px) and mean point error.
- **Default Pinhole Fallback**: Automatic intrinsic model generation from horizontal FOV when uncalibrated.

---

## 🦾 Robotics & Control Subsystem

- **Lie Group Kinematics**: Homogeneous $4 \times 4$ transformations in $SE(3)$ with analytical matrix inversion.
- **Dual Transform Pipelines**:
  - **Relative Mode**: Maps camera delta movements into intuitive robot Cartesian space relative to reference marker orientation.
  - **SE(3) Mode**: Analytical rigid frame composition ($\mathbf{T}_{B \to M} = \mathbf{T}_{B \to C} \cdot \mathbf{T}_{C \to M}$).
- **Signal Filtering**: Adaptive 1 Euro translational filtering with velocity-dependent cutoff and Spherical Linear Interpolation (SLERP) for orientation quaternions.
- **Joint Control**: Target joint position control via PyBullet `setJointMotorControlArray`.

---

## 🔄 Autonomous Manipulation Pipeline

- **State Machine States**: `HOME` $\to$ `SEARCH` $\to$ `APPROACH` $\to$ `PICK` $\to$ `LIFT` $\to$ `MOVE_TO_PLACE` $\to$ `PLACE` $\to$ `RETURN_HOME` $\to$ `SEARCH`.
- **Perception Gating**: Requires consecutive stable detections ($N \ge 5$) of both pick and place markers before target coordinates freeze.
- **Sample Median Aggregation**: Eliminates single-frame optical outliers during target acquisition.
- **Waypoint Timeouts**: Transitions to `ERROR` if motion fails to arrive at waypoints within configured limits.

---

## 🛡️ Safety & Reliability

- **Strict Joint Limit Rejection**: Returns `IKStatus.OUT_OF_LIMITS` and refuses execution when targets require out-of-bound joint angles.
- **Workspace Bounds Clamping**: Strict Cartesian bounding box enforcement ($X, Y, Z$).
- **Time-Based Slew Limiter**: Bounds maximum Cartesian velocity ($\le 0.5\text{ m/s}$) and angular rate ($\le 2.0\text{ rad/s}$) independent of frame rate.
- **Full-Pose Joint-Freeze HOLD**: Spacebar pauses motion and freezes exact current joint targets.
- **NaN / Inf Protection**: Sanitizes all sensor inputs and transformation matrices before computing kinematics.

---

## 🧪 Testing & CI

- **Automated Test Suite**: 49 unit and integration tests passing (`pytest -v`).
- **Continuous Integration**: GitHub Actions automated testing on Windows with Python 3.11 and 3.12.
- **Synthetic Pipeline Validation**: Headless closed-loop end-to-end integration test validating full pick-and-place execution without display dependencies.

---

## 🚀 Installation & Quick Demo

```powershell
# 1. Clone repository
git clone https://github.com/Tanishk756/VisionRobotTwin.git
cd VisionRobotTwin

# 2. Setup environment
setup.bat

# 3. Generate marker assets
python tools\generate_aruco_markers.py

# 4. Run synthetic simulation demo
python main.py --synthetic

# 5. Run physical webcam teleoperation
python main.py --camera 0 --mode manual
```

---

## ⚠️ Known Limitations & Design Boundaries

1. **Monocular Pose Sensitivity**: Metric PnP accuracy depends on camera intrinsic calibration and lighting conditions.
2. **Camera Calibration**: Calibration is hardware-specific; default pinhole model provides qualitative rather than metric accuracy.
3. **Camera Extrinsics**: Camera-to-robot base mounting transform $\mathbf{T}_{B \to C}$ uses nominal coordinates unless explicitly hand-eye calibrated.
4. **Virtual Grasping**: PyBullet grasping uses proximity-gated constraint attachment rather than physical friction finger contact simulation.
5. **Motion Planning**: Employs Cartesian waypoint trajectory generation with slew limiting; does not include a global obstacle-avoiding motion planner (e.g. OMPL/MoveIt).
6. **Hardware Scope**: This release is a software digital twin; physical Franka robot hardware is not directly commanded.

---

## 📊 Physical Validation Status

- **Synthetic End-to-End Simulation**: **`PASS`** (Validated through automated tests)
- **Basic Physical Webcam Smoke Test**: **`USER-CONFIRMED / PASS`** (Manual Marker-0 tracking interaction completed)
- **Physical Calibrated Benchmark**: **`NOT YET MEASURED`** (Hardware-dependent; benchmark tooling provided)
- **Physical Franka Robot Hardware**: **`NOT TESTED`** (Digital twin scope)

---

## 👤 Maintainer & Contact

**Tanishk Singhal**
- **GitHub**: [https://github.com/Tanishk756](https://github.com/Tanishk756)
- **Email**: [tanisksinghal6285@gmail.com](mailto:tanisksinghal6285@gmail.com)
