# Portfolio & Engineering Case Study

## Project: Real-Time Vision-Guided Robotic Manipulator Digital Twin (VisionRobotTwin)

---

## 1. Executive Summary

**VisionRobotTwin** demonstrates end-to-end robotics software engineering integrating real-time computer vision, coordinate frame transformations, numerical inverse kinematics, and physics simulation. The application connects a monocular webcam to a **7-DoF Franka Emika Panda** manipulator simulated in **PyBullet**, enabling live teleoperation tracking or perception-gated autonomous pick-and-place manipulation using **ArUco fiducial markers**.

---

## 2. Technical Stack & Core Competencies

- **Languages & Frameworks**: Python 3.10+, NumPy, SciPy, OpenCV 4.8+ (`opencv-contrib-python`), PyBullet, pytest.
- **Robotics & Math**: $SE(3)$ Lie Group Kinematics, Homogeneous Transformations, Damped Least-Squares Inverse Kinematics, Nullspace Optimization, Quaternion SLERP, Slew-Rate Limiting.
- **Computer Vision**: Camera Intrinsic Calibration (Chessboard / Brown-Conrady model), ArUco Marker Detection, Perspective-n-Point (PnP / IPPE), 1 Euro Adaptive Filtering.
- **Systems & Architecture**: Perception-Gated Finite State Machines (FSM), Joint Position Control, Distance-Gated Constraints, Live OpenCV HUD Telemetry, Automated CI/Testing.

---

## 3. Key Engineering Challenges & Solutions

### Challenge 1: Sensor Noise & Jitter in Optical Pose Estimation
- **Problem**: Monocular PnP estimation suffers from high-frequency pixel jitter, causing rapid micro-movements in joint position controllers.
- **Solution**: Implemented an adaptive **1 Euro Filter** for Cartesian translations and **Spherical Linear Interpolation (SLERP)** for orientation quaternions. The filter automatically tunes its cutoff frequency based on target velocity—applying aggressive low-pass smoothing at standstill while maintaining responsive tracking during rapid motion.

### Challenge 2: Singularity Avoidance and Joint Limit Violations
- **Problem**: Standard pseudo-inverse Jacobian IK ($J^\dagger$) becomes numerically unstable near kinematic singularities and drives joints into physical limits.
- **Solution**: Implemented **Damped Least-Squares (DLS)** IK with nullspace projection toward the manipulator's nominal rest posture ($\mathbf{q}_{\text{home}}$). This exploits the 7-DoF arm's kinematic redundancy to keep joints centered within safe limits while tracking desired tool center point poses.

### Challenge 3: Perception-Gating and Preventing Remote / Fake Grasping
- **Problem**: Naive autonomous sequences start moving before markers are confirmed and attach virtual constraints regardless of distance.
- **Solution**: Built a perception-gated state machine requiring consecutive verified detections ($N \ge 5$) before freezing targets and synchronizing the simulation cube location. The virtual gripper enforces a strict physical proximity check ($< 5.5\text{ cm}$), transitioning to `ERROR` if the arm fails to reach the object.

---

## 4. Technical Interview Deep-Dive Q&A

### Q1: How does Perspective-n-Point (PnP) calculate 3D translation and orientation from a 2D image?
> **Answer**: PnP solves for camera pose relative to an object given known 3D corner coordinates in the marker frame and corresponding 2D projections on the image plane. Using the calibrated camera matrix $\mathbf{K}$, pixel coordinates are unprojected into normalized camera rays. For planar 4-corner ArUco markers, closed-form solvers like IPPE (Infinitesimal Plane-based Pose Estimation) compute the homography and factorize it into rotation matrix $\mathbf{R} \in SO(3)$ and translation vector $\mathbf{t} \in \mathbb{R}^3$.

### Q2: Why is SE(3) transformation chaining separate from workspace mapping?
> **Answer**: The vision sensor measures marker pose in camera optical frame $\mathcal{F}_C$. The robot controller commands motion in base frame $\mathcal{F}_B$. Applying the rigid homogeneous transform $\mathbf{T}_{B \to M} = \mathbf{T}_{B \to C} \cdot \mathbf{T}_{C \to M}$ accounts for physical camera mounting extrinsics. Separating rigid frame transformations from workspace mapping allows independent configuration of intuitive user interaction scaling while preserving geometric accuracy.

### Q3: How does the nullspace projection work in redundant 7-DoF manipulator IK?
> **Answer**: A 7-DoF arm possesses 1 degree of kinematic redundancy when performing 6-DoF task-space positioning. The nullspace projection matrix $\mathbf{N} = (\mathbf{I} - \mathbf{J}^\dagger \mathbf{J})$ projects secondary objective gradients $\nabla H(\mathbf{q})$ (such as distance from joint limits or preferred elbow posture) into the nullspace of $\mathbf{J}$. This ensures secondary motion does not perturb the primary end-effector Cartesian target.

---

## 5. Resume Bullet Points

- **Robotics Software Engineer Bullet**:
  > *Developed a real-time vision-guided robotic manipulation digital twin integrating OpenCV-based ArUco 6-DoF pose estimation with a PyBullet Franka Panda manipulator; implemented SE(3) coordinate-frame transformations, inverse kinematics, adaptive 1 Euro pose filtering, workspace constraints, trajectory visualization, tracking-loss recovery, and perception-gated autonomous pick-and-place state control.*

- **Computer Vision / Controls Engineer Bullet**:
  > *Engineered an end-to-end perception-to-control pipeline in Python/PyBullet; implemented PnP 6-DoF pose estimation, camera intrinsic calibration, Damped Least-Squares IK, and quaternion SLERP smoothing, verified via 27 automated unit/integration tests and CI workflows.*

---

## 6. LinkedIn Project Post Template

🚀 **Excited to share my latest robotics software project: VisionRobotTwin (v1.1 Engineering Hardening)!**

I built a real-time, closed-loop vision-guided robotic manipulator digital twin connecting a webcam to a simulated 7-DoF Franka Emika Panda arm in PyBullet.

🔍 **Key Highlights**:
- **Computer Vision**: OpenCV 4.8+ ArUco detection, camera intrinsic calibration, and 6-DoF Perspective-n-Point (PnP) pose estimation.
- **Kinematics & Control**: Homogeneous $SE(3)$ transformations, Damped Least-Squares Inverse Kinematics, and joint position control.
- **Signal Filtering & Safety**: Adaptive 1 Euro filtering, quaternion SLERP smoothing, time-based slew-rate velocity limiting, and workspace boundary clamping.
- **Autonomy**: Perception-gated Finite State Machine (FSM) executing multi-waypoint pick-and-place manipulation with physical distance-validated grasping.
- **Testing & CI**: 27 unit & integration tests running automatically via GitHub Actions on Windows.

💻 **Check out the code & documentation on GitHub**: https://github.com/Tanishk756/VisionRobotTwin

#Robotics #ComputerVision #Python #PyBullet #OpenCV #Kinematics #ControlSystems #DigitalTwin #FrankaPanda

---

## 7. GitHub Publishing Checklist

- [x] All 27 automated pytest tests passing (`pytest -v`)
- [x] Marker generator and printable assets generated (`tools/generate_aruco_markers.py`)
- [x] Unmistakable synthetic stream indicators and explicit camera failure modes
- [x] Physical distance-gated virtual grasping and perception-gated autonomous sequencing
- [x] Time-based slew-rate limiting independent of frame rate
- [x] GitHub Actions CI workflow running on Windows with Python 3.11/3.12
- [x] Clean `.gitignore` excluding `.venv/`, `.pytest_cache/`, `logs/*.log`, `calibration/*.npz`
- [x] Verified zero machine-specific absolute paths and zero credentials
- [x] Comprehensive technical documentation (`README.md`, `ARCHITECTURE.md`, `PORTFOLIO.md`, `VALIDATION.md`)
