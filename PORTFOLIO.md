# Portfolio & Engineering Case Study

## Project: Multi-Manipulator Vision-Guided Robotics Research Platform (VisionRobotTwin)

---

## 1. Executive Summary

**VisionRobotTwin** demonstrates full-stack robotics software engineering integrating real-time computer vision, coordinate frame transformations, generic multi-manipulator kinematics, resolved-rate Cartesian velocity control, manipulability analysis, collision checking, RRT-Connect motion planning, and physics simulation. The platform natively supports multiple 7-DoF industrial manipulators (**Franka Emika Panda** and **KUKA LBR iiwa**) in **PyBullet**, enabling live teleoperation tracking, perception-gated autonomous pick-and-place manipulation, and reproducible cross-robot benchmarking.

---

## 2. Technical Stack & Core Competencies

- **Languages & Frameworks**: Python 3.10+, NumPy, SciPy, OpenCV 4.8+ (`opencv-contrib-python`), PyBullet, pytest.
- **Robotics & Kinematics**: $SE(3)$ Lie Group Kinematics, Geometric Spatial Jacobian ($6 \times n$), SVD Yoshikawa Manipulability, Condition Number, Adaptive Damped Least-Squares (DLS) Pseudoinverse, Null-Space Joint Centering, Slew-Rate Limiting.
- **Differential Control & Planning**: Resolved-Rate Cartesian Velocity Control, Joint Quintic Polynomial Trajectories, Cartesian SE(3) Quaternion SLERP, Bidirectional RRT-Connect, Randomized Path Shortcutting.
- **Computer Vision**: Camera Intrinsic Calibration (Chessboard / Brown-Conrady model), World-Anchor Extrinsic Calibration (Marker ID 10), ArUco Marker Detection, Perspective-n-Point (PnP / IPPE), 1 Euro Adaptive Filtering.
- **Systems & Architecture**: Model Registry Pattern, Capability Descriptors, Perception-Gated Finite State Machines (FSM), Distance-Gated Constraints, Live OpenCV HUD Telemetry, Automated Multi-Python CI.

---

## 3. Key Engineering Challenges & Solutions

### Challenge 1: Robot-Agnostic Multi-Manipulator Abstraction
- **Problem**: Monolithic codebases couple control and state machines to a single manipulator (e.g., Franka Panda joint names and link indices).
- **Solution**: Designed a strongly typed `RobotModelSpec` and `RobotCapabilities` registry pattern. `GenericRobotController` and `GenericIKSolver` dynamically inspect URDF joint hierarchies, identify arm and gripper joints, resolve end-effector links, and enforce capability-aware constraints (safely disabling autonomous pick/place for gripperless robots like KUKA iiwa without crashing).

### Challenge 2: Singularity Management & Dynamic Velocity Scaling
- **Problem**: Inverting the manipulator Jacobian $\mathbf{J}$ near kinematic singularities causes joint velocities to explode, inducing severe physical instability.
- **Solution**: Implemented an **Adaptive Damped Least-Squares (DLS)** pseudoinverse $\mathbf{J}_{\text{dls}} = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1}$ where damping factor $\lambda$ scales continuously based on the smallest singular value $\sigma_{\min}$. Live telemetry monitors Yoshikawa index $w(\mathbf{q})$ and condition number $\kappa(\mathbf{J})$, triggering automatic velocity reduction and singularity warning alerts.

### Challenge 3: Redundancy Optimization in Task-Space Velocity Control
- **Problem**: 7-DoF manipulators tracking 6-DoF Cartesian twists have an infinite family of valid joint velocity solutions. Unconstrained motion causes the arm to drift into joint limits.
- **Solution**: Implemented null-space projection $(\mathbf{I} - \mathbf{J}_{\text{dls}} \mathbf{J}) \dot{\mathbf{q}}_{\text{null}}$ using a quadratic joint-centering potential $H(\mathbf{q})$ biased towards nominal rest posture. This guarantees secondary joint centering occurs strictly without perturbing primary end-effector tracking.

### Challenge 4: Collision-Aware Motion Planning Around Obstacles
- **Problem**: Direct joint interpolation collides when physical obstacles are present in the workspace.
- **Solution**: Implemented a bidirectional **RRT-Connect** joint-space planner with state-preserving collision queries and randomized path shortcutting.

---

## 4. Technical Interview Deep-Dive Q&A

### Q1: How does the geometric Jacobian relate joint velocities to Cartesian spatial twists?
> **Answer**: The manipulator Jacobian $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times n}$ is the differential mapping $\mathbf{v} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}$, where $\mathbf{v} = [\mathbf{v}_{\text{lin}}^T, \boldsymbol{\omega}_{\text{ang}}^T]^T$. The top 3 rows $\mathbf{J}_{\text{linear}}$ map joint rates to linear velocity of the end-effector tool center point, while the bottom 3 rows $\mathbf{J}_{\text{angular}}$ map joint rates to angular velocity in the base coordinate frame.

### Q2: How is Yoshikawa's manipulability index derived, and what does it measure?
> **Answer**: Yoshikawa's manipulability measure $w(\mathbf{q}) = \sqrt{\det(\mathbf{J} \mathbf{J}^T)}$ is proportional to the volume of the velocity manipulability ellipsoid in task space. Through Singular Value Decomposition (SVD) of $\mathbf{J}$, $w(\mathbf{q}) = \prod_{i=1}^6 \sigma_i$. Near kinematic singularities where the Jacobian loses rank, $\sigma_{\min} \to 0$ and $w(\mathbf{q}) \to 0$, indicating that the arm cannot generate velocity along at least one Cartesian direction.

### Q3: How does null-space projection guarantee zero perturbation of the primary Cartesian task?
> **Answer**: For any arbitrary joint velocity vector $\mathbf{z}$, the projection onto the null space is $\dot{\mathbf{q}}_{\text{null}} = (\mathbf{I} - \mathbf{J}^\dagger \mathbf{J}) \mathbf{z}$. Premultiplying by the Jacobian yields $\mathbf{J} \dot{\mathbf{q}}_{\text{null}} = \mathbf{J} (\mathbf{I} - \mathbf{J}^\dagger \mathbf{J}) \mathbf{z} = (\mathbf{J} - \mathbf{J} \mathbf{J}^\dagger \mathbf{J}) \mathbf{z} = (\mathbf{J} - \mathbf{J}) \mathbf{z} = \mathbf{0}$. Therefore, the secondary joint motion produces exactly zero velocity at the end effector.

---

## 5. Resume Bullet Points

- **Robotics Software Engineer Bullet**:
  > *Architected a modular multi-manipulator robotics digital twin in Python/PyBullet supporting Franka Emika Panda and KUKA LBR iiwa; developed geometric Jacobian solvers, adaptive DLS resolved-rate control, Yoshikawa manipulability telemetry, null-space redundancy optimization, and collision-aware RRT-Connect motion planning, verified via 106 automated unit/integration tests.*

- **Computer Vision / Controls Engineer Bullet**:
  > *Engineered real-time 6-DoF visual teleoperation using monocular OpenCV ArUco pose estimation and camera calibration; implemented closed-loop differential IK, 1 Euro adaptive signal filtering, quaternion SLERP interpolation, and cross-robot benchmarking suites.*

---

## 6. Verification Status

- [x] All 106 automated pytest tests passing (`pytest -v`)
- [x] Tested across Franka Emika Panda and KUKA LBR iiwa
- [x] Geometric Jacobian, SVD manipulability, and condition number verified
- [x] Resolved-rate differential controller with adaptive DLS and null-space centering verified
- [x] Joint quintic polynomial and Cartesian SE(3) SLERP trajectories verified
- [x] Collision checking and state preservation verified
- [x] Bidirectional RRT-Connect motion planner and path shortcutting verified
- [x] Headless cross-robot and cross-controller benchmark tools verified
- [x] Full backward compatibility for existing v1.1 and v1.2 camera calibration features maintained
