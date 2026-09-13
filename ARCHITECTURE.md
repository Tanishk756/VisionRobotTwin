# Systems Architecture & Technical Specification

## VisionRobotTwin (v1.2.0-dev Multi-Manipulator Architecture)

---

## 1. System Overview

VisionRobotTwin is architected as a modular, robot-agnostic digital twin and vision-guided robotics control framework. The system strictly separates perception, geometric transformation, kinematics, differential control, trajectory generation, collision checking, motion planning, and physics simulation.

```mermaid
graph TB
    subgraph Perception ["1. Perception Subsystem"]
        Cam[Hardware Camera / Synthetic Stream] --> Det[cv2.aruco.ArucoDetector]
        Det --> PnP[PnP 6-DoF Pose Solver]
        PnP --> Calib[Intrinsics / Extrinsics Models]
        PnP --> PoseFilt[PoseFilter: EMA / 1-Euro & SLERP]
    end

    subgraph Kinematics ["2. Multi-Robot Kinematics Layer"]
        PoseFilt --> SE3["SE(3) Coordinate Transformation"]
        SE3 --> WSMapper[Workspace Mapper & Bounds Enforcer]
        WSMapper --> SlewRate[Time-Based Slew Rate Limiter]
        SlewRate --> Router{Controller Type}
        Router -->|Position Control| IKSolver[Generic Damped Least-Squares IK]
        Router -->|Velocity Control| DiffIK[Resolved-Rate Cartesian Velocity Controller]
    end

    subgraph Planning ["3. Trajectory & Motion Planning"]
        IKSolver --> ColCheck[CollisionChecker: Self & Environment]
        DiffIK --> ColCheck
        ColCheck -->|Path Blocked| RRT[Bidirectional RRT-Connect Planner]
        RRT --> Shortcut[Randomized Path Shortcutting]
        Shortcut --> TrajGen[Quintic Joint / Cartesian SE(3) Trajectory]
        ColCheck -->|Path Free| TrajGen
    end

    subgraph Simulation ["4. PyBullet Multi-Robot Digital Twin"]
        TrajGen --> RobotCtrl[GenericRobotController: Panda / KUKA iiwa]
        RobotCtrl --> PhysStep[PyBullet Physics Stepping at 240 Hz]
        RobotCtrl --> GripperMgr[Virtual Gripper Manager (if supported)]
    end

    subgraph Monitoring ["5. Telemetry & Analytics"]
        RobotCtrl --> Jacob[Geometric Jacobian & SVD Manipulability]
        Jacob --> HUD[Live OpenCV Heads-Up Display HUD]
        PhysStep --> TrajViz[3D Trajectory Debug Lines]
    end
```

---

## 2. Multi-Robot Abstraction & Registry

### 2.1 Model Registry Pattern
Manipulator models are decoupled from control logic through `RobotModelSpec` and `RobotRegistry`:
- `RobotCapabilities`: Immutable capability descriptor (`has_gripper`, `supports_pick_place`, `supports_velocity_control`, `supports_self_collision`).
- `RobotModelSpec`: Strongly typed metadata containing URDF path, base pose, joint name regex patterns, end-effector link candidates, joint limit overrides, and default home posture.
- `RobotRegistry`: Thread-safe registry providing dynamic lookup, validation, and adapter factory methods.

### 2.2 Supported Robot Specifications

| Specification | Franka Emika Panda | KUKA LBR iiwa |
| :--- | :--- | :--- |
| **Model ID** | `panda` | `kuka_iiwa` |
| **Arm DoF** | 7 Revolute Joints | 7 Revolute Joints |
| **URDF File** | `franka_panda/panda.urdf` | `kuka_iiwa/model.urdf` |
| **End-Effector Link** | `panda_grasptarget` (Link 11) | `lbr_iiwa_link_7` (Link 6) |
| **Controllable Arm Joints** | `[0, 1, 2, 3, 4, 5, 6]` | `[0, 1, 2, 3, 4, 5, 6]` |
| **Gripper Joints** | `[9, 10]` (`panda_finger_joint1/2`) | None (Flange Mount Only) |
| **Spherical Reach** | $0.855\text{ m}$ | $0.820\text{ m}$ |

---

## 3. Kinematics, Control, & Differential Solvers

### 3.1 Geometric Jacobian Calculation
For an $n$-DoF manipulator, the geometric Jacobian $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times n}$ maps joint velocities $\dot{\mathbf{q}}$ to the end-effector Cartesian spatial twist $\mathbf{v} = [\mathbf{v}_{\text{lin}}^T, \boldsymbol{\omega}_{\text{ang}}^T]^T$:

$$\mathbf{v} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}} = \begin{bmatrix} \mathbf{J}_{\text{linear}}(\mathbf{q}) \\ \mathbf{J}_{\text{angular}}(\mathbf{q}) \end{bmatrix} \dot{\mathbf{q}}$$

### 3.2 Yoshikawa Manipulability & Singularity Telemetry
Manipulability is evaluated via Singular Value Decomposition (SVD) of $\mathbf{J} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T$:
- **Yoshikawa Index**: $w(\mathbf{q}) = \sqrt{\det(\mathbf{J} \mathbf{J}^T)} = \prod_{i=1}^6 \sigma_i$
- **Condition Number**: $\kappa(\mathbf{J}) = \frac{\sigma_{\max}}{\sigma_{\min}}$
- **Singularity Warning State**: Triggered when $\sigma_{\min} < \sigma_{\text{threshold}}$ (default $0.05$).

### 3.3 Adaptive DLS Resolved-Rate Cartesian Controller
To track Cartesian velocity demands while avoiding infinite joint speeds near singularities, the controller computes the Damped Least-Squares (DLS) pseudoinverse:

$$\mathbf{J}_{\text{dls}} = \mathbf{J}^T \left( \mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I}_6 \right)^{-1}$$

The damping factor $\lambda(\sigma_{\min})$ adapts continuously:

$$\lambda = \begin{cases}
\lambda_{\min} & \text{if } \sigma_{\min} \ge \sigma_{\text{threshold}} \\
\sqrt{\lambda_{\min}^2 + (1 - (\sigma_{\min}/\sigma_{\text{threshold}})^2)(\lambda_{\max}^2 - \lambda_{\min}^2)} & \text{if } \sigma_{\min} < \sigma_{\text{threshold}}
\end{cases}$$

### 3.4 Null-Space Joint Centering
For redundant 7-DoF manipulators ($n > 6$), secondary joint centering projects the gradient of a joint-centering potential $H(\mathbf{q})$ onto the Jacobian null space:

$$\dot{\mathbf{q}} = \mathbf{J}_{\text{dls}} \mathbf{v}_{\text{task}} + (\mathbf{I}_n - \mathbf{J}_{\text{dls}} \mathbf{J}) \left( -k_{\text{null}} \nabla H(\mathbf{q}) \right)$$

Where $H(\mathbf{q}) = \sum_{i=1}^n \left( \frac{q_i - q_{\text{rest}, i}}{q_{\text{high}, i} - q_{\text{low}, i}} \right)^2$.

---

## 4. Trajectory Generation & Motion Planning

### 4.1 Joint-Space Quintic Polynomials
Trajectories between configurations $\mathbf{q}_0$ and $\mathbf{q}_1$ over duration $T$ enforce $C^2$ continuity:
- Boundary conditions: $\mathbf{q}(0) = \mathbf{q}_0$, $\mathbf{q}(T) = \mathbf{q}_1$, $\dot{\mathbf{q}}(0) = \dot{\mathbf{q}}(T) = \mathbf{0}$, $\ddot{\mathbf{q}}(0) = \ddot{\mathbf{q}}(T) = \mathbf{0}$.
- Evaluator generates smooth, bounded positions, velocities, and accelerations at arbitrary time $t \in [0, T]$.

### 4.2 Cartesian SE(3) Trajectory (Quintic Position + Quaternion SLERP)
- Position: Interpolated via 3-axis quintic polynomial.
- Orientation: Interpolated along the shortest geodesic arc on $SO(3)$ via Spherical Linear Interpolation (SLERP):
  $$\mathbf{q}(t) = \frac{\sin((1 - \alpha)\theta)}{\sin\theta} \mathbf{q}_0 + \frac{\sin(\alpha\theta)}{\sin\theta} \mathbf{q}_1, \quad \alpha = t / T$$

### 4.3 Collision-Aware RRT-Connect Planner
- **Direct Path Optimization**: Checks linear joint interpolation first. If collision-free, executes immediately.
- **Bidirectional RRT-Connect**: Grows two trees rooted at $\mathbf{q}_{\text{start}}$ and $\mathbf{q}_{\text{goal}}$ with goal bias $\beta = 0.05$ and step size $\Delta q = 0.10\text{ rad}$.
- **Randomized Shortcutting**: Post-processes the RRT path by randomly sampling non-adjacent waypoint pairs and connecting them directly if the line segment is collision-free.

---

## 5. Perception-Gated Autonomous State Machine

The FSM checks robot hardware capabilities at runtime:
- **Panda (Gripper Supported)**: Enables full 11-state autonomous Pick-and-Place sequence (`HOME` $\to$ `SEARCH` $\to$ `APPROACH` $\to$ `PICK` $\to$ `LIFT` $\to$ `MOVE_TO_PLACE` $\to$ `PLACE` $\to$ `RETURN_HOME`).
- **KUKA iiwa (No Gripper)**: Safely disables pick-and-place states, providing full 6-DoF manual tracking, waypoint positioning, and trajectory planning without runtime failures.
