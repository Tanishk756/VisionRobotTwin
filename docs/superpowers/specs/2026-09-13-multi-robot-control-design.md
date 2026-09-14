# Multi-Robot Control, Kinematics, and Motion Planning Architecture Design

**Document ID**: `SPEC-2026-09-13-MULTI-ROBOT-CONTROL`  
**Version**: `1.2.0-dev`  
**Target Release**: VisionRobotTwin v1.2.0  
**Theme**: Multi-Manipulator Kinematics, Differential Control, Trajectory Planning, Collision Avoidance, and Cross-Robot Benchmarking.

---

## 1. Executive Summary & Goals

VisionRobotTwin v1.0 and v1.1 established a robust, real-time ArUco vision-guided digital twin specifically for the 7-DoF Franka Emika Panda manipulator in PyBullet. Version 1.2 expands VisionRobotTwin into a **robot-agnostic, multi-manipulator robotics research and teleoperation platform**.

### Primary Objectives
1. **Generic Robot Abstraction**: Decouple the digital twin simulation, kinematics, and control loop from Panda-specific constants, URDFs, and joint topologies.
2. **Supported Multi-Robot Fleet**: Full support for both **Franka Emika Panda** and **KUKA LBR iiwa** (7-DoF revolute manipulators).
3. **Robot Capabilities Model**: Explicit modeling of end-effector capabilities (e.g., parallel jaw gripper on Panda vs. bare mounting flange on KUKA iiwa) preventing illegal states (such as grasping without a physical gripper).
4. **Kinematic & Differential Motion Control**:
   - Generic numerical and optimization-based Inverse Kinematics (IK).
   - Real-time Geometric Jacobian computation ($6 \times N$).
   - Yoshikawa manipulability index, condition number, and singular value analysis.
   - Resolved-rate Cartesian velocity control with adaptive Damped Least-Squares (DLS) pseudoinverse and null-space joint limit centering.
5. **Trajectory Generation & Smoothing**:
   - Quintic polynomial joint-space trajectory generation with $C^2$ boundary conditions ($\dot{q}(0)=\dot{q}(T)=0, \ddot{q}(0)=\ddot{q}(T)=0$).
   - Cartesian SE(3) trajectory interpolation with linear/quintic translation and quaternion spherical linear interpolation (SLERP).
6. **Collision Avoidance & Motion Planning**:
   - Discrete and continuous collision checking (robot self-collision, ground plane/table, dynamic obstacles) with explicit allowed-contact semantics.
   - Bidirectional RRT-Connect joint-space planner with adaptive shortcut smoothing.
7. **Rigorous Benchmarking & Telemetry**:
   - Cross-robot kinematic and planning performance benchmarking (`tools/compare_robots.py`).
   - Cross-controller tracking error and energy benchmarking (`tools/compare_controllers.py`).
   - Extended HUD and console telemetry.

---

## 2. Generic Robot Architecture & Model Registry

```
                    ┌────────────────────────┐
                    │     RobotRegistry      │
                    └───────────┬────────────┘
                                │ creates
                                ▼
                    ┌────────────────────────┐
                    │    RobotModelSpec      │
                    │  - robot_id            │
                    │  - urdf_path           │
                    │  - arm_joints          │
                    │  - ee_link_candidates  │
                    │  - capabilities        │
                    └───────────┬────────────┘
                                │ configures
                                ▼
                    ┌────────────────────────┐
                    │  GenericRobotController│
                    │  - Forward Kinematics  │
                    │  - Position/Vel Control│
                    │  - Joint Limit Gating  │
                    └─────┬────────────┬─────┘
                          │            │
             ┌────────────┴──┐      ┌──┴────────────┐
             ▼               ▼      ▼               ▼
     ┌──────────────┐ ┌──────────┐ ┌─────────┐ ┌──────────┐
     │ PandaAdapter │ │KukaAdapter│ │GenericIK│ │Jacobian/ │
     │ (w/ Gripper) │ │ (Flange) │ │ Solver  │ │Diff. IK  │
     └──────────────┘ └──────────┘ └─────────┘ └──────────┘
```

### 2.1 Robot Capabilities Structure
```python
@dataclass(frozen=True)
class RobotCapabilities:
    has_gripper: bool
    supports_pick_place: bool
    supports_velocity_control: bool
    supports_self_collision: bool
    max_payload_kg: float
```

### 2.2 Supported Robot Specifications
| Parameter | Franka Emika Panda | KUKA LBR iiwa 7 R800 / 14 R820 |
| :--- | :--- | :--- |
| **Robot ID** | `panda` | `kuka_iiwa` |
| **Arm DoF** | 7 revolute joints | 7 revolute joints |
| **URDF Source** | `franka_panda/panda.urdf` (pybullet_data) | `kuka_iiwa/model.urdf` (pybullet_data) |
| **End-Effector Link** | Link 11 (`panda_grasptarget` / `panda_hand`) | Link 6 (`lbr_iiwa_link_7` tool flange) |
| **Gripper Included** | Yes (2-finger parallel jaw prismatic) | No (bare tool flange) |
| **Pick & Place Capable**| Yes | No (requires custom gripper attachment) |
| **Home Pose (rad)** | `[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]` | `[0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]` |
| **Reach Sphere** | $0.855\text{ m}$ | $0.820\text{ m}$ |

---

## 3. Kinematics, Jacobian, and Differential Control

### 3.1 Generic Forward Kinematics (FK)
Given joint state vector $\mathbf{q} \in \mathbb{R}^n$, FK computes the transformation from the robot base frame to the designated end-effector link frame:
$$\mathbf{T}_{\text{base}\to\text{ee}}(\mathbf{q}) = \begin{bmatrix} \mathbf{R}(\mathbf{q}) & \mathbf{p}(\mathbf{q}) \\ \mathbf{0}_{1\times 3} & 1 \end{bmatrix} \in \text{SE}(3)$$

### 3.2 Generic Inverse Kinematics (IK)
Given target pose $(\mathbf{p}_{\text{target}}, \mathbf{q}_{\text{target}})$, PyBullet's damped least-squares numerical solver calculates $\mathbf{q} \in \mathbb{R}^n$ bounded strictly by $[\mathbf{q}_{\text{lower}}, \mathbf{q}_{\text{upper}}]$.

**IK Result Schema (`IKResult`)**:
- `success: bool`
- `joint_positions: List[float]`
- `status: IKStatus` (`SOLUTION_RETURNED`, `OUT_OF_LIMITS`, `UNREACHABLE`, `INVALID_TARGET`, `IK_ERROR`)
- `status_message: str`
- `position_error_m: float`
- `orientation_error_rad: float`
- `solve_time_ms: float`

### 3.3 Geometric Jacobian Computation
The spatial geometric Jacobian $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times n}$ maps joint velocities $\dot{\mathbf{q}}$ to the end-effector spatial twist $\mathbf{v} = [\mathbf{v}_{\text{lin}}^T, \boldsymbol{\omega}_{\text{ang}}^T]^T$:
$$\mathbf{v} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}} = \begin{bmatrix} \mathbf{J}_{\text{linear}}(\mathbf{q}) \\ \mathbf{J}_{\text{angular}}(\mathbf{q}) \end{bmatrix} \dot{\mathbf{q}}$$

### 3.4 Manipulability & Singularity Metrics
1. **Yoshikawa Manipulability**:
   $$w(\mathbf{q}) = \sqrt{\det\left(\mathbf{J}(\mathbf{q}) \mathbf{J}(\mathbf{q})^T\right)} = \prod_{i=1}^6 \sigma_i$$
2. **Condition Number**:
   $$\kappa(\mathbf{J}) = \frac{\sigma_{\max}(\mathbf{J})}{\sigma_{\min}(\mathbf{J})} \ge 1.0$$
3. **Singularity Classification**:
   - `NORMAL`: $\sigma_{\min} \ge \sigma_{\text{thresh}}$ (default $0.05$).
   - `WARNING / NEAR_SINGULARITY`: $\sigma_{\min} < \sigma_{\text{thresh}}$.

### 3.5 Adaptive Damped Least-Squares (DLS) & Resolved-Rate Control
To prevent joint velocity explosion near singular configurations ($\det(\mathbf{J}\mathbf{J}^T) \to 0$), the damped inverse is calculated as:
$$\mathbf{J}_{\text{dls}}^{\dagger} = \mathbf{J}^T \left(\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I}_{6 \times 6}\right)^{-1}$$

**Adaptive Damping Profile**:
$$\lambda(\sigma_{\min}) = \begin{cases} \lambda_{\min}, & \sigma_{\min} \ge \sigma_0 \\ \sqrt{\lambda_{\min}^2 + \left(1 - \left(\frac{\sigma_{\min}}{\sigma_0}\right)^2\right) (\lambda_{\max}^2 - \lambda_{\min}^2)}, & \sigma_{\min} < \sigma_0 \end{cases}$$

**Resolved-Rate Control Law with Null-Space Joint Limit Centering**:
For 7-DoF redundant manipulators ($n=7 > 6$), the joint velocity command is:
$$\dot{\mathbf{q}}_{\text{cmd}} = \mathbf{J}_{\text{dls}}^{\dagger} \mathbf{v}_{\text{twist}} + \left(\mathbf{I}_{n \times n} - \mathbf{J}_{\text{dls}}^{\dagger} \mathbf{J}\right) \dot{\mathbf{q}}_{\text{null}}$$
where:
$$\dot{\mathbf{q}}_{\text{null}} = -k_{\text{null}} \nabla H(\mathbf{q}), \quad H(\mathbf{q}) = \sum_{i=1}^n \left(\frac{q_i - q_{\text{rest}, i}}{q_{\text{upper}, i} - q_{\text{lower}, i}}\right)^2$$
and the Cartesian feedback twist is:
$$\mathbf{v}_{\text{twist}} = \begin{bmatrix} k_p (\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{ee}}) \\ k_o \mathbf{e}_{\text{rot}}(\mathbf{q}_{\text{target}}, \mathbf{q}_{\text{ee}}) \end{bmatrix}$$

---

## 4. Trajectory Generation & Motion Planning

### 4.1 Joint-Space Quintic Polynomial Trajectory
For each joint $j \in \{1,\dots,n\}$, a 5th-order polynomial $s(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5$ is solved for boundary conditions:
$$s(0) = q_{0, j}, \quad s(T) = q_{f, j}, \quad \dot{s}(0) = \dot{s}(T) = 0, \quad \ddot{s}(0) = \ddot{s}(T) = 0$$
$$\begin{aligned}
a_0 &= q_0, \quad a_1 = 0, \quad a_2 = 0 \\
a_3 &= \frac{10 (q_f - q_0)}{T^3}, \quad a_4 = \frac{-15 (q_f - q_0)}{T^4}, \quad a_5 = \frac{6 (q_f - q_0)}{T^5}
\end{aligned}$$

### 4.2 Cartesian SE(3) Trajectory
- Position $\mathbf{p}(t)$: Quintic or linear interpolation between $\mathbf{p}_0$ and $\mathbf{p}_f$.
- Orientation $\mathbf{q}(t)$: Unit quaternion SLERP:
$$\text{SLERP}(\mathbf{q}_0, \mathbf{q}_f, \tau) = \frac{\sin((1-\tau)\theta)}{\sin\theta} \mathbf{q}_0 + \frac{\sin(\tau \theta)}{\sin\theta} \mathbf{q}_f, \quad \cos\theta = \mathbf{q}_0 \cdot \mathbf{q}_f$$

### 4.3 Collision Checker & Allowed Contact Semantics
- Validates contact pairs between robot links and collision objects using PyBullet `getContactPoints` and `getClosestPoints`.
- **Allowed Contact Policy**: During object grasping, contact between finger tips and the target pick object is whitelisted and not flagged as a forbidden collision.

### 4.4 Bidirectional RRT-Connect Motion Planner
- **Algorithm**: Bidirectional trees rooted at $\mathbf{q}_{\text{start}}$ and $\mathbf{q}_{\text{goal}}$.
- **Steering Step**: $\Delta q = \min(\text{step\_size}, \|\mathbf{q}_{\text{rand}} - \mathbf{q}_{\text{near}}\|)$.
- **Collision Checking**: Interpolated checks along joint edges with resolution $\Delta \theta = 0.05\text{ rad}$.
- **Post-Processing Shortcutter**: Random edge replacement to reduce waypoints and overall joint path length without introducing collisions.

---

## 5. Telemetry & FSM Compatibility Matrix

### 5.1 FSM Robot Adaptation
- **Panda**: Full state machine (HOME $\to$ SEARCH $\to$ TRACK $\to$ APPROACH $\to$ GRASP $\to$ LIFT $\to$ TRANSPORT $\to$ PLACE $\to$ RETRACT).
- **KUKA iiwa**: Safe restriction (HOME $\to$ SEARCH $\to$ TRACK). If user initiates `--mode auto` on KUKA iiwa, CLI rejects execution with a clear error:
  `"Robot 'kuka_iiwa' does not provide a gripper; autonomous pick/place is unavailable."`

---

## 6. Verification Plan & Test Strategy

| Test Module | Coverage | Hardware Requirement |
| :--- | :--- | :--- |
| `test_robot_registry.py` | Registration, URDF discovery, capabilities, lookup errors | None (Synthetic) |
| `test_multi_robot_controller.py` | Joint limits, FK, joint reading/setting for Panda & KUKA | None (PyBullet DIRECT) |
| `test_multi_robot_ik.py` | Generic IK solver, tolerances, limits, reachability | None (PyBullet DIRECT) |
| `test_jacobian.py` | Matrix dimensions, finite values, numerical differential FK | None (PyBullet DIRECT) |
| `test_differential_ik.py` | Resolved-rate convergence, velocity limits, null-space projection | None (PyBullet DIRECT) |
| `test_manipulability.py` | Yoshikawa metric, condition number, singularity warnings | None (PyBullet DIRECT) |
| `test_trajectory.py` | Quintic boundary constraints, Cartesian SLERP normalization | None (Synthetic math) |
| `test_collision.py` | Free/blocked paths, allowed-contact policy, state restoration | None (PyBullet DIRECT) |
| `test_planning.py` | Direct path, RRT-Connect, obstacle bypass, deterministic seeds | None (PyBullet DIRECT) |
| `test_robot_benchmark.py` | Cross-robot benchmarking execution and JSON export | None (PyBullet DIRECT) |
| `test_controller_benchmark.py` | Cross-controller benchmarking execution and JSON export | None (PyBullet DIRECT) |

---

## 7. Limitations & Invariants
1. **Kinematic Reach Limits**: Franka Panda max reach $\approx 0.855\text{ m}$; KUKA iiwa max reach $\approx 0.820\text{ m}$.
2. **Gripper Isolation**: KUKA iiwa uses a bare tool flange; gripper operations are strictly prevented.
3. **No ROS/MoveIt Dependencies**: All algorithms run as pure, self-contained Python/PyBullet/NumPy implementations compatible with Windows and headless CI.
