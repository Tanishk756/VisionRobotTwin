# Contributing to VisionRobotTwin

Thank you for your interest in contributing to **VisionRobotTwin**! We welcome improvements to kinematics, computer vision, simulation fidelity, ROS2 middleware backends, safety supervisors, and documentation.

---

## 🛠️ Development & Branching Workflow

VisionRobotTwin uses a structured branching model:
- **`main`**: Stable integrated checkpoint branch.
- **`develop/v1.3.0`**: Active development branch for the v1.3 milestone.

### Standard Contribution Steps
1. **Fork and Clone**:
   ```powershell
   git clone https://github.com/YOUR_USERNAME/VisionRobotTwin.git
   cd VisionRobotTwin
   ```

2. **Branch from `develop/v1.3.0`**:
   ```powershell
   git checkout develop/v1.3.0
   git pull origin develop/v1.3.0
   git checkout -b feature/your-feature-name
   ```

3. **Set Up Environment**:
   ```powershell
   setup.bat
   ```

4. **Implement Changes & Tests**:
   - Write clean, type-annotated Python code following the repository architecture.
   - Add unit and contract tests under `tests/` covering any new kinematics, CV, safety, or backend logic.

5. **Verify Locally**:
   ```powershell
   # Run automated test suite
   python -m pytest -q

   # Check bytecode compilation
   python -m compileall .

   # Run bounded headless synthetic smoke test
   python main.py --synthetic --headless --max-frames 120
   ```
   *Note*: ROS2-specific middleware and `ros2_control` E2E tests require Ubuntu 22.04 with ROS2 Humble Hawksbill installed.

6. **Submit a Pull Request**:
   - Push your branch and open a PR against `develop/v1.3.0` (unless maintainers instruct a direct PR to `main` for integration checkpoints).
   - Provide a clear summary of changes, validation evidence, and test results.
   - Do not force-push to shared branches.

---

## 📐 Engineering Principles & Safety Rules

- **Separation of Concerns**: Keep execution backends (`RobotBackend`), kinematic modeling (`KinematicsProvider`), collision queries (`CollisionProvider`), and model metadata (`ResolvedRobotModel`) decoupled.
- **Defensive Non-Safety-Rated Software**: All software guards, limit checkers, and watchdogs are non-safety-rated defensive software layers only.
- **Physical vs Synthetic Distinction**: Explicitly distinguish synthetic PyBullet/ROS2 simulation results from physical hardware measurements. Never claim physical hardware validation without verified empirical evidence.
- **Zero Credentials / Secrets Policy**: Never commit passwords, tokens, API keys, private robot network IP addresses, or proprietary hardware secrets.
- **Platform Compatibility**: Ensure all core robotics and vision modules import cleanly on Windows without mandatory ROS or vendor SDK dependencies.

