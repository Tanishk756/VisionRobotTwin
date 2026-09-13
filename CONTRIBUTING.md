# Contributing to VisionRobotTwin

Thank you for your interest in contributing to **VisionRobotTwin**! We welcome improvements to kinematics, computer vision, simulation fidelity, filtering algorithms, and documentation.

---

## 🛠️ Development Workflow

1. **Fork and Clone**:
   ```powershell
   git clone https://github.com/YOUR_USERNAME/VisionRobotTwin.git
   cd VisionRobotTwin
   ```

2. **Set Up Environment**:
   ```powershell
   setup.bat
   ```

3. **Create a Feature Branch**:
   ```powershell
   git checkout -b feature/your-feature-name
   ```

4. **Implement Changes & Tests**:
   - Write clean, type-annotated Python code.
   - Add unit/integration tests under `tests/` covering any new kinematics, CV, or control logic.

5. **Verify Locally**:
   ```powershell
   # Run automated test suite
   pytest -v

   # Check bytecode compilation
   python -m compileall .

   # Run bounded headless smoke test
   python main.py --synthetic --headless --max-frames 120
   ```

6. **Submit a Pull Request**:
   - Push your branch and open a PR against `main`.
   - Provide a clear summary of changes and validation evidence.

---

## 📐 Engineering Principles

- **Separation of Concerns**: Keep rigid $SE(3)$ frame transformations mathematically distinct from user workspace scaling.
- **Evidence-Grounded Metrics**: Never commit fabricated FPS, millimeter tracking accuracy, or benchmark claims.
- **Physical vs Synthetic Distinction**: Explicitly distinguish synthetic PyBullet simulation results from physical hardware measurements.
- **Platform Compatibility**: Ensure all scripts and paths work cleanly on Windows environments.
- **Safety First**: Verify joint limits, workspace bounds, and numerical safety (NaN/Inf sanitization) on all motor commands.
