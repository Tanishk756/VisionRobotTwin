# Security & Simulation Safety Policy

## Reporting Security or Safety Vulnerabilities

If you discover a security vulnerability or safety-critical defect in VisionRobotTwin, please report it privately by contacting the maintainer:

- **Contact**: Tanishk Singhal
- **Email**: [tanisksinghal6285@gmail.com](mailto:tanisksinghal6285@gmail.com)

Please do not open public issues for security vulnerabilities until a fix or advisory has been coordinated.

---

## ⚠️ Robotics & Physical Safety Advisory

1. **Simulation Scope**: **VisionRobotTwin v1.1.0** is designed, tested, and validated as a **software simulation and digital twin system** in PyBullet.
2. **Physical Robot Disclaimer**: This software must **not** be assumed safe for direct, unmediated control of physical industrial robot manipulators (including real Franka Emika Panda arms) without independent hardware safety interlocks, emergency stop (E-Stop) circuitry, collision-avoidance verification, and formal compliance certification.
3. **Sensor Limitations**: Monocular optical pose estimation is subject to occlusion, lighting variations, and calibration drift. High-consequence physical robotic deployments require redundant hardware sensing and certified safety controllers.
