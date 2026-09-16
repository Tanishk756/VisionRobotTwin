# Security & Robotics Safety Policy

## Reporting Security or Safety Vulnerabilities

If you discover a security vulnerability or safety-critical defect in VisionRobotTwin, please report it privately by contacting the maintainer:

- **Maintainer**: Tanishk Singhal
- **Email**: [tanisksinghal6285@gmail.com](mailto:tanisksinghal6285@gmail.com)

### What to Include in Your Report
To help us investigate and remediate the issue effectively, please include:
- A clear description of the vulnerability or defect.
- Steps to reproduce the issue, including minimal example scripts or configuration files.
- Affected application version or commit SHA.
- Potential security or physical safety implications.

Please do not open public issues or disclose exploitable vulnerabilities until a coordinated remediation has been deployed.

---

## ⚠️ Robotics & Physical Safety Advisory

1. **Non-Safety-Rated Software Disclaimer**:  
   All software safety supervisors (`GuardedRobotBackend`, `CommandSafetySupervisor`), monotonic watchdogs, velocity limiters, position jump guards, and software stop routines in VisionRobotTwin are **defensive software layers only and are strictly NON-SAFETY-RATED**. They do not constitute functional safety certification under ISO 13849-1, ISO 10218, or IEC 62061.

2. **Physical Robot Safety Requirements**:  
   This software must **never** be assumed sufficient on its own for the safe operation of physical industrial manipulators (e.g. Franka Emika Panda, KUKA LBR iiwa, Universal Robots). Physical deployments strictly require independent, hardwired physical safety systems, including:
   - Dedicated physical Emergency Stop (E-Stop) buttons and circuits.
   - Hardware-level Safe Torque Off (STO).
   - Physical barriers, safety fences, or certified optical light curtains.
   - Manufacturer-certified safety controllers.

3. **Perception & Teleoperation Limits**:  
   Monocular optical pose estimation and marker-based teleoperation are inherently subject to visual occlusion, lighting variations, camera vibration, and calibration inaccuracies. Physical applications require redundant multi-modal sensing and certified fault-tolerant safety monitoring.

