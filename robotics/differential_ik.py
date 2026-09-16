"""Resolved-Rate Cartesian Velocity Controller with Null-Space Joint Centering.

Implements closed-loop differential inverse kinematics mapping Cartesian
tracking errors to joint velocity commands via adaptive Damped Least-Squares (DLS)
pseudoinverse and secondary null-space projection for 7-DoF redundant manipulators.
"""

from typing import List, Optional, Tuple, Union
import numpy as np

from robotics.robot_controller import GenericRobotController
from robotics.kinematics import compute_manipulability, compute_damped_pseudoinverse, ManipulabilityMetrics
from robotics.kinematics_provider import KinematicsProvider, PyBulletKinematicsProvider
from utils.logger import get_logger

logger = get_logger("Robotics.DifferentialIK")


class ResolvedRateController:
    """Resolved-Rate Cartesian Velocity Controller with Null-Space Redundancy Optimization."""

    def __init__(
        self,
        physics_client_id: Optional[int] = None,
        robot_controller: Optional[GenericRobotController] = None,
        kp_pos: float = 4.0,
        kp_orn: float = 2.5,
        max_joint_velocity_radps: Optional[float] = None,
        enable_nullspace: bool = True,
        nullspace_gain: float = 1.0,
        singularity_threshold: float = 0.05,
        kinematics_provider: Optional[KinematicsProvider] = None,
    ):
        self.client_id = int(physics_client_id) if physics_client_id is not None else -1
        self.controller = robot_controller
        self.kp_pos = float(kp_pos)
        self.kp_orn = float(kp_orn)
        self.max_joint_vel = (
            float(max_joint_velocity_radps)
            if max_joint_velocity_radps is not None
            else float(robot_controller.spec.max_joint_velocity_radps)
        )
        self.enable_nullspace = bool(enable_nullspace)
        self.nullspace_gain = float(nullspace_gain)
        self.singularity_threshold = float(singularity_threshold)

        if kinematics_provider is not None:
            self.kinematics_provider: KinematicsProvider = kinematics_provider
        elif hasattr(robot_controller, "kinematics_provider") and robot_controller.kinematics_provider is not None:
            self.kinematics_provider = robot_controller.kinematics_provider
        else:
            self.kinematics_provider = PyBulletKinematicsProvider(
                physics_client_id=self.client_id,
                robot_body_id=robot_controller.robot_id,
                arm_joint_indices=robot_controller.arm_joint_indices,
                end_effector_link_index=robot_controller.ee_link_index,
            )

        lows, highs, ranges, rests = self.controller.get_joint_limits()
        self.lower_limits = np.array(lows, dtype=np.float64)
        self.upper_limits = np.array(highs, dtype=np.float64)
        self.joint_ranges = np.array(ranges, dtype=np.float64)
        self.rest_poses = np.array(rests, dtype=np.float64)

    def reset(self) -> None:
        """Resets internal controller state."""
        pass

    def compute_step(
        self,
        target_position: Union[np.ndarray, List[float], Tuple[float, float, float]],
        target_orientation: Optional[Union[np.ndarray, List[float], Tuple[float, float, float, float]]] = None,
        dt: float = 1.0 / 240.0,
    ) -> Tuple[np.ndarray, ManipulabilityMetrics]:
        """Calculates joint velocity commands to drive end-effector toward Cartesian target.

        Args:
            target_position: Desired [x, y, z] target in robot base frame.
            target_orientation: Optional desired unit quaternion [x, y, z, w].
            dt: Control cycle delta time.

        Returns:
            (q_dot_command, manipulability_metrics)
        """
        n = len(self.controller.arm_joint_indices)
        target_p = np.asarray(target_position, dtype=np.float64).flatten()

        # NaN / Inf validation
        if len(target_p) != 3 or not np.all(np.isfinite(target_p)):
            zero_qdot = np.zeros(n, dtype=np.float64)
            dummy_metrics = ManipulabilityMetrics(0.0, float("inf"), 0.0, 0.0, True)
            return zero_qdot, dummy_metrics

        current_q = np.array(self.controller.get_current_joint_positions(), dtype=np.float64)
        current_p, current_orn = self.kinematics_provider.compute_fk(list(current_q))

        # 1. Position Error & Linear Velocity Feedback
        pos_error = target_p - current_p
        v_lin = self.kp_pos * pos_error

        # 2. Orientation Error & Angular Velocity Feedback
        if target_orientation is not None:
            target_q = np.asarray(target_orientation, dtype=np.float64).flatten()
            q_norm = np.linalg.norm(target_q)
            if q_norm > 1e-6:
                target_q = target_q / q_norm
            else:
                target_q = np.array([1.0, 0.0, 0.0, 0.0])

            # Quaternion error: e_rot = 2 * (q_target * q_current^-1).xyz
            q_curr_conj = np.array([-current_orn[0], -current_orn[1], -current_orn[2], current_orn[3]])
            w1, x1, y1, z1 = target_q[3], target_q[0], target_q[1], target_q[2]
            w2, x2, y2, z2 = q_curr_conj[3], q_curr_conj[0], q_curr_conj[1], q_curr_conj[2]
            err_w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            err_x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            err_y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            err_z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            
            # Shortest arc path
            sign = 1.0 if err_w >= 0.0 else -1.0
            e_rot = 2.0 * sign * np.array([err_x, err_y, err_z], dtype=np.float64)
            v_ang = self.kp_orn * e_rot
        else:
            v_ang = np.zeros(3, dtype=np.float64)

        # Form full 6-DoF spatial twist
        spatial_twist = np.concatenate([v_lin, v_ang])  # (6,)

        # 3. Compute Jacobian via KinematicsProvider
        _, _, J = self.kinematics_provider.compute_jacobian(list(current_q))

        metrics = compute_manipulability(J, singularity_threshold=self.singularity_threshold)

        # 4. Adaptive Damped Least-Squares Inverse
        J_dls = compute_damped_pseudoinverse(
            J,
            lambda_min=0.01,
            lambda_max=0.25,
            sigma_threshold=self.singularity_threshold,
        )  # (n, 6)

        # Primary task joint velocity
        q_dot_primary = J_dls @ spatial_twist  # (n,)

        # 5. Secondary Null-Space Joint Centering Objective (for redundant manipulators n >= 7)
        if self.enable_nullspace and n >= 7:
            # Objective: H(q) = sum( (q_i - q_rest_i) / range_i )^2
            grad_H = 2.0 * (current_q - self.rest_poses) / (np.maximum(self.joint_ranges, 1e-4) ** 2)
            q_dot_null = -self.nullspace_gain * grad_H

            # Null-space projector: (I - J_dls @ J)
            null_projector = np.eye(n) - (J_dls @ J)
            q_dot_secondary = null_projector @ q_dot_null
            q_dot_total = q_dot_primary + q_dot_secondary
        else:
            q_dot_total = q_dot_primary

        # 6. Joint Limit Boundary Damping & Velocity Clamping
        # Slow down joints moving toward hard limits
        for i in range(n):
            dist_to_low = current_q[i] - self.lower_limits[i]
            dist_to_high = self.upper_limits[i] - current_q[i]
            buffer = 0.08  # 0.08 rad buffer zone

            if dist_to_low < buffer and q_dot_total[i] < 0:
                q_dot_total[i] *= max(0.0, dist_to_low / buffer)
            elif dist_to_high < buffer and q_dot_total[i] > 0:
                q_dot_total[i] *= max(0.0, dist_to_high / buffer)

        # Absolute velocity limit clamping
        max_vel = float(self.max_joint_vel)
        q_dot_clamped = np.clip(q_dot_total, -max_vel, max_vel)

        return q_dot_clamped, metrics
