"""Kinematic references for a fixed-foot squat (before an MPC is connected)."""

import numpy as np
import pinocchio as pin

from go2_control.model import GO2_FOOT_FRAMES


def squat_height_reference(
    time, standing_height=0.28, depth=0.04, period=4.0, settling_duration=2.0,
):
    """Return height, vertical velocity and acceleration in world coordinates.

    The hold-to-cosine transition is continuous in position and velocity;
    acceleration starts with a finite step at settling_duration.
    """
    if period <= 0 or depth < 0 or settling_duration < 0:
        raise ValueError("Expected period > 0, depth >= 0 and settling_duration >= 0")
    if time < settling_duration:
        return float(standing_height), 0.0, 0.0
    omega = 2.0 * np.pi / period
    phase = omega * (time - settling_duration)
    height = standing_height - 0.5 * depth * (1.0 - np.cos(phase))
    velocity = -0.5 * depth * omega * np.sin(phase)
    acceleration = -0.5 * depth * omega**2 * np.cos(phase)
    return float(height), float(velocity), float(acceleration)


class FixedFootReference:
    """Solve joint q_des and v_des for a base trajectory and fixed world feet.

    This Go2 helper expects a free flyer followed by twelve scalar joints.
    It uses reference kinematics, not measured joint positions, as the IK seed.
    """

    def __init__(self, model, initial_q, foot_positions_world):
        if model.nq != 19 or model.nv != 18:
            raise ValueError("Expected Go2 free-flyer model with nq=19, nv=18")
        self.model = model
        self.data = model.createData()
        self.q = np.array(initial_q, dtype=float, copy=True)
        self.feet = np.array(foot_positions_world, dtype=float, copy=True)
        if self.q.shape != (19,) or self.feet.shape != (4, 3):
            raise ValueError("Expected initial_q (19,) and foot_positions_world (4, 3)")
        self.frame_ids = []
        for name in GO2_FOOT_FRAMES:
            if not model.existFrame(name):
                raise KeyError(f"Foot frame not found: {name}")
            self.frame_ids.append(model.getFrameId(name))
        self.position_tolerance = 1e-9
        self.maximum_iterations = 25

    def solve(
        self, base_position, base_rotation, linear_velocity_world,
        angular_velocity_local=None,
    ):
        q = self.q.copy()
        q[:3] = base_position
        q[3:7] = pin.Quaternion(base_rotation).coeffs()

        # IK: J_j delta_q_j = p_foot_des - p_foot(q_des).
        for _ in range(self.maximum_iterations):
            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            positions = np.array([
                self.data.oMf[frame_id].translation.copy()
                for frame_id in self.frame_ids
            ])
            error = (self.feet - positions).ravel()
            jacobian = np.vstack([
                pin.getFrameJacobian(
                    self.model, self.data, frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )[:3, :].copy()
                for frame_id in self.frame_ids
            ])
            if np.linalg.norm(error) < self.position_tolerance:
                break
            delta = np.zeros(self.model.nv)
            delta[6:] = np.linalg.solve(jacobian[:, 6:], error)
            delta[6:] *= min(1.0, 0.2 / max(np.max(np.abs(delta[6:])), 1e-12))
            q = pin.integrate(self.model, q, delta)
            q[7:] = np.clip(
                q[7:], self.model.lowerPositionLimit[7:],
                self.model.upperPositionLimit[7:],
            )
        else:
            raise RuntimeError("Fixed-foot IK did not converge within joint limits")

        # Velocity reference: J_b v_b_des + J_j v_j_des = 0.
        v = np.zeros(self.model.nv)
        v[:3] = np.asarray(base_rotation).T @ linear_velocity_world
        if angular_velocity_local is not None:
            v[3:6] = angular_velocity_local
        v[6:] = np.linalg.solve(jacobian[:, 6:], -jacobian[:, :6] @ v[:6])
        if not np.isfinite(q).all() or not np.isfinite(v).all():
            raise RuntimeError("Non-finite joint reference")
        if np.any(np.abs(v[6:]) > self.model.velocityLimit[6:]):
            raise RuntimeError("Joint reference exceeds velocity limits")
        self.q = q.copy()
        return q, v


def joint_pd_feedback(model, q, v, desired_q, desired_v, kp, kd):
    """Return twelve feedback torques in Pinocchio v[6:] order."""
    position_error = pin.difference(model, q, desired_q)[6:]
    velocity_error = desired_v[6:] - v[6:]
    return kp * position_error + kd * velocity_error
