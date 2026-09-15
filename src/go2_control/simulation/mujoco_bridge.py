from __future__ import annotations

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import (
    sdk_joint_mapping,
)

class MujocoPinocchioBridge:
    """Convert Go2 states and torques between MuJoCo and Pinocchio."""

    def __init__(
        self,
        mujoco_model: mujoco.MjModel,
        pinocchio_model: pin.Model,
    ):
        self.mujoco_model = mujoco_model
        self.pinocchio_model = pinocchio_model

        if mujoco_model.nq != 19:
            raise ValueError(
                f"Expected MuJoCo nq=19, got {mujoco_model.nq}"
            )

        if mujoco_model.nv != 18:
            raise ValueError(
                f"Expected MuJoCo nv=18, got {mujoco_model.nv}"
            )

        if mujoco_model.nu != 12:
            raise ValueError(
                f"Expected MuJoCo nu=12, got {mujoco_model.nu}"
            )

        if pinocchio_model.nq != 19:
            raise ValueError(
                f"Expected Pinocchio nq=19, got {pinocchio_model.nq}"
            )

        if pinocchio_model.nv != 18:
            raise ValueError(
                f"Expected Pinocchio nv=18, got {pinocchio_model.nv}"
            )

        joint_mapping = sdk_joint_mapping(
            pinocchio_model
        )

        mujoco_qpos_indices = []
        mujoco_qvel_indices = []
        pinocchio_q_indices = []
        pinocchio_v_indices = []

        for entry in joint_mapping:
            joint_name = entry["name"]

            mujoco_joint_id = mujoco.mj_name2id(
                mujoco_model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name,
            )

            if mujoco_joint_id == -1:
                raise KeyError(
                    f"MuJoCo joint not found: {joint_name}"
                )

            mujoco_qpos_indices.append(
                int(mujoco_model.jnt_qposadr[mujoco_joint_id])
            )

            mujoco_qvel_indices.append(
                int(mujoco_model.jnt_dofadr[mujoco_joint_id])
            )

            pinocchio_q_indices.append(
                int(entry["q_index"])
            )

            pinocchio_v_indices.append(
                int(entry["v_index"])
            )

        self.mujoco_qpos_indices = np.array(
            mujoco_qpos_indices,
            dtype=int,
        )

        self.mujoco_qvel_indices = np.array(
            mujoco_qvel_indices,
            dtype=int,
        )

        self.pinocchio_q_indices = np.array(
            pinocchio_q_indices,
            dtype=int,
        )

        self.pinocchio_v_indices = np.array(
            pinocchio_v_indices,
            dtype=int,
        )

        pinocchio_torque_index_by_name = {
            entry["name"]: int(entry["v_index"]) - 6
            for entry in joint_mapping
        }

        actuator_pinocchio_torque_indices = []

        for actuator_id in range(mujoco_model.nu):
            mujoco_joint_id = int(mujoco_model.actuator_trnid[
                actuator_id,
                0,
            ])

            joint_name = mujoco.mj_id2name(
                mujoco_model,
                mujoco.mjtObj.mjOBJ_JOINT,
                mujoco_joint_id,
            )

            if(joint_name not in pinocchio_torque_index_by_name):
                raise KeyError(
                    "MuJoCo actuator targets an "
                    f"unexpected joint: {joint_name}"
                )

            actuator_pinocchio_torque_indices.append(
                pinocchio_torque_index_by_name[joint_name]
            )

        self.actuator_pinocchio_torque_indices = (
            np.array(actuator_pinocchio_torque_indices,dtype=int)
        )

    def configuration(
        self,
        mujoco_data:mujoco.MjData,
    ) -> np.ndarray:
        """Convert MuJoCo qpos to Pinocchio q."""

        q = pin.neutral(self.pinocchio_model)
        q[:3] = mujoco_data.qpos[:3]

        # MuJoCo:    [w, x, y, z]
        # Pinocchio: [x, y, z, w]
        q[3:7] = np.array([
            mujoco_data.qpos[4],
            mujoco_data.qpos[5],
            mujoco_data.qpos[6],
            mujoco_data.qpos[3],
        ])

        q[self.pinocchio_q_indices] = mujoco_data.qpos[self.mujoco_qpos_indices]

        return q

    def velocity(
        self,
        mujoco_data: mujoco.MjData,
        q: np.ndarray | None = None,
    ) -> np.ndarray:
        """Convert MuJoCo qvel to Pinocchio v."""

        if q is None:
            q = self.configuration(mujoco_data)

        v = np.zeros(self.pinocchio_model.nv)

        world_rotation_base = (pin.XYZQUATToSE3(q[:7]).rotation)

        # MuJoCo linear velocity is expressed in world.
        # Pinocchio free-flyer velocity is expressed in body.
        v[:3] = (
            world_rotation_base.T @ mujoco_data.qvel[:3]
        )
        # Both angular velocities use the body frame.
        v[3:6] = mujoco_data.qvel[3:6]
        v[self.pinocchio_v_indices] = mujoco_data.qvel[self.mujoco_qvel_indices]

        return v

    def state(
        self,
        mujoco_data: mujoco.MjData,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return Pinocchio q and v from the same MuJoCo state."""

        q = self.configuration(mujoco_data)
        v = self.velocity(mujoco_data, q=q)

        return q,v

    def pinocchio_torques_to_mujoco_controls(
        self,
        pinocchio_motor_torques: np.ndarray,
    ) -> np.ndarray:
        """Convert Pinocchio v[6:] torque order to MuJoCo ctrl order."""

        pinocchio_motor_torques = np.asarray(
            pinocchio_motor_torques,
            dtype=float,
        )

        expected_shape = (
            self.pinocchio_model.nv - 6,
        )

        if (
            pinocchio_motor_torques.shape
            != expected_shape
        ):
            raise ValueError(
                "Expected motor torque shape "
                f"{expected_shape}, got "
                f"{pinocchio_motor_torques.shape}"
            )

        return pinocchio_motor_torques[self.actuator_pinocchio_torque_indices].copy()