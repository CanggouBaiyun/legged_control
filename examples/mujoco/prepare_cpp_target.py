"""Prepare existing Python IK targets and mapping checks for native C++ control.

This script runs once before simulation; it is absent from the control loop.
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.centroidal import (
    get_joint_indices, pinocchio_from_state, state_from_pinocchio,
)
from go2_control.model import build_go2_simulation_model
from go2_control.mpc_reference import OCS2_CONTACT_ORDER, OCS2_MODE_CONTACTS
from go2_control.simulation.mujoco_bridge import MujocoPinocchioBridge
from go2_control.wbc.reference import FixedFootReference
from ocs2_wbc_standing import (
    forward_motion_goal, standing_height_goal, turn_yaw_goal, mpc_foot_references,
)


def prepare(output, forward=False, turn_angle_deg=0.0):
    root = Path(__file__).resolve().parents[2]
    mj = mujoco.MjModel.from_xml_path(str(
        root / "third_party/unitree_mujoco/unitree_robots/go2/scene.xml"
    ))
    md = mujoco.MjData(mj)
    mujoco.mj_resetDataKeyframe(mj, md, mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_KEY, "home"))
    mujoco.mj_forward(mj, md)
    model = build_go2_simulation_model()
    data = model.createData()
    q, v = MujocoPinocchioBridge(mj, model).state(md)
    qi, vi = get_joint_indices(model)
    joint_names = [str(model.names[j]) for j in range(1, model.njoints)
                   if model.joints[j].nq == 1]
    joint_names.sort(key=lambda name: model.joints[model.getJointId(name)].idx_q)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    # FixedFootReference uses the SDK / WBC foot order.
    feet_names = ("FR_foot", "FL_foot", "RR_foot", "RL_foot")
    feet = np.array([data.oMf[model.getFrameId(name)].translation for name in feet_names])
    feet[:, 2] = 0.022
    fixed = FixedFootReference(model, q, feet)
    rotation = pin.XYZQUATToSE3(q[:7]).rotation
    times = np.linspace(0.0, 20.0, 401)
    states = []
    for stamp in times:
        displacement, speed = forward_motion_goal(stamp) if forward else (0.0, 0.0)
        yaw, rate = turn_yaw_goal(stamp, turn_angle_deg)
        position = q[:3].copy()
        position[0] += displacement
        position[2] = standing_height_goal(stamp)
        rz = pin.rpy.rpyToMatrix(np.array([0.0, 0.0, yaw]))
        target_rotation = rz @ rotation
        nominal_feet = feet.copy()
        nominal_feet[:, :2] = position[:2] + (rz[:2, :2] @ (feet[:, :2] - q[:2]).T).T
        omega = np.array([0.0, 0.0, rate])
        foot_velocity = np.tile([speed, 0.0, 0.0], (4, 1))
        foot_velocity += np.cross(omega, nominal_feet-position)
        qr, vr = fixed.solve(
            position, target_rotation, np.array([speed, 0.0, 0.0]),
            angular_velocity_local=target_rotation.T @ omega,
            foot_positions_world=nominal_feet,
            foot_velocities_world=foot_velocity,
        )
        states.append(state_from_pinocchio(model, data, qr, vr).tolist())

    def pack(point_q, point_v):
        return {
            "base_q": point_q[:7].tolist(), "base_v": point_v[:6].tolist(),
            "joint_q": point_q[qi].tolist(), "joint_v": point_v[vi].tolist(),
        }

    checks = []
    for index, (mode, future_mode) in enumerate(((15, 15), (9, 9), (6, 15))):
        qc = q.copy()
        qc[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0.04, -0.05, 0.1]))).coeffs()
        vc = np.linspace(-0.1, 0.12, 18)
        qf = qc.copy()
        qf[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0.0401, -0.0502, 0.1003]))).coeffs()
        qf[qi] += 0.003
        vf = vc + np.linspace(-0.005, 0.006, 18)
        state = state_from_pinocchio(model, data, qc, vc)
        future_state = state_from_pinocchio(model, data, qf, vf)
        control = np.zeros(24)
        control[:12] = np.arange(12) + 1.0
        control[12:] = vc[vi]
        future_input = control.copy()
        future_input[12:] = vf[vi]
        # Reconstruct exactly as in the running Python controller.
        qr, vr = pinocchio_from_state(model, data, state, control[12:])
        qfr, vfr = pinocchio_from_state(model, data, future_state, future_input[12:])
        current_feet = mpc_foot_references(model, data, state, control)
        future_feet = mpc_foot_references(model, data, future_state, future_input)
        rot = pin.XYZQUATToSE3(qr[:7]).rotation
        frot = pin.XYZQUATToSE3(qfr[:7]).rotation
        measured_rot = pin.XYZQUATToSE3(q[:7]).rotation
        swings = {}
        for name in OCS2_CONTACT_ORDER:
            if name in OCS2_MODE_CONTACTS[mode]:
                continue
            p, velocity = current_feet[name]
            acc = np.zeros(3)
            if name not in OCS2_MODE_CONTACTS[future_mode]:
                acc = (future_feet[name][1]-velocity)/0.005
            swings[name] = {"position": p.tolist(), "velocity": velocity.tolist(), "acceleration": acc.tolist()}
        checks.append({
            "name": f"mode_{mode}_future_{future_mode}",
            "mode": mode, "future_mode": future_mode,
            "measured": pack(q, v), "point": pack(qr, vr),
            "state": state.tolist(), "input": control.tolist(),
            "future_state": future_state.tolist(), "future_input": future_input.tolist(),
            "linear_velocity_world": (rot @ vr[:3]).tolist(),
            "angular_velocity_local": (measured_rot.T @ rot @ vr[3:6]).tolist(),
            "linear_acceleration_world": ((frot @ vfr[:3]-rot @ vr[:3])/0.005).tolist(),
            "angular_acceleration_local": (measured_rot.T @ (frot @ vfr[3:6]-rot @ vr[3:6])/0.005).tolist(),
            "contact_forces": [control[3*OCS2_CONTACT_ORDER.index(name):3*OCS2_CONTACT_ORDER.index(name)+3].tolist()
                               for name in OCS2_MODE_CONTACTS[mode]],
            "swings": swings,
        })
    payload = {"schema": 1, "joint_names": joint_names, "initial": pack(q, v),
               "times": times.tolist(), "states": states,
               "forward": forward, "turn_angle_deg": turn_angle_deg, "checks": checks}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, allow_nan=False, indent=2) + "\n")
    print(f"Prepared {len(states)} target points and {len(checks)} mapping checks: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--forward", action="store_true")
    parser.add_argument("--turn-angle-deg", type=float, default=0.0)
    args = parser.parse_args()
    if not np.isfinite(args.turn_angle_deg):
        parser.error("turn angle must be finite")
    prepare(args.output, args.forward, args.turn_angle_deg)
