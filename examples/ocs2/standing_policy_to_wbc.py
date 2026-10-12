"""Evaluate WBC at a captured MPC trajectory knot; no actuation."""

import json
from pathlib import Path

import numpy as np

from go2_control.centroidal import pinocchio_from_state
from go2_control.model import build_go2_simulation_model
from go2_control.mpc_reference import standing_reference_from_mpc
from go2_control.wbc import StandingWBC

def main():
    snapshot = json.loads(
        Path("/tmp/go2_standing_policy.json").read_text(
            encoding="utf-8"
        )
    )

    state = np.asarray(snapshot["state"], dtype=float)
    control = np.asarray(snapshot["input"], dtype=float)

    if(
        state.shape != (24, )
        or control.shape != (24, )
        or not np.all(np.isfinite(state))
        or not np.all(np.isfinite(control))
    ):
        raise ValueError("Invalid MPC snapshot")

    model = build_go2_simulation_model()
    controller = StandingWBC(model)

    # 1. 从 MPC 轨迹点恢复构型和广义速度。
    # 这里是预测状态，不是 MuJoCo 实测状态
    q, v = pinocchio_from_state(
        model,
        model.createData(),
        state,
        control[12:],
    )

    # 2. 将 MPC 参考转换成 WBC 接口需要的格式
    reference = standing_reference_from_mpc(
        model,
        model.createData(),
        q,
        state,
        control,
        controller.contact_frame_names,
        mode=snapshot["mode"],
    )

    # 3. WBC 联合求解加速度、接触力和关节力矩
    result = controller.solve(
        q = q,
        v = v,
        **reference,
    )

    if not np.all(np.isfinite(result.motor_torques)):
        raise RuntimeError("WBC produced non-finite torques")

    np.set_printoptions(precision=6, suppress=True)

    print(f"Policy knot time: {snapshot['time']:.4f} s")
    print(f"WBC status: {result.solver_status}")

    print("\nNormal forces: MPC reference -> WBC result [N]")
    for name, expected, actual in zip(
        controller.contact_frame_names,
        reference["desired_contact_forces"],
        result.contact_forces,
    ):
        print(
            f"{name:10s}: "
            f"{expected[2]:9.4f} -> {actual[2]:9.4f}"
        )

    print("\nMaximum absolute motor torque [Nm]:")
    print(np.max(np.abs(result.motor_torques)))

    print("\nDynamics residual norm:")
    print(result.dynamics_residual_norm)

    print("\nContact acceleration residual norm:")
    print(result.contact_acceleration_residual_norm)


if __name__ == "__main__":
    main()