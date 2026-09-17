import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_simulation_model,
    nominal_configuration,
)
from go2_control.wbc.reference import FixedFootReference


model = build_go2_simulation_model()
data = model.createData()

q_initial = nominal_configuration(model)

swing_foot = "FR_foot"

stance_feet = [
    name 
    for name in GO2_FOOT_FRAMES
    if name != swing_foot
]

# 1. 保存初始四足位置，作为身体移动时的固定足端目标。
pin.forwardKinematics(model, data, q_initial)
pin.updateFramePlacements(model, data)

fixed_foot_positions = np.array([
    data.oMf[model.getFrameId(name)].translation.copy()
    for name in GO2_FOOT_FRAMES
])

initial_base_position = q_initial[:3].copy()
initial_base_rotation = pin.XYZQUATToSE3(
    q_initial[:7]
).rotation.copy()

# x负方向：向后；y正方向：向左
shifts = [
    (0.00, 0.00),
    (-0.01, 0.01),
    (-0.02, 0.02),
    (-0.03, 0.03),
]

print(
    " dx[m]  dy[m]   com_x[m]  com_y[m]"
    "    FL[N]    RR[N]    RL[N]"
    "   residual   tau_ratio"
)

for dx, dy in shifts:
    #每组试验都从同一个初始构型开始求 IK
    reference = FixedFootReference(
        model,
        q_initial,
        fixed_foot_positions,
    )

    desired_base_position = (
        initial_base_position + np.array([dx, dy, 0.0])
    )

    #2.身体移动，四足固定，求新的关节角度
    q_shifted, _ = reference.solve(
        desired_base_position,
        initial_base_rotation,
        np.zeros(3),
    )

    #3. 在新构型下，重新计算重力和 Jacobian
    gravity = pin.computeGeneralizedGravity(
        model,
        data,
        q_shifted,
     ).copy()

    pin.computeJointJacobians(
        model,
        data,
        q_shifted,
    )
    pin.updateFramePlacements(model, data)

    vertical_force_columns = []

    for foot_name in stance_feet:
        foot_id = model.getFrameId(foot_name)

        foot_jacobian = pin.getFrameJacobian(
            model,
            data,
            foot_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :].copy()

        vertical_force_columns.append(
            foot_jacobian.T[:, 2].copy()
        )

    force_mapping = np.column_stack(
        vertical_force_columns
    )

    # 4. 右前脚不出力，求其余三脚的静态竖直接触力。
    normal_forces = np.linalg.lstsq(
        force_mapping[:6, :],
        gravity[:6],
        rcond=None,
    )[0]

    generalized_contact_force = (
        force_mapping @ normal_forces
    )

    residual = np.linalg.norm(
        generalized_contact_force[:6] - gravity[:6]
    )

    motor_torques = (
        gravity[6:] - generalized_contact_force[6:]
    )

    torque_utilization = np.max(
        np.abs(motor_torques) / model.effortLimit[6:]
    )

    # 5. 看实际质心位置，而不只看基座位置。
    com = pin.centerOfMass(
        model,
        data,
        q_shifted,
    ).copy()

    print(
        f"{dx:6.3f} {dy:6.3f}"
        f" {com[0]:10.5f} {com[1]:9.5f}"
        f" {normal_forces[0]:8.3f}"
        f" {normal_forces[1]:8.3f}"
        f" {normal_forces[2]:8.3f}"
        f" {residual:10.2e}"
        f" {torque_utilization:11.3f}"
    )