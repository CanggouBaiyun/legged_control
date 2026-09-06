import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    nominal_configuration,
)

model = build_go2_model()
q = nominal_configuration(model)

data = model.createData()

foot_id = model.getFrameId("FR_foot")

joint_names = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
]

v_indices = []

for name in joint_names:
    joint_id = model.getJointId(name)
    v_indices.append(model.joints[joint_id].idx_v)


# ============================================================
# 1. 计算初始足端位置
# ============================================================

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

initial_position = data.oMf[foot_id].translation.copy()

#让右前脚沿世界z方向向上移动5cm
desired_position = (
    initial_position + np.array([0.0, 0.0, 0.05])
)

# ============================================================
# 2. 迭代求解位置IK
# ============================================================

gain = 5.0
dt = 0.01
tolerance = 1e-5
max_iterations = 1000

converged = False

for iteration in range(max_iterations):

    #当前构型下的足端位置
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    current_position = (
        data.oMf[foot_id].translation.copy()
    )

    #位置误差
    position_error = (
        desired_position - current_position
    )

    error_norm = np.linalg.norm(position_error)

    if iteration % 20 == 0 :
        print(
            f"iteration={iteration:4d}, "
            f"error={error_norm:.8f}"
        )

    if error_norm < tolerance:
        converged = True
        break

    #当前构型下的足端Jacobian
    jacobian_6d =pin.computeFrameJacobian(
        model,
        data,
        q,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )

    jacobian_linear = jacobian_6d[:3, :]
    jacobian_leg = jacobian_linear[:, v_indices]

    #把位置误差转换成期望足端速度
    desired_velocity = gain * position_error

    #速度逆运动学
    damping = 1e-3

    matrix = (
        jacobian_leg @ jacobian_leg.T +damping**2 * np.eye(3)
    )
    joint_velocity = (
        jacobian_leg.T @ np.linalg.solve(matrix, desired_velocity)
    )

    #放回完整18维广义速度
    v = np.zeros(model.nv)

    for index, velocity in zip(
        v_indices,
        joint_velocity,
    ):
        v[index] = velocity

    #积分到下一构型
    q = pin.integrate(
        model,
        q,
        v * dt
    )

# ============================================================
# 3. 输出最终结果
# ============================================================

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

final_position = data.oMf[foot_id].translation.copy()
final_error = desired_position - final_position

print("\nConverged:")
print(converged)

print("\nIterations:")
print(iteration)

print("\nInitial foot position [m]:")
print(initial_position)

print("\nDesired foot position [m]:")
print(desired_position)

print("\nFinal foot position [m]:")
print(final_position)

print("\nFinal position error [m]:")
print(final_error)

print("\nFinal error norm:")
print(np.linalg.norm(final_error))

print("\nFinal FR joint angles [rad]:")

for name in joint_names:
    joint_id = model.getJointId(name)
    q_index = model.joints[joint_id].idx_q
    print(name, q[q_index])