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
q_indices = []
v_indices = []

for name in joint_names:
    joint_id = model.getJointId(name)

    q_index = model.joints[joint_id].idx_q
    v_index = model.joints[joint_id].idx_v

    q_indices.append(q_index)
    v_indices.append(v_index)

lower_position_limits = (
    model.lowerPositionLimit[q_indices]
)

upper_position_limits = (
    model.upperPositionLimit[q_indices]
)

   





# ============================================================
# 1. 计算初始足端位置
# ============================================================

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

initial_position = data.oMf[foot_id].translation.copy()

#让右前脚沿世界z方向向上移动5cm
desired_position = (
    initial_position + np.array([0.0, 0.0, 0.05])
    #initial_position + np.array([0.0, 0.0, 0.25])
)

# ============================================================
# 2. 迭代求解位置IK
# ============================================================

gain = 5.0
dt = 0.01
tolerance = 1e-5
max_iterations = 1000
max_joint_speed = 1.0

limit_active_count = 0
position_limit_active_count = 0

maximum_raw_joint_speed = 0.0
maximum_commanded_joint_speed = 0.0

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

    largest_joint_speed = np.max(
        np.abs(joint_velocity)
    )

    maximum_raw_joint_speed = max(
        maximum_raw_joint_speed,
        largest_joint_speed,
    )

    if largest_joint_speed > max_joint_speed:

        limit_active_count += 1

        scale = (
            max_joint_speed / largest_joint_speed
        )

        joint_velocity = (scale * joint_velocity)

    # 根据当前位置， 计算下一步允许的关节速度范围
    current_joint_positions = q[q_indices]

    minimum_velocity_from_position = (
        lower_position_limits - current_joint_positions
    )/dt

    maximum_velocity_from_position = (
        upper_position_limits - current_joint_positions
    )/dt


    #保存位置限制之前的速度，只用于统计
    joint_velocity_before_position_limit = (
        joint_velocity.copy()
    )
    #防止下一次积分后越过关节位置限制
    joint_velocity = np.clip(
        joint_velocity,
        minimum_velocity_from_position,
        maximum_velocity_from_position,
    )

    #检查这次迭代有没有出发位置限制
    if np.any(
        np.abs(
            joint_velocity - joint_velocity_before_position_limit
        ) > 1e-12
    ):
        position_limit_active_count += 1

    #此时的joint_velocity才是真正准备执行的速度
    maximum_commanded_joint_speed = max(
        maximum_commanded_joint_speed,
        np.max(np.abs(joint_velocity)),
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

for name, q_index in zip(
    joint_names,
    q_indices,
):
    print(name, q[q_index])

print("\nPosition-limited iterations:")
print(position_limit_active_count)

print("\nLower position limits [rad]:")
print(lower_position_limits)

print("\nUpper position limits [rad]:")
print(upper_position_limits)

print("\nFinal joint positions [rad]:")
print(q[q_indices])

print("\nNumber of velocity-limited iterations:")
print(limit_active_count)

print("\nMaximum raw joint speed [rad/s]:")
print(maximum_raw_joint_speed)

print("\nMaximum commanded joint speed [rad/s]:")
print(maximum_commanded_joint_speed)