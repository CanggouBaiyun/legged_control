"""Verify Go2 centroidal momentum under uniform translation."""

import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_simulation_model,
    nominal_configuration,
)

np.set_printoptions(precision=8, suppress=True)

# 1. 使用与 MuJoCo 同源的机器人模型。
model = build_go2_simulation_model()
data = model.createData()

q = nominal_configuration(model)
v = np.zeros(model.nv)

# 保证身体坐标轴与世界坐标轴对齐。
# Pinocchio 四元数顺序为 [x, y, z, w]。
q[3:7] = np.array([0.0, 0.0, 0.0, 1.0])

# 2. 基座速度保持为零，只指定右前大腿关节速度
joint_id = model.getJointId("FR_thigh_joint")
v_index = model.joints[joint_id].idx_v

v[v_index] = 1.0 # rad/s

total_mass = pin.computeTotalMass(model)

# 3. 计算世界坐标系下的质心位置和速度
com_position = pin.centerOfMass(
    model, data, q, v,
).copy()

com_velocity = data.vcom[0].copy()

# 4. 计算整机线动量，以及绕质心的角动量
momentum = pin.computeCentroidalMomentum(
    model, data, q, v
)

linear_momentum = momentum.linear.copy()
angular_momentum = momentum.angular.copy()

# OCS2 质心状态使用质量归一化动量
normalized_momentum = np.concatenate([
    linear_momentum,
    angular_momentum,
]) / total_mass

# 5. 验证线动量 l = m * v_com

expected_linear_momentum = total_mass * com_velocity

print("Total mass [kg]:")
print(total_mass)

print("\nCOM position in world [m]:")
print(com_position)

print("\nCOM velocity in world [m/s]:")
print(com_velocity)

print("\nLinear momentum [kg*m/s]:")
print(linear_momentum)

print("\nExpected linear momentum m * v_com:")
print(expected_linear_momentum)

print("\nLinear momentum verification error:")
print(np.linalg.norm(
    linear_momentum - expected_linear_momentum
))

print("\nAngular momentum about COM [kg*m^2/s]:")
print(angular_momentum)

print("\nMass-normalized momentum [linear; angular]:")
print(normalized_momentum)


# 6. 验证质心动量矩阵：h_G = A_G(q) @ v

centroidal_map = pin.computeCentroidalMap(
    model,
    data,
    q,
).copy()

momentum_from_matrix = centroidal_map @ v

momentum_direct = np.concatenate([
    linear_momentum,
    angular_momentum,
])

print("\nCentroidal momentum matrix shape:")
print(centroidal_map.shape)

print("\nMomentum from A_G @ v:")
print(momentum_from_matrix)

print("\nMomentum from computeCentroidalMomentum:")
print(momentum_direct)

print("\nMomentum mapping error norm:")
print(np.linalg.norm(
    momentum_from_matrix - momentum_direct
))

# 当前只有 FR_thigh_joint 的速度为 1 rad/s。
print("\nFR thigh velocity index:")
print(v_index)

print("\nFR thigh column of A_G:")
print(centroidal_map[:, v_index])


# 7. 已知总动量和关节速度，反求基座速度

base_momentum_map = centroidal_map[:, :6]
joint_momentum_map = centroidal_map[:, 6:]

joint_velocity = v[6:].copy()

recovered_base_velocity = np.linalg.solve(
    base_momentum_map,
    momentum_direct - joint_momentum_map @ joint_velocity
)

print("\nOriginal base velocity:")
print(v[:6])

print("\nRecovered base velocity:")
print(recovered_base_velocity)

print("\nBase velocity recovery error:")
print(np.linalg.norm(
    recovered_base_velocity - v[:6]
))

# 8. 按 OCS2 质心模型的布局组装状态
base_rotation = pin.XYZQUATToSE3(q[:7]).rotation

#Pinocchio 返回[roll, pitch, yaw]
rpy = pin.rpy.matrixToRpy(base_rotation)

# OCS2 此处需要 [yaw, pitch, roll]
euler_zyx = rpy[::-1].copy()

# 按当前模型 q 的顺序收集十二个单自由度关节
joint_entries = []

for joint_id in range(1, model.njoints):
    joint = model.joints[joint_id]

    if joint.nq == 1 and joint.nv == 1:
        joint_entries.append((
            joint.idx_q,
            model.names[joint_id],
        ))

joint_entries.sort()

if len(joint_entries) != 12:
    raise RuntimeError("Expected twelve scalar joints")

joint_q_indices = [
    q_index for q_index, _ in joint_entries
]


mpc_state = np.zeros(24)

mpc_state[:6] = normalized_momentum
mpc_state[6:9] = q[:3]
mpc_state[9:12] = euler_zyx
mpc_state[12:] = q[joint_q_indices]

print("\nMPC state shape:")
print(mpc_state.shape)

print("\nNormalized momentum:")
print(mpc_state[:6])

print("\nBase position in world [m]:")
print(mpc_state[6:9])

print("\nBase orientation [yaw, pitch, roll] [rad]:")
print(mpc_state[9:12])

print("\nJoint state mapping:")
for index, (q_index, joint_name) in enumerate(joint_entries):
    print(
        f"state[{12 + index:2d}] <- q[{q_index:2d}] "
        f"{joint_name:20s} {mpc_state[12 + index]: .6f}"
    )

# 9. 独立验证姿态排列，不改变上面实验的 q。
test_rpy = np.deg2rad([10.0, -5.0, 30.0])

test_rotation = pin.rpy.rpyToMatrix(test_rpy)
test_zyx = pin.rpy.matrixToRpy(test_rotation)[::-1].copy()

reconstructed_rotation = pin.rpy.rpyToMatrix(
    test_zyx[::-1].copy()
)

print("\nOrientation check [yaw, pitch, roll] [deg]:")
print(np.rad2deg(test_zyx))

print("\nRotation reconstruction error:")
print(np.linalg.norm(
    reconstructed_rotation - test_rotation
))

# 10. 按本实验约定组装 MPC 输入。
# 后续必须与 OCS2 实际配置的接触顺序核对

contact_names = (
    "FR_foot",
    "FL_foot",
    "RR_foot",
    "RL_foot",
)

# 暂时人为指定四脚均分重力，只用于动力学计算
contact_forces = np.zeros((4,3))
contact_forces[:,2] = (
    total_mass * abs(model.gravity.linear[2] / 4.0)
)

# 关节速度采用与 mpc_state[12:] 相同的关节顺序
joint_v_indices = []

for _, joint_name in joint_entries:
    joint_id = model.getJointId(joint_name)
    joint_v_indices.append(model.joints[joint_id].idx_v)

mpc_input = np.zeros(24)
mpc_input[:12] = contact_forces.ravel()
mpc_input[12:] = v[joint_v_indices]

# 11. 获取各脚在世界坐标系的位置
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

foot_positions = np.array([
    data.oMf[model.getFrameId(name)].translation.copy()
    for name in contact_names
])

# 12. 根据外力计算质心动量变化率
gravity_world = model.gravity.linear.copy()

linear_momentum_rate = (
    np.sum(contact_forces, axis=0) + total_mass * gravity_world
)

angular_momentum_rate = np.zeros(3)

for foot_position, force in zip(foot_positions, contact_forces):
    lever_arm = foot_position - com_position
    angular_momentum_rate += np.cross(lever_arm, force)

normalized_momentum_rate = np.concatenate([
    linear_momentum_rate,
    angular_momentum_rate,
]) / total_mass

print("\nMPC input shape")
print(mpc_input.shape)

print("\nContact forces in input order [N]:")
for name, force in zip(contact_names, contact_forces):
    print(f"{name:10s}: {force}")

print("\nJoint velocities in state joint order [rad/s]:")
print(mpc_input[12:])

print("\nLinear momentum rate [N]:")
print(linear_momentum_rate)

print("\nAngular momentum rate about COM [Nm]:")
print(angular_momentum_rate)

print("\nNormalized momentum rate:")
print(normalized_momentum_rate)

# 13. 从 MPC 状态和输入恢复当前基座速度

#将输入中的关节速度映射回Pinocchio的速度顺序
velocity_from_input = np.zeros(model.nv)
velocity_from_input[joint_v_indices] = mpc_input[12:]

#状态前6维是归一化动量，乘质量恢复实际动量
momentum_from_state = total_mass * mpc_state[:6]

base_velocity = np.linalg.solve(
    centroidal_map[:, :6],
    momentum_from_state - centroidal_map[:, 6:] @ velocity_from_input[6:],
)

# 14. 身体线速度 -> 世界系基座位置导数
yaw, pitch, roll = mpc_state[9:12]

rotation_world_base = pin.rpy.rpyToMatrix(
    np.array([roll, pitch, yaw])
)

base_position_rate = (
    rotation_world_base @ base_velocity[:3]
)

# 15. 身体角速度 -> ZYX 欧拉角导数
# omega_body = E @ [yaw_dot, pitch_dot, roll_dot]
if abs(np.cos(pitch)) < 1e-6:
    raise ValueError("ZYX Euler angles are near a singular configuration")

euler_rate_to_body_omega = np.array([
    [-np.sin(pitch),                0.0,           1.0],
    [np.sin(roll) * np.cos(pitch),  np.cos(roll),  0.0],
    [np.cos(roll) * np.cos(pitch), -np.sin(roll),  0.0],
])

base_orientation_rate = np.linalg.solve(
    euler_rate_to_body_omega,
    base_velocity[3:6],
)

# 16. 组装完整的 24 维状态导数

mpc_state_rate = np.zeros(24)

mpc_state_rate[:6] = normalized_momentum_rate
mpc_state_rate[6:9] = base_position_rate
mpc_state_rate[9:12] = base_orientation_rate
mpc_state_rate[12:] = mpc_input[12:]

print("\nMPC state derivative shape:")
print(mpc_state_rate.shape)

print("\nNormalized momentum rate:")
print(mpc_state_rate[:6])

print("\nBase position rate in world [m/s]:")
print(mpc_state_rate[6:9])

print("\nEuler angle rates [yaw, pitch, roll] [rad/s]:")
print(mpc_state_rate[9:12])

print("\nJoint angle rates [rad/s]:")
print(mpc_state_rate[12:])
