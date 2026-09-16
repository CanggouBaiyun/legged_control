import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_simulation_model,
    nominal_configuration,
)

np.set_printoptions(precision=6, suppress=True)

# 1. 创建模型，取初始站立构型。
model = build_go2_simulation_model()
data = model.createData()

q = nominal_configuration(model)

# 2. 指定右前脚为摆动脚，其余三脚为支撑脚。
swing_foot = "FR_foot"

stance_feet = [
    name 
    for name in GO2_FOOT_FRAMES
    if name != swing_foot
]

# 3. 计算当前构型下的关节 Jacobian 和 frame 位姿。
pin.computeJointJacobians(model, data, q)
pin.updateFramePlacements(model, data)

# 4. 分别保存支撑脚和摆动脚的线速度 Jacobian。
stance_jacobians = []
swing_jacobian = None

for foot_name in GO2_FOOT_FRAMES:
    foot_id = model.getFrameId(foot_name)

    frame_jacobian = pin.getFrameJacobian(
        model,
        data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    linear_jacobian = frame_jacobian[:3, :].copy()

    if foot_name == swing_foot:
        swing_jacobian = linear_jacobian
    else:
        stance_jacobians.append(linear_jacobian)

contact_jacobian = np.vstack(stance_jacobians)

# 5. 检查维度。
print("Stance feet:")
print(stance_feet)

print("\nSwing foot:")
print(swing_foot)

print("\nContact Jacobian shape:")
print(contact_jacobian.shape)

print("\nSwing Jacobian shape:")
print(swing_jacobian.shape)

print("\nContact Jacobian rank:")
print(np.linalg.matrix_rank(contact_jacobian))

# 6. 
# 将接触 Jacobian 按列分成基座部分和关节部分
base_jacobian = contact_jacobian[:, :6]
joint_jacobian = contact_jacobian[:, 6:]

# 指定基座向前运动
v = np.zeros(model.nv)
v[:3] = [0.1, 0.0, 0.0]

# 求配套的关节速度，让三只支撑脚的速度为零
v[6:] = np.linalg.lstsq(
    joint_jacobian,
    -base_jacobian @ v[:6],
    rcond=None,
)[0]

print("\nContact velocity norm:")
print(np.linalg.norm(contact_jacobian @ v))


# 将广义加速度设为零，用足端加速度提取 J_dot @ v。
zero_acceleration = np.zeros(model.nv)

pin.forwardKinematics(
    model,
    data,
    q,
    v,
    zero_acceleration,
)

pin.updateFramePlacements(model, data)

contact_bias_parts = []

for foot in stance_feet:
    foot_id = model.getFrameId(foot_name)

    foot_acceleration = pin.getFrameClassicalAcceleration(
        model,
        data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    contact_bias_parts.append(
        foot_acceleration.linear.copy()
    )

contact_bias = np.concatenate(contact_bias_parts)

# 固定接触条件：J_c @ a = -J_dot_c @ v
contact_acceleration_target = -contact_bias


print("\nJ_dot_c @ v:")
print(contact_bias)

print("\nContact acceleration target:")
print(contact_acceleration_target)

print("\nTarget shape:")
print(contact_acceleration_target.shape)

# 7. 指定这一瞬间的基座广义加速度为零
a = np.zeros(model.nv)

# 求关节加速度，使支撑脚加速度为零
joint_acceleration_target = (
    contact_acceleration_target - base_jacobian @ a[:6]
)

a[6:] = np.linalg.lstsq(
    joint_jacobian,
    joint_acceleration_target,
    rcond = None,
)[0]

# 用公式检查。
contact_acceleration_from_formula = (
    contact_jacobian @ a + contact_bias
)

# 用 Pinocchio 独立计算同一状态下的足端加速度。
pin.forwardKinematics(
    model,
    data,
    q,
    v,
    a,
)

pin.updateFramePlacements(model, data)

actual_acceleration_parts = []

for foot_name in stance_feet:
    foot_id = model.getFrameId(foot_name)

    foot_acceleration = pin.getFrameClassicalAcceleration(
        model,
        data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    actual_acceleration_parts.append(
        foot_acceleration.linear.copy()
    )

contact_acceleration_from_pinocchio = np.concatenate(
    actual_acceleration_parts
)

print("\nSolved joint acceleration [rad/s^2]:")
print(a[6:])

print("\nContact acceleration norm from formula:")
print(np.linalg.norm(contact_acceleration_from_formula))

print("\nContact acceleration norm from Pinocchio:")
print(np.linalg.norm(contact_acceleration_from_pinocchio))

print("\nFormula-Pinocchio difference norm:")
print(np.linalg.norm(
    contact_acceleration_from_formula
    - contact_acceleration_from_pinocchio
))

# 8. 单独做三足静力平衡检查。
# 本部分采用 v=0、a=0，不使用前面运动学实验的速度和加速度。

gravity = pin.computeGeneralizedGravity(
    model,
    data,
    q,
).copy()

#每只支撑脚暂时只考虑向上的竖直接触力
vertical_force_columns = []

for foot_jacobian in stance_jacobians:
    vertical_force_columns.append(
        foot_jacobian.T[:, 2].copy()
    )

# 三个竖直接触力 → 18维广义接触力。
vertical_force_mapping = np.column_stack(
    vertical_force_columns
)

#只取基座对应的前6行
base_force_mapping = vertical_force_mapping[:6, :]

normal_forces = np.linalg.lstsq(
    base_force_mapping,
    gravity[:6],
    rcond=None,
)[0]

generalized_contact_force = (
    vertical_force_mapping @ normal_forces
)

base_residual = (
    generalized_contact_force[:6] - gravity[:6]
)

#静力平衡:g = S.T @ tau + J.T @ force

motor_torques = (
    gravity[6:] - generalized_contact_force[6:]
)

full_residual = (
    gravity - np.concatenate([np.zeros(6), motor_torques]) - generalized_contact_force
)

print("\n--- Three-foot static balance ---")

print("\nBase force mapping shape:")
print(base_force_mapping.shape)

print("\nSolved vertical forces [N]:")
for foot_name, force in zip(stance_feet, normal_forces):
    print(f"{foot_name:10s}: {force: .6f}")

print("\nMinimum vertical force [N]:")
print(np.min(normal_forces))

print("\nBase balance residual norm:")
print(np.linalg.norm(base_residual))

print("\nFull static balance residual norm:")
print(np.linalg.norm(full_residual))

print("\nMaximum absolute motor torque [Nm]:")
print(np.max(np.abs(motor_torques)))

print("\nMaximum motor torque utilization:")
print(np.max(
    np.abs(motor_torques) / model.effortLimit[6:]
))