import numpy as np
import pinocchio as pin

from go2_control.model import(
    build_go2_model,
    nominal_configuration,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

model = build_go2_model()
q = nominal_configuration(model)

foot_name = "FR_foot"
foot_id = model.getFrameId(foot_name)

joint_names = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
]

v_indices = []

for joint_name in joint_names:
    joint_id = model.getJointId(joint_name)
    v_indices.append(
        model.joints[joint_id].idx_v
    )

#当前广义速度：只有FR三个关节正在运动

v = np.zeros(model.nv)
v[v_indices] = np.array([
    0.4,
    -0.3,
    0.2,
])

# ============================================================
# 1. 计算 J 和 J_dot
# ============================================================

jacobian_data = model.createData()

pin.computeJointJacobiansTimeVariation(
    model,
    jacobian_data,
    q,
    v,
)
#更新所有frame的位姿
pin.updateFramePlacements(
    model,
    jacobian_data,
)

frame_jacobian = pin.getFrameJacobian(
    model,
    jacobian_data,
    foot_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
)

frame_jacobian_time_variation = (
    pin.getFrameJacobianTimeVariation(
        model,
        jacobian_data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )
)

linear_jacobian = frame_jacobian[:3, :]
linear_jacobian_time_variation = (
    frame_jacobian_time_variation[:3, :]
)

# 即使广义加速度为零，足端仍可能有 J_dot * v
bias_acceleration = (
    linear_jacobian_time_variation @ v
)

# ============================================================
# 2. 用 Pinocchio 直接验证 J_dot * v
# ============================================================

bias_data = model.createData()
zero_acceleration = np.zeros(model.nv)

pin.forwardKinematics(
    model,
    bias_data,
    q,
    v,
    zero_acceleration,
)

pin.updateFramePlacements(
    model,
    bias_data,
)

bias_acceleration_from_pinocchio = (
    pin.getFrameClassicalAcceleration(
        model,
        bias_data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    ).linear.copy()
)

# ============================================================
# 3. 求使 FR 足端加速度为零的关节加速度
# ============================================================

leg_jacobian = linear_jacobian[
    :,
    v_indices,
]

joint_acceleration = np.linalg.solve(
    leg_jacobian,
    -bias_acceleration
)

generalized_acceleration = np.zeros(model.nv)
generalized_acceleration[v_indices] = (
    joint_acceleration
)

predicted_foot_acceleration = (
    linear_jacobian @ generalized_acceleration + bias_acceleration
)

# ============================================================
# 4. 用 Pinocchio 直接验证最终足端加速度
# ============================================================

acceleration_data = model.createData()

pin.forwardKinematics(
    model,
    acceleration_data,
    q,
    v,
    generalized_acceleration,
)

pin.updateFramePlacements(
    model,
    acceleration_data,
)

foot_acceleration_from_pinocchio = (
    pin.getFrameClassicalAcceleration(
        model,
        acceleration_data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    ).linear.copy()
)

print("FR joint velocity indices:")
print(v_indices)

print("\nJ_dot @ v [m/s^2]:")
print(bias_acceleration)

print("\nPinocchio result with a=0 [m/s^2]:")
print(bias_acceleration_from_pinocchio)

print("\nJ_dot @ v verification error:")
print(
    np.linalg.norm(
        bias_acceleration
        - bias_acceleration_from_pinocchio
    )
)

print("\nSolved FR joint acceleration [rad/s^2]:")
print(joint_acceleration)

print("\nJ @ a + J_dot @ v [m/s^2]:")
print(predicted_foot_acceleration)

print("\nPinocchio foot acceleration [m/s^2]:")
print(foot_acceleration_from_pinocchio)

print("\nZero foot acceleration norm:")
print(
    np.linalg.norm(
        foot_acceleration_from_pinocchio
    )
)

