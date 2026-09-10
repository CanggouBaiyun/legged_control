import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_model,
    nominal_configuration,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

model = build_go2_model()
q = nominal_configuration(model)

# ============================================================
# 1. 堆叠四个足端的线性 Jacobian
# ============================================================

position_data = model.createData()

pin.computeJointJacobians(
    model,
    position_data,
    q,
)

pin.updateFramePlacements(
    model,
    position_data,
)

foot_jacobians = []

for foot_name in GO2_FOOT_FRAMES:
    foot_id = model.getFrameId(foot_name)

    frame_jacobian = pin.getFrameJacobian(
        model,
        position_data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    foot_jacobians.append(
        frame_jacobian[:3, :].copy()
    )

contact_jacobian = np.vstack(
    foot_jacobians
)

base_contact_jacobian = (
    contact_jacobian[:,:6]
)

joint_contact_jacobian = (
    contact_jacobian[:,6:]
)

# ============================================================
# 2. 求满足 J_c v = 0 的关节速度
# ============================================================

v = np.zeros(model.nv)

#名义姿态下机身沿世界x方向运动
v[:6] = np.array([
    0.1,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
])

joint_velocity = np.linalg.solve(
    joint_contact_jacobian,
    -base_contact_jacobian @ v[:6],
)

v[6:] = joint_velocity

contact_velocity = (
    contact_jacobian @ v
)

# ============================================================
# 3. 使用接触一致的速度计算 J_dot @ v
# ============================================================

variation_data = model.createData()

pin.computeJointJacobiansTimeVariation(
    model,
    variation_data,
    q,
    v,
)

pin.updateFramePlacements(
    model,
    variation_data,
)

foot_jacobians = []
foot_jacobian_time_variations = []

for foot_name in GO2_FOOT_FRAMES:
    foot_id = model.getFrameId(foot_name)

    frame_jacobian = pin.getFrameJacobian(
        model,
        variation_data,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    frame_jacoboian_time_variation = (
        pin.getFrameJacobianTimeVariation(
            model,
            variation_data,
            foot_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
    )

    foot_jacobians.append(
        frame_jacobian[:3, :].copy()
    )
    foot_jacobian_time_variations.append(
        frame_jacoboian_time_variation[:3, :].copy()
    )

contact_jacobian = np.vstack(
    foot_jacobians
)

contact_jacobian_time_variation = np.vstack(
    foot_jacobian_time_variations
)
#偏置加速度，表示如果此刻广义加速度 \(\dot v=0\)，机器人继续保持当前广义速度 \(v\)，四只脚由于运动路径弯曲而产生的加速度。
contact_bias_acceleration = (
    contact_jacobian_time_variation @ v
)

base_contact_jacobian = (
    contact_jacobian[:, :6]
)

joint_contact_jacobian = (
    contact_jacobian[:, 6:]
)

# ============================================================
# 4. 求满足 J_c a + J_dot_c v = 0 的关节加速度
# ============================================================
generalized_acceleration = np.zeros(model.nv)

generalized_acceleration[:6] = np.array([
    0.0,
    0.0,
    0.2,
    0.0,
    0.0,
    0.0,
])

joint_acceleration_target = (
    -base_contact_jacobian @ generalized_acceleration[:6] - contact_bias_acceleration
)

joint_acceleration = np.linalg.solve(
    joint_contact_jacobian,
    joint_acceleration_target,
)

generalized_acceleration[6:] = (
    joint_acceleration
)

contact_acceleration = (
    contact_jacobian @ generalized_acceleration + contact_bias_acceleration
)

# ============================================================
# 5. 使用 Pinocchio 直接验证四个足端加速度
# ============================================================

verification_data = model.createData()

pin.forwardKinematics(
    model,
    verification_data,
    q,
    v,
    generalized_acceleration,
)

pin.updateFramePlacements(
    model,
    verification_data,
)

pinocchio_contact_acceleration = []

for foot_name in GO2_FOOT_FRAMES:
    foot_id = model.getFrameId(foot_name)

    foot_accleration = (
        pin.getFrameClassicalAcceleration(
            model,
            verification_data,
            foot_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        ).linear.copy()
    )

    pinocchio_contact_acceleration.append(
        foot_accleration
    )

pinocchio_contact_acceleration = np.concatenate(
    pinocchio_contact_acceleration
)

print("Contact Jacobian shape:")
print(contact_jacobian.shape)

print("\nContact Jacobian rank:")
print(np.linalg.matrix_rank(contact_jacobian))

print("\nJoint block shape:")
print(joint_contact_jacobian.shape)

print("\nJoint block rank:")
print(np.linalg.matrix_rank(joint_contact_jacobian))

print("\nDesired base velocity:")
print(v[:6])

print("\nSolved joint velocity:")
print(v[6:])

print("\nContact velocity norm:")
print(np.linalg.norm(contact_velocity))

print("\nJ_dot_c @ v norm:")
print(np.linalg.norm(contact_bias_acceleration))

print("\nDesired base acceleration:")
print(generalized_acceleration[:6])

print("\nSolved joint acceleration:")
print(generalized_acceleration[6:])

print("\nContact acceleration norm:")
print(np.linalg.norm(contact_acceleration))

print("\nPinocchio contact acceleration norm:")
print(
    np.linalg.norm(
        pinocchio_contact_acceleration
    )
)

print("\nFormula-Pinocchio difference norm:")
print(
    np.linalg.norm(
        contact_acceleration
        - pinocchio_contact_acceleration
    )
)