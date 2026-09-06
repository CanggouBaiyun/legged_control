import numpy as np
import pinocchio as pin

from go2_control.model import(
    build_go2_model,
    nominal_configuration,
)

np.set_printoptions(precision=8, suppress=False)

model = build_go2_model()
data = model.createData()
q = nominal_configuration(model)

foot_id = model.getFrameId("FR_foot")

joint_names = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
]
#把腿设置成接近伸直、但没完全伸直姿态
joint_angles = [
    0.0,
    0.05,
    -0.10,
]

v_indices = []

for joint_name, joint_angle in zip(joint_names, joint_angles):
    joint_id = model.getJointId(joint_name)

    q_index = model.joints[joint_id].idx_q
    v_index = model.joints[joint_id].idx_v

    q[q_index] = joint_angle
    v_indices.append(v_index)

jacobian_6d = pin.computeFrameJacobian(
    model,
    data,
    q,
    foot_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
)

leg_jacobian = jacobian_6d[:3, v_indices]
#奇异值分解
singular_values = np.linalg.svd(
    leg_jacobian,
    compute_uv=False,
)

desired_velocity = np.array([0.0, 0.0, 0.05])

#普通伪逆
joint_velocity_pinv =(
    np.linalg.pinv(leg_jacobian) @ desired_velocity
)
#阻尼最小二乘

damping_values = [
    1e-4,
    1e-3,
    1e-2,
    1e-1,
]
print("\nDamping comparison:")

print(
    "damping      "
    "joint velocity norm    "
    "velocity error norm    "
    "achieved z velocity"
)

achieved_velocity_pinv = (
        leg_jacobian @ joint_velocity_pinv
    )

for damping in damping_values:
    matrix =(
        leg_jacobian @ leg_jacobian.T +damping**2 * np.eye(3)
    )

    x = np.linalg.solve(matrix, desired_velocity)

    joint_velocity_dls = leg_jacobian.T @ x

    

    achieved_velocity_dls =(
        leg_jacobian @ joint_velocity_dls
    )

    velocity_error_dls = (
        desired_velocity - achieved_velocity_dls
    )

    print(
        f"{damping:8.1e}    "
        f"{np.linalg.norm(joint_velocity_dls):19.8f}    "
        f"{np.linalg.norm(velocity_error_dls):19.8f}    "
        f"{achieved_velocity_dls[2]:19.8f}"
    )

print("Leg Jacobian:")
print(leg_jacobian)

print("\nSingular values:")
print(singular_values)

print("\nDesired foot velocity [m/s]:")
print(desired_velocity)

print("\nJoint velocity from pseudoinverse [rad/s]:")
print(joint_velocity_pinv)

print("\nJoint velocity norm from pseudoinverse:")
print(np.linalg.norm(joint_velocity_pinv))

print("\nAchieved foot velocity from pseudoinverse [m/s]:")
print(achieved_velocity_pinv)



