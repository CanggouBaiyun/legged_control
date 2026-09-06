import numpy as np
import pinocchio as pin

from go2_control.model import(
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
    v_index = model.joints[joint_id].idx_v
    v_indices.append(v_index)

jacobian_6d = pin.computeFrameJacobian(
    model,
    data,
    q,
    foot_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
)

jacobian_linear = jacobian_6d[:3, :]

#只读取右前腿三个关节的列
jacobian_leg = jacobian_linear[:, v_indices]
#期望足端速度
desired_foot_velocity = np.array([
    0.0,
    0.0,
    0.05,
])


joint_velocity = (
    np.linalg.pinv(jacobian_leg) @ desired_foot_velocity 
)

achieved_foot_velocity = (
    jacobian_leg @ joint_velocity
)

velocity_error = (
    achieved_foot_velocity - desired_foot_velocity
)

# 7. 输出
print("Joint names:")
print(joint_names)

print("\nv indices:")
print(v_indices)

print("\nLeg Jacobian:")
print(jacobian_leg)

print("\nDesired foot velocity [m/s]:")
print(desired_foot_velocity)

print("\nSolved joint velocity [rad/s]:")
print(joint_velocity)

print("\nAchieved foot velocity [m/s]:")
print(achieved_foot_velocity)

print("\nVelocity error:")
print(velocity_error)

print("\nError norm:")
print(np.linalg.norm(velocity_error))
