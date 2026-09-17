import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_simulation_model,
    nominal_configuration,
)
from go2_control.wbc import StandingWBC
from go2_control.wbc.reference import FixedFootReference


np.set_printoptions(precision=6, suppress=True)

model = build_go2_simulation_model()
data = model.createData()

q_initial = nominal_configuration(model)

stance_feet = (
    "FL_foot",
    "RR_foot",
    "RL_foot",
)

# 1. 保存初始四足世界位置
pin.forwardKinematics(model, data, q_initial)
pin.updateFramePlacements(model, data)

fixed_foot_positions = np.array([
    data.oMf[model.getFrameId(name)].translation.copy()
    for name in GO2_FOOT_FRAMES
])

# 2. 构造已经完成移重的姿态
desired_base_position = (
    q_initial[:3] + np.array([-0.03, 0.03, 0.0])
)

desired_base_rotation = pin.XYZQUATToSE3(
    q_initial[:7]
).rotation.copy()

reference = FixedFootReference(
    model,
    q_initial,
    fixed_foot_positions,
)

q, _ = reference.solve(
    desired_base_position,
    desired_base_rotation,
    np.zeros(3),
)

v = np.zeros(model.nv)

# 记录移重完成后右前脚的位置，作为保持目标。
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

swing_foot_name = "FR_foot"
swing_frame_id = model.getFrameId(swing_foot_name)

current_swing_position = (
    data.oMf[swing_frame_id].translation.copy()
)

lift_height = 0.03

desired_swing_position = (
    current_swing_position + np.array([0.0, 0.0, lift_height])
)


#3. 创建三足支撑控制器
controller = StandingWBC(
    model,
    contact_frame_names=stance_feet,
)

# 4. 当前姿态就是目标姿态，测试在此处保持。
result = controller.solve(
    q=q,
    v=v,
    desired_base_position=desired_base_position,
    desired_base_rotation=desired_base_rotation,
    swing_foot_name=swing_foot_name,
    desired_swing_position=desired_swing_position,
)

# 5. 输出关键结果。
print("QP status:")
print(result.solver_status)

print("\nDecision variable size:")
print(controller.number_of_decision_variables)

print("\nSolved base acceleration:")
print(result.generalized_acceleration[:6])

print("\nStance contact forces [N]:")
for name, force in zip(
    stance_feet,
    result.contact_forces,
):
    print(
        f"{name:10s}"
        f" fx={force[0]:9.4f}"
        f" fy={force[1]:9.4f}"
        f" fz={force[2]:9.4f}"
    )

print("\nMinimum normal force [N]:")
print(np.min(result.contact_forces[:, 2]))

print("\nDynamics residual norm:")
print(result.dynamics_residual_norm)

print("\nContact acceleration residual norm:")
print(result.contact_acceleration_residual_norm)

print("\nMaximum motor torque utilization:")
print(np.max(
    np.abs(result.motor_torques) / model.effortLimit[6:]
))


print("\nSolved joint accelerations [rad/s^2]:")

for joint_name in (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
):
    joint_id = model.getJointId(joint_name)
    v_index = model.joints[joint_id].idx_v

    print(
        f"{joint_name:20s}: "
        f"{result.generalized_acceleration[v_index]: .6f}"
    )

total_force = result.contact_forces.sum(axis=0)
total_mass = pin.computeTotalMass(model)

com_acceleration_world = (
    total_force / total_mass
    + np.asarray(model.gravity.linear)
)

print("\nSum of contact forces [N]:")
print(total_force)

print("\nCOM acceleration from force balance [m/s^2]:")
print(com_acceleration_world)


# 检查 QP 解对应的右前脚线加速度。
pin.forwardKinematics(
    model,
    data,
    q,
    v,
    result.generalized_acceleration,
)

pin.updateFramePlacements(model, data)

swing_acceleration = pin.getFrameClassicalAcceleration(
    model,
    data,
    swing_frame_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
).linear.copy()

print("\nSwing foot acceleration [m/s^2]:")
print(swing_acceleration)

print("\nSwing foot acceleration norm:")
print(np.linalg.norm(swing_acceleration))



# 与 standing.py 中当前的 swing_kp 保持一致。
swing_kp = 100.0

expected_swing_acceleration = (
    swing_kp * (desired_swing_position - current_swing_position)
)

print("\nDesired foot displacement [m]:")
print(desired_swing_position - current_swing_position)

print("\nExpected swing acceleration [m/s^2]:")
print(expected_swing_acceleration)

print("\nSwing acceleration tracking error [m/s^2]:")
print(swing_acceleration - expected_swing_acceleration)