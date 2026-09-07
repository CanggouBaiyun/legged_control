import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    nominal_configuration,
    sdk_joint_mapping,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

model = build_go2_model()
data = model.createData()
q = nominal_configuration(model)

foot_name = "FR_foot"
foot_id = model.getFrameId(foot_name)

pin.forwardKinematics(
    model,
    data,
    q,
)

pin.updateFramePlacements(
    model,
    data,
)

foot_position = (
    data.oMf[foot_id]
    .translation
    .copy()
)

frame_jacobian = pin.computeFrameJacobian(
    model,
    data,
    q,
    foot_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
)

linear_jacobian = frame_jacobian[:3, :]

contact_force = np.array([
    0.0,
    0.0,
    100.0,
])

generalized_contact_force = (
    linear_jacobian.T @ contact_force
)

# ============================================================
# 手动计算该接触力对浮动基产生的力和力矩
# ============================================================

base_position = q[:3]
#力臂
lever_arm = (
    foot_position - base_position
)

expected_base_force = contact_force
#产生力矩
expected_base_moment = np.cross(
    lever_arm,
    contact_force,
)

expected_base_wrench = np.concatenate([
    expected_base_force,
    expected_base_moment,
])

base_wrench_error = (
    generalized_contact_force[:6] - expected_base_wrench
)

# ============================================================
# 使用功率关系验证 J.T 的映射
# ============================================================
rng = np.random.default_rng(7)

test_velocity = rng.normal(
    scale=0.1,
    size=model.nv,
)

foot_velocity = (
    linear_jacobian
    @ test_velocity
)

contact_power = (
    contact_force
    @ foot_velocity
)

generalized_power = (
    generalized_contact_force
    @ test_velocity
)

# ============================================================
# 输出
# ============================================================

print("Foot position in world [m]:")
print(foot_position)

print("\nContact force on robot [N]:")
print(contact_force)

print("\nLinear Jacobian shape:")
print(linear_jacobian.shape)

print("\nGeneralized contact force J.T @ force:")
print(generalized_contact_force)

print("\nFloating-base wrench from J.T @ force:")
print(generalized_contact_force[:6])

print("\nExpected floating-base wrench [force, moment]:")
print(expected_base_wrench)

print("\nFloating-base wrench error:")
print(base_wrench_error)

print("\nJoint torque contribution in SDK order [Nm]:")

for entry in sdk_joint_mapping(model):
    joint_name = entry["name"]
    v_index = entry["v_index"]

    print(
        f"{joint_name:20s} "
        f"{generalized_contact_force[v_index]: .8f}"
    )

print("\nTest foot velocity J @ v [m/s]:")
print(foot_velocity)

print("\nContact power force.T @ foot_velocity [W]:")
print(contact_power)

print("\nGeneralized power (J.T @ force).T @ v [W]:")
print(generalized_power)

print("\nPower difference [W]:")
print(contact_power - generalized_power)