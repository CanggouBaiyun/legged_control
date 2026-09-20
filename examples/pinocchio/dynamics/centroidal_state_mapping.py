"""Check the round-trip conversion of centroidal state."""

import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_simulation_model,
    nominal_configuration,
)
from go2_control.centroidal import (
    get_joint_indices,
    state_from_pinocchio,
    pinocchio_from_state,
    centroidal_dynamics,
)

model = build_go2_simulation_model()
data = model.createData()

q = nominal_configuration(model)

# 不只检查水平姿态：给基座添加一个旋转。
configuration_increment = np.zeros(model.nv)
configuration_increment[3:6] = [0.15, -0.10, 0.30]
q = pin.integrate(model, q, configuration_increment)

# 基座和关节都在运动，避免全零数据掩盖转换错误。
rng = np.random.default_rng(7)
v = rng.normal(0.0, 0.2, model.nv)

_, v_indices = get_joint_indices(model)


# 正向转换：q、v -> MPC 状态。
state = state_from_pinocchio(model, data, q, v)

# 反向转换：MPC 状态、关节速度 -> q、v。
recovered_q, recovered_v = pinocchio_from_state(
    model,
    data,
    state,
    v[v_indices],
)

configuration_error = pin.difference(
    model, q, recovered_q
)
velocity_error = recovered_v - v

print("State shape:")
print(state.shape)

print("\nConfiguration recovery error norm:")
print(np.linalg.norm(configuration_error))

print("\nVelocity recovery error norm:")
print(np.linalg.norm(velocity_error))

assert np.linalg.norm(configuration_error) < 1e-10
assert np.linalg.norm(velocity_error) < 1e-10

# 构造测试输入：四脚均分重力，关节速度沿用当前值
mass = pin.computeTotalMass(model)

control = np.zeros(24)
contact_forces = control[:12].reshape(4, 3)
contact_forces[:, 2] = mass * abs(model.gravity.linear[2]) / 4.0
control[12:] = v[v_indices]

state_rate = centroidal_dynamics(
    model, data, state, control
)

print("\nState derivative shape:")
print(state_rate.shape)

print("\nNormalized momentum rate:")
print(state_rate[:6])

# 验证后十八维：位置、姿态、关节角的导数。
# 用原始 q、v 做一个很小的构型积分，再通过正运动学状态转换比较
dt = 1e-7
q_next = pin.integrate(model, q, v * dt)

state_next = state_from_pinocchio(
    model, data, q_next, v
)

finite_difference = (
    state_next[6:] - state[6:]
) / dt

kinematic_error = np.linalg.norm(
    finite_difference - state_rate[6:]
)

print("\nKinematic derivative error norm:")
print(kinematic_error)

assert state_rate.shape == (24,)
assert np.all(np.isfinite(state_rate))
assert np.linalg.norm(state_rate[:3]) < 1e-10
assert kinematic_error < 1e-6
