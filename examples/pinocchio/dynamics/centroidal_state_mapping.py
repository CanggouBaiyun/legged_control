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

