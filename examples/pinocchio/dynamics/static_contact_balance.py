import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_model,
    nominal_configuration,
    sdk_joint_mapping,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

model = build_go2_model()
q = nominal_configuration(model)

gravity = pin.computeGeneralizedGravity(
    model,
    model.createData(),
    q,
).copy()

vertical_unit_force = np.array([
    0.0,
    0.0,
    1.0,
])

foot_jacobians = []
base_wrench_columns = []

# ============================================================
# 1. 计算每个足端的 Jacobian
# ============================================================

for foot_name in GO2_FOOT_FRAMES:
    foot_id = model.getFrameId(foot_name)
    data = model.createData()

    frame_jacobian = pin.computeFrameJacobian(
        model,
        data,
        q,
        foot_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )

    linear_jacobian = (
        frame_jacobian[:3, :]
        .copy()
    )

    foot_jacobians.append(
        linear_jacobian
    )

    generalized_unit_force = (
        linear_jacobian.T @ vertical_unit_force
    )

    base_wrench_columns.append(
        generalized_unit_force[:6]
    )

# ============================================================
# 2. 求四只脚的竖直接触力
# ============================================================

#6x4matrix
base_force_mapping = np.column_stack(
    base_wrench_columns
)

normal_forces, _, mapping_rank, singular_values = (
    np.linalg.lstsq(
        base_force_mapping,
        gravity[:6],
        rcond=None,
    )
)

# ============================================================
# 3. 构造完整接触 Jacobian 和接触力向量
# ============================================================

contact_jacobian = np.vstack(
    foot_jacobians
)
#四只脚，每只脚三维力
contact_force_vector = np.zeros(
    3 * len(GO2_FOOT_FRAMES)
)

for foot_index, normal_force in enumerate(
    normal_forces
):
    contact_force_vector[
        3 * foot_index + 2
    ] = normal_force

generalized_contact_force = (
    contact_jacobian.T @ contact_force_vector
)

# ============================================================
# 4. 根据关节动力学求电机力矩
# ============================================================

actuated_generalized_force = np.zeros(model.nv)

actuated_generalized_force[6:] = (
    gravity[6:] - generalized_contact_force[6:]
)

# ============================================================
# 5. 验证完整静力平衡
# ============================================================

static_balance_residual = (
    gravity
    - actuated_generalized_force
    - generalized_contact_force
)

print("Base force mapping shape:")
print(base_force_mapping.shape)

print("\nBase force mapping rank:")
print(mapping_rank)

print("\nBase force mapping singular values:")
print(singular_values)

print("\nSolved vertical contact forces [N]:")

for foot_name, normal_force in zip(
    GO2_FOOT_FRAMES,
    normal_forces,
):
    print(
        f"{foot_name:10s} "
        f"{normal_force: .8f}"
    )

print("\nSum of vertical forces [N]:")
print(np.sum(normal_forces))

print("\nRobot weight [N]:")
print(gravity[2])

print("\nTarget base generalized gravity:")
print(gravity[:6])

print("\nBase generalized contact force:")
print(generalized_contact_force[:6])

print("\nBase balance residual:")
print(
    gravity[:6]
    - generalized_contact_force[:6]
)

print("\nMotor torques in SDK order [Nm]:")

for entry in sdk_joint_mapping(model):
    joint_name = entry["name"]
    v_index = entry["v_index"]

    print(
        f"{joint_name:20s} "
        f"{actuated_generalized_force[v_index]: .8f}"
    )

print("\nFull static balance residual:")
print(static_balance_residual)

print("\nFull static balance residual norm:")
print(
    np.linalg.norm(
        static_balance_residual
    )
)