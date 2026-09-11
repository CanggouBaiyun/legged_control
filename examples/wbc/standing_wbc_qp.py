import numpy as np
import pinocchio as pin
from scipy import sparse

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_model,
    nominal_configuration,
)

from go2_control.optimization.qp_solvers import (
    solve_with_osqp,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

def compute_contact_kinematics(
        model,
        q,
        v,
):
    data = model.createData()

    pin.computeJointJacobiansTimeVariation(
        model,
        data,
        q,
        v,
    )

    pin.updateFramePlacements(
        model,
        data,
    )

    foot_jacobians = []
    foot_jacobian_time_variations = []

    for foot_name in GO2_FOOT_FRAMES:
        foot_id = model.getFrameId(foot_name)

        frame_jacobian = pin.getFrameJacobian(
            model,
            data,
            foot_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        frame_jacobian_time_variation = (
                pin.getFrameJacobianTimeVariation(
                    model,
                    data,
                    foot_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )
            )   

        foot_jacobians.append(
            frame_jacobian[:3, :].copy()
        )

        foot_jacobian_time_variations.append(
            frame_jacobian_time_variation[:3, :].copy()
        )

    contact_jacobian = np.vstack(
        foot_jacobians
    )

    contact_jacbian_time_variation = np.vstack(
        foot_jacobian_time_variations
    )

    contact_bias_acceleration = (
        contact_jacbian_time_variation @ v
    )

    return (
        contact_jacobian,
        contact_bias_acceleration,
    )


model = build_go2_model()
q = nominal_configuration(model)
v = np.zeros(model.nv)

# ============================================================
# 1. 计算动力学和接触运动学
# ============================================================
dynamics_data = model.createData()

mass_matrix_upper = pin.crba(
    model,
    dynamics_data,
    q,
)

mass_matrix = (
    np.triu(mass_matrix_upper + np.triu(mass_matrix_upper, k = 1).T)
)

nonlinear_effects = pin.nonLinearEffects(
    model,
    dynamics_data,
    q,
    v,
).copy()

(
    contact_jacobian,
    contact_bias_acceleration,
) = compute_contact_kinematics(
    model,
    q,
    v,
)

# ============================================================
# 2. 定义42维决策变量的索引
# ============================================================
number_of_accelerations = model.nv
number_of_contact_forces = contact_jacobian.shape[0]
number_of_motor_torques = model.nv - 6

number_of_decision_variables = (
    number_of_accelerations + number_of_contact_forces + number_of_motor_torques
)

acceleration_slice = slice(
    0,
    number_of_accelerations,
)

contact_force_slice = slice(
    number_of_accelerations,
    number_of_accelerations + number_of_contact_forces,
)

motor_torque_slice = slice(
    number_of_accelerations + number_of_contact_forces,
    number_of_decision_variables,
)

# ============================================================
# 3. 构造执行器选择矩阵 S
# ============================================================

selection_matrix = np.zeros(
    (
        number_of_motor_torques,
        model.nv,
    )
)

selection_matrix[:, 6:] = np.eye(
    number_of_motor_torques
)

# ============================================================
# 4. 构造完整动力学等式
# ============================================================

dynamics_constraint = np.zeros(
    (
        model.nv,
        number_of_decision_variables
    )
)


dynamics_constraint[
    :,
    acceleration_slice,
] = mass_matrix

dynamics_constraint[
    :,
    contact_force_slice,
] = -contact_jacobian.T

dynamics_constraint[
    :,
    motor_torque_slice,
] = -selection_matrix.T

dynamics_target = -nonlinear_effects

# ============================================================
# 5. 构造四足接触加速度等式
# ============================================================

contact_constraint = np.zeros(
    (
        number_of_contact_forces,
        number_of_decision_variables,
    )
)

contact_constraint[
    :,
    acceleration_slice,
] = contact_jacobian

contact_target = (
    -contact_bias_acceleration
)

# ============================================================
# 6. 合并两组等式约束
# ============================================================
equality_matrix = np.vstack(
    [
        dynamics_constraint,
        contact_constraint,
    ]
)

equality_target = np.concatenate(
    [
        dynamics_target,
        contact_target,
    ]
)

# ============================================================
# 7. 构造站立目标函数
# ============================================================
total_mass = pin.computeTotalMass(model)

reference_contact_force = np.tile(
    np.array([
        0.0,
        0.0,
        total_mass * 9.81 / 4.0,
    ]),
    len(GO2_FOOT_FRAMES),
)

reference_decision = np.zeros(
    number_of_decision_variables
)

reference_decision[contact_force_slice] = reference_contact_force

decision_weights = np.concatenate(
    [
        np.full(
            number_of_accelerations,
            1e-3,
        ),
        np.ones(
            number_of_contact_forces,
        ),
        np.full(
            number_of_motor_torques,
            1e-4,
        ),
    ]
)

decision_weights[:6] = 1e5

quadratic_cost = sparse.diags(
    decision_weights,
    format="csc",
)

linear_cost = (
    -decision_weights * reference_decision
)


# ============================================================
# 8. 当前版本暂时没有不等式约束
# ============================================================
inequality_matrix = sparse.csc_matrix(
    (
        0,
        number_of_decision_variables,
    )
)

inequality_lower_bound = np.empty(0)
inequality_upper_bound = np.empty(0)


# ============================================================
# 9. 求解 WBC-QP
# ============================================================
solution, solver_info = solve_with_osqp(
    quadratic_cost=quadratic_cost,
    linear_cost=linear_cost,
    equality_matrix=equality_matrix,
    equality_target=equality_target,
    inequality_matrix=inequality_matrix,
    inequality_lower_bound=inequality_lower_bound,
    inequality_upper_bound=inequality_upper_bound,
)

generalized_acceleration = solution[
    acceleration_slice
]

contact_force_vector = solution[
    contact_force_slice
]

motor_torques = solution[
    motor_torque_slice
]


# ============================================================
# 10. 验证约束残差
# ============================================================

dynamics_residual = (
    mass_matrix @ generalized_acceleration
    + nonlinear_effects
    - selection_matrix.T @ motor_torques
    - contact_jacobian.T @ contact_force_vector
)

contact_acceleration_residual = (
    contact_jacobian @ generalized_acceleration
    + contact_bias_acceleration
)

contact_forces_by_foot = (
    contact_force_vector.reshape(
        len(GO2_FOOT_FRAMES),
        3,
    )
)

print(
    f"OSQP: status={solver_info.status}, "
    f"iterations={solver_info.iter}"
)

print("\nDecision variable size:")
print(number_of_decision_variables)

print("\nEquality matrix shape:")
print(equality_matrix.shape)

print("\nSolved base acceleration:")
print(generalized_acceleration[:6])

print("\nSolved contact forces [N]:")

for foot_name, force in zip(
    GO2_FOOT_FRAMES,
    contact_forces_by_foot,
):
    print(
        f"{foot_name:10s} "
        f"fx={force[0]: .6f}, "
        f"fy={force[1]: .6f}, "
        f"fz={force[2]: .6f}"
    )

print("\nSum of contact forces [N]:")
print(
    np.sum(
        contact_forces_by_foot,
        axis=0,
    )
)

print("\nMaximum absolute motor torque [Nm]:")
print(
    np.max(
        np.abs(motor_torques)
    )
)

print("\nDynamics residual norm:")
print(
    np.linalg.norm(
        dynamics_residual
    )
)

print("\nContact acceleration residual norm:")
print(
    np.linalg.norm(
        contact_acceleration_residual
    )
)
