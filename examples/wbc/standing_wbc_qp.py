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

    contact_jacobian_time_variation = np.vstack(
        foot_jacobian_time_variations
    )

    contact_bias_acceleration = (
        contact_jacobian_time_variation @ v
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
    np.triu(mass_matrix_upper) + np.triu(mass_matrix_upper, k = 1).T
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

current_base_height = q[2]
current_base_vertical_velocity = v[2]

desired_base_height = 0.28
#desired_base_height = 0.27
#desired_base_height = 0.26
desired_base_vertical_velocity = 0.0

height_kp = 50.0
height_kd = 10.0

height_error = (
    desired_base_height - current_base_height
)

vertical_velocity_error = (
    desired_base_vertical_velocity - current_base_vertical_velocity
)

desired_vertical_acceleration = (
    height_kp * height_error + height_kd * vertical_velocity_error
)

desired_vertical_acceleration = np.clip(
    desired_vertical_acceleration,
    -2.0,
    2.0,
)

desired_base_acceleration = np.zeros(6)
desired_base_acceleration[2] = (
    desired_vertical_acceleration
)

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
reference_decision[:6] = desired_base_acceleration

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
# 8. 构造摩擦、法向力和电机力矩约束
# ============================================================

friction_coefficient = 0.5
maximum_normal_force = 100.0

#单脚接触力顺序：[fx, fy, fz]
friction_block = np.array([
    [1.0, 0.0, -friction_coefficient],
    [-1.0, 0.0, -friction_coefficient],
    [0.0, 1.0, -friction_coefficient],
    [0.0, -1.0, -friction_coefficient],
    [0.0, 0.0, 1.0],
])

friction_force_matrix = sparse.block_diag(
    [friction_block] * len(GO2_FOOT_FRAMES),
    format="csc",
).toarray()

friction_constraint = np.zeros(
    (
        friction_force_matrix.shape[0],
        number_of_decision_variables,
    )
)

friction_constraint[
    :,
    contact_force_slice,
] = friction_force_matrix

single_foot_friction_lower_bound = np.array([
    -np.inf,
    -np.inf,
    -np.inf,
    -np.inf,
    0.0,
])

single_foot_friction_upper_bound = np.array([
    0.0,
    0.0,
    0.0,
    0.0,
    maximum_normal_force,
])

friction_lower_bound = np.tile(
    single_foot_friction_lower_bound,
    len(GO2_FOOT_FRAMES),
)

friction_upper_bound = np.tile(
    single_foot_friction_upper_bound,
    len(GO2_FOOT_FRAMES),
)

motor_torque_limits = (
    model.effortLimit[6:].copy()
)

motor_torque_constraint = np.zeros(
    (
        number_of_motor_torques,
        number_of_decision_variables,
    )
)

motor_torque_constraint[
    :,
    motor_torque_slice,
] = np.eye(
    number_of_motor_torques
)

motor_torque_lower_bound = (
    -motor_torque_limits
)

motor_torque_upper_bound = (
    motor_torque_limits
)

inequality_matrix = sparse.csc_matrix(
    np.vstack(
        [
            friction_constraint,
            motor_torque_constraint,
        ]
    )
)

inequality_lower_bound = np.concatenate(
    [
        friction_lower_bound,
        motor_torque_lower_bound,
    ]
)


inequality_upper_bound = np.concatenate(
    [
        friction_upper_bound,
        motor_torque_upper_bound,
    ]
)


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

base_acceleration_error = (
    generalized_acceleration[:6] - desired_base_acceleration
)


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

normal_forces = (
    contact_forces_by_foot[:, 2]
)

friction_utilization = (
    np.max(
        np.abs(
            contact_forces_by_foot[:, :2]
        ),
        axis=1,
    )
    / (
        friction_coefficient
        * normal_forces
    )
)

motor_torque_utilization = (
    np.abs(motor_torques)
    / motor_torque_limits
)

print(
    f"OSQP: status={solver_info.status}, "
    f"iterations={solver_info.iter}"
)

print("\nDecision variable size:")
print(number_of_decision_variables)

print("\nEquality matrix shape:")
print(equality_matrix.shape)

print("\nInequality matrix shape:")
print(inequality_matrix.shape)

print("\nDesired base acceleration:")
print(desired_base_acceleration)

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

print("\nMinimum normal force [N]:")
print(np.min(normal_forces))

print("\nMaximum normal force [N]:")
print(np.max(normal_forces))

print("\nMaximum friction utilization:")
print(np.max(friction_utilization))

print("\nMaximum motor torque utilization:")
print(np.max(motor_torque_utilization))

print("\nBase acceleration tracking error:")
print(base_acceleration_error)

print("\nBase acceleration tracking error norm:")
print(np.linalg.norm(base_acceleration_error))
