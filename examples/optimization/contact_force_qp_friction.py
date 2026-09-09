import numpy as np
import pinocchio as pin
from scipy import sparse

from go2_control.model import(
    GO2_FOOT_FRAMES,
    build_go2_model,
    nominal_configuration,
)

from go2_control.optimization.qp_solvers import(
    solve_with_osqp,
    solve_with_proxqp,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

model = build_go2_model()
q = nominal_configuration(model)

v = np.zeros(model.nv)
desired_acceleration = np.zeros(model.nv)

#希望浮动基座沿世界x、z轴向上加速0.5 m/s²
desired_acceleration[0] = 0.5
desired_acceleration[2] = 0.5

dynamics_data = model.createData()

mass_matrix_upper = pin.crba(
    model,
    dynamics_data,
    q,
)

mass_matrix = (
    np.triu(mass_matrix_upper) + np.triu(mass_matrix_upper, k=1).T
)

nonlinear_effects = pin.nonLinearEffects(
    model,
    dynamics_data,
    q,
    v,
).copy()

required_generalized_force = (
    mass_matrix @ desired_acceleration + nonlinear_effects
)

foot_jacobians = []

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

contact_jacobian = np.vstack(
    foot_jacobians
)

# ============================================================
# 2. 求四只脚的接触力
# ============================================================

base_force_mapping = (
   contact_jacobian.T[:6, :].copy()
)

number_of_feet = len(GO2_FOOT_FRAMES)

number_of_contact_force_variables = (
    3 * number_of_feet
)

reference_force_per_foot = (
    required_generalized_force[:3] / number_of_feet
)

reference_contact_forces = np.tile(
    reference_force_per_foot,
    number_of_feet,
)
# ============================================================
# 3. 构造并求解 12 维接触力 QP
# ============================================================

#目标函数
quadratic_cost = sparse.eye(
    number_of_contact_force_variables,
    format = "csc",
)
linear_cost = -reference_contact_forces

# ============================================================
# 4. 动力学约束和线性化摩擦锥
# ============================================================

friction_coefficient = 0.05
maximum_normal_force = 100.0

dynamics_constraint = sparse.csc_matrix(
    base_force_mapping
)

# 单脚变量顺序：[fx, fy, fz]
#
# 第1行： fx - mu*fz <= 0
# 第2行：-fx - mu*fz <= 0
# 第3行： fy - mu*fz <= 0
# 第4行：-fy - mu*fz <= 0
# 第5行： 0 <= fz <= fz_max
friction_block = np.array([
    [ 1.0,  0.0, -friction_coefficient],
    [-1.0,  0.0, -friction_coefficient],
    [ 0.0,  1.0, -friction_coefficient],
    [ 0.0, -1.0, -friction_coefficient],
    [ 0.0,  0.0,  1.0],
])

friction_constraint = sparse.block_diag(
    [friction_block] * number_of_feet,
    format="csc",
)


single_foot_lower_bound = np.array([
    -np.inf,
    -np.inf,
    -np.inf,
    -np.inf,
    0.0,
])

single_foot_upper_bound = np.array([
    0.0,
    0.0,
    0.0,
    0.0,
    maximum_normal_force,
])

friction_lower_bound = np.tile(
    single_foot_lower_bound,
    number_of_feet,
)

friction_upper_bound = np.tile(
    single_foot_upper_bound,
    number_of_feet,
)


contact_force_vector, osqp_info = solve_with_osqp(
    quadratic_cost=quadratic_cost,
    linear_cost=linear_cost,
    equality_matrix=dynamics_constraint,
    equality_target=required_generalized_force[:6],
    inequality_matrix=friction_constraint,
    inequality_lower_bound=friction_lower_bound,
    inequality_upper_bound=friction_upper_bound,
)

proxqp_contact_force_vector, proxqp_info = (
    solve_with_proxqp(
        quadratic_cost=quadratic_cost,
        linear_cost=linear_cost,
        equality_matrix=dynamics_constraint,
        equality_target=required_generalized_force[:6],
        inequality_matrix=friction_constraint,
        inequality_lower_bound=friction_lower_bound,
        inequality_upper_bound=friction_upper_bound,
    )
)

solver_solution_difference = np.linalg.norm(
    contact_force_vector
    - proxqp_contact_force_vector
)




generalized_contact_force = (
    contact_jacobian.T @ contact_force_vector
)

# ============================================================
# 4. 根据关节动力学求电机力矩
# ============================================================

number_of_actuated_joints = model.nv - 6

selection_matrix = np.zeros((number_of_actuated_joints, model.nv))

selection_matrix[:, 6:] = np.eye(number_of_actuated_joints)

motor_torques = (
    required_generalized_force[6:] - generalized_contact_force[6:]
)

actuated_generalized_force = (
    selection_matrix.T @ motor_torques
)

# ============================================================
# 5. 验证完整浮动基动力学
# ============================================================

dynamics_residual = (
    required_generalized_force - actuated_generalized_force - generalized_contact_force
)

contact_forces_by_foot = (
    contact_force_vector.reshape(
        number_of_feet,
        3,
    )
)

friction_utilization = (
    np.max(
        np.abs(
            contact_forces_by_foot[:, :2]
        ),
        axis=1,
    )/(friction_coefficient * contact_forces_by_foot[:, 2])
)

contact_force_tracking_cost = (
    0.5
    * np.linalg.norm(
        contact_force_vector
        - reference_contact_forces
    ) ** 2
)

base_dynamics_residual = (
    required_generalized_force[:6]
    - generalized_contact_force[:6]
)

print(
    f"OSQP: status={osqp_info.status}, "
    f"iterations={osqp_info.iter}, "
    f"primal_residual={osqp_info.prim_res:.3e}, "
    f"dual_residual={osqp_info.dual_res:.3e}"
)

print(
    f"ProxQP: "
    f"status={proxqp_info.status}, "
    f"iterations={proxqp_info.iter}"
)

print("\nOSQP-ProxQP solution difference:")
print(solver_solution_difference)

print(
    "\nBase force mapping: "
    f"shape={base_force_mapping.shape}, "
    f"rank={np.linalg.matrix_rank(base_force_mapping)}"
)

print("\nSolved contact forces [N]:")

for foot_name, force, utilization in zip(
    GO2_FOOT_FRAMES,
    contact_forces_by_foot,
    friction_utilization,
):
    print(
    f"{foot_name:10s} "
    f"fx={force[0]: .6f}, "
    f"fy={force[1]: .6f}, "
    f"fz={force[2]: .6f}, "
    f"friction={utilization:.3f}"
)

print("\nSum of contact forces [N]:")
print(
    np.sum(
        contact_forces_by_foot,
        axis=0,
    )
)

print("\nRequired resultant force [N]:")
print(required_generalized_force[:3])

print("\nContact-force tracking cost:")
print(contact_force_tracking_cost)

print("\nBase dynamics residual norm:")
print(np.linalg.norm(base_dynamics_residual))

print("\nFull dynamics residual norm:")
print(np.linalg.norm(dynamics_residual))

print("\nMaximum absolute motor torque [Nm]:")
print(np.max(np.abs(motor_torques)))