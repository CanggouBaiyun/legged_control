import numpy as np
import pinocchio as pin

from go2_control.model import(
    build_go2_model,
    nominal_configuration,
)

np.set_printoptions(
    precision=10,
    suppress=False,
)

model = build_go2_model()
q = nominal_configuration(model)

rng = np.random.default_rng(7)

velocity = rng.normal(
    scale = 0.1,
    size = model.nv,
)

acceleration = rng.normal(
    scale=0.2,
    size=model.nv,
)

# ============================================================
# 1. 直接使用 RNEA 求逆动力学
# ============================================================

rnea_data = model.createData()

generalized_force_rnea = pin.rnea(
    model,
    rnea_data,
    q,
    velocity,
    acceleration,
).copy()


# ============================================================
# 2. 分别计算 M(q) 和 h(q, v)
# ============================================================
crba_data = model.createData()

mass_matrix_upper = pin.crba(
    model,
    crba_data,
    q,
).copy()

mass_matrix = (
    np.triu(mass_matrix_upper) + np.triu(mass_matrix_upper, k=1).T
)

nonlinear_data = model.createData()

nonlinear_effects = pin.nonLinearEffects(
    model,
    nonlinear_data,
    q,
    velocity,
).copy()

# ============================================================
# 3. 使用 M(q) * acceleration + h(q, v)
# ============================================================

inertial_force = (
    mass_matrix @ acceleration
)

generalized_force_equation = (
    inertial_force + nonlinear_effects
)

# ============================================================
# 4. 将 h(q, v) 分成重力和速度相关项
# ============================================================

gravity_data = model.createData()

gravity = pin.computeGeneralizedGravity(
    model,
    gravity_data,
    q,
).copy()

coriolis_centrifugal = (
    nonlinear_effects - gravity
)

# ============================================================
# 5. 比较两种逆动力学结果
# ============================================================

difference = (
    generalized_force_rnea - generalized_force_equation
)

print("Generalized velocity v:")
print(velocity)

print("\nGeneralized acceleration a:")
print(acceleration)

print("\nInertial term M(q) @ a:")
print(inertial_force)

print("\nGravity term g(q):")
print(gravity)

print("\nCoriolis and centrifugal term:")
print(coriolis_centrifugal)

print("\nNonlinear effects h(q, v):")
print(nonlinear_effects)

print("\nGeneralized force from RNEA:")
print(generalized_force_rnea)

print("\nGeneralized force from M @ a + h:")
print(generalized_force_equation)

print("\nDifference:")
print(difference)

print("\nDifference norm:")
print(np.linalg.norm(difference))

print("\nMaximum absolute difference:")
print(np.max(np.abs(difference)))



