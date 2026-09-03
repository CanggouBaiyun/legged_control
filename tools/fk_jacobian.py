import pinocchio as pin
import numpy as np

from go2_control.model import(
    build_go2_model,
    nominal_configuration,
)

np.set_printoptions(precision=10, suppress=False)

# ------------------------------------------------------------------
# 1. Load the Go2 model and nominal configuration
# ------------------------------------------------------------------

model = build_go2_model()
q = nominal_configuration(model)


foot_id = model.getFrameId("FR_foot")
joint_id = model.getJointId("FR_thigh_joint")


q_index = model.joints[joint_id].idx_q
v_index = model.joints[joint_id].idx_v


# ============================================================
# 2. 正运动学：计算 p_before = f(q)
# ============================================================

data_before = model.createData()

pin.forwardKinematics(model, data_before, q)
pin.updateFramePlacements(model, data_before)

p_before = data_before.oMf[foot_id].translation.copy()

# ============================================================
# 3. 只让 FR_thigh_joint 变化 epsilon
# ============================================================

epsilon = 1e-7

delta_q = np.zeros(model.nv)
delta_q[v_index] = epsilon

q_after = pin.integrate(model, q, delta_q)

# ============================================================
# 4. 再做一次正运动学：p_after = f(q_after)
# ============================================================

data_after = model.createData()

pin.forwardKinematics(model, data_after, q_after)
pin.updateFramePlacements(model, data_after)

p_after = data_after.oMf[foot_id].translation.copy()

jacobian_finite_difference = (
    p_after - p_before
) / epsilon

# ------------------------------------------------------------------
# 5. Ask Pinocchio to compute the analytical Jacobian
# ------------------------------------------------------------------
data_jacobian = model.createData()

jacobian_6d = pin.computeFrameJacobian(
    model,
    data_jacobian,
    q,
    foot_id,
    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
)

jacobian_linear = jacobian_6d[:3, :]

jacobian_pinocchio = jacobian_linear[:, v_index]

# ============================================================
# 6. 比较结果
# ============================================================

print("q index:", q_index)
print("v index:", v_index)

print("\np_before:")
print(p_before)

print("\np_after:")
print(p_after)

print("\np_after - p_before:")
print(p_after - p_before)

print("\nFinite-difference result:")
print(jacobian_finite_difference)

print("\nPinocchio Jacobian column:")
print(jacobian_pinocchio)

print("\nError norm:")
print(
    np.linalg.norm(
        jacobian_finite_difference
        - jacobian_pinocchio
    )
)