import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    nominal_configuration,
)

np.set_printoptions(
    precision=6,
    suppress=True,
)

model = build_go2_model()
q = nominal_configuration(model)

crba_data = model.createData()
#crba算法计算广义质量矩阵，但是只保证上三角
mass_matrix_upper = pin.crba(
    model,
    crba_data,
    q,
).copy()

mass_matrix = (
    np.triu(mass_matrix_upper) + np.triu(mass_matrix_upper, k=1).T
)

base_base_block = mass_matrix[:6, :6]
base_joint_block = mass_matrix[:6, 6:]
joint_base_block = mass_matrix[6:, :6]
joint_joint_block = mass_matrix[6:, 6:]

#检查质量矩阵的完整性
symmetry_error = np.linalg.norm(
    mass_matrix - mass_matrix.T
)
#计算实对称矩阵的特征值
eigenvalues = np.linalg.eigvalsh(
    mass_matrix
)
#检查两个耦合块是否互为转置
coupling_symmetry_error = np.linalg.norm(
    base_joint_block - joint_base_block.T
)

rng = np.random.default_rng(7)

test_velocity = rng.normal(
    scale=0.2,
    size=model.nv,
)
#手动使用质量矩阵计算动能
kinetic_energy_from_matrix = (
    0.5 * test_velocity @ mass_matrix @ test_velocity
)

energy_data = model.createData()
#pinocchio计算动能
kinetic_energy_from_pinocchio = (
    pin.computeKineticEnergy(
        model,
        energy_data,
        q,
        test_velocity,
    )
)

print("model.nq:")
print(model.nq)

print("\nmodel.nv:")
print(model.nv)

print("\nMass matrix shape:")
print(mass_matrix.shape)

print("\nBase-base block M_bb:")
print(base_base_block)

print("\nJoint-joint diagonal:")
print(np.diag(joint_joint_block))

print("\nBase-joint coupling norm:")
print(np.linalg.norm(base_joint_block))

print("\nCoupling symmetry error:")
print(coupling_symmetry_error)

print("\nFull symmetry error:")
print(symmetry_error)

print("\nMinimum eigenvalue:")
print(eigenvalues.min())

print("\nMaximum eigenvalue:")
print(eigenvalues.max())

print("\nTest generalized velocity:")
print(test_velocity)

print("\nKinetic energy from 0.5 * v.T * M * v:")
print(kinetic_energy_from_matrix)

print("\nKinetic energy from Pinocchio:")
print(kinetic_energy_from_pinocchio)

print("\nKinetic energy difference:")
print(
    kinetic_energy_from_matrix
    - kinetic_energy_from_pinocchio
)