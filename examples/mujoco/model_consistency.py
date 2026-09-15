from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    build_go2_simulation_model,
    sdk_joint_mapping,
)
from go2_control.simulation.mujoco_bridge import (
    MujocoPinocchioBridge,
)


np.set_printoptions(
    precision=8,
    suppress=True,
)


# ============================================================
# 1. 创建 MuJoCo 和 Pinocchio 模型
# ============================================================

project_root = Path(__file__).resolve().parents[2]

scene_path = (
    project_root
    / "third_party"
    / "unitree_mujoco"
    / "unitree_robots"
    / "go2"
    / "scene.xml"
)

mujoco_model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

mujoco_data = mujoco.MjData(
    mujoco_model
)

pinocchio_model = build_go2_model()

pinocchio_data = pinocchio_model.createData()

bridge = MujocoPinocchioBridge(
    mujoco_model,
    pinocchio_model,
)

# ============================================================
# 2. 将 MuJoCo 设置为 home 构型
# ============================================================

home_keyframe_id = mujoco.mj_name2id(
    mujoco_model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

if home_keyframe_id == -1:
    raise RuntimeError(
        "MuJoCo model does not contain the home keyframe."
    )

mujoco.mj_resetDataKeyframe(
    mujoco_model,
    mujoco_data,
    home_keyframe_id,
)

# home 中保存了电机输入，但这里仅检查模型，所以清零。
mujoco_data.ctrl[:] = 0.0

mujoco.mj_forward(
    mujoco_model,
    mujoco_data,
)

pinocchio_q, pinocchio_v = bridge.state(
    mujoco_data
)


# ============================================================
# 3. 比较总质量
# ============================================================

mujoco_total_mass = float(
    np.sum(mujoco_model.body_mass)
)

pinocchio_total_mass = float(
    pin.computeTotalMass(pinocchio_model)
)

mass_difference = (
    mujoco_total_mass
    - pinocchio_total_mass
)

relative_mass_difference = (
    abs(mass_difference)
    / pinocchio_total_mass
    * 100.0
)

print("Total mass comparison:")
print(f"MuJoCo:     {mujoco_total_mass:.9f} kg")
print(f"Pinocchio:  {pinocchio_total_mass:.9f} kg")
print(f"Difference: {mass_difference:.9f} kg")
print(
    "Relative difference: "
    f"{relative_mass_difference:.3f} %"
)


# ============================================================
# 4. 比较整机质心
# ============================================================

base_body_id = mujoco.mj_name2id(
    mujoco_model,
    mujoco.mjtObj.mjOBJ_BODY,
    "base_link",
)

if base_body_id == -1:
    raise RuntimeError(
        "MuJoCo model does not contain base_link."
    )

# base_link 子树包含整个机器人。
mujoco_center_of_mass = (
    mujoco_data.subtree_com[base_body_id].copy()
)

pinocchio_center_of_mass = pin.centerOfMass(
    pinocchio_model,
    pinocchio_data,
    pinocchio_q,
).copy()

center_of_mass_difference = (
    mujoco_center_of_mass
    - pinocchio_center_of_mass
)

print("\nCenter of mass in world frame [m]:")

print("MuJoCo:")
print(mujoco_center_of_mass)

print("Pinocchio:")
print(pinocchio_center_of_mass)

print("Difference:")
print(center_of_mass_difference)

print("Difference norm [m]:")
print(
    np.linalg.norm(
        center_of_mass_difference
    )
)


# ============================================================
# 5. 比较关节被动动力学参数
# ============================================================

joint_mapping = sdk_joint_mapping(
    pinocchio_model
)

print("\nJoint passive parameters:")
print(
    f"{'joint':22s}"
    f"{'MJ damping':>12s}"
    f"{'MJ friction':>13s}"
    f"{'MJ armature':>13s}"
    f"{'Pin damping':>14s}"
    f"{'Pin friction':>14s}"
    f"{'Pin armature':>14s}"
)

for entry in joint_mapping:
    joint_name = entry["name"]

    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    mujoco_v_index = int(
        mujoco_model.jnt_dofadr[
            mujoco_joint_id
        ]
    )

    pinocchio_v_index = int(
        entry["v_index"]
    )

    print(
        f"{joint_name:22s}"
        f"{mujoco_model.dof_damping[mujoco_v_index]:12.5f}"
        f"{mujoco_model.dof_frictionloss[mujoco_v_index]:13.5f}"
        f"{mujoco_model.dof_armature[mujoco_v_index]:13.5f}"
        f"{pinocchio_model.damping[pinocchio_v_index]:14.5f}"
        f"{pinocchio_model.friction[pinocchio_v_index]:14.5f}"
        f"{pinocchio_model.armature[pinocchio_v_index]:14.5f}"
    )

# ============================================================
# 6. 比较静止状态下的重力广义力
# ============================================================
if np.linalg.norm(pinocchio_v) > 1e-12:
    raise RuntimeError(
        "Gravity comparison requires zero velocity."
    )

# Pinocchio：
# h(q, v) = C(q, v)v + g(q)
#
# 当前 v=0，所以：
# h(q, 0) = g(q)
pinocchio_bias_force = pin.nonLinearEffects(
    pinocchio_model,
    pinocchio_data,
    pinocchio_q,
    pinocchio_v,
)

# MuJoCo 的 qfrc_bias 包含重力、科里奥利力和离心力。
# 当前速度为零，因此这里同样主要是重力广义力。
mujoco_bias_force = (
    mujoco_data.qfrc_bias.copy()
)

# 将 Pinocchio 的关节部分按照关节名称，
# 排列到 MuJoCo qvel 的顺序中。
pinocchio_bias_in_mujoco_order = np.zeros(
    mujoco_model.nv
)

# 当前 home 构型的基座姿态为单位四元数，
# 两边基座广义力可以直接比较。
pinocchio_bias_in_mujoco_order[:6] = (
    pinocchio_bias_force[:6]
)

for entry in joint_mapping:
    joint_name = entry["name"]

    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    mujoco_v_index = int(
        mujoco_model.jnt_dofadr[
            mujoco_joint_id
        ]
    )

    pinocchio_v_index = int(
        entry["v_index"]
    )

    pinocchio_bias_in_mujoco_order[
        mujoco_v_index
    ] = pinocchio_bias_force[
        pinocchio_v_index
    ]


bias_force_difference = (
    mujoco_bias_force
    - pinocchio_bias_in_mujoco_order
)

gravity_magnitude = float(
    -mujoco_model.opt.gravity[2]
)

expected_vertical_difference = (
    mass_difference
    * gravity_magnitude
)


print("\nGravity generalized force comparison:")

print("MuJoCo base part:")
print(mujoco_bias_force[:6])

print("Pinocchio base part:")
print(pinocchio_bias_in_mujoco_order[:6])

print("Base difference:")
print(bias_force_difference[:6])

print("\nExpected vertical difference from mass [N]:")
print(expected_vertical_difference)

print("Actual vertical generalized-force difference [N]:")
print(bias_force_difference[2])

print("\nJoint gravity-force difference:")
print(bias_force_difference[6:])

print("Full difference norm:")
print(
    np.linalg.norm(
        bias_force_difference
    )
)


# ============================================================
# 7. 比较质量矩阵
# ============================================================

# 构造速度坐标转换矩阵：
#
# v_pinocchio = T @ v_mujoco
velocity_transform = np.zeros(
    (
        pinocchio_model.nv,
        mujoco_model.nv,
    )
)

world_rotation_base = pin.XYZQUATToSE3(
    pinocchio_q[:7]
).rotation

# MuJoCo 基座线速度使用世界坐标系，
# Pinocchio 使用基座坐标系。
velocity_transform[:3, :3] = (
    world_rotation_base.T
)

# 两边基座角速度都使用基座坐标系。
velocity_transform[3:6, 3:6] = np.eye(3)

# 根据关节名称建立关节速度映射。
for entry in joint_mapping:
    joint_name = entry["name"]

    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    mujoco_v_index = int(
        mujoco_model.jnt_dofadr[
            mujoco_joint_id
        ]
    )

    pinocchio_v_index = int(
        entry["v_index"]
    )

    velocity_transform[
        pinocchio_v_index,
        mujoco_v_index,
    ] = 1.0


# ---------- Pinocchio 质量矩阵 ----------

pinocchio_mass_matrix_upper = pin.crba(
    pinocchio_model,
    pinocchio_data,
    pinocchio_q,
).copy()

# CRBA 主要写入矩阵上三角，需要手动补成对称矩阵。
pinocchio_mass_matrix = np.triu(
    pinocchio_mass_matrix_upper
)

pinocchio_mass_matrix += np.triu(
    pinocchio_mass_matrix_upper,
    1,
).T


# 将 Pinocchio 质量矩阵转换到 MuJoCo 速度坐标中。
pinocchio_mass_matrix_in_mujoco_coordinates = (
    velocity_transform.T
    @ pinocchio_mass_matrix
    @ velocity_transform
)


# ---------- MuJoCo 质量矩阵 ----------

mujoco_mass_matrix = np.zeros(
    (
        mujoco_model.nv,
        mujoco_model.nv,
    )
)

mujoco.mj_fullM(
    mujoco_model,
    mujoco_mass_matrix,
    mujoco_data.qM,
)


# MuJoCo 的 armature 会加到质量矩阵对角线上。
mujoco_mass_matrix_without_armature = (
    mujoco_mass_matrix
    - np.diag(mujoco_model.dof_armature)
)


# ---------- 比较结果 ----------

mass_matrix_difference = (
    mujoco_mass_matrix
    - pinocchio_mass_matrix_in_mujoco_coordinates
)

mass_matrix_difference_without_armature = (
    mujoco_mass_matrix_without_armature
    - pinocchio_mass_matrix_in_mujoco_coordinates
)

joint_slice = slice(6, None)

print("\nJoint mass-matrix diagonal:")

print("MuJoCo with armature:")
print(
    np.diag(mujoco_mass_matrix)[joint_slice]
)

print("MuJoCo without armature:")
print(
    np.diag(
        mujoco_mass_matrix_without_armature
    )[joint_slice]
)

print("Pinocchio:")
print(
    np.diag(
        pinocchio_mass_matrix_in_mujoco_coordinates
    )[joint_slice]
)


print("\nFull mass-matrix difference norm:")

print("With MuJoCo armature:")
print(
    np.linalg.norm(
        mass_matrix_difference
    )
)

print("Without MuJoCo armature:")
print(
    np.linalg.norm(
        mass_matrix_difference_without_armature
    )
)


print("\nJoint block difference norm:")

print("With MuJoCo armature:")
print(
    np.linalg.norm(
        mass_matrix_difference[
            joint_slice,
            joint_slice,
        ]
    )
)

print("Without MuJoCo armature:")
print(
    np.linalg.norm(
        mass_matrix_difference_without_armature[
            joint_slice,
            joint_slice,
        ]
    )
)

# ============================================================
# 8. 验证同源 MJCF Pinocchio 模型
# ============================================================

simulation_pinocchio_model = (
    build_go2_simulation_model()
)

simulation_pinocchio_data = (
    simulation_pinocchio_model.createData()
)

simulation_bridge = MujocoPinocchioBridge(
    mujoco_model,
    simulation_pinocchio_model,
)

simulation_q, simulation_v = (
    simulation_bridge.state(mujoco_data)
)


simulation_total_mass = float(
    pin.computeTotalMass(
        simulation_pinocchio_model
    )
)

simulation_bias_force = pin.nonLinearEffects(
    simulation_pinocchio_model,
    simulation_pinocchio_data,
    simulation_q,
    simulation_v,
)

simulation_mass_matrix_upper = pin.crba(
    simulation_pinocchio_model,
    simulation_pinocchio_data,
    simulation_q,
).copy()

simulation_mass_matrix = np.triu(
    simulation_mass_matrix_upper
)

simulation_mass_matrix += np.triu(
    simulation_mass_matrix_upper,
    1,
).T

simulation_mass_matrix_in_mujoco_coordinates = (
    velocity_transform.T
    @ simulation_mass_matrix
    @ velocity_transform
)

simulation_bias_force_in_mujoco_coordinates = (
    velocity_transform.T
    @ simulation_bias_force
)


print("\nSame-MJCF model verification:")

print("MuJoCo total mass [kg]:")
print(mujoco_total_mass)

print("MJCF Pinocchio total mass [kg]:")
print(simulation_total_mass)

print("Mass difference [kg]:")
print(
    mujoco_total_mass
    - simulation_total_mass
)

print("\nBias-force difference norm:")
print(
    np.linalg.norm(
        mujoco_bias_force
        - simulation_bias_force_in_mujoco_coordinates
    )
)

print("Mass-matrix difference norm:")
print(
    np.linalg.norm(
        mujoco_mass_matrix
        - simulation_mass_matrix_in_mujoco_coordinates
    )
)

print("Maximum absolute mass-matrix difference:")
print(
    np.max(
        np.abs(
            mujoco_mass_matrix
            - simulation_mass_matrix_in_mujoco_coordinates
        )
    )
)