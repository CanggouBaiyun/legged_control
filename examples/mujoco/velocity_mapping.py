from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import(
    GO2_FOOT_FRAMES,
    build_go2_model,
    sdk_joint_mapping,
)

np.set_printoptions(
    precision=10,
    suppress=False,
)

project_root = Path(__file__).resolve().parents[2]

scene_path = (
    project_root
    / "third_party"
    / "unitree_mujoco"
    / "unitree_robots"
    / "go2"
    / "scene.xml"
)

# ============================================================
# 1. 创建两个模型
# ============================================================

mujoco_model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

mujoco_data = mujoco.MjData(
    mujoco_model
)

pinocchio_model = build_go2_model()

pinocchio_data = (
    pinocchio_model.createData()
)

# ============================================================
# 2. 设置 MuJoCo 测试状态
# ============================================================

home_keyframe_id = mujoco.mj_name2id(
    mujoco_model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

mujoco.mj_resetDataKeyframe(
    mujoco_model,
    mujoco_data,
    home_keyframe_id,
)

mujoco_data.ctrl[:] = 0.0

# 给基座设置非零偏航角。
yaw_angle = np.deg2rad(30.0)

mujoco_data.qpos[3:7] = np.array([
    np.cos(yaw_angle / 2.0),
    0.0,
    0.0,
    np.sin(yaw_angle / 2.0),
])

# MuJoCo 基座线速度使用世界坐标系表达。
mujoco_data.qvel[:3] = np.array([
    0.20,
    -0.10,
    0.05,
])

# MuJoCo 基座角速度使用基座坐标系表达。
mujoco_data.qvel[3:6] = np.array([
    0.10,
    -0.20,
    0.30,
])

test_joint_velocities = {
    "FR_hip_joint": 0.10,
    "FR_thigh_joint": -0.20,
    "FR_calf_joint": 0.30,

    "FL_hip_joint": -0.15,
    "FL_thigh_joint": 0.25,
    "FL_calf_joint": -0.35,

    "RR_hip_joint": 0.12,
    "RR_thigh_joint": -0.22,
    "RR_calf_joint": 0.32,

    "RL_hip_joint": -0.17,
    "RL_thigh_joint": 0.27,
    "RL_calf_joint": -0.37,
}

for joint_name, joint_velocity in test_joint_velocities.items():
    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    mujoco_qvel_index = (
        mujoco_model.jnt_dofadr[mujoco_joint_id]
    )

    mujoco_data.qvel[mujoco_qvel_index] = joint_velocity

    mujoco.mj_forward(mujoco_model,mujoco_data)

# ============================================================
# 3. 转换构型 q
# ============================================================
pinocchio_q = pin.neutral(
    pinocchio_model
)

pinocchio_q[:3] = (
    mujoco_data.qpos[:3]
)

# MuJoCo:    [w, x, y, z]
# Pinocchio: [x, y, z, w]
pinocchio_q[3:7] = np.array([
    mujoco_data.qpos[4],
    mujoco_data.qpos[5],
    mujoco_data.qpos[6],
    mujoco_data.qpos[3],
])

joint_mapping = sdk_joint_mapping(
    pinocchio_model
)

for entry in joint_mapping:
    joint_name = entry["name"]

    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    mujoco_qpos_index = int(
        mujoco_model.jnt_qposadr[
            mujoco_joint_id
        ]
    )

    pinocchio_q_index = int(
        entry["q_index"]
    )

    pinocchio_q[pinocchio_q_index] = mujoco_data.qpos[mujoco_qpos_index]

# ============================================================
# 4. 转换广义速度 v
# ============================================================

pinocchio_v = np.zeros(pinocchio_model.nv)

base_body_id = mujoco.mj_name2id(
    mujoco_model,
    mujoco.mjtObj.mjOBJ_BODY,
    "base_link",
)

world_rotation_base = (
    mujoco_data.xmat[base_body_id].reshape(3, 3).copy()
)

# MuJoCo 的基座线速度：世界坐标系
# Pinocchio 的基座线速度：基座坐标系
pinocchio_v[:3] = (
    world_rotation_base.T @ mujoco_data.qvel[:3]
)

# 两个库的基座角速度都用基座坐标系表达。
pinocchio_v[3:6] = (
    mujoco_data.qvel[3:6]
)

for entry in joint_mapping:
    joint_name = entry["name"]

    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    mujoco_qvel_index = int(
        mujoco_model.jnt_dofadr[mujoco_joint_id]
    )

    pinocchio_v_index = int(
        entry["v_index"]
    )

    pinocchio_v[pinocchio_v_index] = mujoco_data.qvel[mujoco_qvel_index]

# ============================================================
# 5. 计算 Pinocchio Jacobian
# ============================================================

pin.computeJointJacobians(
    pinocchio_model,
    pinocchio_data,
    pinocchio_q,
)

pin.updateFramePlacements(
    pinocchio_model,
    pinocchio_data,
)


# ============================================================
# 6. 比较四个足端速度
# ============================================================

maximum_velocity_error = 0.0


print("MuJoCo base linear velocity in world:")
print(mujoco_data.qvel[:3])

print("\nPinocchio base linear velocity in body:")
print(pinocchio_v[:3])

print("\nBase angular velocity in body:")
print(pinocchio_v[3:6])

print("\nFoot velocity comparison:")


for foot_name in GO2_FOOT_FRAMES:
    # ---------- MuJoCo ----------

    mujoco_body_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_BODY,
        foot_name,
    )

    foot_position = (
        mujoco_data.xpos[mujoco_body_id].copy()
    )

    mujoco_linear_jacobian = np.zeros(
        (3, mujoco_model.nv)
    )

    mujoco_angular_jacobian = np.zeros(
        (3, mujoco_model.nv)
    )

    mujoco.mj_jac(
        mujoco_model,
        mujoco_data,
        mujoco_linear_jacobian,
        mujoco_angular_jacobian,
        foot_position,
        mujoco_body_id,
    )

    mujoco_foot_velocity = (
        mujoco_linear_jacobian @ mujoco_data.qvel
    )

    # ---------- Pinocchio ----------

    pinocchio_frame_id = (
        pinocchio_model.getFrameId(foot_name)
    )

    pinocchio_frame_jacobian = (
        pin.getFrameJacobian(
            pinocchio_model,
            pinocchio_data,
            pinocchio_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
    )

    pinocchio_foot_velocity = (
        pinocchio_frame_jacobian[:3, :] @ pinocchio_v
    )

     # ---------- 比较 ----------

    velocity_difference = (
        mujoco_foot_velocity - pinocchio_foot_velocity
    )

    velocity_error_norm = np.linalg.norm(
        velocity_difference
    )

    maximum_velocity_error = max(
        maximum_velocity_error,
        velocity_error_norm,
    )

    print(f"\n{foot_name}")

    print("MuJoCo:")
    print(mujoco_foot_velocity)

    print("Pinocchio:")
    print(pinocchio_foot_velocity)

    print("Difference:")
    print(velocity_difference)

    print("Error norm:")
    print(velocity_error_norm)


print("\nMaximum foot velocity error [m/s]:")
print(maximum_velocity_error)
