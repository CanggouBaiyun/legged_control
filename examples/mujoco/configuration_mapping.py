from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import (
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
# 1. 创建 MuJoCo 和 Pinocchio 模型
# ============================================================

mujoco_model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

mujoco_data = mujoco.MjData(
    mujoco_model
)

pinocchio_model = build_go2_model()

pinocchio_data = pinocchio_model.createData()


# ============================================================
# 2. 将 MuJoCo 设置到 home 构型
# ============================================================
home_keyframe_id = mujoco.mj_name2id(
    mujoco_model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

if home_keyframe_id == -1:
    raise RuntimeError(
        "The MJCF model does not contain the 'home' keyframe."
    )

mujoco.mj_resetDataKeyframe(
    mujoco_model,
    mujoco_data,
    home_keyframe_id,
)

# home 关键帧包含非零电机输入，必须清零。
mujoco_data.ctrl[:] = 0.0

# ============================================================
# 3. 给四条腿设置不同的测试角度
# ============================================================

test_joint_positions = {
    "FR_hip_joint": 0.10,
    "FR_thigh_joint": 0.80,
    "FR_calf_joint": -1.60,

    "FL_hip_joint": -0.10,
    "FL_thigh_joint": 0.85,
    "FL_calf_joint": -1.70,

    "RR_hip_joint": 0.05,
    "RR_thigh_joint": 1.00,
    "RR_calf_joint": -2.00,

    "RL_hip_joint": -0.05,
    "RL_thigh_joint": 1.10,
    "RL_calf_joint": -2.10,
}

for joint_name, joint_position in test_joint_positions.items():
    mujoco_joint_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_name,
    )

    if mujoco_joint_id == -1:
        raise KeyError(
            f"MuJoCo joint not found: {joint_name}"
        )

    mujoco_qpos_index = (
        mujoco_model.jnt_qposadr[
            mujoco_joint_id
        ]
    )

    mujoco_data.qpos[
        mujoco_qpos_index
    ] = joint_position

mujoco.mj_forward(
    mujoco_model,
    mujoco_data,
)

# ============================================================
# 4. MuJoCo qpos 转换成 Pinocchio q
# ============================================================
pinocchio_q = pin.neutral(
    pinocchio_model
)

# 浮动基座位置
pinocchio_q[:3] = mujoco_data.qpos[:3]

# MuJoCo quaternion:    [w, x, y, z]
# Pinocchio quaternion: [x, y, z, w]
pinocchio_q[3:7] = np.array([
    mujoco_data.qpos[4],
    mujoco_data.qpos[5],
    mujoco_data.qpos[6],
    mujoco_data.qpos[3],
])

# 按照关节名称转换，不依赖数组排列顺序。
joint_mapping = sdk_joint_mapping(
    pinocchio_model
)

print("Joint position mapping:")
print(
    f"{'joint':22s} "
    f"{'MuJoCo q index':>16s} "
    f"{'Pinocchio q index':>20s}"
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
    print(
        f"{joint_name:22s} "
        f"{mujoco_qpos_index:16d} "
        f"{pinocchio_q_index:20d}"
    )

# ============================================================
# 5. Pinocchio 正运动学
# ============================================================

pin.forwardKinematics(
    pinocchio_model,
    pinocchio_data,
    pinocchio_q,
)

pin.updateFramePlacements(
    pinocchio_model,
    pinocchio_data,
)


# ============================================================
# 6. 比较四个足端位置
# ============================================================

maximum_foot_position_error = 0.0


print("\nFoot position comparison:")

for foot_name in GO2_FOOT_FRAMES:
    mujoco_body_id = mujoco.mj_name2id(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_BODY,
        foot_name,
    )

    if mujoco_body_id == -1:
        raise KeyError(f"MuJoCo body not found: {foot_name}")

    mujoco_foot_position = (
        mujoco_data.xpos[mujoco_body_id].copy()
    )

    pinocchio_frame_id = (
        pinocchio_model.getFrameId(foot_name)
    )

    pinocchio_foot_position = (
        pinocchio_data.oMf[pinocchio_frame_id].translation.copy()
    )

    position_difference = (
        mujoco_foot_position - pinocchio_foot_position
    )

    position_error_norm = np.linalg.norm(
        position_difference
    )

    maximum_foot_position_error = max(
        maximum_foot_position_error,
        position_error_norm,
    )

    print(f"\n{foot_name}")

    print("MuJoCo:")
    print(mujoco_foot_position)

    print("Pinocchio:")
    print(pinocchio_foot_position)

    print("Difference:")
    print(position_difference)

    print("Error norm:")
    print(position_error_norm)


print("\nMaximum foot position error [m]:")
print(maximum_foot_position_error)