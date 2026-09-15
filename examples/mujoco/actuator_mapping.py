from pathlib import Path

import mujoco
import numpy as np

from go2_control.model import (
    build_go2_model,
)

np.set_printoptions(
    precision=6,
    suppress=True,
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

# ============================================================
# 2. 设置 MuJoCo 初始状态
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

mujoco_data.qvel[:] = 0.0
mujoco_data.ctrl[:] = 0.0

# ============================================================
# 3. 模拟一组 Pinocchio/WBC 输出力矩
# ============================================================

pinocchio_motor_torques = np.arange(
    1.0,
    13.0,
)

print("Pinocchio motor torques in v[6:] order:")
print(pinocchio_motor_torques)

# ============================================================
# 4. 按关节名称映射到 MuJoCo ctrl
# ============================================================

mujoco_controls = np.zeros(
    mujoco_model.nu
)

expected_mujoco_generalized_force = np.zeros(
    mujoco_model.nv
)

print("\nActuator mapping:")
print(
    f"{'actuator':12s} "
    f"{'joint':18s} "
    f"{'Pin torque index':>17s} "
    f"{'ctrl value':>12s}"
)

for actuator_id in range(mujoco_model.nu):
    actuator_name = mujoco.mj_id2name(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        actuator_id,
    )

    mujoco_joint_id = int(
        mujoco_model.actuator_trnid[
            actuator_id,
            0,
        ]
    ) 

    joint_name = mujoco.mj_id2name(
        mujoco_model,
        mujoco.mjtObj.mjOBJ_JOINT,
        mujoco_joint_id,
    )

    pinocchio_joint_id = (
        pinocchio_model.getJointId(joint_name)
    )

    pinocchio_velocity_index = int(
        pinocchio_model.joints[pinocchio_joint_id].idx_v
    )

    # motor_torques[0] 对应 Pinocchio v[6]
    pinocchio_torque_index = (
        pinocchio_velocity_index - 6
    )

    motor_torque = (
        pinocchio_motor_torques[pinocchio_torque_index]
    )

    mujoco_controls[actuator_id] = motor_torque

    mujoco_velocity_index = int(mujoco_model.jnt_dofadr[mujoco_joint_id])

    expected_mujoco_generalized_force[mujoco_velocity_index] = motor_torque

    print(
        f"{str(actuator_name):12s} "
        f"{str(joint_name):18s} "
        f"{pinocchio_torque_index:17d} "
        f"{motor_torque:12.6f}"
    )

# ============================================================
# 5. 将力矩写入 MuJoCo
# ============================================================
mujoco_data.ctrl[:] = (
    mujoco_controls
)

mujoco.mj_forward(
    mujoco_model,
    mujoco_data,
)

# ============================================================
# 6. 检查 MuJoCo 实际产生的广义驱动力
# ============================================================
actuator_force_error = (
    mujoco_data.qfrc_actuator - expected_mujoco_generalized_force
)

print("\nMuJoCo ctrl in actuator order:")
print(mujoco_data.ctrl)

print("\nExpected MuJoCo generalized actuator force:")
print(expected_mujoco_generalized_force)

print("\nActual MuJoCo generalized actuator force:")
print(mujoco_data.qfrc_actuator)

print("\nActuator force error:")
print(actuator_force_error)

print("\nActuator force error norm:")
print(
    np.linalg.norm(
        actuator_force_error
    )
)
