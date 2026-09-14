from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

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
# 1. 创建 MuJoCo 模型和数据
# ============================================================

model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

data = mujoco.MjData(model)

# ============================================================
# 2. 恢复 home 初始构型
# ============================================================
home_keyframe_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

if home_keyframe_id == -1:
    raise RuntimeError(
        "The MJCF model does not contain the 'home' keyframe."
    )

mujoco.mj_resetDataKeyframe(
    model,
    data,
    home_keyframe_id,
)

# ============================================================
# 3. 建立 actuator -> joint 状态索引
# ============================================================

actuator_joint_ids = (
    model.actuator_trnid[:,0].astype(int)
)

joint_qpos_indices = (
    model.jnt_qposadr[actuator_joint_ids]
)

joint_qvel_indices = (
    model.jnt_dofadr[actuator_joint_ids]
)

# 按照MuJoCo actuator 顺序保存 home 目标关节角
desired_joint_positions = (
    data.qpos[joint_qpos_indices].copy()
)

desired_joint_velocities = np.zeros(model.nu)

# home 关键帧包含非零 ctrl，必须清零。
data.ctrl[:] = 0.0

mujoco.mj_forward(
    model,
    data,
)

# ============================================================
# 4. 设置关节 PD 参数
# ============================================================
# 单腿顺序：hip、thigh、calf
single_leg_kp = np.array([
    60.0,
    60.0,
    80.0,
])

single_leg_kd = np.array([
    2.0,
    3.0,
    3.0,
])

# MuJoCo actuator 顺序中，每条腿都是 hip、thigh、calf。
kp = np.tile(
    single_leg_kp,
    4,
)

kd = np.tile(
    single_leg_kd,
    4,
)

minimum_motor_torque = (
    model.actuator_ctrlrange[:, 0]
)

maximum_motor_torque = (
    model.actuator_ctrlrange[:, 1]
)

# ============================================================
# 5. 启动仿真和 Viewer
# ============================================================

simulation_duration = 20.0
next_print_time = 0.0

with mujoco.viewer.launch_passive(
    model,
    data,
) as viewer:
    with viewer.lock():
        viewer.cam.lookat[:] = np.array([
            0.0,
            0.0,
            0.2,
        ])
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -20.0

    viewer.sync()

    while(
        viewer.is_running()
        and data.time < simulation_duration
    ):
        step_start_time = time.perf_counter()

        # --------------------------------------------
        # 读取当前关节状态，顺序与 actuator 一致
        # --------------------------------------------

        current_joint_positions = (
            data.qpos[joint_qpos_indices].copy()
        )

        current_joint_velocities = (
            data.qvel[joint_qvel_indices].copy()
        )

        # --------------------------------------------
        # 关节 PD 控制
        # --------------------------------------------

        position_error = (
            desired_joint_positions - current_joint_positions
        )

        velocity_error = (
            desired_joint_velocities - current_joint_velocities
        )

        motor_torques = (
            kp * position_error + kd * velocity_error
        )

        motor_torques = np.clip(
            motor_torques,
            minimum_motor_torque,
            maximum_motor_torque,
        )

        data.ctrl[:] = motor_torques

        # --------------------------------------------
        # 推进一个 MuJoCo 仿真步
        # --------------------------------------------

        mujoco.mj_step(
            model,
            data,
        )

        viewer.sync()

        # 每1秒打印一次状态
        if data.time >= next_print_time:
            print(
                f"time={data.time:6.2f} s, "
                f"base_z={data.qpos[2]:.4f} m, "
                f"max_torque="
                f"{np.max(np.abs(motor_torques)):.4f} Nm"
            )

            next_print_time += 1.0

        # 使仿真速度大致与真实时间一致
        computation_time = (
            time.perf_counter() - step_start_time
        )

        remaining_time = (
            model.opt.timestep - computation_time
        )

        if remaining_time > 0.0:
            time.sleep(
                remaining_time
            )

print("\nSimulation finished.")

print("Final base position:")
print(data.qpos[:3])

print("\nFinal base quaternion [w, x, y, z]:")
print(data.qpos[3:7])

print("\nFinal joint position error norm [rad]:")
print(
    np.linalg.norm(
        desired_joint_positions
        - data.qpos[joint_qpos_indices]
    )
)