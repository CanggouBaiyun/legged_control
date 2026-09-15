from pathlib import Path

import time
import copy

import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin

from go2_control.model import GO2_FOOT_FRAMES
from go2_control.wbc.kinematics import (
    compute_contact_kinematics,
)
from go2_control.model import build_go2_simulation_model
from go2_control.simulation.mujoco_bridge import (
    MujocoPinocchioBridge,
)
from go2_control.wbc import StandingWBC

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
# 1. 创建 MuJoCo 和 Pinocchio 模型
# ============================================================

mujoco_model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

mujoco_data = mujoco.MjData(
    mujoco_model
)

pinocchio_model = build_go2_simulation_model()

# ============================================================
# 2. 恢复 MuJoCo 的 home 初始构型
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

# home 关键帧中可能保存了非零 ctrl。
mujoco_data.ctrl[:] = 0.0

mujoco.mj_forward(
    mujoco_model,
    mujoco_data,
)

# ============================================================
# 3. 创建模型桥和 WBC
# ============================================================

bridge = MujocoPinocchioBridge(
    mujoco_model=mujoco_model,
    pinocchio_model=pinocchio_model,
)

controller = StandingWBC(
    pinocchio_model
)

# ============================================================
# 4. 设置期望基座位姿
# ============================================================

standing_height = 0.28
squat_depth = 0.04
squat_period = 4.0
settling_duration = 2.0

desired_base_position = np.array([
    0.0,
    0.0,
    standing_height,
])

desired_base_linear_velocity_world = np.zeros(3)
desired_base_rotation = np.eye(3)

# 关节位置反馈 Kp 暂时为零，仅启用速度反馈。
joint_kd = 1.0

# 用于计算期望关节速度的运动学缓存。
velocity_reference_data = (
    pinocchio_model.createData()
)
# 对照 A：只关闭关节干摩擦。
no_friction_model = copy.copy(mujoco_model)
no_friction_model.dof_frictionloss[:] = 0.0

# 对照 B：在 A 的基础上，再关闭关节黏性阻尼。
no_friction_damping_model = copy.copy(
    no_friction_model
)
no_friction_damping_model.dof_damping[:] = 0.0

# ============================================================
# 5. 仿真循环
# ============================================================

simulation_duration = 10.0
next_print_time = 0.0

with mujoco.viewer.launch_passive(
    mujoco_model,
    mujoco_data,
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

    while (
        viewer.is_running()
        and mujoco_data.time < simulation_duration
    ):
        step_start_time = time.perf_counter()

        # --------------------------------------------
        # 生成周期蹲起轨迹
        # --------------------------------------------
        if mujoco_data.time < settling_duration:
            trajectory_phase = 0.0
        else:
            trajectory_phase = (
                2.0 * np.pi * (mujoco_data.time - settling_duration) / squat_period
            )
        squat_progress = 0.5 * (1.0 - np.cos(trajectory_phase))

        desired_height = (
            standing_height - squat_depth * squat_progress
        )

        desired_height_velocity = (
            -squat_depth * np.pi / squat_period * np.sin(trajectory_phase)
        )

        desired_base_position[2] = (
            desired_height
        )

        desired_base_linear_velocity_world[2] = (
            desired_height_velocity
        )

        # MuJoCo 状态转换为 Pinocchio 的 q、v。
        q, v = bridge.state(
            mujoco_data
        )

        # 根据当前状态求解一次 WBC QP。
        result = controller.solve(
            q=q,
            v=v,
            desired_base_position=desired_base_position,
            desired_base_rotation=desired_base_rotation,
            desired_base_linear_velocity_world=(
                desired_base_linear_velocity_world
            ),
        )

        # --------------------------------------------
        # 根据基座期望速度，计算四足不滑动时的关节速度
        # --------------------------------------------

        contact_jacobian, contact_bias_acceleration = compute_contact_kinematics(
            model=pinocchio_model,
            data=velocity_reference_data,
            q=q,
            v=v,
            contact_frame_names=GO2_FOOT_FRAMES,
        )

        base_jacobian = contact_jacobian[:, :6]
        joint_jacobian = contact_jacobian[:, 6:]

        current_base_rotation = (
            pin.XYZQUATToSE3(q[:7]).rotation
        )

        desired_base_velocity = np.zeros(6)

        # 轨迹线速度用世界坐标表达，
        # 转成 Pinocchio 基座速度所用的基座坐标。
        desired_base_velocity[:3] = (
            current_base_rotation.T @ desired_base_linear_velocity_world
        )

        # 蹲起轨迹保持姿态，期望角速度为零。
        desired_base_velocity[3:6] = 0.0

        desired_joint_velocities = np.linalg.solve(
            joint_jacobian, -base_jacobian @ desired_base_velocity,
        )

        # --------------------------------------------
        # 关节速度反馈：速度误差转换为附加力矩
        # --------------------------------------------

        joint_velocity_error = (
            desired_joint_velocities - v[6:]
        )

        velocity_feedback_torques = (
            joint_kd * joint_velocity_error
        )

        commanded_motor_torques = (
            result.motor_torques + velocity_feedback_torques
        )

        # Pinocchio 关节顺序转换为 MuJoCo actuator 顺序。
        mujoco_controls = (
            bridge.pinocchio_torques_to_mujoco_controls(
                commanded_motor_torques
            )
        )

        # 最后一层安全保护。
        mujoco_controls = np.clip(
            mujoco_controls,
            mujoco_model.actuator_ctrlrange[:, 0],
            mujoco_model.actuator_ctrlrange[:, 1],
        )

        mujoco_data.ctrl[:] = mujoco_controls

        # 在推进仿真前，用状态副本比较不同力矩和被动参数。
        if mujoco_data.time >= next_print_time:
            # 复制当前状态和已经下发的 ctrl。
            # 在副本中计算，避免诊断改变主仿真的数据。
            diagnostic_data = copy.copy(
                mujoco_data
            )

            mujoco.mj_forward(
                mujoco_model,
                diagnostic_data,
            )

            # 同一状态、同一 ctrl，同时关闭摩擦与阻尼。
            no_friction_damping_data = copy.copy(
                mujoco_data
            )

            mujoco.mj_forward(
                no_friction_damping_model,
                no_friction_damping_data,
            )

            # 将 Pinocchio 基座加速度转换为世界系线加速度。
            rotation_correction = np.cross(
                v[3:6],
                v[:3],
            )

            wbc_acceleration_world = (
                current_base_rotation
                @ (
                    result.generalized_acceleration[:3] + rotation_correction
                )
            )

            # 关闭关节摩擦和阻尼，仅施加 WBC 原始力矩。
            wbc_only_data = copy.copy(mujoco_data)

            wbc_only_controls = (
                bridge.pinocchio_torques_to_mujoco_controls(
                    result.motor_torques
                )
            )

            clipped_wbc_controls = np.clip(
                wbc_only_controls,
                mujoco_model.actuator_ctrlrange[:, 0],
                mujoco_model.actuator_ctrlrange[:, 1],
            )

            control_clip_error = np.max(
                np.abs(clipped_wbc_controls - wbc_only_controls)
            )

            wbc_only_data.ctrl[:] = clipped_wbc_controls

            mujoco.mj_forward(
                no_friction_damping_model,
                wbc_only_data,
            )

            # 世界系线加速度转换为 Pinocchio 基座速度的导数。
            actual_acceleration_pin = np.zeros(pinocchio_model.nv)
            actual_acceleration_pin[:3] = (
                current_base_rotation.T @ wbc_only_data.qacc[:3]
                - rotation_correction
            )
            actual_acceleration_pin[3:6] = wbc_only_data.qacc[3:6]
            actual_acceleration_pin[
                bridge.pinocchio_v_indices
            ] = wbc_only_data.qacc[bridge.mujoco_qvel_indices]

            # 检查四个足端坐标系原点的 a_foot = J_c @ a + J_dot_c @ v。
            actual_foot_acceleration = (
                contact_jacobian @ actual_acceleration_pin
                + contact_bias_acceleration
            )
            actual_foot_acceleration_norm = np.linalg.norm(
                actual_foot_acceleration
            )

            # 当前 MuJoCo 模型的浮动基平移使用世界坐标。
            mujoco_vertical_acceleration = (
                diagnostic_data.qacc[2]
            )

            if next_print_time == 0.0:
                print(
                    " time      e_z    a_wbc     a_mj"
                    "  a_no_both      a_ff    foot_a      clip"
                )

            print(
                f"{mujoco_data.time:5.2f} "
                f"{desired_height - q[2]:8.4f} "
                f"{wbc_acceleration_world[2]:8.3f} "
                f"{mujoco_vertical_acceleration:8.3f} "
                f"{no_friction_damping_data.qacc[2]:10.3f} "
                f"{wbc_only_data.qacc[2]:9.3f} "
                f"{actual_foot_acceleration_norm:9.4f} "
                f"{control_clip_error:9.2e}"
            )

            next_print_time += 1.0

        # 完成诊断后，再推进一个仿真时间步。
        mujoco.mj_step(
            mujoco_model,
            mujoco_data,
        )

        viewer.sync()

        computation_time = (
            time.perf_counter() - step_start_time
        )

        remaining_time = (
            mujoco_model.opt.timestep - computation_time
        )

        if remaining_time > 0.0:
            time.sleep(
                remaining_time
            )

print("\nSimulation finished.")

print("Final base position:")
print(mujoco_data.qpos[:3])

print("\nFinal base quaternion [w, x, y, z]:")
print(mujoco_data.qpos[3:7])
