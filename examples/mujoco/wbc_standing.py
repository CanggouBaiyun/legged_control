from pathlib import Path

import time

import mujoco
import mujoco.viewer
import numpy as np

from go2_control.model import build_go2_model
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

pinocchio_model = build_go2_model()

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

desired_base_position = np.array([
    0.0,
    0.0,
    0.27,
])

desired_base_rotation = np.eye(3)

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
        )

        # Pinocchio 关节顺序转换为 MuJoCo actuator 顺序。
        mujoco_controls = (
            bridge.pinocchio_torques_to_mujoco_controls(
                result.motor_torques
            )
        )

        # 最后一层安全保护。
        mujoco_controls = np.clip(
            mujoco_controls,
            mujoco_model.actuator_ctrlrange[:, 0],
            mujoco_model.actuator_ctrlrange[:, 1],
        )

        mujoco_data.ctrl[:] = mujoco_controls

        # 让真实接触动力学推进一个时间步。
        mujoco.mj_step(mujoco_model,mujoco_data)

        viewer.sync()

        if mujoco_data.time >= next_print_time:
            print(
                f"time={mujoco_data.time:6.2f} s, "
                f"base_z={mujoco_data.qpos[2]:.4f} m, "
                f"contacts={mujoco_data.ncon:2d}, "
                f"max_torque="
                f"{np.max(np.abs(mujoco_controls)):.4f} Nm, "
                f"QP={result.solver_status}"
            )

            next_print_time += 1.0

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