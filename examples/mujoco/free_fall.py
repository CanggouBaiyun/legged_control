from pathlib import Path

import mujoco
import numpy as np

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

model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

data = mujoco.MjData(model)

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

# 将机器人抬高，避免实验过程中足端接触地面。
data.qpos[2] = 1.0

# 自由落体初速度为零。
data.qvel[:] = 0.0

# home 关键帧包含非零 ctrl，这里必须显式清零。
data.ctrl[:] = 0.0

mujoco.mj_forward(
    model,
    data,
)

initial_base_position = data.qpos[:3].copy()
initial_joint_positions = data.qpos[7:].copy()
initial_acceleration = data.qacc.copy()

gravity = model.opt.gravity.copy()
time_step = model.opt.timestep

simulation_duration = 0.1

number_of_steps = round(
    simulation_duration / time_step
)

for _ in range(number_of_steps):
    mujoco.mj_step(
        model,
        data,
    )

elapsed_time = data.time

expected_base_z = (
    initial_base_position[2] + 0.5 * gravity[2] * elapsed_time**2
)

expected_base_z_velocity = (
    gravity[2] * elapsed_time
)


print("Gravity [m/s^2]:")
print(gravity)

print("\nInitial generalized acceleration:")
print(initial_acceleration)

print("\nInitial base linear acceleration:")
print(initial_acceleration[:3])

print("\nElapsed time [s]:")
print(elapsed_time)

print("\nSimulated base position:")
print(data.qpos[:3])

print("\nExpected base z position [m]:")
print(expected_base_z)

print("\nBase z position error [m]:")
print(data.qpos[2] - expected_base_z)

print("\nSimulated base linear velocity:")
print(data.qvel[:3])

print("\nExpected base z velocity [m/s]:")
print(expected_base_z_velocity)

print("\nBase z velocity error [m/s]:")
print(data.qvel[2] - expected_base_z_velocity)

print("\nJoint position drift norm [rad]:")
print(
    np.linalg.norm(
        data.qpos[7:]
        - initial_joint_positions
    )
)

print("\nJoint velocity norm [rad/s]:")
print(
    np.linalg.norm(
        data.qvel[6:]
    )
)