from pathlib import Path

import mujoco
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

model = mujoco.MjModel.from_xml_path(
    str(scene_path)
)

data = mujoco.MjData(model)


print("Model dimensions:")
print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)
print("njnt =", model.njnt)
print("nbody =", model.nbody)
print("simulation timestep =", model.opt.timestep)

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

mujoco.mj_forward(
    model,
    data,
)


print("\nHome configuration qpos:")
print(data.qpos)

print("\nBase position [x, y, z]:")
print(data.qpos[:3])

print("\nBase quaternion [w, x, y, z]:")
print(data.qpos[3:7])


print("\nJoint mapping:")
print(
    f"{'joint':22s} "
    f"{'qpos address':>12s} "
    f"{'qvel address':>12s}"
)


for joint_id in range(model.njnt):
    joint_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_id,
    )

    qpos_address = model.jnt_qposadr[joint_id]
    qvel_address = model.jnt_dofadr[joint_id]

    print(
        f"{str(joint_name):22s} "
        f"{qpos_address:12d} "
        f"{qvel_address:12d}"
    )


print("\nActuator mapping:")
print(
    f"{'actuator':22s} "
    f"{'joint':22s} "
    f"{'minimum':>10s} "
    f"{'maximum':>10s}"
)

for actuator_id in range(model.nu):
    actuator_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        actuator_id,
    )

    joint_id = model.actuator_trnid[
        actuator_id,
        0,
    ]

    joint_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        joint_id,
    )

    minimum_control = model.actuator_ctrlrange[
        actuator_id,
        0,
    ]

    maximum_control = model.actuator_ctrlrange[
        actuator_id,
        1,
    ]

    print(
        f"{str(actuator_name):22s} "
        f"{str(joint_name):22s} "
        f"{minimum_control:10.4f} "
        f"{maximum_control:10.4f}"
    )


print("\nInitial control:")
print(data.ctrl)