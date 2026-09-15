import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    load_model_config,
    nominal_configuration,
)

from go2_control.simulation import (
    MujocoPinocchioBridge,
)


def create_bridge():
    config = load_model_config()

    mujoco_model = (
        mujoco.MjModel.from_xml_path(
            str(config["mjcf"])
        )
    )

    mujoco_data = mujoco.MjData(
        mujoco_model
    )

    pinocchio_model = build_go2_model()

    bridge = MujocoPinocchioBridge(
        mujoco_model=mujoco_model,
        pinocchio_model=pinocchio_model,
    )

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

    return (
        mujoco_model,
        mujoco_data,
        pinocchio_model,
        bridge,
    )


def test_home_configuration_mapping():
    (
        _,
        mujoco_data,
        pinocchio_model,
        bridge,
    ) = create_bridge()

    q = bridge.configuration(
        mujoco_data
    )

    expected_q = nominal_configuration(
        pinocchio_model
    )

    np.testing.assert_allclose(
        q,
        expected_q,
        atol=1e-12,
    )


def test_rotated_base_velocity_mapping():
    (
        mujoco_model,
        mujoco_data,
        _,
        bridge,
    ) = create_bridge()

    yaw = np.deg2rad(30.0)

    mujoco_data.qpos[3:7] = np.array([
        np.cos(yaw / 2.0),
        0.0,
        0.0,
        np.sin(yaw / 2.0),
    ])

    mujoco_data.qvel[:3] = np.array([
        0.2,
        -0.1,
        0.05,
    ])

    mujoco_data.qvel[3:6] = np.array([
        0.1,
        -0.2,
        0.3,
    ])

    mujoco.mj_forward(
        mujoco_model,
        mujoco_data,
    )

    q, v = bridge.state(
        mujoco_data
    )

    rotation = pin.XYZQUATToSE3(
        q[:7]
    ).rotation

    np.testing.assert_allclose(
        v[:3],
        rotation.T @ mujoco_data.qvel[:3],
        atol=1e-12,
    )

    np.testing.assert_allclose(
        v[3:6],
        mujoco_data.qvel[3:6],
        atol=1e-12,
    )


def test_torque_mapping():
    (
        _,
        _,
        _,
        bridge,
    ) = create_bridge()

    pinocchio_torques = np.arange(
        1.0,
        13.0,
    )

    mujoco_controls = (
        bridge.pinocchio_torques_to_mujoco_controls(
            pinocchio_torques
        )
    )

    expected_controls = np.array([
        4.0,
        5.0,
        6.0,
        1.0,
        2.0,
        3.0,
        10.0,
        11.0,
        12.0,
        7.0,
        8.0,
        9.0,
    ])

    np.testing.assert_allclose(
        mujoco_controls,
        expected_controls,
        atol=0.0,
    )