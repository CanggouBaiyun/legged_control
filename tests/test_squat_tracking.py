"""Regression checks for MJCF world placement and closed-loop squat references."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pinocchio as pin
import pytest

from go2_control.model import build_go2_simulation_model, nominal_configuration, load_model_config, GO2_FOOT_FRAMES
from go2_control.simulation import MujocoPinocchioBridge
from go2_control.wbc.reference import FixedFootReference, joint_pd_feedback, squat_height_reference
from go2_control.wbc.tasks import compute_base_acceleration_task


def test_mjcf_absolute_world_placements():
    mm = mujoco.MjModel.from_xml_path(str(load_model_config()["mjcf"]))
    md = mujoco.MjData(mm)
    pm = build_go2_simulation_model()
    pd = pm.createData()
    bridge = MujocoPinocchioBridge(mm, pm)
    mujoco.mj_resetDataKeyframe(mm, md, mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_KEY, "home"))
    rotation = pin.rpy.rpyToMatrix(np.array([0.1, -0.2, 0.3]))
    quaternion = pin.Quaternion(rotation).coeffs()
    md.qpos[:3] = [0.2, -0.1, 0.5]
    md.qpos[3:7] = quaternion[[3, 0, 1, 2]]
    mujoco.mj_forward(mm, md)
    q, v = bridge.state(md)
    pin.forwardKinematics(pm, pd, q)
    pin.updateFramePlacements(pm, pd)
    for name in ("base_link", *GO2_FOOT_FRAMES):
        body = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_BODY, name)
        assert body >= 0
        frame = pd.oMf[pm.getFrameId(name)]
        np.testing.assert_allclose(frame.translation, md.xpos[body], atol=1e-12)
        np.testing.assert_allclose(frame.rotation, md.xmat[body].reshape(3, 3), atol=1e-12)
    base = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    np.testing.assert_allclose(pin.centerOfMass(pm, pd, q), md.subtree_com[base], atol=1e-12)
    # At zero velocity, compare generalized gravity in the same coordinates.
    transform = np.zeros((18, 18))
    transform[:3, :3] = rotation
    transform[3:6, 3:6] = np.eye(3)
    transform[bridge.mujoco_qvel_indices, bridge.pinocchio_v_indices] = 1
    np.testing.assert_allclose(pin.nonLinearEffects(pm, pd, q, v), transform.T @ md.qfrc_bias, atol=1e-11)
    mass = np.zeros((18, 18))
    mujoco.mj_fullM(mm, mass, md.qM)
    np.testing.assert_allclose(pin.crba(pm, pd, q), transform.T @ mass @ transform, atol=1e-11)


def test_squat_reference_derivatives():
    assert squat_height_reference(1) == (0.28, 0.0, 0.0)
    t, dt = 3.3, 1e-5
    z, v, a = squat_height_reference(t)
    before = squat_height_reference(t - dt)
    after = squat_height_reference(t + dt)
    np.testing.assert_allclose((after[0] - before[0]) / (2*dt), v, atol=1e-10)
    np.testing.assert_allclose((after[1] - before[1]) / (2*dt), a, atol=1e-10)


def make_reference():
    model = build_go2_simulation_model()
    q = nominal_configuration(model)
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    feet = np.array([data.oMf[model.getFrameId(n)].translation.copy() for n in GO2_FOOT_FRAMES])
    feet[:, 2] = 0.022
    return model, FixedFootReference(model, q, feet), feet


def test_fixed_feet_and_reference_velocity():
    model, reference, feet = make_reference()
    for z in [0.28, 0.26, 0.24, 0.26, 0.28]:
        q, v = reference.solve(np.array([0., 0., z]), np.eye(3), np.array([0., 0., 0.02]))
        data = model.createData()
        pin.forwardKinematics(model, data, q, v)
        pin.updateFramePlacements(model, data)
        for i, name in enumerate(GO2_FOOT_FRAMES):
            frame = model.getFrameId(name)
            np.testing.assert_allclose(data.oMf[frame].translation, feet[i], atol=1e-9)
            np.testing.assert_allclose(pin.getFrameVelocity(model, data, frame, pin.LOCAL_WORLD_ALIGNED).linear, 0, atol=1e-10)
        q_next, _ = reference.solve(np.array([0., 0., z + 0.02e-4]), np.eye(3), np.array([0., 0., 0.02]))
        np.testing.assert_allclose(pin.difference(model, q, q_next) / 1e-4, v, atol=3e-6)


def test_unreachable_reference_fails():
    _, reference, _ = make_reference()
    with pytest.raises(RuntimeError):
        reference.solve(np.array([0., 0., 2.]), np.eye(3), np.zeros(3))


def test_joint_pd_includes_position_and_velocity():
    model = build_go2_simulation_model()
    q = nominal_configuration(model)
    q_ref = q.copy()
    q_ref[7] += 0.1
    v = np.zeros(18)
    v_ref = v.copy()
    v_ref[7] = 0.2
    expected = np.zeros(12)
    expected[0], expected[1] = 2.0, 0.2
    np.testing.assert_allclose(joint_pd_feedback(model, q, v, q_ref, v_ref, 20, 1), expected, atol=1e-12)


def test_trajectory_acceleration_coordinate_conversion():
    model = build_go2_simulation_model()
    q = nominal_configuration(model)
    rotation = pin.rpy.rpyToMatrix(np.array([0.1, 0.2, 0.3]))
    q[3:7] = pin.Quaternion(rotation).coeffs()
    v = np.zeros(18)
    v[:6] = [0.1, 0.2, -0.1, 0.2, -0.1, 0.3]
    world_acceleration = np.array([0.2, -0.1, 0.4])
    a = compute_base_acceleration_task(
        q, v, q[:3], rotation, rotation @ v[:3], v[3:6],
        30, 8, 40, 8, 2, 5,
        desired_linear_acceleration_world=world_acceleration,
    )
    np.testing.assert_allclose(rotation @ (a[:3] + np.cross(v[3:6], v[:3])), world_acceleration, atol=1e-12)


@pytest.mark.filterwarnings("ignore:The default value of raise_error:PendingDeprecationWarning")
def test_headless_squat_tracking():
    path = Path(__file__).resolve().parents[1] / "examples/mujoco/wbc_squat.py"
    spec = importlib.util.spec_from_file_location("squat_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.run(SimpleNamespace(
        headless=True, mode="tracking", joint_kp=20., joint_kd=1.,
        depth=0.04, period=4., duration=6.1, csv=None,
    ))
    assert result["height_rmse_mm"] < 4
    assert result["height_max_error_mm"] < 6
    assert result["minimum_stance_feet"] == 4
    assert result["clipped_steps"] == result["qp_failures"] == 0
