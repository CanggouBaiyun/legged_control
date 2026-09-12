import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    nominal_configuration,
)
from go2_control.wbc.tasks import (
    compute_base_acceleration_task,
)

def compute_test_task(
    q,
    v,
    desired_rotation=None,
):
    if desired_rotation is None:
        desired_rotation = np.eye(3)

    return compute_base_acceleration_task(
        q=q,
        v=v,
        desired_position=np.array([
            0.0,
            0.0,
            0.28,
        ]),
        desired_rotation=desired_rotation,
        desired_linear_velocity_world=np.zeros(3),
        desired_angular_velocity_local=np.zeros(3),
        position_kp=np.array([
            30.0,
            30.0,
            50.0,
        ]),
        position_kd=np.array([
            8.0,
            8.0,
            10.0,
        ]),
        orientation_kp=40.0,
        orientation_kd=8.0,
        maximum_linear_acceleration=2.0,
        maximum_angular_acceleration=5.0,
    )

def test_base_height_error_generates_upward_acceleration():
    model = build_go2_model()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    acceleration = compute_test_task(q, v)

    np.testing.assert_allclose(
        acceleration,
        np.array([
            0.0,
            0.0,
            0.5,
            0.0,
            0.0,
            0.0,
        ]),
        atol=1e-12,
    )

def test_positive_x_error_generates_negative_x_acceleration():
    model = build_go2_model()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    q[0] = 0.02

    acceleration = compute_test_task(q, v)

    np.testing.assert_allclose(
        acceleration[:3],
        np.array([
            -0.6,
            0.0,
            0.5,
        ]),
        atol=1e-12,
    )


def test_positive_roll_generates_negative_roll_acceleration():
    model = build_go2_model()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    roll_angle = np.deg2rad(5.0)

    current_rotation = pin.rpy.rpyToMatrix(
        np.array([
            roll_angle,
            0.0,
            0.0,
        ])
    )

    q[3:7] = pin.Quaternion(
        current_rotation
    ).coeffs()

    acceleration = compute_test_task(q, v)

    expected_roll_acceleration = (
        -40.0 * roll_angle
    )

    np.testing.assert_allclose(
        acceleration[3:],
        np.array([
            expected_roll_acceleration,
            0.0,
            0.0,
        ]),
        atol=1e-10,
    )

