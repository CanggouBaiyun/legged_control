import numpy as np

from go2_control.model import (
    GO2_FOOT_FRAMES,
    build_go2_model,
    nominal_configuration,
)
from go2_control.wbc.kinematics import (
    compute_contact_kinematics,
)


def create_test_state():
    model = build_go2_model()
    data = model.createData()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    return model, data, q, v

def test_four_foot_contact_kinematics():
    model, data, q, v = create_test_state()

    contact_jacobian, contact_bias = (
        compute_contact_kinematics(
            model=model,
            data=data,
            q=q,
            v=v,
            contact_frame_names=GO2_FOOT_FRAMES,
        )
    )

    assert contact_jacobian.shape == (
        12,
        model.nv,
    )
    assert contact_bias.shape == (12,)
    assert np.linalg.matrix_rank(
        contact_jacobian
    ) == 12

    np.testing.assert_allclose(
        contact_bias,
        np.zeros(12),
        atol=1e-12,
    )

def test_diagonal_two_foot_contact_kinematics():
    model, data, q, v = create_test_state()

    contact_frame_names = (
        "FR_foot",
        "RL_foot",
    )

    contact_jacobian, contact_bias = (
        compute_contact_kinematics(
            model=model,
            data=data,
            q=q,
            v=v,
            contact_frame_names=contact_frame_names,
        )
    )

    assert contact_jacobian.shape == (
        6,
        model.nv,
    )
    assert contact_bias.shape == (6,)
    assert np.linalg.matrix_rank(
        contact_jacobian
    ) == 6

def test_zero_contact_kinematics():
    model, data, q, v = create_test_state()

    contact_jacobian, contact_bias = (
        compute_contact_kinematics(
            model=model,
            data=data,
            q=q,
            v=v,
            contact_frame_names=(),
        )
    )

    assert contact_jacobian.shape == (
        0,
        model.nv,
    )
    assert contact_bias.shape == (0,)
