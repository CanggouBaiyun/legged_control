import numpy as np

from go2_control.model import (
    build_go2_model,
    nominal_configuration,
)

from go2_control.wbc import (
    StandingWBC,
)


def test_nominal_standing_wbc():
    model = build_go2_model()

    controller = StandingWBC(
        model
    )

    q = nominal_configuration(
        model
    )

    v = np.zeros(
        model.nv
    )

    result = controller.solve(
        q=q,
        v=v,
        desired_base_position=np.array([
            0.0,
            0.0,
            0.28,
        ]),
        desired_base_rotation=np.eye(3),
    )

    assert result.solver_status.lower().startswith(
        "solved"
    )

    assert result.generalized_acceleration.shape == (
        18,
    )

    assert result.contact_forces.shape == (
        4,
        3,
    )

    assert result.motor_torques.shape == (
        12,
    )

    assert (
        result.dynamics_residual_norm
        < 1e-8
    )

    assert (
        result.contact_acceleration_residual_norm
        < 1e-8
    )

    base_acceleration_error = (
        result.generalized_acceleration[:6]
        - result.desired_base_acceleration
    )

    assert (
        np.linalg.norm(
            base_acceleration_error
        )
        < 1e-3
    )

    normal_forces = (
        result.contact_forces[:, 2]
    )

    assert np.all(
        normal_forces >= 0.0
    )

    assert np.all(
        normal_forces <= 100.0
    )

    assert np.all(
        np.abs(
            result.motor_torques
        )
        <= model.effortLimit[6:] + 1e-10
    )