import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    build_go2_simulation_model,
    foot_positions,
    nominal_configuration,
)

from go2_control.wbc import (
    StandingWBC,
)
from go2_control.wbc.kinematics import compute_contact_kinematics


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


def test_two_swing_feet_enter_same_wbc_problem():
    model = build_go2_simulation_model()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)
    feet = foot_positions(model, q)
    support_force = pin.computeTotalMass(model) * 9.81 / 2.0

    contact_pairs = (
        (("FL_foot", "RR_foot"), ("FR_foot", "RL_foot")),
        (("FR_foot", "RL_foot"), ("FL_foot", "RR_foot")),
    )

    for stance_feet, swing_feet in contact_pairs:
        controller = StandingWBC(model, contact_frame_names=stance_feet)
        heights = (0.02, 0.03)
        targets = {
            name: (
                np.asarray(feet[name]) + np.array([0.0, 0.0, height]),
                None,
                None,
            )
            for name, height in zip(swing_feet, heights)
        }

        result = controller.solve(
            q=q,
            v=v,
            desired_base_position=q[:3].copy(),
            desired_base_rotation=np.eye(3),
            desired_contact_forces=np.tile(
                [0.0, 0.0, support_force], (2, 1)
            ),
            swing_targets=targets,
        )

        assert result.solver_status.lower() == "solved"
        assert result.contact_forces.shape == (2, 3)
        assert result.dynamics_residual_norm < 1e-7
        assert result.contact_acceleration_residual_norm < 1e-7

        for name, height in zip(swing_feet, heights):
            jacobian, bias = compute_contact_kinematics(
                model, model.createData(), q, v, (name,)
            )
            achieved_acceleration = (
                jacobian @ result.generalized_acceleration + bias
            )
            np.testing.assert_allclose(
                achieved_acceleration[2],
                100.0 * height,
                atol=0.05,
            )
