import numpy as np
import pinocchio as pin

from go2_control.centroidal import state_from_pinocchio
from go2_control.model import (
    build_go2_simulation_model,
    nominal_configuration,
)
from go2_control.mpc_reference import standing_reference_from_mpc
from go2_control.wbc import StandingWBC


def test_mpc_force_order():
    model = build_go2_simulation_model()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    state = state_from_pinocchio(
        model, model.createData(), q, v
    )

    control = np.zeros(24)

    # Distinct values reveal ordering mistakes.
    control[:12] = np.array([
        [0.0, 0.0, 10.0],  # FL
        [0.0, 0.0, 20.0],  # FR
        [0.0, 0.0, 30.0],  # RL
        [0.0, 0.0, 40.0],  # RR
    ]).reshape(-1)

    reference = standing_reference_from_mpc(
        model,
        model.createData(),
        q,
        state,
        control,
        ("FR_foot", "FL_foot", "RR_foot", "RL_foot"),
    )

    np.testing.assert_allclose(
        reference["desired_contact_forces"][:, 2],
        [20.0, 10.0, 40.0, 30.0],
    )


def test_wbc_accepts_mpc_standing_reference():
    model = build_go2_simulation_model()
    controller = StandingWBC(model)

    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    state = state_from_pinocchio(
        model, model.createData(), q, v
    )

    control = np.zeros(24)
    forces = control[:12].reshape(4, 3)
    forces[:, 2] = (
        pin.computeTotalMass(model)
        * abs(model.gravity.linear[2])
        / 4.0
    )

    reference = standing_reference_from_mpc(
        model,
        model.createData(),
        q,
        state,
        control,
        controller.contact_frame_names,
    )

    result = controller.solve(q=q, v=v, **reference)

    assert result.solver_status.lower() == "solved"
    assert np.all(np.isfinite(result.motor_torques))
    assert result.dynamics_residual_norm < 1e-7
    assert result.contact_acceleration_residual_norm < 1e-7

    # Explicitly passing the default force reference must preserve
    # the original solver behavior.
    baseline = controller.solve(
        q=q,
        v=v,
        desired_base_position=q[:3].copy(),
        desired_base_rotation=np.eye(3),
    )

    np.testing.assert_allclose(
        result.motor_torques,
        baseline.motor_torques,
        atol=1e-7,
        rtol=1e-7,
    )