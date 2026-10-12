import numpy as np
import pinocchio as pin

from go2_control.centroidal import state_from_pinocchio
from go2_control.model import (
    build_go2_simulation_model,
    nominal_configuration,
)
from go2_control.mpc_reference import (
    mode_at_time,
    standing_reference_from_mpc,
    swing_interval_at_time,
)
from go2_control.wbc import StandingWBC


def test_mode_at_time_matches_ocs2_switch_boundary():
    events = [0.30, 0.35, 0.35, 0.65]
    modes = [15, 9, 15, 6, 15]

    assert mode_at_time(events, modes, 0.29) == 15
    assert mode_at_time(events, modes, 0.30) == 15
    assert mode_at_time(events, modes, 0.31) == 9
    assert mode_at_time(events, modes, 0.35) == 9
    assert mode_at_time(events, modes, 0.36) == 6
    assert mode_at_time(events, modes, 0.65) == 6
    assert mode_at_time(events, modes, 0.66) == 15


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

    cases = (
        (15, ("FR_foot", "FL_foot", "RR_foot", "RL_foot"),
         [20.0, 10.0, 40.0, 30.0]),
        (9, ("RR_foot", "FL_foot"), [40.0, 10.0]),
        (6, ("RL_foot", "FR_foot"), [30.0, 20.0]),
    )

    for mode, contact_names, expected_forces in cases:
        reference = standing_reference_from_mpc(
            model,
            model.createData(),
            q,
            state,
            control,
            contact_names,
            mode=mode,
        )

        np.testing.assert_allclose(
            reference["desired_contact_forces"][:, 2],
            expected_forces,
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

def test_swing_intervals_follow_diagonal_modes():
    events = [1.00, 1.30, 1.35, 1.65]
    modes = [15, 9, 15, 6, 15]

    assert swing_interval_at_time(events, modes, 1.00, "FR_foot") is None
    assert swing_interval_at_time(events, modes, 1.15, "FR_foot") == (1.00, 1.30)
    assert swing_interval_at_time(events, modes, 1.15, "RL_foot") == (1.00, 1.30)
    assert swing_interval_at_time(events, modes, 1.15, "FL_foot") is None
    assert swing_interval_at_time(events, modes, 1.30, "FR_foot") == (1.00, 1.30)
    assert swing_interval_at_time(events, modes, 1.35, "FR_foot") is None
    assert swing_interval_at_time(events, modes, 1.50, "FL_foot") == (1.35, 1.65)
    assert swing_interval_at_time(events, modes, 1.50, "RR_foot") == (1.35, 1.65)
