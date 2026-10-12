"""Convert Go2 OCS2 references to the standing WBC interface."""

import numpy as np
import pinocchio as pin

from go2_control.centroidal import pinocchio_from_state

# Order used by config/ocs2/go2/task.info.
OCS2_CONTACT_ORDER = (
    "FL_foot",
    "FR_foot",
    "RL_foot",
    "RR_foot",
)

def standing_reference_from_mpc(
    model,
    data,
    measured_q,
    state_reference,
    input_reference,
    contact_frame_names,
    mode = 15,
):
    """Convert one evaluated MPC reference; four-foot stance only."""
    if mode != 15:
        raise ValueError("This adapter currently supports stance mode only")

    state = np.asarray(state_reference, dtype=float)
    control = np.asarray(input_reference, dtype=float)

    if state.shape != (24,) or control.shape != (24,) :
        raise ValueError("Expected 24-dimensional MPC state and input")

    if not np.all(np.isfinite(state)) or not np.all(np.isfinite(control)):
        raise ValueError("MPC reference contains non-finite values")

    contact_names = tuple(contact_frame_names)

    if(
        len(contact_names) != 4
        or set(contact_names) != set(OCS2_CONTACT_ORDER)
    ):
        raise ValueError("Expected the four Go2 stance feet")

    # Recover reference configuration and velocity from momentum
    # and the reference joint velocities.
    q_reference, v_reference = pinocchio_from_state(
        model,
        data,
        state,
        control[12:],
    )

    reference_rotation = pin.XYZQUATToSE3(
        q_reference[:7]
    ).rotation.copy()

    measured_rotation = pin.XYZQUATToSE3(
        measured_q[:7]
    ).rotation.copy()

    # Reference body-frame linear velocity -> world
    linear_velocity_world = (
        reference_rotation @ v_reference[:3]
    )

    # WBC expects angular velocity expressed in the CURRENT body frame.
    angular_velocity_world = (
        reference_rotation @ v_reference[3:6]
    )

    angular_velocity_current_body = (
        measured_rotation.T @ angular_velocity_world
    )

    # Reorder world-frame forces by foot NAME, not by assumption
    forces_in_ocs2_order = control[:12].reshape(4, 3)

    force_by_name = dict(zip(
        OCS2_CONTACT_ORDER,
        forces_in_ocs2_order,
    ))

    forces_in_wbc_order = np.array([
        force_by_name[name]
        for name in contact_names
    ])

    return {
        "desired_base_position": q_reference[:3].copy(),
        "desired_base_rotation": reference_rotation,
        "desired_base_linear_velocity_world": linear_velocity_world,
        "desired_base_angular_velocity_local": (
            angular_velocity_current_body
        ),
        "desired_contact_forces": forces_in_wbc_order,
    }