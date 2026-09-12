import numpy as np
import pinocchio as pin


def compute_dynamics_terms(
    model,
    data,
    q,
    v,
):
    """Compute the symmetric mass matrix and nonlinear effects."""

    mass_matrix_upper = pin.crba(
        model,
        data,
        q,
    )

    mass_matrix = (
        np.triu(mass_matrix_upper) + np.triu(mass_matrix_upper, k=1).T
    )

    nonlinear_effects = pin.nonLinearEffects(
        model,
        data,
        q,
        v,
    ).copy()

    return (
        mass_matrix,
        nonlinear_effects,
    )