import numpy as np
import pinocchio as pin

from go2_control.model import (
    build_go2_model,
    nominal_configuration,
)
from go2_control.wbc.dynamics import (
    compute_dynamics_terms,
)


def create_test_state():
    model = build_go2_model()
    data = model.createData()
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    return model, data, q, v


def test_mass_matrix_is_symmetric():
    model, data, q, v = create_test_state()

    mass_matrix, _ = compute_dynamics_terms(
        model=model,
        data=data,
        q=q,
        v=v,
    )

    assert mass_matrix.shape == (
        model.nv,
        model.nv,
    )

    np.testing.assert_allclose(
        mass_matrix,
        mass_matrix.T,
        atol=1e-12,
    )


def test_mass_matrix_is_positive_definite():
    model, data, q, v = create_test_state()

    mass_matrix, _ = compute_dynamics_terms(
        model=model,
        data=data,
        q=q,
        v=v,
    )

    eigenvalues = np.linalg.eigvalsh(
        mass_matrix
    )

    assert np.min(eigenvalues) > 0.0


def test_dynamics_terms_match_rnea():
    model = build_go2_model()
    q = nominal_configuration(model)

    random_generator = np.random.default_rng(
        seed=7
    )

    v = random_generator.normal(
        scale=0.1,
        size=model.nv,
    )

    acceleration = random_generator.normal(
        scale=0.2,
        size=model.nv,
    )

    mass_matrix, nonlinear_effects = (
        compute_dynamics_terms(
            model=model,
            data=model.createData(),
            q=q,
            v=v,
        )
    )

    generalized_force_from_equation = (
        mass_matrix @ acceleration
        + nonlinear_effects
    )

    generalized_force_from_rnea = pin.rnea(
        model,
        model.createData(),
        q,
        v,
        acceleration,
    ).copy()

    np.testing.assert_allclose(
        generalized_force_from_equation,
        generalized_force_from_rnea,
        rtol=1e-10,
        atol=1e-10,
    )