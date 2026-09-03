"""Numerical smoke tests for the official Go2 free-flyer model."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    GO2_SDK_JOINT_ORDER,
    build_go2_model,
    nominal_configuration,
    sdk_joint_mapping,
)


def test_model_dimensions_and_required_names() -> None:
    model = build_go2_model()
    mapping = sdk_joint_mapping(model)

    assert model.nq == 19
    assert model.nv == 18
    assert len(mapping) == 12
    assert tuple(entry["name"] for entry in mapping) == GO2_SDK_JOINT_ORDER
    for frame_name in GO2_FOOT_FRAMES:
        assert model.existFrame(frame_name)


def test_mass_matrix_is_symmetric_positive_definite() -> None:
    model = build_go2_model()
    q = nominal_configuration(model)
    mass_matrix = pin.crba(model, model.createData(), q)
    mass_matrix = (mass_matrix + mass_matrix.T) * 0.5

    np.testing.assert_allclose(mass_matrix, mass_matrix.T, atol=1e-12)
    assert np.linalg.eigvalsh(mass_matrix).min() > 0.0


def test_rnea_matches_mass_matrix_and_nonlinear_effects() -> None:
    model = build_go2_model()
    q = nominal_configuration(model)
    rng = np.random.default_rng(7)
    v = rng.normal(scale=0.1, size=model.nv)
    acceleration = rng.normal(scale=0.2, size=model.nv)

    mass_matrix = pin.crba(model, model.createData(), q)
    mass_matrix = (mass_matrix + mass_matrix.T) * 0.5
    nonlinear_effects = pin.nonLinearEffects(model, model.createData(), q, v)
    inverse_dynamics = pin.rnea(model, model.createData(), q, v, acceleration)

    np.testing.assert_allclose(
        inverse_dynamics,
        mass_matrix @ acceleration + nonlinear_effects,
        rtol=1e-10,
        atol=1e-10,
    )
