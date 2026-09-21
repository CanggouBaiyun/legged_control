"""Verify the exported URDF against the MJCF-derived rigid-body model."""

from pathlib import Path

import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    GO2_SDK_JOINT_ORDER,
    build_go2_simulation_model,
    nominal_configuration,
)

project_root = Path(__file__).resolve().parents[1]
urdf_path = project_root / "models/ocs2/go2_mujoco_inertial.urdf"

if not urdf_path.is_file():
    raise FileNotFoundError(urdf_path)

# Both models use the same free-flyer coordinates for this comparison.
mjcf_model = build_go2_simulation_model()
urdf_model = pin.buildModelFromUrdf(
    str(urdf_path),
    pin.JointModelFreeFlyer(),
)

for model in (mjcf_model, urdf_model):
    if (model.nq, model.nv) != (19, 18):
        raise ValueError("Expected nq=19 and nv=18")

    for name in GO2_FOOT_FRAMES:
        if not model.existFrame(name):
            raise ValueError(f"Missing foot frame: {name}")

# Confirm coordinate ordering before comparing arrays directly.
for name in GO2_SDK_JOINT_ORDER:
    if not mjcf_model.existJointName(name):
        raise ValueError(f"Missing MJCF joint: {name}")
    if not urdf_model.existJointName(name):
        raise ValueError(f"Missing URDF joint: {name}")

    mj_joint = mjcf_model.joints[mjcf_model.getJointId(name)]
    urdf_joint = urdf_model.joints[urdf_model.getJointId(name)]

    mj_layout = (
        mj_joint.idx_q, mj_joint.idx_v,
        mj_joint.nq, mj_joint.nv,
    )
    urdf_layout = (
        urdf_joint.idx_q, urdf_joint.idx_v,
        urdf_joint.nq, urdf_joint.nv,
    )

    if mj_layout != urdf_layout:
        raise ValueError(f"Joint coordinate mismatch: {name}")

# Compare rigid-body dynamics only.
# These are local model objects; no source file or WBC model is modified.
mjcf_model.armature[:] = 0.0
urdf_model.armature[:] = 0.0

mjcf_data = mjcf_model.createData()
urdf_data = urdf_model.createData()


def evaluate(model, data, q, v):
    com = pin.centerOfMass(model, data, q).copy()

    gravity = pin.computeGeneralizedGravity(model, data, q).copy()
    bias = pin.nonLinearEffects(model, data, q, v).copy()

    upper = pin.crba(model, data, q).copy()
    mass_matrix = np.triu(upper) + np.triu(upper, 1).T

    momentum_map = pin.computeCentroidalMap(
        model, data, q
    ).copy()

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    feet = np.array([
        data.oMf[model.getFrameId(name)].translation.copy()
        for name in GO2_FOOT_FRAMES
    ])

    return {
        "foot_position": feet,
        "com_position": com,
        "gravity": gravity,
        "nonlinear_effects": bias,
        "rigid_body_mass_matrix": mass_matrix,
        "centroidal_momentum_matrix": momentum_map,
    }


mjcf_mass = float(pin.computeTotalMass(mjcf_model))
urdf_mass = float(pin.computeTotalMass(urdf_model))
mass_error = abs(mjcf_mass - urdf_mass)

# Check the nominal pose and 20 perturbed poses/velocities.
rng = np.random.default_rng(7)
q_nominal = nominal_configuration(mjcf_model)
maximum_errors = {}

for sample in range(21):
    if sample == 0:
        q = q_nominal.copy()
        v = np.zeros(mjcf_model.nv)
    else:
        increment = rng.normal(0.0, 0.08, mjcf_model.nv)
        q = pin.integrate(mjcf_model, q_nominal, increment)
        v = rng.normal(0.0, 0.2, mjcf_model.nv)

    mj_values = evaluate(mjcf_model, mjcf_data, q, v)
    urdf_values = evaluate(urdf_model, urdf_data, q, v)

    for name, mj_value in mj_values.items():
        urdf_value = urdf_values[name]

        if not (
            np.all(np.isfinite(mj_value))
            and np.all(np.isfinite(urdf_value))
        ):
            raise RuntimeError(f"Non-finite result: {name}")

        error = float(np.max(np.abs(mj_value - urdf_value)))
        maximum_errors[name] = max(
            maximum_errors.get(name, 0.0), error
        )

print(f"MJCF mass: {mjcf_mass:.12f} kg")
print(f"URDF mass: {urdf_mass:.12f} kg")
print(f"Mass error: {mass_error:.3e} kg")
print("\nMaximum absolute errors across 21 samples:")

for name, error in maximum_errors.items():
    print(f"{name:30s}: {error:.3e}")

tolerances = {
    "foot_position": 1e-9,
    "com_position": 1e-9,
    "gravity": 1e-8,
    "nonlinear_effects": 1e-8,
    "rigid_body_mass_matrix": 1e-8,
    "centroidal_momentum_matrix": 1e-8,
}

failed = [
    name for name, error in maximum_errors.items()
    if error > tolerances[name]
]

if not np.isfinite(mass_error) or mass_error > 1e-10:
    failed.append("total_mass")

if failed:
    raise RuntimeError("Model verification failed: " + ", ".join(failed))

print("\nPASS: exported URDF matches the MJCF-derived rigid-body model.")