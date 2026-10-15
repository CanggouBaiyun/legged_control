"""Compare C++ Go2 model/dynamics output with the existing Python implementation."""

import argparse
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    GO2_SDK_JOINT_ORDER,
    build_go2_simulation_model,
    foot_positions,
)
from go2_control.wbc.dynamics import compute_dynamics_terms
from go2_control.wbc.kinematics import compute_contact_kinematics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cpp_json", type=Path)
    args = parser.parse_args()
    payload = json.loads(args.cpp_json.read_text())
    if payload["schema"] != 1:
        raise ValueError("Unsupported C++ output schema")
    model = build_go2_simulation_model()
    motors = payload["motors"]
    if len(motors) != 12 or {m["name"] for m in motors} != set(GO2_SDK_JOINT_ORDER):
        raise ValueError("C++ motor names do not match Go2")
    if {m["q_index"] for m in motors} != set(range(7, 19)) or {
        m["v_index"] for m in motors
    } != set(range(6, 18)):
        raise ValueError("C++ motor indices must cover all actuated coordinates")
    feet = payload["foot_names"]
    if len(feet) != 4 or set(feet) != set(GO2_FOOT_FRAMES):
        raise ValueError("C++ foot names do not match Go2")
    q_order = np.arange(model.nq)
    v_order = np.arange(model.nv)
    for motor in motors:
        joint = model.joints[model.getJointId(motor["name"])]
        q_order[motor["q_index"]] = joint.idx_q
        v_order[motor["v_index"]] = joint.idx_v

    errors = {}

    def compare(label, cpp, python):
        cpp = np.asarray(cpp, dtype=float)
        python = np.asarray(python, dtype=float)
        if cpp.shape != python.shape or not np.isfinite(cpp).all() or not np.isfinite(python).all():
            raise ValueError(f"Invalid array for {label}: {cpp.shape} vs {python.shape}")
        error = float(np.max(np.abs(cpp - python)))
        errors[label] = error
        np.testing.assert_allclose(
            cpp, python, atol=1e-10, rtol=1e-9, err_msg=label,
        )
        print(f"{label:35s} max_abs_error={error:.3e}")

    compare("model.mass_kg", payload["mass_kg"], pin.computeTotalMass(model))
    compare("model.armature", payload["armature"], model.armature[v_order])
    compare("model.effort_limits", payload["effort_limits"], model.effortLimit[v_order[6:]])
    compare("model.velocity_limits", payload["velocity_limits"], model.velocityLimit[v_order[6:]])
    expected = {"nominal", "tilted_moving", "opposite_moving"}
    if len(payload["cases"]) != 3 or {c["name"] for c in payload["cases"]} != expected:
        raise ValueError("Missing or duplicate model/dynamics cases")
    for case in payload["cases"]:
        q_cpp = np.asarray(case["q"], dtype=float)
        v_cpp = np.asarray(case["v"], dtype=float)
        if q_cpp.shape != (19,) or v_cpp.shape != (18,) or not (
            np.isfinite(q_cpp).all() and np.isfinite(v_cpp).all()
        ) or abs(np.linalg.norm(q_cpp[3:7]) - 1.0) > 1e-8:
            raise ValueError("Invalid q/v in C++ sample")
        q = np.empty(model.nq)
        v = np.empty(model.nv)
        q[q_order] = q_cpp
        v[v_order] = v_cpp
        mass_matrix, nonlinear = compute_dynamics_terms(model, model.createData(), q, v)
        jacobian, bias = compute_contact_kinematics(model, model.createData(), q, v, feet)
        positions = foot_positions(model, q)
        if np.linalg.eigvalsh(mass_matrix).min() <= 0:
            raise ValueError("Mass matrix is not positive definite")
        if case["name"] != "nominal" and np.linalg.norm(bias) < 1e-6:
            raise ValueError("Moving sample must exercise nonzero Jdot*v")
        prefix = case["name"]
        compare(f"{prefix}.M", case["M"], mass_matrix[np.ix_(v_order, v_order)])
        compare(f"{prefix}.h", case["h"], nonlinear[v_order])
        compare(f"{prefix}.J", case["J"], jacobian[:, v_order])
        compare(f"{prefix}.Jdot_v", case["Jdot_v"], bias)
        compare(f"{prefix}.feet", case["feet"], [positions[name] for name in feet])

    summary_path = args.cpp_json.with_name(args.cpp_json.stem + "_comparison.json")
    summary = {
        "cpp_pinocchio": payload["pinocchio_version"],
        "python_pinocchio": pin.__version__,
        "cases": 3,
        "atol": 1e-10,
        "rtol": 1e-9,
        "maximum_absolute_errors": errors,
    }
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print("PASS: model limits, armature, FK, M, h, J and Jdot*v match in all 3 cases.")
    print(f"Saved comparison to {summary_path}")


if __name__ == "__main__":
    main()
