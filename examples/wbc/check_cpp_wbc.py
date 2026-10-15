"""Prepare Python WBC fixtures, then compare a native C++ WBC replay."""
import argparse
import json
import math
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pinocchio as pin
from scipy import sparse

from go2_control.model import (
    GO2_SDK_JOINT_ORDER, GO2_FOOT_FRAMES,
    build_go2_simulation_model, nominal_configuration, foot_positions,
)
from go2_control.mpc_reference import OCS2_MODE_CONTACTS
from go2_control.wbc.standing import StandingWBC
import go2_control.wbc.standing as standing


def serializable(value):
    if sparse.issparse(value):
        value = value.toarray()
    if isinstance(value, np.ndarray):
        return serializable(value.tolist())
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [serializable(item) for item in value]
    if isinstance(value, (float, np.floating)):
        if math.isinf(value):
            return "-inf" if value < 0 else "inf"
        if not math.isfinite(value):
            raise ValueError("Non-finite fixture data")
        return float(value)
    return value


def prepare(path):
    model = build_go2_simulation_model()
    motors = []
    for name in GO2_SDK_JOINT_ORDER:
        joint = model.joints[model.getJointId(name)]
        motors.append(dict(name=name, q_index=joint.idx_q, v_index=joint.idx_v))
    payload = dict(schema=1, python_pinocchio=pin.__version__, motors=motors, cases=[])
    original_solver = standing.solve_with_osqp
    specs = [(15, False), (15, True), (9, False), (9, True), (6, False), (6, True), (15, False)]
    for index, (mode, moving) in enumerate(specs):
        contacts = OCS2_MODE_CONTACTS[mode]
        q = nominal_configuration(model)
        v = np.zeros(model.nv)
        if moving:
            displacement = np.linspace(-0.015, 0.025, model.nv)
            displacement[:6] = [0.02, -0.01, 0.005, 0.08, -0.05, 0.12]
            q = pin.integrate(model, q, displacement)
            v = np.linspace(-0.08, 0.1, model.nv)
        rotation = pin.XYZQUATToSE3(q[:7]).rotation.copy()
        reference = dict(
            desired_base_position=q[:3] + [0.003, -0.002, 0.01],
            desired_base_rotation=rotation @ pin.rpy.rpyToMatrix(0.015, -0.01, 0.02),
            desired_base_linear_velocity_world=rotation @ v[:3] * 0.8,
            desired_base_angular_velocity_local=v[3:6] * 0.7,
            desired_base_linear_acceleration_world=np.array([0.10, -0.05, 0.02]),
            desired_base_angular_acceleration_local=np.array([0.04, -0.02, 0.03]),
        )
        if index != 0:
            reference["desired_contact_forces"] = np.array([
                [0.5 * i, -0.3 * i, pin.computeTotalMass(model) * 9.81 / len(contacts) + i]
                for i in range(len(contacts))
            ])
        if index == 6:
            reference["normal_force_upper_bounds"] = np.array([20.0, 100.0, 100.0, 100.0])
        positions = foot_positions(model, q)
        swings = {
            name: dict(
                position=np.asarray(positions[name]) + [0.005, -0.003, 0.025],
                velocity=np.array([0.01, -0.02, 0.04]),
                acceleration=np.array([0.1, -0.1, 0.2]),
            )
            for name in GO2_FOOT_FRAMES if name not in contacts
        }
        if swings:
            reference["swing_targets"] = swings
        kwargs = dict(reference)
        if swings:
            kwargs["swing_targets"] = {
                name: (target["position"], target["velocity"], target["acceleration"])
                for name, target in swings.items()
            }
        captured = {}

        def capture(**solver_args):
            for output, key in dict(
                H="quadratic_cost", g="linear_cost", Aeq="equality_matrix", beq="equality_target",
                C="inequality_matrix", l="inequality_lower_bound", u="inequality_upper_bound",
            ).items():
                captured[output] = serializable(solver_args[key])
            return original_solver(**solver_args)

        controller = StandingWBC(model, contact_frame_names=contacts)
        with patch.object(standing, "solve_with_osqp", side_effect=capture):
            result = controller.solve(q=q, v=v, **kwargs)
        if result.solver_status.lower() != "solved":
            raise RuntimeError("Python fixture did not solve")
        captured["base_acceleration"] = result.desired_base_acceleration
        captured["x"] = np.concatenate([
            result.generalized_acceleration, result.contact_forces.ravel(), result.motor_torques,
        ])
        payload["cases"].append(dict(
            name=f"case_{index}_mode_{mode}", mode=mode, contacts=contacts,
            q=q, v=v, reference=reference, expected=captured,
        ))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(serializable(payload), indent=2, allow_nan=False) + "\n")
    print(f"Saved {len(specs)} Python WBC cases to {path}")


def compare(input_path, output_path):
    fixtures = json.loads(input_path.read_text())
    replay = json.loads(output_path.read_text())
    if fixtures["schema"] != 1 or replay["schema"] != 1:
        raise ValueError("Unsupported schema")
    source_motors = {entry["name"]: entry for entry in fixtures["motors"]}
    v_order = np.arange(18)
    if len(replay["motors"]) != 12 or {m["name"] for m in replay["motors"]} != set(source_motors):
        raise ValueError("Wrong C++ motor metadata")
    for motor in replay["motors"]:
        v_order[motor["v_index"]] = source_motors[motor["name"]]["v_index"]
    if set(v_order) != set(range(18)):
        raise ValueError("Invalid coordinate permutation")
    results = {case["name"]: case for case in replay["cases"]}
    if len(results) != len(replay["cases"]) or set(results) != {c["name"] for c in fixtures["cases"]}:
        raise ValueError("Missing, duplicate or unexpected replay cases")
    report = []
    for fixture in fixtures["cases"]:
        cpp = results[fixture["name"]]
        expected = fixture["expected"]
        nf = 3 * len(fixture["contacts"])
        variables = np.concatenate([v_order, np.arange(18, 18 + nf), 18 + nf + v_order[6:] - 6])
        equations = np.concatenate([v_order, np.arange(18, 18 + nf)])
        inequalities = np.concatenate([np.arange(5 * len(fixture["contacts"])), 5 * len(fixture["contacts"]) + v_order[6:] - 6])
        as_array = lambda key: np.asarray(expected[key], dtype=float)
        python = dict(
            H=as_array("H")[np.ix_(variables, variables)], g=as_array("g")[variables],
            Aeq=as_array("Aeq")[np.ix_(equations, variables)], beq=as_array("beq")[equations],
            C=as_array("C")[np.ix_(inequalities, variables)],
            l=as_array("l")[inequalities], u=as_array("u")[inequalities],
            base_acceleration=as_array("base_acceleration"), x=as_array("x")[variables],
        )
        matrix_errors = {}
        for key, reference in python.items():
            actual = np.asarray(cpp[key], dtype=float)
            np.testing.assert_allclose(
                actual, reference, atol=1e-4 if key == "x" else 1e-8,
                rtol=1e-5 if key == "x" else 1e-9, equal_nan=False, err_msg=f"{fixture['name']}.{key}",
            )
            mask = np.isfinite(reference)
            matrix_errors[key] = float(np.max(np.abs(actual[mask] - reference[mask])))
        if cpp["status"].lower() != "solved" or cpp["dynamics_residual"] > 1e-3 or cpp["contact_residual"] > 1e-3:
            raise AssertionError("C++ WBC status/residual failed")
        x = np.asarray(cpp["x"], dtype=float)
        constrained = np.asarray(cpp["C"], dtype=float) @ x
        violation = max(0.0, float(np.max(python["l"] - constrained)), float(np.max(constrained - python["u"])))
        if violation > 1e-3 or cpp["update_count"] < 1:
            raise AssertionError("Inequality constraint or workspace reuse check failed")
        torque_error = float(np.max(np.abs(x[-12:] - python["x"][-12:])))
        solution_error = float(np.max(np.abs(x - python["x"])))
        print(f"{fixture['name']}: max_torque_error={torque_error:.3e} Nm, max_solution_error={solution_error:.3e}, inequality_violation={violation:.3e}")
        report.append(dict(name=fixture["name"], torque_error_nm=torque_error, solution_error=solution_error,
                           dynamics_residual=cpp["dynamics_residual"], contact_residual=cpp["contact_residual"],
                           inequality_violation=violation, setup_count=cpp["setup_count"], update_count=cpp["update_count"],
                           maximum_absolute_errors=matrix_errors))
    summary_path = output_path.with_name(output_path.stem + "_comparison.json")
    summary_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("PASS: C++ WBC QP matrices, solutions and torque outputs match Python in all 7 cases.")
    print(f"Saved comparison to {summary_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--prepare", type=Path)
    group.add_argument("--check", nargs=2, type=Path, metavar=("INPUT_JSON", "CPP_OUTPUT_JSON"))
    args = parser.parse_args()
    if args.prepare:
        prepare(args.prepare)
    else:
        compare(*args.check)


if __name__ == "__main__":
    main()
