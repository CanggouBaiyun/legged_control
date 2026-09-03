#!/usr/bin/env python3
"""Inspect and numerically validate the official Go2 Pinocchio model."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

from go2_control.model import (
    GO2_FOOT_FRAMES,
    PROJECT_ROOT,
    build_go2_model,
    foot_positions,
    load_model_config,
    nominal_configuration,
    sdk_joint_mapping,
)


def inspect_model() -> dict[str, object]:
    config = load_model_config()
    model = build_go2_model(config["urdf"])
    q = nominal_configuration(model)
    v = np.zeros(model.nv)

    mass_matrix = pin.crba(model, model.createData(), q)
    mass_matrix = (mass_matrix + mass_matrix.T) * 0.5
    eigenvalues = np.linalg.eigvalsh(mass_matrix)
    nonlinear_effects = pin.nonLinearEffects(model, model.createData(), q, v)
    mapping = sdk_joint_mapping(model)

    return {
        "source_urdf": str(config["urdf"]),
        "source_mjcf": str(config["mjcf"]),
        "model_name": model.name,
        "nq": model.nq,
        "nv": model.nv,
        "joint_count_including_universe": model.njoints,
        "frame_count": len(model.frames),
        "actuated_dof": len(mapping),
        "total_mass_kg": float(pin.computeTotalMass(model)),
        "sdk_joint_order": [entry["name"] for entry in mapping],
        "sdk_to_pinocchio": mapping,
        "foot_frames": list(GO2_FOOT_FRAMES),
        "nominal_foot_positions_world_m": foot_positions(model, q),
        "mass_matrix_symmetry_error": float(
            np.linalg.norm(mass_matrix - mass_matrix.T, ord=np.inf)
        ),
        "mass_matrix_min_eigenvalue": float(eigenvalues.min()),
        "gravity_generalized_force_norm": float(np.linalg.norm(nonlinear_effects)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "go2_model_summary.json",
    )
    args = parser.parse_args()

    summary = inspect_model()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"\nSaved model report to {args.output}")


if __name__ == "__main__":
    main()

