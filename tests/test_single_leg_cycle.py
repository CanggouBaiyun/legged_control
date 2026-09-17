"""Regression test for the complete single-leg lift and touchdown cycle."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


@pytest.mark.filterwarnings(
    "ignore:The default value of raise_error:PendingDeprecationWarning"
)
def test_headless_single_leg_cycle(tmp_path):
    # Load the example without invoking its command-line entry point.
    project_root = Path(__file__).resolve().parents[1]
    script_path = project_root / "examples/mujoco/wbc_single_leg_cycle.py"
    spec = importlib.util.spec_from_file_location(
        "single_leg_cycle_example", script_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    csv_path = tmp_path / "single_leg_cycle.csv"
    summary = module.run(SimpleNamespace(
        headless=True, mode="tracking", duration=20.0,
        joint_kp=20.0, joint_kd=1.0, csv=csv_path,
    ))

    records = np.genfromtxt(csv_path, delimiter=",", names=True)
    for name in records.dtype.names:
        assert np.isfinite(records[name]).all(), name
    assert records["time_s"][-1] >= 19.9

    # Overall stability and command validity.
    assert summary["qp_failures"] == 0
    assert summary["clipped_steps"] == 0
    assert summary["height_rmse_mm"] < 5.0
    assert summary["height_max_error_mm"] < 8.0
    assert summary["maximum_tilt_deg"] < 3.0

    # Verify physical lift-off, not just the requested contact mode.
    hold = records[
        (records["time_s"] >= 11.5) & (records["time_s"] < 13.0)
    ]
    assert len(hold) > 0
    assert np.all(hold["stance_feet"] == 3)
    assert np.max(np.abs(hold["fr_normal_force_n"])) < 0.5
    assert np.min(hold["fr_z_m"]) > 0.022 + 0.015
    foot_height_error = hold["fr_ref_z_m"] - hold["fr_z_m"]
    assert np.max(np.abs(foot_height_error)) < 0.006

    # Touchdown must occur during lowering and recovery must finish.
    touchdown_time = summary["touchdown_time_s"]
    assert touchdown_time is not None
    assert 13.0 <= touchdown_time <= 17.0
    assert summary["four_contact_recovery_completed"]

    # Verify sustained contact and actual load during the final second.
    final = records[records["time_s"] >= 19.0]
    assert len(final) > 0
    assert np.all(final["stance_feet"] == 4)
    assert np.min(final["fr_normal_force_n"]) > 5.0
    final_position = np.asarray(summary["final_base_position"])
    target_position = np.array([-0.03, 0.03, 0.28])
    assert np.isfinite(final_position).all()
    assert np.linalg.norm(final_position[:2] - target_position[:2]) < 0.005
    assert abs(final_position[2] - target_position[2]) < 0.005
