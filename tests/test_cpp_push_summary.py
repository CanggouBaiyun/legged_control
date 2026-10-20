"""Check that pulse accounting and recovery reports cannot pass incomplete data."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "cpp_summary", Path(__file__).resolve().parents[1] / "tools/summarize_cpp_control.py"
)
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def samples():
    names = ["time_s", "push_enabled", "push_start_s", "push_duration_s"]
    names += [f"push_f{a}_n" for a in "xyz"] + [f"push_point_{a}_world_m" for a in "xyz"]
    names += [f"{prefix}_{a}_m" for prefix in ("goal", "measured") for a in "xyz"]
    names += [f"truth_base_v{a}_world_mps" for a in "xyz"]
    names += ["measured_roll_rad", "measured_pitch_rad", "measured_yaw_rad", "goal_yaw_rad",
              "estimator_position_error_x_m", "estimator_position_error_y_m",
              "max_applied_torque_nm", "max_torque_clip_nm"]
    data = np.zeros(7000, dtype=[(n, float) for n in names])
    data["time_s"] = np.arange(len(data)) * 0.002
    data["push_enabled"], data["push_start_s"], data["push_duration_s"] = 1, 8, 0.2
    data["push_fy_n"][4000:4100] = -20
    return data


def test_signed_impulse_and_continuous_recovery():
    data = samples()
    data["measured_y_m"][4000:4500] = 0.04
    data["measured_y_m"][4300:4310] = 0  # A brief crossing is not recovery.
    result = summary.summarize_push(data)
    np.testing.assert_allclose(result["impulse_world_ns"], [0, -4, 0])
    assert result["recovery_time_after_force_end_s"] == pytest.approx(0.8)
    assert result["recovery_confirmed_at_s"] == pytest.approx(9.5)


def test_persistent_offset_is_not_recovered():
    data = samples()
    data["measured_y_m"][4000:] = 0.04
    assert summary.summarize_push(data)["recovery_time_after_force_end_s"] is None


def test_incomplete_or_leaking_pulse_is_rejected():
    data = samples()
    with pytest.raises(RuntimeError, match="incomplete"):
        summary.summarize_push(data[:4050])
    data["push_fy_n"][4200] = 1
    with pytest.raises(RuntimeError, match="single constant pulse"):
        summary.summarize_push(data)


def test_moving_goal_does_not_claim_position_recovery():
    data = samples()
    data["goal_x_m"] = 0.1 * data["time_s"]
    result = summary.summarize_push(data)
    assert not result["recovery_applicable"]
    assert result["recovery_time_after_force_end_s"] is None
