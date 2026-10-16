"""Summarize a completed native C++ simulation CSV (outside the control loop)."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize(path, duration=20.0):
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.ndim != 1 or len(data) != round(duration / 0.002):
        raise RuntimeError(f"Incomplete run: expected {round(duration / 0.002)} rows")
    for name in data.dtype.names:
        if name.startswith("swing_target_"):
            continue
        if not np.isfinite(data[name]).all():
            raise RuntimeError(f"Non-finite data: {name}")
    if not np.allclose(data["time_s"], np.arange(len(data)) * 0.002, atol=1e-9, rtol=0):
        raise RuntimeError("CSV does not contain consecutive 2 ms samples")
    if (np.diff(data["policy_sequence"]) < 0).any() or len(np.unique(data["policy_reset_epoch"])) != 1:
        raise RuntimeError("Policy identity moved backward or changed epoch")
    if (data["policy_sequence"] <= 0).any() or (data["policy_reset_epoch"] <= 0).any():
        raise RuntimeError("Missing policy identity")
    if (data["policy_age_ms"] < -0.001).any() or data["policy_age_ms"].max() > 100.0:
        raise RuntimeError("Policy exceeded the accepted age")
    if data["dynamics_residual"].max() > 1e-3 or data["contact_acceleration_residual"].max() > 1e-3:
        raise RuntimeError("WBC residual exceeded its limit")
    evaluated = data[data["time_s"] >= 2.0]
    if not len(evaluated):
        raise RuntimeError("Run must include samples after 2 s")

    def rmse(values):
        return float(np.sqrt(np.mean(values ** 2)))

    def distribution(values):
        return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)),
                "p99": float(np.percentile(values, 99)), "max": float(values.max())}

    result = {
        "samples": len(data), "duration_s": duration, "evaluation_start_s": 2.0,
        "mode_samples": {str(int(mode)): int((data["mode"] == mode).sum()) for mode in np.unique(data["mode"])},
        "first_non_stance_s": None,
        "final_x_m": float(data["measured_x_m"][-1]),
        "final_goal_x_error_mm": float(1000 * (data["measured_x_m"][-1] - data["goal_x_m"][-1])),
        "final_yaw_deg": float(np.rad2deg(data["measured_yaw_rad"][-1])),
        "final_goal_yaw_error_deg": float(np.rad2deg(data["measured_yaw_rad"][-1] - data["goal_yaw_rad"][-1])),
        "height_range_m": [float(data["measured_z_m"].min()), float(data["measured_z_m"].max())],
        "goal_height_rmse_mm": 1000 * rmse(evaluated["measured_z_m"] - evaluated["goal_z_m"]),
        "mpc_height_tracking_rmse_mm": 1000 * rmse(evaluated["measured_z_m"] - evaluated["mpc_z_m"]),
        "max_roll_pitch_deg": float(np.rad2deg(max(np.abs(data["measured_roll_rad"]).max(), np.abs(data["measured_pitch_rad"]).max()))),
        "maximum_applied_torque_nm": float(data["max_applied_torque_nm"].max()),
        "clipped_steps": int((data["max_torque_clip_nm"] > 1e-9).sum()),
        "maximum_dynamics_residual": float(data["dynamics_residual"].max()),
        "maximum_contact_residual": float(data["contact_acceleration_residual"].max()),
        "policy_age_ms": distribution(evaluated["policy_age_ms"]),
        "timing_ms": {name: distribution(evaluated[name]) for name in
                      ("state_ms", "policy_ms", "mrt_ms", "reference_ms", "wbc_ms", "command_ms",
                       "control_ms", "simulation_ms", "diagnostics_ms", "viewer_ms", "loop_ms", "control_start_lateness_ms")},
        "control_over_2ms": int((evaluated["control_ms"] > 2).sum()),
        "loop_over_2ms": int((evaluated["loop_ms"] > 2).sum()),
        "timing_evaluated_samples": len(evaluated),
        "stance_contact_fraction_over_1n": {},
        "raised_swing_contact_samples": {},
        "swing_height_tracking_rmse_mm": {},
    }
    if "policy_controller_type" in data.dtype.names:
        controller_names = {1: "feedforward", 2: "linear"}
        types = np.unique(data["policy_controller_type"])
        if any(value not in controller_names for value in types):
            raise RuntimeError("Unsupported policy controller type in CSV")
        result["policy_controller_samples"] = {
            controller_names[int(value)]: int((data["policy_controller_type"] == value).sum())
            for value in types
        }
    gain_columns = ("joint_kp", "joint_kd")
    if any(name in data.dtype.names for name in gain_columns):
        if not all(name in data.dtype.names for name in gain_columns):
            raise RuntimeError("Incomplete joint-gain metadata in CSV")
        result["joint_gains"] = {}
        for name in gain_columns:
            values = np.unique(data[name])
            if len(values) != 1 or values[0] < 0:
                raise RuntimeError(f"Invalid or changing joint gain: {name}")
            result["joint_gains"][name] = float(values[0])
    if "centroidal_base_accel" in data.dtype.names:
        sources = np.unique(data["centroidal_base_accel"])
        if len(sources) != 1 or sources[0] not in (0, 1):
            raise RuntimeError("Invalid or changing base acceleration source")
        result["base_acceleration_source"] = "centroidal" if sources[0] else "tracking"
    wbc_fields = (
        "swing_kp", "swing_kd", "swing_acceleration_feedforward",
        "base_weight", "swing_weight", "force_weight",
        "joint_regularization", "torque_regularization",
    )
    if any(f"wbc_{name}" in data.dtype.names for name in wbc_fields):
        if not all(f"wbc_{name}" in data.dtype.names for name in wbc_fields):
            raise RuntimeError("Incomplete WBC settings in CSV")
        result["wbc_settings"] = {}
        for name in wbc_fields:
            values = np.unique(data[f"wbc_{name}"])
            if len(values) != 1:
                raise RuntimeError(f"Changing WBC setting: {name}")
            value = float(values[0])
            if name == "swing_acceleration_feedforward":
                if value not in (0, 1):
                    raise RuntimeError("Invalid WBC acceleration-feedforward flag")
                value = bool(value)
            elif (value < 0 if name in ("swing_kp", "swing_kd") else value <= 0):
                raise RuntimeError(f"Invalid WBC setting: {name}")
            result["wbc_settings"][name] = value
    walking = data[data["mode"] != 15]
    if len(walking):
        result["first_non_stance_s"] = float(walking["time_s"][0])
        result["last_non_stance_s"] = float(walking["time_s"][-1])
    moving = data[(data["time_s"] >= 8) & (data["time_s"] <= 12)]
    if len(moving):
        result["moving_goal_x_rmse_mm"] = 1000 * rmse(moving["measured_x_m"] - moving["goal_x_m"])
    for leg, bit in zip(("fl", "fr", "rl", "rr"), (8, 4, 2, 1)):
        stance = (evaluated["mode"].astype(int) & bit) != 0
        swing = ~stance
        force = evaluated[f"mj_floor_fz_{leg}_n"]
        target = evaluated[f"swing_target_z_{leg}_m"]
        result["stance_contact_fraction_over_1n"][leg] = float((force[stance] > 1).mean()) if stance.any() else None
        result["raised_swing_contact_samples"][leg] = int((swing & (target > 0.04) & (force > 1)).sum())
        result["swing_height_tracking_rmse_mm"][leg] = (
            1000 * rmse(evaluated[f"foot_z_{leg}_m"][swing] - target[swing]) if swing.any() else None
        )
    output = path.with_suffix(".json")
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Completed {duration:g} s / {len(data)} steps; modes: {result['mode_samples']}")
    if "policy_controller_samples" in result:
        print(f"Policy controller samples: {result['policy_controller_samples']}")
    if "joint_gains" in result:
        gains = result["joint_gains"]
        print(f"Joint gains: Kp={gains['joint_kp']:g}, Kd={gains['joint_kd']:g}")
    if "base_acceleration_source" in result:
        print(f"Base acceleration source: {result['base_acceleration_source']}")
    if "wbc_settings" in result:
        print(f"WBC settings: {result['wbc_settings']}")
    print(f"Final x: {result['final_x_m']:.6f} m; yaw: {result['final_yaw_deg']:.3f} deg; "
          f"torque: {result['maximum_applied_torque_nm']:.3f} Nm; clipped steps: {result['clipped_steps']}")
    for name in ("control_ms", "loop_ms", "control_start_lateness_ms"):
        stats = result["timing_ms"][name]
        print(f"{name}: mean={stats['mean']:.4f}, p95={stats['p95']:.4f}, "
              f"p99={stats['p99']:.4f}, max={stats['max']:.4f}")
    print(f"Steps over 2 ms after 2 s: control={result['control_over_2ms']}, "
          f"loop={result['loop_over_2ms']} / {len(evaluated)}")
    print(f"Summary saved: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()
    summarize(args.csv, args.duration)
