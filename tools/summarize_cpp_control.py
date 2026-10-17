"""Summarize a completed native C++ simulation CSV (outside the control loop)."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize_commands(path, allow_empty=False):
    """Summarize commands sampled when targets were published, outside control."""
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    required = (
        "target_sequence", "time_s", "wall_elapsed_s", "command_sequence",
        "command_age_s", "command_valid", "command_timed_out",
        "requested_vx_mps", "requested_vy_mps", "requested_yaw_rate_radps",
        "limited_vx_mps", "limited_vy_mps", "limited_yaw_rate_radps",
        "effective_vx_mps", "effective_vy_mps", "effective_yaw_rate_radps",
        "target_vx_world_mps", "target_vy_world_mps", "target_x_m",
        "target_y_m", "target_yaw_rad", "target_end_time_s",
        "target_end_x_m", "target_end_y_m", "target_end_yaw_rad",
    )
    if not set(required).issubset(data.dtype.names or ()):
        raise RuntimeError("Missing online command samples or columns")
    if not len(data):
        if allow_empty:
            return {"path": str(path), "target_publications": 0, "received_command_count": 0,
                    "effective_command_segments": [], "note": "Session ended before the first online target publication."}
        raise RuntimeError("Missing online command samples or columns")
    for name in required:
        # Non-finite rejected requests may be recorded; generated targets must be finite.
        values = data[name][data["command_valid"] == 1] if name.startswith("requested_") else data[name]
        if not np.isfinite(values).all():
            raise RuntimeError(f"Non-finite online command data: {name}")
    if ((np.diff(data["target_sequence"]) != 1).any() or
            (np.diff(data["command_sequence"]) < 0).any() or
            (np.diff(data["time_s"]) < -1e-9).any() or
            (np.diff(data["wall_elapsed_s"]) <= 0).any()):
        raise RuntimeError("Online command log sequence or time moved backward")
    for name in ("command_valid", "command_timed_out"):
        if not np.isin(data[name], (0, 1)).all():
            raise RuntimeError(f"Invalid command flag: {name}")
    effective_fields = ("effective_vx_mps", "effective_vy_mps", "effective_yaw_rate_radps")
    effective = np.column_stack([data[name] for name in effective_fields])
    if (np.abs(effective) > np.array([0.10, 0.05, 0.20]) + 1e-9).any():
        raise RuntimeError("Effective online velocity exceeded its configured limit")
    zero_required = (data["command_timed_out"] == 1) | (data["command_valid"] == 0)
    if (np.abs(effective[zero_required]) > 1e-9).any():
        raise RuntimeError("Invalid or expired command produced nonzero velocity")
    changed = np.ones(len(data), dtype=bool)
    changed[1:] = ((np.abs(np.diff(effective, axis=0)) > 1e-9).any(axis=1) |
                   (np.diff(data["command_valid"]) != 0) |
                   (np.diff(data["command_timed_out"]) != 0))
    starts = np.flatnonzero(changed)
    segments = []
    for start, end in zip(starts, list(starts[1:]) + [len(data)]):
        segments.append({
            "first_sample_s": float(data["time_s"][start]),
            "last_sample_s": float(data["time_s"][end - 1]),
            "vx_mps": float(effective[start, 0]),
            "vy_mps": float(effective[start, 1]),
            "yaw_rate_radps": float(effective[start, 2]),
            "command_valid": bool(data["command_valid"][start]),
            "timed_out": bool(data["command_timed_out"][start]),
        })
    gaps = np.diff(data["time_s"])
    result = {
        "path": str(path), "target_publications": len(data),
        "received_command_count": int(data["command_sequence"].max()),
        "first_observation_time_s": float(data["time_s"][0]),
        "last_observation_time_s": float(data["time_s"][-1]),
        "max_target_observation_gap_s": float(gaps.max()) if len(gaps) else 0.0,
        "samples_with_expired_received_command": int(((data["command_sequence"] > 0) &
                                                       (data["command_timed_out"] == 1)).sum()),
        "max_abs_effective_body_velocity": {
            name: float(np.abs(effective[:, i]).max())
            for i, name in enumerate(("vx_mps", "vy_mps", "yaw_rate_radps"))
        },
        "effective_command_segments": segments,
        "note": "Commands are sampled at target publication; active-policy goal fields are in the control CSV.",
    }
    reference_fields = ("integrated_reference", "reference_lead_x_m",
                        "reference_lead_y_m", "reference_lead_yaw_rad")
    if any(name in data.dtype.names for name in reference_fields):
        if not all(name in data.dtype.names for name in reference_fields):
            raise RuntimeError("Incomplete position-reference metadata")
        for name in reference_fields:
            if not np.isfinite(data[name]).all():
                raise RuntimeError(f"Non-finite position-reference metadata: {name}")
        modes = np.unique(data["integrated_reference"])
        if len(modes) != 1 or modes[0] not in (0, 1):
            raise RuntimeError("Invalid or changing position-reference mode")
        result["position_reference_mode"] = "integrated" if modes[0] else "measured"
        translating_or_turning = (np.abs(effective) > 1e-9).any(axis=1)
        lead_xy = np.hypot(data["reference_lead_x_m"], data["reference_lead_y_m"])
        lead_yaw = np.abs(data["reference_lead_yaw_rad"])
        result["max_moving_position_lead_m"] = float(lead_xy[translating_or_turning].max()) if translating_or_turning.any() else 0.0
        result["max_moving_yaw_lead_rad"] = float(lead_yaw[translating_or_turning].max()) if translating_or_turning.any() else 0.0
        if modes[0] and ((lead_xy[translating_or_turning] > 0.05 + 1e-9).any() or
                         (lead_yaw[translating_or_turning] > 0.20 + 1e-9).any()):
            raise RuntimeError("Integrated reference exceeded its lead limit")
    return result


def summarize(path, duration=20.0, commands=None):
    if not np.isfinite(duration) or duration < 0:
        raise RuntimeError("Duration must be finite and non-negative")
    interactive = duration == 0
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    if interactive and not len(data):
        result = {"samples": 0, "duration_s": 0.0, "requested_duration_s": 0.0,
                  "session_mode": "interactive", "status": "closed_before_first_control_step"}
        path.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print("Interactive session ended before the first control sample; empty CSV saved.")
        return
    if data.ndim != 1 or (not interactive and len(data) != round(duration / 0.002)):
        raise RuntimeError(f"Incomplete run: expected {round(duration / 0.002)} rows")
    if interactive:
        duration = len(data) * 0.002
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
    evaluation_start = 2.0
    if not len(evaluated):
        if not interactive:
            raise RuntimeError("Run must include samples after 2 s")
        evaluated = data
        evaluation_start = 0.0

    def rmse(values):
        return float(np.sqrt(np.mean(values ** 2)))

    def distribution(values):
        return {"mean": float(values.mean()), "p95": float(np.percentile(values, 95)),
                "p99": float(np.percentile(values, 99)), "max": float(values.max())}

    result = {
        "samples": len(data), "duration_s": duration, "evaluation_start_s": evaluation_start,
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
    if interactive:
        result["session_mode"] = "interactive"
        result["requested_duration_s"] = 0.0
    online = False
    if "online_mode" in data.dtype.names:
        modes = np.unique(data["online_mode"])
        if len(modes) != 1 or modes[0] not in (0, 1):
            raise RuntimeError("Invalid or changing online mode")
        online = bool(modes[0])
        result["target_mode"] = "online" if online else "scripted"
        result["goal_source"] = "active_policy_target" if online else "prepared_target_json"
    if online:
        fields = ("goal_y_m", "measured_y_m", "goal_com_vx_world_mps", "goal_com_vy_world_mps",
                  "measured_com_vx_world_mps", "measured_com_vy_world_mps",
                  "active_target_start_s", "active_target_end_s")
        if not set(fields).issubset(data.dtype.names):
            raise RuntimeError("Incomplete online control metadata")
        if (data["active_target_end_s"] < data["time_s"] - 1e-6).any():
            raise RuntimeError("Online policy used an expired target")
        command_path = Path(commands) if commands is not None else path.with_suffix(".commands.csv")
        result["online_commands"] = summarize_commands(command_path, allow_empty=interactive)
        result["online_tracking"] = {
            "evaluation_start_s": evaluation_start,
            "active_goal_x_rmse_mm": 1000 * rmse(evaluated["measured_x_m"] - evaluated["goal_x_m"]),
            "active_goal_y_rmse_mm": 1000 * rmse(evaluated["measured_y_m"] - evaluated["goal_y_m"]),
            "com_vx_rmse_mps": rmse(evaluated["measured_com_vx_world_mps"] - evaluated["goal_com_vx_world_mps"]),
            "com_vy_rmse_mps": rmse(evaluated["measured_com_vy_world_mps"] - evaluated["goal_com_vy_world_mps"]),
            "final_y_m": float(data["measured_y_m"][-1]),
        }
        translating = evaluated[(np.abs(evaluated["goal_com_vx_world_mps"]) > 1e-6) |
                                 (np.abs(evaluated["goal_com_vy_world_mps"]) > 1e-6)]
        result["online_tracking"]["translation_command_samples"] = len(translating)
        if len(translating):
            result["online_tracking"]["during_translation_commands"] = {
                "mean_goal_com_vx_world_mps": float(translating["goal_com_vx_world_mps"].mean()),
                "mean_measured_com_vx_world_mps": float(translating["measured_com_vx_world_mps"].mean()),
                "mean_goal_com_vy_world_mps": float(translating["goal_com_vy_world_mps"].mean()),
                "mean_measured_com_vy_world_mps": float(translating["measured_com_vy_world_mps"].mean()),
                "mpc_x_tracking_rmse_mm": 1000 * rmse(translating["measured_x_m"] - translating["mpc_x_m"]),
            }
    elif commands is not None:
        raise RuntimeError("Command log supplied for a CSV without online mode")
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
    if not online and len(moving):
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
    if "target_mode" in result:
        print(f"Target mode: {result['target_mode']}; goal source: {result['goal_source']}")
    if online:
        command_result = result["online_commands"]
        print(f"Online targets: {command_result['target_publications']}; "
              f"received commands: {command_result['received_command_count']}; "
              f"command log: {command_result['path']}")
        if "position_reference_mode" in command_result:
            print(f"Online position reference: {command_result['position_reference_mode']}; "
                  f"max moving lead: {1000 * command_result['max_moving_position_lead_m']:.3f} mm")
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
    print(f"Steps over 2 ms after {evaluation_start:g} s: control={result['control_over_2ms']}, "
          f"loop={result['loop_over_2ms']} / {len(evaluated)}")
    print(f"Summary saved: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Expected duration; 0 infers the duration of an interactive session")
    parser.add_argument("--commands", type=Path, help="Online command CSV; defaults to <control>.commands.csv")
    args = parser.parse_args()
    summarize(args.csv, args.duration, args.commands)
