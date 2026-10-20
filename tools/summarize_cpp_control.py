"""Summarize a completed native C++ simulation CSV (outside the control loop)."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize_push(data):
    """Describe a single pulse; recovery thresholds are a scoped experiment definition."""
    fields = ("push_enabled", "push_start_s", "push_duration_s", "push_fx_n", "push_fy_n", "push_fz_n",
              "push_point_x_world_m", "push_point_y_world_m", "push_point_z_world_m")
    if not any(name in data.dtype.names for name in fields):
        return None  # Historical CSVs remain readable.
    if not all(name in data.dtype.names for name in fields):
        raise RuntimeError("Incomplete push metadata")
    for name in fields:
        if not np.isfinite(data[name]).all():
            raise RuntimeError(f"Non-finite push metadata: {name}")
    enabled = np.unique(data["push_enabled"])
    if len(enabled) != 1 or enabled[0] not in (0, 1):
        raise RuntimeError("Invalid push enable flag")
    force = np.column_stack([data[f"push_f{axis}_n"] for axis in "xyz"])
    if not enabled[0]:
        if np.any(force):
            raise RuntimeError("Disabled push contains force")
        return {"enabled": False}
    for name in ("push_start_s", "push_duration_s"):
        if len(np.unique(data[name])) != 1:
            raise RuntimeError("Push configuration changed during run")
    start, duration = float(data["push_start_s"][0]), float(data["push_duration_s"][0])
    if start < 3 or duration < 0.002 or not np.allclose(
            np.array([start, duration]) / 0.002, np.round(np.array([start, duration]) / 0.002), atol=1e-7, rtol=0):
        raise RuntimeError("Invalid pulse timing")
    t, end = data["time_s"], start + duration
    active = (t >= start - 1e-9) & (t < end - 1e-9)
    if active.sum() != round(duration / 0.002) or not (t >= end - 1e-9).any():
        raise RuntimeError("Push pulse or post-push observation incomplete")
    vector = force[active][0]
    if not np.any(vector) or not np.allclose(force[active], vector, atol=1e-10, rtol=0) or np.any(force[~active]):
        raise RuntimeError("Push force differs from a single constant pulse")
    # This protocol measures return to the previous tracking offset of a fixed horizontal target.
    window = (t >= start - 0.5 - 1e-9) & (t < end + 5 - 1e-9)
    baseline = (t >= start - 0.5 - 1e-9) & (t < start - 1e-9)
    if baseline.sum() != 250:
        raise RuntimeError("Missing 0.5 s push baseline")
    goals = np.column_stack([data[f"goal_{axis}_m"] for axis in "xyz"])
    yaw_error = np.arctan2(np.sin(data["measured_yaw_rad"] - data["goal_yaw_rad"]),
                           np.cos(data["measured_yaw_rad"] - data["goal_yaw_rad"]))
    error = np.column_stack([data[f"measured_{axis}_m"] for axis in "xyz"]) - goals
    position_change = error - error[baseline].mean(axis=0)
    velocity = np.column_stack([data[f"truth_base_v{axis}_world_mps"] for axis in "xyz"])
    velocity_change = velocity - velocity[baseline].mean(axis=0)
    tilt = np.maximum(np.abs(data["measured_roll_rad"]), np.abs(data["measured_pitch_rad"]))
    horizontal = np.linalg.norm(position_change[:, :2], axis=1)
    speed = np.linalg.norm(velocity_change[:, :2], axis=1)
    after = (t >= end - 1e-9) & window
    during_and_after = (t >= start - 1e-9) & window
    fixed_target = (np.ptp(goals[window, :2], axis=0).max() < 1e-8 and
                    np.ptp(np.unwrap(data["goal_yaw_rad"][window])) < 1e-8)
    good = (horizontal <= 0.02) & (np.abs(position_change[:, 2]) <= 0.02) & (speed <= 0.03) & (
        tilt <= np.deg2rad(3)) & (np.abs(yaw_error) <= np.deg2rad(5))
    recovery, confirmed, count = None, None, 0
    if fixed_target:
        for i in np.flatnonzero(after):
            count = count + 1 if good[i] else 0
            if count >= 250:  # All 250 consecutive 2 ms intervals satisfy the thresholds.
                recovery = max(0.0, float(t[i - 249] - end))
                confirmed = float(t[i] + 0.002)
                break
    points = np.column_stack([data[f"push_point_{axis}_world_m"] for axis in "xyz"])
    result = {
        "enabled": True, "start_s": start, "duration_s": duration,
        "force_world_n": vector.tolist(), "impulse_world_ns": (force.sum(axis=0) * 0.002).tolist(),
        "application": "base_link center of mass (not whole-robot COM), world axes; no direct torque",
        "point_at_onset_world_m": points[active][0].tolist(),
        "baseline_window_s": [start - 0.5, start],
        "evaluation_end_s": float(t[after][-1] + 0.002),
        "max_horizontal_tracking_offset_change_m": float(horizontal[during_and_after].max()),
        "max_roll_pitch_deg": float(np.rad2deg(tilt[during_and_after].max())),
        "max_horizontal_velocity_change_mps": float(speed[during_and_after].max()),
        "max_estimated_horizontal_position_error_m": (float(np.hypot(
            data["estimator_position_error_x_m"][during_and_after], data["estimator_position_error_y_m"][during_and_after]).max())
            if "estimator_shadow_enabled" in data.dtype.names and (data["estimator_shadow_enabled"][during_and_after] == 1).all()
            else None),
        "maximum_applied_torque_nm": float(data["max_applied_torque_nm"][during_and_after].max()),
        "clipped_steps": int((data["max_torque_clip_nm"][during_and_after] > 1e-9).sum()),
        "recovery_applicable": bool(fixed_target), "recovery_time_after_force_end_s": recovery,
        "recovery_confirmed_at_s": confirmed,
        "thresholds": {"horizontal_offset_change_m": 0.02, "vertical_offset_change_m": 0.02,
                       "horizontal_velocity_change_mps": 0.03, "absolute_roll_pitch_deg": 3,
                       "absolute_yaw_tracking_error_deg": 5, "continuous_duration_s": 0.5},
        "note": "0.5 s pre-push mean is the baseline. Evaluate at most 5 s after force end. "
                "Recovery is not evaluated for moving horizontal/yaw targets; height tracking offset is baseline-relative. "
                "No mouse interaction during quantitative runs. A recovered window is not a guarantee for larger pushes.",
    }
    return result


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
    requested_duration = duration
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    session_path = path.with_suffix(".session.json")
    session = None
    if session_path.exists():
        session = json.loads(session_path.read_text())
        if (int(session["schema"]) != 1 or session["state"] != "stopped" or
                session["csv_flushed"] not in (True, "true") or
                int(session["samples"]) != len(data) or
                abs(float(session["time_s"]) - len(data) * 0.002) > 1e-8):
            raise RuntimeError("Session report does not describe a completed, flushed CSV")
        controlled_stop = session["controlled_stop"] in (True, "true")
        if controlled_stop != (session["reason"] == "controlled_stop"):
            raise RuntimeError("Inconsistent controlled-stop report")
        if abs(float(session["requested_duration_s"]) - duration) > 1e-8:
            raise RuntimeError("Requested duration differs from session report")
        intentional_early_exit = controlled_stop or session["reason"] == "viewer_closed"
        if intentional_early_exit and duration > 0 and len(data) <= round(duration / 0.002):
            duration = len(data) * 0.002
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
    if session is not None:
        result["control_session"] = session
        result["requested_duration_s"] = requested_duration
        if requested_duration > 0 and duration < requested_duration:
            result["session_mode"] = "ended_early"
        if "control_session_state" not in data.dtype.names or not np.isin(data["control_session_state"], (1, 2, 3, 4)).all():
            raise RuntimeError("Invalid control-session state samples")
        if session["reason"] == "controlled_stop":
            if not len(data) or data["control_session_state"][-1] != 4:
                raise RuntimeError("Controlled stop has no stopping state samples")
            settled = data[data["time_s"] >= data["time_s"][-1] - 0.3 + 1e-9]
            if (settled["mode"] != 15).any() or (settled["startup_pd_weight"] != 0).any():
                raise RuntimeError("Controlled stop did not finish in running four-foot stance")
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
    startup_fields = ("sensor_startup_enabled", "startup_stage", "startup_pd_weight", "control_uses_estimate")
    if any(name in data.dtype.names for name in startup_fields):
        if not all(name in data.dtype.names for name in startup_fields):
            raise RuntimeError("Incomplete sensor-startup metadata")
        enabled = np.unique(data["sensor_startup_enabled"])
        if len(enabled) != 1 or enabled[0] not in (0, 1):
            raise RuntimeError("Invalid or changing sensor-startup mode")
        if enabled[0]:
            stage, weight = data["startup_stage"], data["startup_pd_weight"]
            if (not np.isin(stage, (1, 2, 3)).all() or stage[0] != 1 or
                    (np.diff(stage) < 0).any() or (np.diff(stage) > 1).any() or
                    (weight < 0).any() or (weight > 1).any() or
                    (np.diff(weight) > 1e-9).any() or
                    (data["control_uses_estimate"] != 1).any()):
                raise RuntimeError("Invalid sensor-startup stage, blend or state source")
            for axis in ("x", "y", "z"):
                if (not np.allclose(data[f"control_base_{axis}_m"], data[f"estimated_{axis}_m"], atol=1e-10, rtol=0) or
                        not np.allclose(data[f"control_base_v{axis}_world_mps"], data[f"estimated_v{axis}_world_mps"], atol=1e-10, rtol=0)):
                    raise RuntimeError("Sensor-startup control did not use the estimated base state")
            preparing, blending, running = stage == 1, stage == 2, stage == 3
            if ((data["mode"][~running] != 15).any() or
                    (weight[preparing] != 1).any() or (weight[running] != 0).any() or
                    (data["max_wbc_torque_nm"][preparing] != 0).any() or
                    (data["estimator_control_active"][preparing] != 0).any() or
                    (data["estimator_control_active"][~preparing] != 1).any()):
                raise RuntimeError("Invalid preparation or MPC/WBC handoff")
            if not interactive and (not blending.any() or not running.any()):
                raise RuntimeError("Sensor startup did not complete its handoff")
            handoff = float(data["time_s"][blending][0]) if blending.any() else None
            completed = float(data["time_s"][running][0]) if running.any() else None
            if handoff is not None and not 2.2 - 1e-9 <= handoff < 3.0:
                raise RuntimeError("Sensor startup activated outside its readiness window")
            if completed is not None and abs(completed - handoff - 0.2) > 0.002 + 1e-9:
                raise RuntimeError("Sensor startup did not blend for 0.2 seconds")
            result["sensor_startup"] = {
                "stage_samples": {str(int(value)): int((stage == value).sum()) for value in np.unique(stage)},
                "handoff_start_s": handoff, "handoff_complete_s": completed,
                "control_uses_estimate_from_first_sample": True,
                "preparation_joint_kp": 60.0, "preparation_joint_kd": 3.0,
                "preparation_wbc_executed": False,
                "note": "Flat-ground nominal preparation pose; ideal IMU attitude and simulation contact flags. MPC prewarmed before physics starts.",
            }
    command_columns = [name for name in data.dtype.names if name.startswith("cmd_")]
    if command_columns:
        joints = [name[4:-3] for name in command_columns if name.endswith("_kp")]
        fields = ("q_rad", "dq_radps", "kp", "kd", "tau_ff_nm")
        expected = {f"cmd_{joint}_{field}" for joint in joints for field in fields}
        if len(joints) != 12 or set(command_columns) != expected:
            raise RuntimeError("Incomplete joint-command fields")
        weight = data["startup_pd_weight"]
        for joint in joints:
            kp, kd = data[f"cmd_{joint}_kp"], data[f"cmd_{joint}_kd"]
            if (kp < 0).any() or (kd < 0).any():
                raise RuntimeError("Negative joint-command gains")
            if (not np.allclose(kp, 60 * weight + (1 - weight) * data["joint_kp"], rtol=0, atol=1e-10) or
                    not np.allclose(kd, 3 * weight + (1 - weight) * data["joint_kd"], rtol=0, atol=1e-10)):
                raise RuntimeError("Joint-command gains disagree with the handoff")
            if (data[f"cmd_{joint}_tau_ff_nm"][weight == 1] != 0).any():
                raise RuntimeError("Preparation command unexpectedly contained feedforward torque")
        result["joint_command"] = {
            "fields": list(fields), "order": joints,
            "execution": "MuJoCo evaluates tau_ff + Kp*(q_des-q_sensor) + Kd*(dq_des-dq_sensor), then clips total torque to actuator limits.",
            "hardware_mapping_verified": False,
        }
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
    push = summarize_push(data)
    if push is not None:
        result["push"] = push
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
    if "sensor_startup" in result:
        startup = result["sensor_startup"]
        print(f"Sensor startup: handoff {startup['handoff_start_s']} -> {startup['handoff_complete_s']} s; "
              "estimated base state used from the first sample")
    if session is not None:
        print(f"Control session: {session['reason']}; report: {session_path}")
    if "joint_command" in result:
        print("Joint commands: q/dq/Kp/Kd/tau_ff; all 12 joints and handoff gains verified")
    if push is not None and push["enabled"]:
        print(f"Push: {push['force_world_n']} N for {push['duration_s']:g} s; "
              f"impulse={push['impulse_world_ns']} N*s")
        print(f"Push response: max roll/pitch={push['max_roll_pitch_deg']:.3f} deg; "
              f"horizontal offset change={1000 * push['max_horizontal_tracking_offset_change_m']:.3f} mm; "
              f"recovery after force end={push['recovery_time_after_force_end_s']} s (None=not confirmed/applicable)")
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
