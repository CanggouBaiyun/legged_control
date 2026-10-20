"""Run a fixed-parameter push matrix in MuJoCo; no robot connection required.

Uses the existing launcher and recovery definition. Every case has independent
logs; failed simulations are preserved and do not hide later cases.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DIRECTIONS = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}


def save_report(path, cases, force, complete):
    report = {
        "complete": complete, "passed": complete and all(c.get("passed", False) for c in cases),
        "force_n": force, "push_start_s": 8.0, "push_duration_s": 0.2,
        "application": "world axes, base_link own COM; no added torque",
        "control": "sensor startup, SQP, Kp=0/Kd=1, centroidal, upstream_a1_tasks_go2.json",
        "recovery_definition": "Existing summarize_cpp_control.py thresholds, unchanged; 0.5 s continuous recovery within 5 s after force end.",
        "note": "Single trial per case on ideal flat-ground simulation. Timing is reported, not accepted as hard real time. No robot commands sent.",
        "cases": cases,
    }
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)
    rows = ["| Case | Status | Peak roll/pitch (deg) | XY offset change (mm) | Recovery (s) | Control/loop >2ms |",
            "| --- | --- | ---: | ---: | ---: | ---: |"]
    for case in cases:
        metrics = case.get("metrics", {})
        def number(name, scale=1):
            value = metrics.get(name)
            return "—" if value is None else f"{scale * value:.3f}"
        timing = case.get("timing", {})
        rows.append(f"| {case['case']} | {case['status']} | {number('peak_roll_pitch_deg')} | "
                    f"{number('horizontal_offset_change_m', 1000)} | {number('recovery_s')} | "
                    f"{timing.get('control_over_2ms', '—')}/{timing.get('loop_over_2ms', '—')} |")
    path.with_suffix(".md").write_text("# Simulation push matrix\n\n" + "\n".join(rows) +
                                     "\n\nSee results.json and per-case logs for failures and exact criteria.\n")


def terminate(proc):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def evaluate(case, stem):
    errors = []
    if case["exit_code"] != 0:
        errors.append(f"Launcher exit code {case['exit_code']}")
    session_path = stem.with_suffix(".session.json")
    if session_path.exists():
        session = json.loads(session_path.read_text())
        case["session"] = session
        if (session["state"] != "stopped" or session["reason"] != "duration_limit" or
                int(session["samples"]) != 10000 or session["csv_flushed"] not in (True, "true")):
            errors.append(f"Session {session['state']}: {session['reason']}")
    else:
        errors.append("Missing session report")
    summary_path = stem.with_suffix(".json")
    if summary_path.exists():
        summary = json.loads(summary_path.read_text())
        push = summary["push"]
        case["metrics"] = {
            "peak_roll_pitch_deg": push["max_roll_pitch_deg"],
            "horizontal_offset_change_m": push["max_horizontal_tracking_offset_change_m"],
            "recovery_s": push["recovery_time_after_force_end_s"],
            "recovery_confirmed_at_s": push["recovery_confirmed_at_s"],
            "max_horizontal_velocity_change_mps": push["max_horizontal_velocity_change_mps"],
            "max_estimated_horizontal_position_error_m": push["max_estimated_horizontal_position_error_m"],
            "maximum_applied_torque_nm": summary["maximum_applied_torque_nm"],
            "clipped_steps": summary["clipped_steps"],
        }
        case["timing"] = {key: summary[key] for key in ("control_over_2ms", "loop_over_2ms", "timing_evaluated_samples")}
        case["timing"]["distributions_ms"] = summary["timing_ms"]
        case["recovery_thresholds"] = push["thresholds"]
        if summary["samples"] != 10000 or abs(summary["duration_s"] - 20) > 1e-8:
            errors.append("Incomplete 20 s simulation")
        if not summary["sensor_startup"]["control_uses_estimate_from_first_sample"]:
            errors.append("Sensor-driven control not enabled")
        if summary["clipped_steps"]:
            errors.append("Actuator torque clipped")
        expected = case["force_world_n"]
        if (not push["enabled"] or abs(push["start_s"] - 8) > 1e-8 or abs(push["duration_s"] - 0.2) > 1e-8 or
                any(abs(a - b) > 1e-8 for a, b in zip(push["force_world_n"], expected)) or
                any(abs(a - 0.2 * b) > 1e-8 for a, b in zip(push["impulse_world_ns"], expected))):
            errors.append("Pulse configuration or impulse mismatch")
        recovery = push["recovery_time_after_force_end_s"]
        if (not push["recovery_applicable"] or recovery is None or not 0 <= recovery <= 4.5 + 1e-8 or
                push["recovery_confirmed_at_s"] is None or push["recovery_confirmed_at_s"] > 13.2 + 1e-8 or
                push["evaluation_end_s"] < 13.2 - 1e-8):
            errors.append("No confirmed continuous recovery within 5 s")
        modes = summary["mode_samples"]
        if case["gait"] == "trot":
            if not all(modes.get(m, 0) > 0 for m in ("6", "9", "15")) or summary["last_non_stance_s"] >= 19.5:
                errors.append("Trot or final return to stance incomplete")
        elif modes != {"15": 10000}:
            errors.append("Standing case changed gait")
    else:
        errors.append("Missing completed simulation summary (inspect partial CSV and session report)")
    text = stem.with_suffix(".run.log").read_text()
    rejected = re.findall(r"Completed 10000 steps; rejected policies=(\d+)", text)
    case["rejected_policies"] = int(rejected[-1]) if rejected else None
    if case["rejected_policies"] != 0:
        errors.append("Policy rejection or missing native completion record")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; refuses existing results")
    parser.add_argument("--force", type=float, default=40.0, help="Force magnitude in N (default: 40)")
    parser.add_argument("--gait", choices=("standing", "trot", "both"), default="both")
    parser.add_argument("--direction", choices=tuple(DIRECTIONS), help="Omit for all four horizontal directions")
    parser.add_argument("--case-timeout", type=float, default=120.0, help="Wall seconds, including MPC prewarm")
    args = parser.parse_args()
    if not math.isfinite(args.force) or args.force <= 0 or not math.isfinite(args.case_timeout) or args.case_timeout < 30:
        parser.error("Force must be positive and finite; timeout must be finite and >=30 s")
    directory = args.output_dir if args.output_dir.is_absolute() else ROOT / args.output_dir
    if directory.exists():
        parser.error(f"Refusing existing result directory: {directory}")
    directory.mkdir(parents=True)
    gaits = ("standing", "trot") if args.gait == "both" else (args.gait,)
    directions = (args.direction,) if args.direction else tuple(DIRECTIONS)
    cases = []
    for gait in gaits:
        for direction in directions:
            dx, dy = DIRECTIONS[direction]
            cases.append({"case": f"{gait}_{direction}", "gait": gait, "direction": direction,
                          "force_world_n": [args.force * dx, args.force * dy, 0.0], "status": "not_run"})
    report_path = directory / "results.json"
    save_report(report_path, cases, args.force, False)
    for index, case in enumerate(cases):
        stem = directory / case["case"]
        case["status"] = "running"
        case["csv"] = str(stem.with_suffix(".csv"))
        case["run_log"] = str(stem.with_suffix(".run.log"))
        save_report(report_path, cases, args.force, False)
        fx, fy, _ = case["force_world_n"]
        cmd = ["bash", str(ROOT / "scripts/run_cpp_simulation.sh"), "--mpc-solver", "sqp", "--headless",
               "--duration", "20", "--sensor-startup", "--joint-kp", "0", "--joint-kd", "1",
               "--centroidal-base-accel", "--wbc-config", "config/wbc/upstream_a1_tasks_go2.json",
               "--push-at", "8", "--push-duration", "0.2", "--push-fx", str(fx), "--push-fy", str(fy),
               "--csv", case["csv"]]
        if case["gait"] == "trot":
            cmd += ["--trot-at", "3", "--stop-trot-at", "15"]
        env = os.environ.copy()
        env.setdefault("ROS_DOMAIN_ID", "219")
        env["GO2_MPC_LOG"] = str(stem) + "_mpc.log"
        env["GO2_TARGET"] = str(stem) + "_target.json"
        print(f"[{index + 1}/{len(cases)}] {case['case']}: {case['force_world_n']} N", flush=True)
        started = time.monotonic()
        interrupted = False
        try:
            with stem.with_suffix(".run.log").open("w") as output:
                proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    case["exit_code"] = proc.wait(timeout=args.case_timeout)
                except subprocess.TimeoutExpired:
                    case["timed_out"] = True
                    terminate(proc)
                    case["exit_code"] = proc.returncode
                except KeyboardInterrupt:
                    interrupted = True
                    terminate(proc)
                    case["exit_code"] = proc.returncode
        except OSError as error:
            case["exit_code"] = -1
            case["launch_error"] = str(error)
        case["wall_elapsed_s"] = time.monotonic() - started
        try:
            errors = evaluate(case, stem)
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors = [f"Result validation error: {error}"]
        if interrupted or case.get("timed_out") or case.get("launch_error"):
            errors.append("Interrupted, timed out, or could not launch")
        case["errors"], case["passed"] = errors, not errors
        case["status"] = "passed" if not errors else "failed"
        save_report(report_path, cases, args.force, False)
        metrics = case.get("metrics", {})
        print(f"  {case['status'].upper()}: recovery={metrics.get('recovery_s')} s; "
              f"peak tilt={metrics.get('peak_roll_pitch_deg')} deg; errors={errors}", flush=True)
        if interrupted:
            print(f"Interrupted; partial matrix preserved: {report_path}", flush=True)
            return 130
    save_report(report_path, cases, args.force, True)
    passed = sum(c["passed"] for c in cases)
    print(f"Completed: {passed}/{len(cases)} passed. Report: {report_path}", flush=True)
    return 0 if passed == len(cases) else 1


if __name__ == "__main__":
    sys.exit(main())
