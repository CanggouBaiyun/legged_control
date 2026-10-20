"""Run native simulation lifecycle checks; source ROS/workspace and use system Python.

Each case owns its processes. Results use a fresh --prefix; existing files are
never overwritten. Fault cases must fail the controller and flush a fault report.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
SENSOR_CASES = ("sensor-delay", "sensor-dropout", "contact-on", "contact-off")
CASES = ("normal", "stop", "sensor-stale", "estimator-invalid", "mpc-exit", "wbc-invalid", "command-exit") + SENSOR_CASES


def run_case(case, prefix, node, observation, velocity, Twist):
    stem = prefix.with_name(prefix.name + "_" + case)
    outputs = [stem.with_suffix(s) for s in
               (".csv", ".json", ".commands.csv", ".session.json", ".failure.json", ".run.log")]
    outputs += [stem.with_name(stem.name + s) for s in ("_mpc.log", "_command.log", "_target.json")]
    if any(path.exists() for path in outputs):
        raise RuntimeError(f"Existing case results: {stem}")
    path = stem.with_suffix(".csv")
    args = ["bash", str(ROOT / "scripts/run_cpp_simulation.sh"), "--mpc-solver", "sqp",
            "--headless", "--duration", "20" if case in ("normal", "stop") + SENSOR_CASES else "8",
            "--sensor-startup", "--joint-kp", "0", "--joint-kd", "1", "--centroidal-base-accel",
            "--wbc-config", "config/wbc/upstream_a1_tasks_go2.json", "--csv", str(path)]
    if case in SENSOR_CASES:
        event = {"sensor-delay": "delay", "sensor-dropout": "dropout"}.get(case, case)
        args += ["--sensor-errors", "config/sensors/moderate_errors.json", "--sensor-event", event,
                 "--sensor-event-at", "9" if case == "contact-off" else "8",
                 "--sensor-event-duration", "0.1" if case in ("sensor-delay", "sensor-dropout") else "0.01"]
        if case in ("contact-on", "contact-off"):
            args += ["--sensor-contact-foot", "fl"]
        args += ["--trot-at", "3", "--stop-trot-at", "15"]
    elif case == "normal":
        args += ["--trot-at", "3", "--stop-trot-at", "15", "--forward"]
    elif case == "stop":
        args += ["--online", "--trot-at", "3", "--request-stop-at", "8"]
    elif case == "command-exit":
        args += ["--online"]
    elif case != "mpc-exit":
        args += ["--inject-fault", case, "--inject-at", "4"]
    env = os.environ.copy()
    env["GO2_MPC_LOG"] = str(stem) + "_mpc.log"
    env["GO2_TARGET"] = str(stem) + "_target.json"
    observation.update(time=-1.0, saw_start=False)
    print(f"Starting {case}: {stem}", flush=True)
    with stem.with_suffix(".run.log").open("w") as output:
        proc = subprocess.Popen(args, cwd=ROOT, env=env, stdout=output,
                                stderr=subprocess.STDOUT, start_new_session=True)
        killed = False
        deadline = time.monotonic() + 100
        try:
            while proc.poll() is None:
                import rclpy
                rclpy.spin_once(node, timeout_sec=0.01)
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Case timed out: {case}")
                stamp = observation["time"]
                if case == "stop":
                    msg = Twist()
                    if observation["saw_start"] and stamp >= 5:
                        msg.linear.x = 0.03
                    # Keep requesting motion even after stop, to verify the latch.
                    velocity.publish(msg)
                if case in ("mpc-exit", "command-exit") and observation["saw_start"] and stamp >= 4 and not killed:
                    children = subprocess.check_output(
                        ["ps", "--ppid", str(proc.pid), "-o", "pid=,args="], text=True)
                    binary = "/legged_robot_sqp_mpc" if case == "mpc-exit" else "/online_command"
                    pids = [int(line.split()[0]) for line in children.splitlines() if binary in line]
                    if len(pids) != 1:
                        raise RuntimeError(f"Could not identify owned {binary}: {children}")
                    os.kill(pids[0], signal.SIGTERM)
                    killed = True
            code = proc.wait()
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
    report = json.loads(stem.with_suffix(".session.json").read_text())
    if report["csv_flushed"] not in (True, "true"):
        raise RuntimeError(f"CSV not flushed: {case}")
    # Count persisted samples without importing a second NumPy installation into ROS.
    with path.open() as csv:
        samples = sum(1 for _ in csv) - 1
    if samples != int(report["samples"]) or abs(float(report["time_s"]) - samples * 0.002) > 1e-8:
        raise RuntimeError(f"Report/CSV mismatch: {case}")
    details = {}
    if case in ("normal", "stop", "contact-on", "contact-off"):
        expected = "controlled_stop" if case == "stop" else "duration_limit"
        if code != 0 or report["state"] != "stopped" or report["reason"] != expected:
            raise RuntimeError(f"Unexpected normal termination: {case}, code={code}, {report}")
        summary = json.loads(path.with_suffix(".json").read_text())
        if summary["clipped_steps"]:
            raise RuntimeError(f"Torque clipping: {case}")
        if case != "stop" and samples != 10000:
            raise RuntimeError("Normal run ended early")
        if case in ("contact-on", "contact-off"):
            import csv as csv_module
            active = changed = 0
            max_changed_trust = 0.0
            with path.open() as csv:
                for row in csv_module.DictReader(csv):
                    if int(float(row["sensor_event_active"])):
                        active += 1
                    if int(float(row["changed_contact_fl"])):
                        changed += 1
                        raw, delivered = int(float(row["raw_contact_fl"])), int(float(row["estimator_sim_contact_fl"]))
                        expected = (0, 1) if case == "contact-on" else (1, 0)
                        if (raw, delivered) != expected:
                            raise RuntimeError("Contact event changed unexpected physical contact")
                        max_changed_trust = max(max_changed_trust, float(row["estimator_foot_fl_contact_trust"]))
            if active != 5 or not 1 <= changed <= 5 or max_changed_trust > 1e-12:
                raise RuntimeError(f"Contact event not exercised or false support trusted: active={active}, changed={changed}, trust={max_changed_trust}")
            event = report["sensor_event"]
            if int(event["changed_contact_samples"]) != changed:
                raise RuntimeError("Contact event report disagrees with CSV")
            if not all(summary["mode_samples"].get(m, 0) > 0 for m in ("6", "9", "15")) or summary["last_non_stance_s"] >= 19.5:
                raise RuntimeError("Contact case did not complete trot and final stance")
            details = {"changed_contact_samples": changed, "active_samples": active,
                       "max_changed_contact_trust": max_changed_trust,
                       "max_horizontal_estimation_error_m": summary["sensor_errors"]["after_2p4s_max_horizontal_position_estimation_error_m"],
                       "maximum_applied_torque_nm": summary["maximum_applied_torque_nm"],
                       "control_over_2ms": summary["control_over_2ms"], "loop_over_2ms": summary["loop_over_2ms"]}
        if case == "stop":
            import csv as csv_module
            with stem.with_suffix(".commands.csv").open() as csv:
                commands = list(csv_module.DictReader(csv))
            stopped = [r for r in commands if r["stop_latched"] == "1"]
            fields = ("effective_vx_mps", "effective_vy_mps", "effective_yaw_rate_radps")
            if (not stopped or any(abs(float(r[f])) > 1e-12 for r in stopped for f in fields) or
                    int(stopped[-1]["command_sequence"]) <= int(stopped[0]["command_sequence"]) or
                    not any(float(r["requested_vx_mps"]) == 0.03 for r in stopped) or
                    summary["final_x_m"] < 0.03):
                raise RuntimeError("Stop did not reject subsequent motion commands, or motion was not exercised")
    else:
        expected = {"sensor-stale": "sensors", "sensor-delay": "sensors", "sensor-dropout": "sensors", "estimator-invalid": "estimator",
                    "mpc-exit": "policy", "wbc-invalid": "wbc", "command-exit": "worker"}[case]
        if code == 0 or report["state"] != "fault" or report["fault_source"] != expected:
            raise RuntimeError(f"Expected {expected} fault: code={code}, {report}")
        if case in ("mpc-exit", "command-exit") and not killed:
            raise RuntimeError("Worker exit was not injected")
        injection_time = 8 if case in ("sensor-delay", "sensor-dropout") else 4
        if not injection_time - 1e-8 <= float(report["time_s"]) < injection_time + 0.5:
            raise RuntimeError(f"Fault was detected outside its expected window: {report}")
        if case in ("sensor-delay", "sensor-dropout"):
            event = report["sensor_event"]
            if (event["active"] not in (True, "true") or abs(float(event["current_time_s"]) - 8) > 1e-8 or
                    abs(float(event["delivered_time_s"]) - 7.998) > 1e-8 or abs(float(event["age_s"]) - 0.002) > 1e-8 or samples != 4000):
                raise RuntimeError("Stale cached measurement/timing not confirmed")
            if case == "sensor-dropout" and int(event["dropped_samples"]) != 1:
                raise RuntimeError("Dropout was not exercised")
            details = {"event": event, "fault_source": report["fault_source"], "fault_reason": report["reason"]}
    print(f"PASS {case}: t={report['time_s']}, samples={samples}, reason={report['reason']}", flush=True)
    return {"case": case, "exit_code": code, "session_report": str(stem.with_suffix(".session.json")),
            "time_s": float(report["time_s"]), "samples": samples, "passed": True, "details": details}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--case", choices=CASES + ("sensor-robustness",), help="Omit for all; sensor-robustness runs four sensor event cases")
    args = parser.parse_args()
    prefix = args.prefix if args.prefix.is_absolute() else ROOT / args.prefix
    if prefix.suffix:
        parser.error("--prefix must be a filename prefix without an extension")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    report_path = prefix.with_suffix(".checks.json")
    if report_path.exists():
        parser.error(f"Existing check report: {report_path}")
    os.environ.setdefault("ROS_DOMAIN_ID", "218")
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/go2_session_check_ros_logs")
    import rclpy
    from geometry_msgs.msg import Twist
    from ocs2_msgs.msg import MpcObservation
    rclpy.init()
    node = rclpy.create_node("go2_session_checks")
    observation = {"time": -1.0, "saw_start": False}
    def observe(msg):
        if msg.time < 1:
            observation["saw_start"] = True
        observation["time"] = msg.time
    subscription = node.create_subscription(MpcObservation, "/legged_robot_mpc_observation", observe, 1)
    velocity = node.create_publisher(Twist, "/cmd_vel", 1)
    try:
        selected = SENSOR_CASES if args.case == "sensor-robustness" else ((args.case,) if args.case else CASES)
        results = []
        for c in selected:
            results.append(run_case(c, prefix, node, observation, velocity, Twist))
            report_path.write_text(json.dumps({"passed": len(results) == len(selected), "complete": len(results) == len(selected), "cases": results}, indent=2) + "\n")
        report_path.write_text(json.dumps({"passed": True, "cases": results}, indent=2) + "\n")
        print(f"All requested session checks passed: {report_path}", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
