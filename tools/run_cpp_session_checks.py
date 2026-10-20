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
CASES = ("normal", "stop", "sensor-stale", "estimator-invalid", "mpc-exit", "wbc-invalid", "command-exit")


def run_case(case, prefix, node, observation, velocity, Twist):
    stem = prefix.with_name(prefix.name + "_" + case)
    outputs = [stem.with_suffix(s) for s in
               (".csv", ".json", ".commands.csv", ".session.json", ".failure.json", ".run.log")]
    outputs += [stem.with_name(stem.name + s) for s in ("_mpc.log", "_command.log", "_target.json")]
    if any(path.exists() for path in outputs):
        raise RuntimeError(f"Existing case results: {stem}")
    path = stem.with_suffix(".csv")
    args = ["bash", str(ROOT / "scripts/run_cpp_simulation.sh"), "--mpc-solver", "sqp",
            "--headless", "--duration", "20" if case in ("normal", "stop") else "8",
            "--sensor-startup", "--joint-kp", "0", "--joint-kd", "1", "--centroidal-base-accel",
            "--wbc-config", "config/wbc/upstream_a1_tasks_go2.json", "--csv", str(path)]
    if case == "normal":
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
    if case in ("normal", "stop"):
        expected = "duration_limit" if case == "normal" else "controlled_stop"
        if code != 0 or report["state"] != "stopped" or report["reason"] != expected:
            raise RuntimeError(f"Unexpected normal termination: {case}, code={code}, {report}")
        summary = json.loads(path.with_suffix(".json").read_text())
        if summary["clipped_steps"]:
            raise RuntimeError(f"Torque clipping: {case}")
        if case == "normal" and samples != 10000:
            raise RuntimeError("Normal run ended early")
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
        expected = {"sensor-stale": "sensors", "estimator-invalid": "estimator",
                    "mpc-exit": "policy", "wbc-invalid": "wbc", "command-exit": "worker"}[case]
        if code == 0 or report["state"] != "fault" or report["fault_source"] != expected:
            raise RuntimeError(f"Expected {expected} fault: code={code}, {report}")
        if case in ("mpc-exit", "command-exit") and not killed:
            raise RuntimeError("Worker exit was not injected")
        if not 4 - 1e-8 <= float(report["time_s"]) < 4.5:
            raise RuntimeError(f"Fault was detected outside its expected window: {report}")
    print(f"PASS {case}: t={report['time_s']}, samples={samples}, reason={report['reason']}", flush=True)
    return {"case": case, "exit_code": code, "session_report": str(stem.with_suffix(".session.json")),
            "time_s": float(report["time_s"]), "samples": samples, "passed": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--case", choices=CASES, help="Omit to run all cases")
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
        results = [run_case(c, prefix, node, observation, velocity, Twist)
                   for c in ((args.case,) if args.case else CASES)]
        report_path.write_text(json.dumps({"passed": True, "cases": results}, indent=2) + "\n")
        print(f"All requested session checks passed: {report_path}", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
