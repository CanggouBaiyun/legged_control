"""Run and evaluate a fixed online motion scenario in the native MuJoCo simulation.

Run with /usr/bin/python3 after sourcing ROS and the OCS2 workspace. The launcher
owns the simulated controller and MPC; this tool only publishes commands in its
selected ROS domain. Existing result files are never overwritten.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
DURATION = 80.0
STANCE_AT = 70.0
FINAL_STANCE_START = 73.0
PHASES = (
    ("forward", 6.0,12.0,(0.10, 0.0, 0.0)),
    ("backward", 15.0, 21.0, (-0.03, 0.0, 0.0)),
    ("left", 24.0, 30.0, (0.0, 0.02, 0.0)),
    ("right", 33.0, 39.0, (0.0, -0.02, 0.0)),
    ("turn_left", 42.0, 48.0, (0.0, 0.0, 0.08)),
    ("turn_right", 51.0, 57.0, (0.0, 0.0, -0.08)),
    ("combined", 60.0, 66.0, (0.02, 0.01, 0.05)),
)

def evaluate(path, require_estimator=False):
    """Evaluate settled intervals in body-heading coordinates, including yaw."""
    fields = ("time_s", "mode", "measured_x_m", "measured_y_m",
              "measured_yaw_rad", "measured_com_vx_world_mps",
              "measured_com_vy_world_mps")
    if require_estimator:
        fields += ("estimator_control_active",)
        fields += tuple(f"estimator_position_error_{axis}_m" for axis in "xyz")
        fields += tuple(f"estimator_velocity_error_{axis}_mps" for axis in "xyz")
    with path.open() as stream:
        reader = csv.DictReader(stream)
        missing = set(fields) - set(reader.fieldnames or ())
        if missing:
            raise RuntimeError(f"Control CSV is missing columns: {sorted(missing)}")
        rows = [{key: float(row[key]) for key in fields} for row in reader]
    if len(rows) != round(DURATION / 0.002):
        raise RuntimeError("Scenario control CSV is incomplete")
    yaw = None
    for row in rows:
        raw = row["measured_yaw_rad"]
        yaw = raw if yaw is None else yaw + math.remainder(raw - yaw, 2 * math.pi)
        row["unwrapped_yaw"] = yaw
        c, s = math.cos(raw), math.sin(raw)
        vx, vy = row["measured_com_vx_world_mps"], row["measured_com_vy_world_mps"]
        row["vx_body"] = c * vx + s * vy
        row["vy_body"] = -s * vx + c * vy
    phases = []
    for name, start, end, command in PHASES:
        # Allow 1 s for settling; exclude the final 0.2 s near command changes.
        selected = [row for row in rows if start + 1.0 <= row["time_s"] < end - 0.2]
        if len(selected) < 100:
            raise RuntimeError(f"Missing samples for {name}")
        mean_t = statistics.fmean(row["time_s"] for row in selected)
        mean_yaw = statistics.fmean(row["unwrapped_yaw"] for row in selected)
        yaw_rate = sum((row["time_s"] - mean_t) * (row["unwrapped_yaw"] - mean_yaw)
                       for row in selected) / sum((row["time_s"] - mean_t) ** 2 for row in selected)
        measured = (statistics.fmean(row["vx_body"] for row in selected),
                    statistics.fmean(row["vy_body"] for row in selected), yaw_rate)
        # Initial low-speed scenario gates, not hardware deployment limits.
        tolerances = (max(0.003, 0.20 * abs(command[0])),
                      max(0.003, 0.20 * abs(command[1])),
                      max(0.01, 0.20 * abs(command[2])))
        errors = tuple(actual - requested for actual, requested in zip(measured, command))
        passed = all(abs(error) <= tolerance for error, tolerance in zip(errors, tolerances))
        passed = passed and all(requested == 0 or actual * requested > 0
                                for actual, requested in zip(measured, command))
        phases.append({
            "name": name, "requested_interval_s": [start, end],
            "evaluation_interval_s": [start + 1.0, end - 0.2],
            "command_body_vx_vy_yaw_rate": list(command),
            "measured_mean_body_vx_vy_yaw_rate": list(measured),
            "errors": list(errors), "absolute_tolerances": list(tolerances),
            "samples": len(selected), "passed": passed,
        })
        print(f"{name}: requested={command}; measured="
              f"({measured[0]:.5f}, {measured[1]:.5f}, {measured[2]:.5f}); "
              f"{'PASS' if passed else 'FAIL'}", flush=True)
    stopped = [row for row in rows if row["time_s"] >= FINAL_STANCE_START]
    x_range = max(row["measured_x_m"] for row in stopped) - min(row["measured_x_m"] for row in stopped)
    y_range = max(row["measured_y_m"] for row in stopped) - min(row["measured_y_m"] for row in stopped)
    yaw_range = max(row["unwrapped_yaw"] for row in stopped) - min(row["unwrapped_yaw"] for row in stopped)
    stop_passed = (all(row["mode"] == 15 for row in stopped) and
                   x_range <= 0.005 and y_range <= 0.005 and yaw_range <= math.radians(1.0))
    summary = json.loads(path.with_suffix(".json").read_text())
    estimator_check = {"requested": require_estimator}
    estimator_passed = True

    if require_estimator:
        active_times = [
            row["time_s"] for row in rows
            if row["estimator_control_active"] == 1
        ]

        first_active = active_times[0] if active_times else None

        estimator_passed = (
            first_active is not None
            and 2.0 <= first_active < 3.0
            and all(
                row["estimator_control_active"] in (0, 1)
                for row in rows
            )
            and all(
                row["estimator_control_active"] == 1
                for row in rows if row["time_s"] >= first_active
            )
        )

        estimator_check.update(
            first_active_s=first_active,
            active_samples=len(active_times),
            passed=estimator_passed,
        )

        print(
            f"Estimator control: first_active={first_active}; "
            f"{'PASS' if estimator_passed else 'FAIL'}",
            flush=True,
        )
        if first_active is not None:
            active = [row for row in rows if row["time_s"] >= first_active]
            max_horizontal_error = max(math.hypot(
                row["estimator_position_error_x_m"],
                row["estimator_position_error_y_m"]) for row in active)
            vertical_velocity_rmse = math.sqrt(statistics.fmean(
                row["estimator_velocity_error_z_mps"] ** 2 for row in active))
            mean_velocity_errors = {}
            for name, start, end, _ in PHASES:
                selected = [row for row in active if start + 1.0 <= row["time_s"] < end - 0.2]
                mean_velocity_errors[name] = [statistics.fmean(
                    row[f"estimator_velocity_error_{axis}_mps"] for row in selected)
                    for axis in "xyz"]
            max_mean_horizontal_velocity_error = max(
                math.hypot(error[0], error[1]) for error in mean_velocity_errors.values())
            # Flat-ground, ideal-sensor simulation acceptance limits. Evaluate
            # drift and sustained bias separately from commanded motion.
            accuracy_passed = (max_horizontal_error <= 0.02 and
                               max_mean_horizontal_velocity_error <= 0.003 and
                               vertical_velocity_rmse <= 0.02)
            estimator_check["accuracy"] = {
                "max_horizontal_position_error_m": max_horizontal_error,
                "phase_mean_velocity_error_xyz_mps": mean_velocity_errors,
                "max_phase_mean_horizontal_velocity_error_mps": max_mean_horizontal_velocity_error,
                "vertical_velocity_rmse_mps": vertical_velocity_rmse,
                "limits": {"horizontal_position_m": 0.02,
                           "phase_mean_horizontal_velocity_mps": 0.003,
                           "vertical_velocity_rmse_mps": 0.02},
                "passed": accuracy_passed,
            }
            estimator_passed = estimator_passed and accuracy_passed
            estimator_check["passed"] = estimator_passed
            print(f"Estimator accuracy: max horizontal error={max_horizontal_error * 1000:.3f} mm; "
                  f"max phase velocity bias={max_mean_horizontal_velocity_error:.6f} m/s; "
                  f"{'PASS' if accuracy_passed else 'FAIL'}", flush=True)
    result = {
        "schema": 1, "control_csv": str(path), "duration_s": DURATION,
        "phases": phases,
        "final_stance": {"evaluation_start_s": FINAL_STANCE_START, "x_range_m": x_range,
                         "y_range_m": y_range, "yaw_range_rad": yaw_range,
                         "passed": stop_passed},
        "passed": (
            all(phase["passed"] for phase in phases)
            and stop_passed
            and estimator_passed
            and summary["clipped_steps"] == 0
        ),
        "note": "Initial flat-ground simulation gates; estimator accuracy is checked when requested. Timing, sensor robustness and historical solver failures remain separate deployment gates.",
        "state_estimator": estimator_check,
    }
    output = path.with_suffix(".scenario.json")
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Scenario result: {output}", flush=True)
    return result["passed"]


def run(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from ocs2_msgs.msg import ModeSchedule, MpcObservation

    path = args.csv if args.csv.is_absolute() else ROOT / args.csv
    path = path.resolve()
    if path.suffix != ".csv":
        raise RuntimeError("--csv must end in .csv")
    paths = [path, path.with_suffix(".json"), path.with_suffix(".commands.csv"),
             path.with_suffix(".scenario.json"), path.with_suffix(".failure.json"),
             path.with_name(path.stem + "_mpc.log"), path.with_name(path.stem + "_command.log"),
             path.with_name(path.stem + "_target.json")]
    if any(item.exists() for item in paths):
        raise RuntimeError("Results already exist; choose a new --csv path")
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOG_DIR"] = f"/tmp/go2_online_scenario_ros_logs_{args.ros_domain_id}"
    environment = os.environ.copy()
    environment["GO2_MPC_LOG"] = str(path.with_name(path.stem + "_mpc.log"))
    environment["GO2_TARGET"] = str(path.with_name(path.stem + "_target.json"))
    # Keep this run's logs together even if the calling shell has overrides.
    environment["GO2_COMMAND_CSV"] = str(path.with_suffix(".commands.csv"))
    environment["GO2_COMMAND_LOG"] = str(path.with_name(path.stem + "_command.log"))
    rclpy.init()
    node = None
    process = None
    try:
        node = rclpy.create_node("go2_online_motion_scenario")
        velocity = node.create_publisher(Twist, "/cmd_vel", 1)
        gait = node.create_publisher(ModeSchedule, "/legged_robot_mpc_mode_schedule", 1)
        latest = {"time": -1.0, "received": 0.0}
        def observation(message):
            latest.update(time=message.time, received=time.monotonic())
        subscription = node.create_subscription(MpcObservation, "/legged_robot_mpc_observation", observation, 1)
        arguments = ["bash", "scripts/run_cpp_simulation.sh", "--online",
                     "--online-reference", "integrated", "--mpc-solver", "sqp",
                     "--duration", str(DURATION), "--joint-kp", "0", "--joint-kd", "1",
                     "--centroidal-base-accel", "--wbc-config", "config/wbc/upstream_a1_tasks_go2.json",
                     "--csv", str(path)]
        if not args.viewer:
            arguments.append("--headless")
        if args.estimator_control:
            arguments.append("--estimator-control")
        process = subprocess.Popen(arguments, cwd=ROOT, env=environment, start_new_session=True)
        deadline = time.monotonic() + 180.0
        next_command = 0.0
        trot_sent = stance_sent = False
        last_phase = None
        while process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError("Scenario startup/run exceeded 180 s")
            stamp = latest["time"]
            if stamp < 0 or now - latest["received"] > 0.2:
                continue
            if stamp >= 3.0 and not trot_sent and gait.get_subscription_count():
                message = ModeSchedule()
                message.event_times = [0.0, 0.30, 0.35, 0.65, 0.70]
                message.mode_sequence = [9, 15, 6, 15]
                gait.publish(message)
                trot_sent = True
                print(f"SCENARIO: trot at observation t={stamp:.3f}", flush=True)
            if stamp >= STANCE_AT and not stance_sent and gait.get_subscription_count():
                message = ModeSchedule()
                message.event_times = [0.0, 0.5]
                message.mode_sequence = [15]
                gait.publish(message)
                stance_sent = True
                print(f"SCENARIO: stance at observation t={stamp:.3f}", flush=True)
            if now >= next_command:
                name, command = "stop", (0.0, 0.0, 0.0)
                for candidate, start, end, values in PHASES:
                    if start <= stamp < end:
                        name, command = candidate, values
                        break
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = command
                velocity.publish(message)
                next_command = now + 0.1
                if name != last_phase:
                    print(f"SCENARIO: {name}, command={command}, observation t={stamp:.3f}", flush=True)
                    last_phase = name
        if process.returncode:
            raise RuntimeError(f"Simulation launcher exited with code {process.returncode}; inspect partial CSV and failure/log files")
        if not trot_sent or not stance_sent:
            raise RuntimeError("Scenario did not send both gait transitions")
        return evaluate(path, require_estimator=args.estimator_control)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=Path("artifacts/cpp_online_directions.csv"))
    parser.add_argument("--viewer", action="store_true", help="Open the official MuJoCo viewer")
    parser.add_argument("--ros-domain-id", type=int, default=217)
    parser.add_argument(
        "--estimator-control", action="store_true",
        help="Use estimated-state control and verify activation stays latched",
    )
    args = parser.parse_args()
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be in [0,232]")
    try:
        return 0 if run(args) else 1
    except (RuntimeError, OSError) as error:
        print(f"online scenario: {error}", flush=True)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
