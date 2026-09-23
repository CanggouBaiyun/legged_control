"""Synchronized OCS2-WBC-MuJoCo standing experiment; simulation only."""

import json
from pathlib import Path
import select
import shlex
import subprocess
import time

import mujoco
import numpy as np
import mujoco.viewer


from go2_control.centroidal import (
    pinocchio_from_state,
    state_from_pinocchio,
)
from go2_control.model import build_go2_simulation_model
from go2_control.mpc_reference import standing_reference_from_mpc
from go2_control.simulation.mujoco_bridge import MujocoPinocchioBridge
from go2_control.wbc import StandingWBC
from go2_control.wbc.reference import joint_pd_feedback


def timed_call(samples, function, *args, **kwargs):
    """Record wall-clock duration of a successful function call."""
    start = time.perf_counter()
    result = function(*args, **kwargs)
    samples.append(time.perf_counter() - start)
    return result


def request_transport(process, message):
    """Send one request to the ROS subprocess and receive its response."""
    process.stdin.write(json.dumps(message, allow_nan=False) + "\n")
    process.stdin.flush()

    ready, _, _ = select.select([process.stdout], [], [], 40.0)
    if not ready:
        raise RuntimeError("ROS transport response timed out")

    line = process.stdout.readline()
    if not line:
        raise RuntimeError("ROS transport exited")

    reply = json.loads(line)
    if "error" in reply:
        raise RuntimeError(reply["error"])
    return reply

def request_policy(process, message):
    """Request a fresh MPC policy."""
    reply = request_transport(process, message)
    
    times = np.asarray(reply["times"], dtype=float)
    states = np.asarray(reply["states"], dtype=float)
    inputs = np.asarray(reply["inputs"], dtype=float)

    if (
        times.ndim != 1
        or len(times) < 2
        or states.shape != (len(times), 24)
        or inputs.shape != (len(times), 24)
        or not np.isfinite(times).all()
        or not np.isfinite(states).all()
        or not np.isfinite(inputs).all()
        or not np.all(np.diff(times) >= 0.0)
        or not np.any(np.diff(times) > 0.0)
    ):
        raise RuntimeError("Invalid standing policy arrays")

    return times, states, inputs


def evaluate_policy(process, stamp, current_state):
    """Obtain references from the C++ MRT service."""

    reply = request_transport(
        process,
        {
            "action": "evaluate",
            "time": float(stamp),
            "state": current_state.tolist(),
        },
    )

    state = np.asarray(reply["state"], dtype=float)
    control = np.asarray(reply["input"], dtype=float)

    if (
        reply["mode"] != 15
        or state.shape != (24,)
        or control.shape != (24,)
        or not np.isfinite(state).all()
        or not np.isfinite(control).all()
    ):
        raise RuntimeError("Invalid reference returned by MRT")

    return state, control

def standing_height_goal(time):
    """Smooth height goal: hold, lower, hold, rise, hold."""

    def smooth_progress(s):
        s = np.clip(s, 0.0, 1.0)
        return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5

    if time < 5.0:
        return 0.28

    if time < 8.0:
        progress = smooth_progress((time - 5.0) / 3.0)
        return 0.28 - 0.01 * progress

    if time < 12.0:
        return 0.27

    if time < 15.0:
        progress = smooth_progress((time - 12.0) / 3.0)
        return 0.27 + 0.01 * progress

    return 0.28

def main():
    root = Path(__file__).resolve().parents[2]
    scene = root / "third_party/unitree_mujoco/unitree_robots/go2/scene.xml"

    mj_model = mujoco.MjModel.from_xml_path(str(scene))
    mj_data = mujoco.MjData(mj_model)
    model = build_go2_simulation_model()
    bridge = MujocoPinocchioBridge(mj_model, model)
    wbc = StandingWBC(model)
    mapping_data = model.createData()

    key = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key < 0:
        raise RuntimeError("Missing home keyframe")

    mujoco.mj_resetDataKeyframe(mj_model, mj_data, key)
    mj_data.ctrl[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)

    q, v = bridge.state(mj_data)
    target = state_from_pinocchio(model, mapping_data, q, v)
    target[:6] = 0.0
    target[8] = 0.28

    # 每0.05s提供一个目标节点，覆盖整个20s实验
    target_times = np.linspace(0.0, 20.0, 401)

    target_states = np.repeat(
        target[np.newaxis, :],
        len(target_times),
        axis = 0,
    )

    target_states[:, 8] = np.array([
        standing_height_goal(stamp)
        for stamp in target_times
    ])

    # 子进程单独加载 ROS 环境，不污染当前 Conda 进程。
    transport = root / "examples/ocs2/standing_mpc_transport.py"
    command = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/bot/Project/ocs2_ws/install/setup.bash && "
    "export RCUTILS_LOGGING_USE_STDOUT=0 && "
    f"exec /usr/bin/python3 {shlex.quote(str(transport))}"
    )
    process = subprocess.Popen(
        ["/bin/bash", "-c", command],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    dt = mj_model.opt.timestep
    steps_per_policy = max(1, round(0.02 / dt))
    total_steps = round(20.0 / dt)
    last_control = np.zeros(24)
    next_print = 0.0
    clipped_steps = 0
    viewer = None
    predicted_next_z = None
    prediction_error_mm = float("nan")
    timings = {
        "plan": [],
        "mrt": [],
        "wbc": [],
        "loop": [],
    }

    try:
        viewer = mujoco.viewer.launch_passive(
            mj_model,
            mj_data,
        )

        # 镜头看向机器人。
        viewer.cam.lookat[:] = mj_data.qpos[:3]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.sync()

        for step in range(total_steps):
            if not viewer.is_running():
                break

            loop_start = time.perf_counter()

            # A. 实测状态：来自 MuJoCo，不来自 MPC 预测。
            q, v = bridge.state(mj_data)
            # 当前目标用于日志；完整时间表在首次请求时交给 MPC。
            target[8] = standing_height_goal(mj_data.time)

            if (
                not np.isfinite(q).all()
                or not np.isfinite(v).all()
                or not 0.12 < q[2] < 0.50
            ):
                raise RuntimeError("Invalid state or unsafe base height")

            if step % steps_per_policy == 0:
                if predicted_next_z is not None:
                    prediction_error_mm = (
                        q[2] - predicted_next_z
                    ) * 1000.0

                state = state_from_pinocchio(model, mapping_data, q, v)

                observation_input = last_control.copy()
                # observation 中使用实际关节速度。
                observation_input[12:] = v[6:]

                times, states, inputs = timed_call(
                    timings["plan"],
                    request_policy,
                    process,
                    {
                        "time": float(mj_data.time),
                        "until": float(
                            mj_data.time + steps_per_policy * dt
                        ),
                        "state": state.tolist(),
                        "input": observation_input.tolist(),
                        "target": target.tolist(),
                        "target_times": target_times.tolist(),
                        "target_states": target_states.tolist(),
                    },
                )

                next_policy_time = (
                    mj_data.time + steps_per_policy * dt
                )

                predicted_next_z = float(np.interp(
                    next_policy_time,
                    times,
                    states[:, 8],
                ))


            # B. 用当前仿真时间取得 MPC 参考。
            current_mpc_state = state_from_pinocchio(
                model,
                mapping_data,
                q,
                v,
            )

            state_ref, input_ref = timed_call(
                timings["mrt"],
                evaluate_policy,
                process,
                mj_data.time,
                current_mpc_state,
            )

            last_control = input_ref.copy()

            reference = standing_reference_from_mpc(
                model,
                mapping_data,
                q,
                state_ref,
                input_ref,
                wbc.contact_frame_names,
            )

            # 同一条策略上，用参考基座世界系速度的变化估计加速度
            acceleration_interval = 0.005
            future_time = mj_data.time + acceleration_interval

            if future_time > times[-1]:
                raise RuntimeError(
                    "Policy does not cover acceleration estimation"
                )

            future_state_ref, future_input_ref = timed_call(
                timings["mrt"],
                evaluate_policy,
                process,
                future_time,
                current_mpc_state,
            )

            future_reference = standing_reference_from_mpc(
                model,
                mapping_data,
                q,
                future_state_ref,
                future_input_ref,
                wbc.contact_frame_names,
            )

            current_velocity_world = reference[
                "desired_base_linear_velocity_world"
            ]

            future_velocity_world = future_reference[
                "desired_base_linear_velocity_world"
            ]

            acceleration_ff_world = (
                future_velocity_world - current_velocity_world
            ) / acceleration_interval

            if not np.isfinite(acceleration_ff_world).all():
                raise RuntimeError(
                    "Non-finite base acceleration feedforward"
                )

            reference["desired_base_linear_acceleration_world"] = acceleration_ff_world

            # C. WBC 必须用实测 q、v 求解。
            result = timed_call(
                timings["wbc"],
                wbc.solve,
                q=q,
                v=v,
                **reference,
            )

            if (
                result.solver_status.lower() != "solved"
                or not np.isfinite(result.motor_torques).all()
                or not np.isfinite(result.dynamics_residual_norm)
                or not np.isfinite(result.contact_acceleration_residual_norm)
                or result.dynamics_residual_norm > 1e-3
                or result.contact_acceleration_residual_norm > 1e-3
            ):
                raise RuntimeError("WBC solve/constraint check failed")

            # D. 关节 PD 的参考也来自当前 MPC 轨迹点。
            q_ref, v_ref = pinocchio_from_state(
                model, mapping_data, state_ref, input_ref[12:]
            )
            feedback = joint_pd_feedback(
                model, q, v, q_ref, v_ref, kp=20.0, kd=1.0
            )
            torque = result.motor_torques + feedback

            controls = bridge.pinocchio_torques_to_mujoco_controls(torque)
            if not np.isfinite(controls).all():
                raise RuntimeError("Non-finite actuator command")

            limited = np.clip(
                controls,
                mj_model.actuator_ctrlrange[:, 0],
                mj_model.actuator_ctrlrange[:, 1],
            )
            clipped_steps += int(np.any(np.abs(limited - controls) > 1e-9))
            mj_data.ctrl[:] = limited

            if mj_data.time >= next_print:
                lookahead_time = mj_data.time + 0.2
                if lookahead_time > times[-1]:
                    raise RuntimeError(
                        "Policy does not cover the 0.2 s diagnostic lookahead"
                    )

                future_z = float(np.interp(
                    lookahead_time,
                    times,
                    states[:, 8],
                ))

                # MuJoCo 浮动基线速度在世界坐标系表达。
                actual_vz = float(mj_data.qvel[2])

                # 适配器已将参考基座线速度转换到世界坐标系。
                reference_vz = float(
                    reference["desired_base_linear_velocity_world"][2]
                )

                # 世界系竖直接触合力 [N]：MPC 参考与 WBC 求解值。
                # 两者都不是 MuJoCo 实际测得的接触力。
                mpc_fz = float(
                    np.sum(input_ref[:12].reshape(4, 3)[:, 2])
                )
                wbc_fz = float(
                    np.sum(result.contact_forces[:, 2])
                )

                print(
                    f"t={mj_data.time:5.2f} "
                    f"goal={target[8]:.4f} "
                    f"z={q[2]:.4f} "
                    f"plan+0.2={future_z:.4f} "
                    f"plan_end={states[-1, 8]:.4f} "
                    f"horizon={times[-1] - mj_data.time:.2f} "
                    f"vz_ref={reference_vz:+.5f} "
                    f"vz={actual_vz:+.5f} "
                    f"pred_err_mm={prediction_error_mm:+.3f} "
                    f"mpc_fz={mpc_fz:.3f} "
                    f"wbc_fz={wbc_fz:.3f} "
                    f"az_ff={acceleration_ff_world[2]:+.3f}",
                    flush=True,
                )

                next_print += 1.0

            # E. 执行力矩，推进真实的仿真动力学。
            mujoco.mj_step(mj_model, mj_data)
            viewer.sync()
            timings["loop"].append(time.perf_counter() - loop_start)

        print("\nFinal base position:", mj_data.qpos[:3])
        print("Clipped steps:", clipped_steps)

        # Inclusive wall-clock measurements; loop already contains all sections.
        # MRT entries are individual round trips (two per control step).
        print("\nTiming summary [ms]:")
        print(
            f"{'section':<10} {'calls':>8} "
            f"{'mean':>10} {'p95':>10} {'max':>10}"
        )
        for name, samples in timings.items():
            if not samples:
                continue
            values_ms = np.asarray(samples) * 1000.0
            print(
                f"{name:<10} {len(samples):>8d} "
                f"{np.mean(values_ms):>10.3f} "
                f"{np.percentile(values_ms, 95):>10.3f} "
                f"{np.max(values_ms):>10.3f}"
            )

        if timings["plan"]:
            print(
                "\nFirst plan request, including initialization [ms]:",
                timings["plan"][0] * 1000.0,
            )

        loop_times = np.asarray(timings["loop"])
        if loop_times.size:
            print("\nSimulation timestep [ms]:", dt * 1000.0)
            print(
                "Loops exceeding simulation timestep:",
                int(np.count_nonzero(loop_times > dt)),
                "/",
                len(loop_times),
            )

    finally:
        # 出错时停止推进仿真，而不是继续执行旧策略。
        mj_data.ctrl[:] = 0.0
        if viewer is not None:
            viewer.close()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
