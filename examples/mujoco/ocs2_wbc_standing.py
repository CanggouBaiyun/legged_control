"""Synchronized OCS2-WBC-MuJoCo standing experiment; simulation only."""

import json
from pathlib import Path
import select
import shlex
import subprocess

import mujoco
import numpy as np

from go2_control.centroidal import (
    pinocchio_from_state,
    state_from_pinocchio,
)
from go2_control.model import build_go2_simulation_model
from go2_control.mpc_reference import standing_reference_from_mpc
from go2_control.simulation.mujoco_bridge import MujocoPinocchioBridge
from go2_control.wbc import StandingWBC
from go2_control.wbc.reference import joint_pd_feedback

def request_policy(process, message):
    """Send measured state; wait while simulation time is paused."""
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
        or not np.all(np.diff(times) > 0.0)
    ):
        raise RuntimeError("Invalid standing policy arrays")

    return times, states, inputs


def evaluate_policy(times, states, inputs, stamp):
    """Interpolate a feedforward stance policy at simulation time."""
    if stamp < times[0] - 1e-6 or stamp > times[-1]:
        raise RuntimeError("Attempted to use an expired/out-of-range policy")

    state = np.array([
        np.interp(stamp, times, states[:, index])
        for index in range(24)
    ])
    control = np.array([
        np.interp(stamp, times, inputs[:, index])
        for index in range(24)
    ])
    return state, control

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
    total_steps = round(5.0 / dt)
    last_control = np.zeros(24)
    next_print = 0.0
    clipped_steps = 0

    try:
        for step in range(total_steps):
            # A. 实测状态：来自 MuJoCo，不来自 MPC 预测。
            q, v = bridge.state(mj_data)

            if (
                not np.isfinite(q).all()
                or not np.isfinite(v).all()
                or not 0.12 < q[2] < 0.50
            ):
                raise RuntimeError("Invalid state or unsafe base height")

            if step % steps_per_policy == 0:
                state = state_from_pinocchio(model, mapping_data, q, v)

                observation_input = last_control.copy()
                # observation 中使用实际关节速度。
                observation_input[12:] = v[6:]

                times, states, inputs = request_policy(
                    process,
                    {
                        "time": float(mj_data.time),
                        "until": float(
                            mj_data.time + steps_per_policy * dt
                        ),
                        "state": state.tolist(),
                        "input": observation_input.tolist(),
                        "target": target.tolist(),
                    },
                )

            # B. 用当前仿真时间取得 MPC 参考。
            state_ref, input_ref = evaluate_policy(
                times, states, inputs, mj_data.time
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

            # C. WBC 必须用实测 q、v 求解。
            result = wbc.solve(q=q, v=v, **reference)

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
                print(
                    f"t={mj_data.time:5.2f} "
                    f"z_ref={state_ref[8]:.4f} "
                    f"z={q[2]:.4f} "
                    f"max_tau={np.max(np.abs(limited)):.3f} "
                    f"QP={result.solver_status}",
                    flush=True,
                )
                next_print += 1.0

            # E. 执行力矩，推进真实的仿真动力学。
            mujoco.mj_step(mj_model, mj_data)

        print("\nFinal base position:", mj_data.qpos[:3])
        print("Clipped steps:", clipped_steps)

    finally:
        # 出错时停止推进仿真，而不是继续执行旧策略。
        mj_data.ctrl[:] = 0.0
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
