"""Synchronized OCS2-WBC-MuJoCo standing experiment; simulation only."""

import argparse
import json
from pathlib import Path
import select
import shlex
import subprocess
import time

import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin

CSV_COLUMNS = (
    "time_s",
    "mode",
    "goal_x_m",
    "mpc_x_m",
    "measured_x_m",
    "goal_yaw_rad",
    "mpc_yaw_rad",
    "measured_yaw_rad",
    "goal_z_m",
    "mpc_z_m",
    "measured_z_m",
    "measured_minus_goal_m",
    "measured_minus_mpc_m",
    "mpc_vz_mps",
    "measured_vz_mps",
    "mpc_fz_fl_n",
    "mpc_fz_fr_n",
    "mpc_fz_rl_n",
    "mpc_fz_rr_n",
    "wbc_fz_fl_n",
    "wbc_fz_fr_n",
    "wbc_fz_rl_n",
    "wbc_fz_rr_n",
    "max_applied_torque_nm",
    "max_feedback_torque_nm",
    "max_torque_clip_nm",
    "dynamics_residual",
    "contact_acceleration_residual",
    "policy_horizon_s",
    "prediction_error_mm",
    "foot_z_fl_m",
    "foot_z_fr_m",
    "foot_z_rl_m",
    "foot_z_rr_m",
    "swing_target_z_fl_m",
    "swing_target_z_fr_m",
    "swing_target_z_rl_m",
    "swing_target_z_rr_m",
    "foot_x_fl_m",
    "foot_x_fr_m",
    "foot_x_rl_m",
    "foot_x_rr_m",
    "swing_target_x_fl_m",
    "swing_target_x_fr_m",
    "swing_target_x_rl_m",
    "swing_target_x_rr_m",
    "mj_floor_fz_fl_n",
    "mj_floor_fz_fr_n",
    "mj_floor_fz_rl_n",
    "mj_floor_fz_rr_n",
    "stance_center_xy_speed_fl_mps",
    "stance_center_xy_speed_fr_mps",
    "stance_center_xy_speed_rl_mps",
    "stance_center_xy_speed_rr_mps",
)

from go2_control.centroidal import (
    pinocchio_from_state,
    state_from_pinocchio,
)
from go2_control.model import (
    build_go2_simulation_model,
    foot_positions,
    GO2_FOOT_FRAMES,
)
from go2_control.wbc.reference import (
    FixedFootReference,
    joint_pd_feedback,
)
from go2_control.mpc_reference import (
    OCS2_CONTACT_ORDER,
    OCS2_MODE_CONTACTS,
    mode_at_time,
    standing_reference_from_mpc,
)
from go2_control.simulation.mujoco_bridge import MujocoPinocchioBridge
from go2_control.wbc import StandingWBC
from go2_control.wbc.trot import TrotWBC


def timed_call(samples, function, *args, **kwargs):
    """Record wall-clock duration of a successful function call."""
    start = time.perf_counter()
    result = function(*args, **kwargs)
    samples.append(time.perf_counter() - start)
    return result


def floor_normal_forces(model, data, floor_id, foot_name_by_geom):
    """Read MuJoCo's solved normal forces for named foot-floor contacts."""
    forces = {name: 0.0 for name in foot_name_by_geom.values()}
    wrench = np.zeros(6)
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        foot_geom = geom2 if geom1 == floor_id else geom1 if geom2 == floor_id else -1
        foot_name = foot_name_by_geom.get(foot_geom)
        if foot_name is None or contact.efc_address < 0:
            continue
        mujoco.mj_contactForce(model, data, contact_id, wrench)
        forces[foot_name] += float(wrench[0])
    return forces


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

    mode_event_times = np.asarray(
        reply["mode_event_times"], dtype=float
    )
    mode_sequence = np.asarray(
        reply["mode_sequence"], dtype=int
    )

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
        or mode_event_times.ndim != 1
        or mode_sequence.ndim != 1
        or len(mode_sequence) != len(mode_event_times) + 1
        or not np.isfinite(mode_event_times).all()
        or not np.all(np.diff(mode_event_times) >= 0.0)
        or any(
            mode not in OCS2_MODE_CONTACTS
            for mode in mode_sequence
        )
    ):
        raise RuntimeError("Invalid standing policy arrays")

    return times, states, inputs, mode_event_times, mode_sequence


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
    mode = reply["mode"]

    if (
        mode not in OCS2_MODE_CONTACTS
        or state.shape != (24,)
        or control.shape != (24,)
        or not np.isfinite(state).all()
        or not np.isfinite(control).all()
    ):
        raise RuntimeError("Invalid reference returned by MRT")

    return state, control, mode

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

def forward_motion_goal(time):
    """Return desired world-X displacement and velocity for a short walk."""
    speed = 0.03 # m/s

    if time <= 8.0:
        return 0.0, 0.0

    if time < 9.0:
        u = time - 8.0
        velocity = speed * (3.0 * u**2 - 2.0 * u**3)
        displacement = speed * (u**3 - 0.5 * u**4)
        return displacement, velocity

    if time < 11.0:
        return 0.5 * speed + speed * (time - 9.0), speed

    if time < 12.0:
        u = time - 11.0
        velocity = speed * (1.0 - 3.0 * u**2 + 2.0 * u**3)
        displacement = speed * (2.5 + u - u**3 + 0.5 * u**4)
        return displacement, velocity

    return 3.0 * speed, 0.0

def turn_yaw_goal(time):
    """Return desired yaw and yaw rate for a 10-degree left turn."""
    rate = np.deg2rad(10.0 / 3.0)

    if time <= 8.0:
        return 0.0, 0.0

    if time < 9.0:
        u = time - 8.0
        yaw_rate = rate * (3.0 * u**2 - 2.0 * u**3)
        yaw = rate * (u**3 - 0.5 * u**4)
        return yaw, yaw_rate

    if time < 11.0:
        return 0.5 * rate + rate * (time - 9.0), rate

    if time < 12.0:
        u = time - 11.0
        yaw_rate = rate * (1.0 - 3.0 * u**2 + 2.0 * u**3)
        yaw = rate * (2.5 + u - u**3 + 0.5 * u**4)
        return yaw, yaw_rate

    return 3.0 * rate, 0.0

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without opening the MuJoCo viewer.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="Simulation duration in seconds, at most 20.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Optional path for per-step CSV output.",
    )
    parser.add_argument(
        "--allow-trot",
        action="store_true",
        help="Enable mode-dependent stance contacts and swing tasks.",
    )
    parser.add_argument(
        "--trot-at",
        type=float,
        help="Send standing_trot when simulation time reaches this value.",
    )
    parser.add_argument(
        "--stop-trot-at",
        type=float,
        help="Send the four-foot stance gait at this simulation time.",
    )
    parser.add_argument(
        "--forward",
        action="store_true",
        help="Use the short forward-motion target during trot.",
    )
    parser.add_argument(
        "--turn",
        action="store_true",
        help="Use the short in-place yaw-turn target during trot.",
    )
    args = parser.parse_args()

    if not 0.0 < args.duration <= 20.0:
        parser.error("--duration must be in (0, 20] seconds")
    if args.trot_at is not None and (
        not args.allow_trot or not 0.0 < args.trot_at < args.duration
    ):
        parser.error("--trot-at requires --allow-trot and a time within the run")
    if args.turn and (not args.allow_trot or args.trot_at is None):
        parser.error("--turn requires --allow-trot and --trot-at")
    if args.forward and (not args.allow_trot or args.trot_at is None):
        parser.error("--forward requires --allow-trot and --trot-at")
    if args.stop_trot_at is not None and (
        args.trot_at is None
        or not args.trot_at < args.stop_trot_at < args.duration
    ):
        parser.error("--stop-trot-at must be after --trot-at and before the run ends")

    return args


def main():
    args = parse_args()

    root = Path(__file__).resolve().parents[2]
    scene = root / "third_party/unitree_mujoco/unitree_robots/go2/scene.xml"

    mj_model = mujoco.MjModel.from_xml_path(str(scene))
    mj_data = mujoco.MjData(mj_model)
    model = build_go2_simulation_model()
    bridge = MujocoPinocchioBridge(mj_model, model)
    wbc = StandingWBC(model)
    trot_wbc = TrotWBC(
        model, 
        forward_goal=forward_motion_goal if args.forward else None,
        yaw_goal=turn_yaw_goal if args.turn else None,                
    ) if args.allow_trot else None
    floor_id = mujoco.mj_name2id(
        mj_model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )
    foot_name_by_geom = {
        mujoco.mj_name2id(
            mj_model, mujoco.mjtObj.mjOBJ_GEOM, name.split("_")[0]
        ): name
        for name in OCS2_CONTACT_ORDER
    }
    if floor_id < 0 or -1 in foot_name_by_geom:
        raise RuntimeError("Missing MuJoCo floor or named foot geom")
    mapping_data = model.createData()

    key = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key < 0:
        raise RuntimeError("Missing home keyframe")

    mujoco.mj_resetDataKeyframe(mj_model, mj_data, key)
    mj_data.ctrl[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)

    q, v = bridge.state(mj_data)
    # 以初始足端水平位置和地面接触高度构造固定足端目标
    feet_by_name = foot_positions(model, q)
    initial_feet = np.asarray(
        [feet_by_name[name] for name in GO2_FOOT_FRAMES]
    )
    # 当前场景的地面 z=0，足端碰撞球半径 0.022 m。
    initial_feet[:, 2] = 0.022
    initial_base_x = float(q[0])
    fixed_foot_reference = FixedFootReference(
        model, q, initial_feet
    )
    base_rotation = pin.XYZQUATToSE3(q[:7]).rotation

    target_times = np.linspace(0.0, 20.0, 401)
    target_states = []

    for stamp in target_times:
        displacement, forward_velocity = (
            forward_motion_goal(stamp) if args.forward else (0.0, 0.0)
        )
        yaw_delta, yaw_rate = (
            turn_yaw_goal(stamp) if args.turn else (0.0, 0.0)
        )

        base_position = q[:3].copy()
        base_position[0] = initial_base_x + displacement
        base_position[2] = standing_height_goal(stamp)

        yaw_rotation = pin.rpy.rpyToMatrix(
            np.array([0.0, 0.0, yaw_delta])
        )
        target_rotation = yaw_rotation @ base_rotation

        # 让名义足端随机身朝向旋转，保持合理的关节姿态参考。
        # 实际支撑脚仍由 WBC 的接触约束固定在世界坐标中。
        nominal_feet = initial_feet.copy()
        relative_feet_xy = initial_feet[:, :2] - q[:2]
        nominal_feet[:, :2] = (
            base_position[:2]
            + (yaw_rotation[:2, :2] @ relative_feet_xy.T).T
        )

        angular_velocity_world = np.array([0.0, 0.0, yaw_rate])
        nominal_foot_velocities = np.tile(
            np.array([forward_velocity, 0.0, 0.0]), (4, 1)
        )
        nominal_foot_velocities += np.cross(
            angular_velocity_world,
            nominal_feet - base_position,
        )

        q_ref, v_ref = fixed_foot_reference.solve(
            base_position,
            target_rotation,
            np.array([forward_velocity, 0.0, 0.0]),
            angular_velocity_local=target_rotation.T @ angular_velocity_world,
            foot_positions_world=nominal_feet,
            foot_velocities_world=nominal_foot_velocities,
        )
        target_states.append(
            state_from_pinocchio(model, mapping_data, q_ref, v_ref)
        )

    target_states = np.asarray(target_states)
    target = target_states[0].copy()

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
    total_steps = round(args.duration / dt)
    last_control = np.zeros(24)
    next_print = 0.0
    clipped_steps = 0
    gait_command_sent = False
    stance_command_sent = False
    previous_feet_by_name = None
    previous_stance = set()
    previous_floor_forces = None
    viewer = None
    predicted_next_z = None
    prediction_error_mm = float("nan")
    rows = []
    timings = {
        "plan": [],
        "mrt": [],
        "wbc": [],
        "loop": [],
    }

    try:
        if not args.headless:
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
            if viewer is not None and not viewer.is_running():
                break

            loop_start = time.perf_counter()

            # A. 实测状态：来自 MuJoCo，不来自 MPC 预测。
            q, v = bridge.state(mj_data)
            # 当前目标用于日志；完整时间表在首次请求时交给 MPC。
            displacement, _ = (
                forward_motion_goal(mj_data.time) if args.forward else (0.0, 0.0)
            )
            target[6] = initial_base_x + displacement
            target[8] = standing_height_goal(mj_data.time)
            target[9] = np.interp(
                mj_data.time, target_times, target_states[:, 9]
            )
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

                send_gait = (
                    args.trot_at is not None
                    and not gait_command_sent
                    and mj_data.time >= args.trot_at
                )

                send_stance = (
                    args.stop_trot_at is not None
                    and gait_command_sent
                    and not stance_command_sent
                    and mj_data.time >= args.stop_trot_at
                )
                gait_command = None
                if send_gait:
                    gait_command = "standing_trot"
                elif send_stance:
                    gait_command = "stance"

                (
                    times,
                    states,
                    inputs,
                    mode_event_times,
                    mode_sequence,
                ) = timed_call(
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
                        **({"gait": gait_command} if gait_command is not None else {}),
                    },
                )
                if send_gait:
                    gait_command_sent = True

                if send_stance:
                    stance_command_sent = True

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

            state_ref, input_ref, mode = timed_call(
                timings["mrt"],
                evaluate_policy,
                process,
                mj_data.time,
                current_mpc_state,
            )

            scheduled_mode = mode_at_time(
                mode_event_times, mode_sequence, mj_data.time
            )
            if mode != scheduled_mode:
                raise RuntimeError(
                    f"MRT mode {mode} differs from policy schedule "
                    f"mode {scheduled_mode} at {mj_data.time:.6f}s"
                )

            if mode != 15 and trot_wbc is None:
                raise RuntimeError(
                    f"Standing simulation requires mode 15, got {mode}"
                )

            swing_targets = {}
            if trot_wbc is not None:
                feet_by_name = {
                    name: np.asarray(position, dtype=float)
                    for name, position in foot_positions(model, q).items()
                }
                wbc, swing_targets = trot_wbc.prepare(
                    mj_data.time,
                    mode,
                    mode_event_times,
                    mode_sequence,
                    feet_by_name,
                    base_position=q[:3],
                )

            last_control = input_ref.copy()

            reference = standing_reference_from_mpc(
                model,
                mapping_data,
                q,
                state_ref,
                input_ref,
                wbc.contact_frame_names,
                mode=mode,
            )

            # 同一条策略上，用参考基座世界系速度的变化估计加速度
            acceleration_interval = 0.005
            future_time = mj_data.time + acceleration_interval

            if future_time > times[-1]:
                raise RuntimeError(
                    "Policy does not cover acceleration estimation"
                )

            future_state_ref, future_input_ref, future_mode = timed_call(
                timings["mrt"],
                evaluate_policy,
                process,
                future_time,
                current_mpc_state,
            )

            scheduled_future_mode = mode_at_time(
                mode_event_times, mode_sequence, future_time
            )
            if future_mode != scheduled_future_mode:
                raise RuntimeError(
                    f"MRT future mode {future_mode} differs from "
                    f"policy schedule mode {scheduled_future_mode} "
                    f"at {future_time:.6f}s"
                )

            if future_mode != 15 and trot_wbc is None:
                raise RuntimeError(
                    f"Standing simulation requires mode 15, got {future_mode}"
                )

            future_wbc = (
                trot_wbc.controllers[future_mode]
                if trot_wbc is not None else wbc
            )

            future_reference = standing_reference_from_mpc(
                model,
                mapping_data,
                q,
                future_state_ref,
                future_input_ref,
                future_wbc.contact_frame_names,
                mode=future_mode,
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
                swing_targets=swing_targets,
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

            # OCS2 force order: FL, FR, RL, RR.
            mpc_forces = input_ref[:12].reshape(4,3)
            # WBC uses a different internal contact order, so convert it to the
            # OCS2 order before recording and comparing the forces.
            wbc_force_by_name = dict(
                zip(wbc.contact_frame_names, result.contact_forces)
            )
            wbc_forces_ocs2_order = np.array(
                [
                    wbc_force_by_name.get(name, np.zeros(3))
                    for name in OCS2_CONTACT_ORDER
                ]
            )

            reference_vz = float(
                reference["desired_base_linear_velocity_world"][2]
            )
            actual_vz = float(mj_data.qvel[2])
            measured_yaw = pin.rpy.matrixToRpy(
                pin.XYZQUATToSE3(q[:7]).rotation
            )[2]

            row = [
                    float(mj_data.time),
                    float(mode),
                    float(target[6]),
                    float(state_ref[6]),
                    float(q[0]),
                    float(target[9]),
                    float(state_ref[9]),
                    float(measured_yaw),
                    float(target[8]),
                    float(state_ref[8]),
                    float(q[2]),
                    float(q[2] - target[8]),
                    float(q[2] - state_ref[8]),
                    reference_vz,
                    actual_vz,
                    *mpc_forces[:, 2].tolist(),
                    *wbc_forces_ocs2_order[:, 2].tolist(),
                    float(np.max(np.abs(limited))),
                    float(np.max(np.abs(feedback))),
                    float(np.max(np.abs(limited - controls))),
                    float(result.dynamics_residual_norm),
                    float(result.contact_acceleration_residual_norm),
                    float(times[-1] - mj_data.time),
                    float(prediction_error_mm),
                    *(
                        [float(feet_by_name[name][2]) for name in OCS2_CONTACT_ORDER]
                        if trot_wbc is not None else [float("nan")] * 4
                    ),
                    *(
                        float(swing_targets[name][0][2])
                        if name in swing_targets else float("nan")
                        for name in OCS2_CONTACT_ORDER
                    ),
                    *(
                        [float(feet_by_name[name][0]) for name in OCS2_CONTACT_ORDER]
                        if trot_wbc is not None else [float("nan")] * 4
                    ),
                    *(
                        float(swing_targets[name][0][0])
                        if name in swing_targets else float("nan")
                        for name in OCS2_CONTACT_ORDER
                    ),
            ]
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

                mpc_fz = float(np.sum(mpc_forces[:, 2]))
                wbc_fz = float(np.sum(wbc_forces_ocs2_order[:, 2]))

                print(
                    f"t={mj_data.time:5.2f} "
                    f"mode={mode} "
                    f"x_goal={target[6]:.4f} "
                    f"x_mpc={state_ref[6]:.4f} "
                    f"x={q[0]:.4f} "
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
            if trot_wbc is not None:
                # mj_step 的接触求解对应本步积分前的 q、v 和 mode。
                floor_forces = floor_normal_forces(
                    mj_model, mj_data, floor_id, foot_name_by_geom
                )
                stance_speeds = {}
                for name in OCS2_CONTACT_ORDER:
                    speed = float("nan")
                    if (
                        name in wbc.contact_frame_names
                        and name in previous_stance
                        and previous_feet_by_name is not None
                        and previous_floor_forces[name] > 1.0
                        and floor_forces[name] > 1.0
                    ):
                        speed = float(np.linalg.norm(
                            feet_by_name[name][:2]
                            - previous_feet_by_name[name][:2]
                        ) / dt)
                    stance_speeds[name] = speed
                row.extend(floor_forces[name] for name in OCS2_CONTACT_ORDER)
                row.extend(stance_speeds[name] for name in OCS2_CONTACT_ORDER)
                previous_feet_by_name = {
                    name: position.copy()
                    for name, position in feet_by_name.items()
                }
                previous_stance = set(wbc.contact_frame_names)
                previous_floor_forces = floor_forces
            else:
                row.extend([float("nan")] * 8)
            rows.append(row)
            if viewer is not None:
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
        if not rows:
            raise RuntimeError("No simulation samples were recorded")

        records = np.asarray(rows, dtype=float)

        if (
            records.ndim != 2
            or records.shape[1] != len(CSV_COLUMNS)
        ):
            raise RuntimeError(
                "CSV row width does not match CSV_COLUMNS"
            )

        column = {
            name: index
            for index, name in enumerate(CSV_COLUMNS)
        }

        # Normal runs ignore the first two seconds of initialization.
        # Short smoke tests use all available samples.
        evaluation_start_s = (
            2.0
            if records[-1, column["time_s"]] >= 2.0
            else 0.0
        )
        active = records[
            records[:, column["time_s"]] >= evaluation_start_s
        ]

        prediction_errors = active[
            :, column["prediction_error_mm"]
        ]
        prediction_errors = prediction_errors[
            np.isfinite(prediction_errors)
        ]

        mpc_total_fz = np.sum(
            active[
                :,
                column["mpc_fz_fl_n"]:
                column["mpc_fz_rr_n"] + 1,
            ],
            axis=1,
        )
        wbc_total_fz = np.sum(
            active[
                :,
                column["wbc_fz_fl_n"]:
                column["wbc_fz_rr_n"] + 1,
            ],
            axis=1,
        )

        timing_summary_ms = {}
        for name, samples in timings.items():
            if not samples:
                continue

            values_ms = np.asarray(samples) * 1000.0
            timing_summary_ms[name] = {
                "calls": len(samples),
                "mean": float(np.mean(values_ms)),
                "p95": float(np.percentile(values_ms, 95)),
                "maximum": float(np.max(values_ms)),
            }

        swing_height_max = {}
        swing_tracking_rmse = {}
        stance_contact_fraction = {}
        mid_swing_contact_samples = {}
        stance_center_speed_p95 = {}
        maximum_floor_force = {}
        for name, key in zip(
            OCS2_CONTACT_ORDER, ("fl", "fr", "rl", "rr")
        ):
            actual_z = records[:, column[f"foot_z_{key}_m"]]
            target_z = records[:, column[f"swing_target_z_{key}_m"]]
            swing_samples = np.isfinite(target_z)
            swing_height_max[name] = (
                float(np.max(actual_z[swing_samples]))
                if np.any(swing_samples) else None
            )
            swing_tracking_rmse[name] = (
                float(1000.0 * np.sqrt(np.mean(
                    (actual_z[swing_samples] - target_z[swing_samples]) ** 2
                )))
                if np.any(swing_samples) else None
            )
            normal_force = active[:, column[f"mj_floor_fz_{key}_n"]]
            stance = np.array([
                name in OCS2_MODE_CONTACTS[int(mode)]
                for mode in active[:, column["mode"]]
            ])
            measured_contact = normal_force > 1.0
            stance_contact_fraction[name] = (
                float(np.mean(measured_contact[stance]))
                if np.any(stance) and np.isfinite(normal_force).any()
                else None
            )
            mid_swing = (
                active[:, column[f"swing_target_z_{key}_m"]] > 0.07
            )
            mid_swing_contact_samples[name] = (
                int(np.count_nonzero(measured_contact & mid_swing))
                if np.isfinite(normal_force).any() else None
            )
            center_speed = active[
                :, column[f"stance_center_xy_speed_{key}_mps"]
            ]
            valid_speed = center_speed[np.isfinite(center_speed)]
            stance_center_speed_p95[name] = (
                float(np.percentile(valid_speed, 95))
                if valid_speed.size else None
            )
            maximum_floor_force[name] = (
                float(np.nanmax(normal_force))
                if np.isfinite(normal_force).any() else None
            )

        summary = {
            "requested_duration_s": args.duration,
            "actual_duration_s": float(mj_data.time),
            "samples": len(records),
            "mode_samples": {
                str(mode): int(np.count_nonzero(
                    records[:, column["mode"]] == mode
                ))
                for mode in OCS2_MODE_CONTACTS
            },
            "swing_foot_height_max_m": swing_height_max,
            "swing_foot_tracking_rmse_mm": swing_tracking_rmse,
            "stance_floor_contact_fraction_over_1n": stance_contact_fraction,
            "mid_swing_floor_contact_samples": mid_swing_contact_samples,
            "stance_center_xy_speed_p95_mps": stance_center_speed_p95,
            "maximum_floor_normal_force_n": maximum_floor_force,
            "simulation_timestep_s": float(dt),
            "evaluation_start_s": evaluation_start_s,
            "goal_height_rmse_mm": float(
                1000.0 * np.sqrt(np.mean(
                    active[:, column["measured_minus_goal_m"]] ** 2
                ))
            ),
            "goal_height_max_error_mm": float(
                1000.0 * np.max(np.abs(
                    active[:, column["measured_minus_goal_m"]]
                ))
            ),
            "mpc_tracking_rmse_mm": float(
                1000.0 * np.sqrt(np.mean(
                    active[:, column["measured_minus_mpc_m"]] ** 2
                ))
            ),
            "mpc_tracking_max_error_mm": float(
                1000.0 * np.max(np.abs(
                    active[:, column["measured_minus_mpc_m"]]
                ))
            ),
            "maximum_motor_torque_nm": float(np.max(
                records[:, column["max_applied_torque_nm"]]
            )),
            "maximum_feedback_torque_nm": float(np.max(
                records[:, column["max_feedback_torque_nm"]]
            )),
            "maximum_torque_clip_nm": float(np.max(
                records[:, column["max_torque_clip_nm"]]
            )),
            "clipped_steps": int(np.count_nonzero(
                records[:, column["max_torque_clip_nm"]] > 1e-9
            )),
            "maximum_dynamics_residual": float(np.max(
                records[:, column["dynamics_residual"]]
            )),
            "maximum_contact_acceleration_residual": float(
                np.max(records[
                    :, column["contact_acceleration_residual"]
                ])
            ),
            "minimum_policy_horizon_s": float(np.min(
                records[:, column["policy_horizon_s"]]
            )),
            "prediction_rmse_mm": (
                float(np.sqrt(np.mean(prediction_errors ** 2)))
                if prediction_errors.size
                else None
            ),
            "maximum_prediction_error_mm": (
                float(np.max(np.abs(prediction_errors)))
                if prediction_errors.size
                else None
            ),
            "mean_mpc_total_fz_n": float(np.mean(mpc_total_fz)),
            "mean_wbc_total_fz_n": float(np.mean(wbc_total_fz)),
            "mean_total_fz_difference_n": float(np.mean(
                np.abs(wbc_total_fz - mpc_total_fz)
            )),
            "final_base_position": mj_data.qpos[:3].tolist(),
            "timing_ms": timing_summary_ms,
        }

        print("\nExperiment summary:")
        print(json.dumps(summary, indent=2))

        if args.csv is not None:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            np.savetxt(
                args.csv,
                records,
                delimiter=",",
                comments="",
                header=",".join(CSV_COLUMNS),
                fmt="%.10g",
            )

            summary_path = args.csv.with_suffix(".json")
            summary_path.write_text(
                json.dumps(summary, indent=2) + "\n",
                encoding="utf-8",
            )

            print(f"\nSaved {len(records)} samples to {args.csv}")
            print(f"Saved summary to {summary_path}")

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
