"""OCS2-WBC-MuJoCo experiment with optional asynchronous MPC; simulation only."""

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

CONTROL_TIMING_PHASES = (
    "state",
    "policy",
    "mrt_pair",
    "reference",
    "wbc",
    "command",
    "logging",
    "simulation",
    "contact",
    "viewer",
)

CONTROL_TIMING_COLUMNS = (
    "step",
    "time_s",
    "mode",
    "initialization_step",
    "plan_requested",
    "observation_published",
    "poll_requested",
    "policy_updated",
    "policy_age_ms",
    "control_start_lateness_ms",
) + tuple(f"{name}_ms" for name in CONTROL_TIMING_PHASES) + (
    "loop_ms",
    "timing_record_ms",
)

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
    "foot_y_fl_m",
    "foot_y_fr_m",
    "foot_y_rl_m",
    "foot_y_rr_m",
    "swing_target_y_fl_m",
    "swing_target_y_fr_m",
    "swing_target_y_rl_m",
    "swing_target_y_rr_m",
    "mj_floor_fz_fl_n",
    "mj_floor_fz_fr_n",
    "mj_floor_fz_rl_n",
    "mj_floor_fz_rr_n",
    "stance_center_xy_speed_fl_mps",
    "stance_center_xy_speed_fr_mps",
    "stance_center_xy_speed_rl_mps",
    "stance_center_xy_speed_rr_mps",
    "mj_contact_tangent_speed_fl_mps",
    "mj_contact_tangent_speed_fr_mps",
    "mj_contact_tangent_speed_rl_mps",
    "mj_contact_tangent_speed_rr_mps",
    "mj_contact_translation_speed_fl_mps",
    "mj_contact_translation_speed_fr_mps",
    "mj_contact_translation_speed_rl_mps",
    "mj_contact_translation_speed_rr_mps",
    "mj_contact_rotation_speed_fl_mps",
    "mj_contact_rotation_speed_fr_mps",
    "mj_contact_rotation_speed_rl_mps",
    "mj_contact_rotation_speed_rr_mps",
)

CSV_COLUMNS += tuple(
    f"pd_swing_{kind}_delta_{axis}_{leg}_{unit}"
    for kind, unit in (("position", "m"), ("velocity", "mps"))
    for axis in ("x", "y", "z")
    for leg in ("fl", "fr", "rl", "rr")
)

CSV_COLUMNS += (
    "policy_reset_epoch",
    "policy_sequence",
    "policy_age_s",
    "policy_updated",
    "control_start_lateness_s",
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


def summarize_control_timings(records, timestep, evaluation_start_s):
    """Summarize disjoint loop phases, excluding the startup interval."""
    records = np.asarray(records, dtype=float)
    if records.ndim != 2 or records.shape[1] != len(CONTROL_TIMING_COLUMNS):
        raise RuntimeError("Control timing row width does not match columns")

    column = {name: index for index, name in enumerate(CONTROL_TIMING_COLUMNS)}
    phase_names = [f"{name}_ms" for name in CONTROL_TIMING_PHASES]
    phase_names.append("timing_record_ms")
    durations = records[:, [column[name] for name in phase_names]]
    loop_ms = records[:, column["loop_ms"]]
    if (
        not np.isfinite(durations).all()
        or not np.isfinite(loop_ms).all()
        or np.any(durations < 0.0)
        or np.any(loop_ms < 0.0)
    ):
        raise RuntimeError("Invalid control timing durations")

    accounting_error = np.abs(np.sum(durations, axis=1) - loop_ms)
    if np.any(accounting_error > 1e-6):
        raise RuntimeError("Control timing phases do not sum to loop duration")

    active = records[:, column["time_s"]] >= evaluation_start_s
    if not np.any(active):
        raise RuntimeError("No control timing samples in evaluation interval")
    active_durations = durations[active]
    active_loop = loop_ms[active]
    overruns = active_loop > timestep * 1000.0

    def statistics(values):
        return {
            "mean": float(np.mean(values)),
            "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)),
            "maximum": float(np.max(values)),
        }

    largest_counts = {name: 0 for name in phase_names}
    overrun_means = None
    if np.any(overruns):
        overrun_durations = active_durations[overruns]
        overrun_means = {
            name: float(np.mean(overrun_durations[:, index]))
            for index, name in enumerate(phase_names)
        }
        largest = np.argmax(overrun_durations, axis=1)
        largest_counts = {
            name: int(np.count_nonzero(largest == index))
            for index, name in enumerate(phase_names)
        }

    return {
        "evaluation_start_s": float(evaluation_start_s),
        "samples": int(np.count_nonzero(active)),
        "overrun_samples": int(np.count_nonzero(overruns)),
        "phase_ms": {
            name: statistics(active_durations[:, index])
            for index, name in enumerate(phase_names)
        },
        "loop_ms": statistics(active_loop),
        "overrun_phase_mean_ms": overrun_means,
        "overrun_largest_phase_counts": largest_counts,
        "maximum_accounting_error_ms": float(np.max(accounting_error)),
    }


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

def floor_contact_tangent_speeds(model, data, floor_id, foot_name_by_geom):
    """Return total, translation and rotation tangent-speed magnitudes."""
    speeds = {
        name: float("nan") for name in foot_name_by_geom.values()
    }
    translation_speeds = speeds.copy()
    rotation_speeds = speeds.copy()
    wrench = np.zeros(6)
    foot_motion = np.zeros(6)
    floor_motion = np.zeros(6)

    # 返回顺序为角速度、线速度；0 表示使用世界系方向。
    mujoco.mj_objectVelocity(
        model, data, mujoco.mjtObj.mjOBJ_GEOM,
        floor_id, floor_motion, 0,
    )

    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        foot_geom = (
            geom2 if geom1 == floor_id
            else geom1 if geom2 == floor_id
            else -1
        )
        foot_name = foot_name_by_geom.get(foot_geom)
        if foot_name is None or contact.efc_address < 0:
            continue

        mujoco.mj_contactForce(model, data, contact_id, wrench)
        if wrench[0] <= 1.0:
            continue

        mujoco.mj_objectVelocity(
            model, data, mujoco.mjtObj.mjOBJ_GEOM,
            foot_geom, foot_motion, 0,
        )
        translation_velocity = foot_motion[3:] - floor_motion[3:]
        rotation_velocity = (
            np.cross(
                foot_motion[:3],
                contact.pos - data.geom_xpos[foot_geom],
            )
            - np.cross(
                floor_motion[:3],
                contact.pos - data.geom_xpos[floor_id],
            )
        )
        relative_velocity = translation_velocity + rotation_velocity

        # 接触坐标系第一行为法向，后两行为切向。
        tangent_axes = contact.frame.reshape(3, 3)[1:]
        speed = float(np.linalg.norm(tangent_axes @ relative_velocity))

        previous = speeds[foot_name]
        # Keep all three magnitudes from the same selected contact.
        if not np.isfinite(previous) or speed > previous:
            speeds[foot_name] = speed
            translation_speeds[foot_name] = float(np.linalg.norm(
                tangent_axes @ translation_velocity
            ))
            rotation_speeds[foot_name] = float(np.linalg.norm(
                tangent_axes @ rotation_velocity
            ))

    return speeds, translation_speeds, rotation_speeds

def mpc_foot_references(model, data, state, control):
    """Read world-frame foot positions and velocities from one MPC point."""
    q_ref, v_ref = pinocchio_from_state(
        model, data, state, control[12:]
    )
    pin.forwardKinematics(model, data, q_ref, v_ref)
    pin.updateFramePlacements(model, data)

    references = {}
    for name in OCS2_CONTACT_ORDER:
        frame_id = model.getFrameId(name)
        position = data.oMf[frame_id].translation.copy()
        velocity = pin.getFrameVelocity(
            model, data, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        ).linear.copy()
        references[name] = (position, velocity)

    return references

def joint_pd_swing_reference_deltas(
    model, data, q, v, q_ref, v_ref, swing_targets,
):
    """Compare joint PD references with WBC swing targets."""
    deltas = np.full((6, 4), np.nan)
    if not swing_targets:
        return deltas.ravel().tolist()

    # 保留实测基座，只替换电机关节的位置和速度。
    q_target = q.copy()
    q_target[7:] = q_ref[7:]
    v_target = v.copy()
    v_target[6:] = v_ref[6:]

    pin.forwardKinematics(model, data, q_target, v_target)
    pin.updateFramePlacements(model, data)

    for index, name in enumerate(OCS2_CONTACT_ORDER):
        if name not in swing_targets:
            continue

        target_position, target_velocity, _ = swing_targets[name]
        if target_velocity is None:
            target_velocity = np.zeros(3)

        frame_id = model.getFrameId(name)
        pd_position = data.oMf[frame_id].translation
        pd_velocity = pin.getFrameVelocity(
            model, data, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        ).linear

        deltas[:3, index] = pd_position - target_position
        deltas[3:, index] = pd_velocity - target_velocity

    return deltas.ravel().tolist()


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
    return parse_policy_reply(reply)
def parse_policy_reply(reply):
    """Validate a policy reply and extract diagnostic trajectories."""
    times = np.asarray(reply["times"], dtype=float)

    if (
        times.ndim != 1
        or len(times) < 2
        or not np.isfinite(times).all()
        or not np.all(np.diff(times) >= 0.0)
        or not np.any(np.diff(times) > 0.0)
    ):
        raise RuntimeError("Invalid policy time trajectory")

    trajectory_format = reply.get("trajectory_format", "full")

    if trajectory_format == "full":
        states = np.asarray(reply["states"], dtype=float)
        inputs = np.asarray(reply["inputs"], dtype=float)

        if (
            states.shape != (len(times), 24)
            or inputs.shape != (len(times), 24)
            or not np.isfinite(states).all()
            or not np.isfinite(inputs).all()
        ):
            raise RuntimeError("Invalid full policy arrays")

        height_trajectory = states[:, 8].copy()
        yaw_trajectory = states[:, 9].copy()

    elif trajectory_format == "height_yaw":
        height_trajectory = np.asarray(
            reply["height_trajectory"], dtype=float
        )
        yaw_trajectory = np.asarray(
            reply["yaw_trajectory"], dtype=float
        )

    else:
        raise RuntimeError(
            f"Unknown policy trajectory format: {trajectory_format}"
        )

    if (
        height_trajectory.shape != (len(times),)
        or yaw_trajectory.shape != (len(times),)
        or not np.isfinite(height_trajectory).all()
        or not np.isfinite(yaw_trajectory).all()
    ):
        raise RuntimeError("Invalid diagnostic trajectories")

    mode_event_times = np.asarray(
        reply["mode_event_times"], dtype=float
    )
    mode_sequence = np.asarray(
        reply["mode_sequence"], dtype=int
    )

    if (
        mode_event_times.ndim != 1
        or mode_sequence.ndim != 1
        or len(mode_sequence) != len(mode_event_times) + 1
        or not np.isfinite(mode_event_times).all()
        or not np.all(np.diff(mode_event_times) >= 0.0)
        or any(
            mode not in OCS2_MODE_CONTACTS
            for mode in mode_sequence
        )
    ):
        raise RuntimeError("Invalid policy mode schedule")

    return (
        times,
        height_trajectory,
        yaw_trajectory,
        mode_event_times,
        mode_sequence,
    )

def poll_policy(process, stamp, until, max_age=0.1):
    """Poll for a usable policy without waiting for MPC to finish."""
    reply = request_transport(
        process,
        {
            "action": "poll",
            "time": float(stamp),
            "until": float(until),
            "max_policy_age": float(max_age),
        },
    )

    age = float(reply["policy_age_s"])
    if (
        not isinstance(reply["updated"], bool)
        or reply["reset_epoch"] <= 0
        or reply["policy_sequence"] <= 0
        or not np.isfinite(age)
        or not -1e-6 <= age <= max_age
    ):
        raise RuntimeError("Invalid policy polling response")

    policy = None
    if reply["updated"]:
        payload = reply["policy"]
        observation_time = float(payload["observation_time"])

        if (
            payload["reset_epoch"] != reply["reset_epoch"]
            or payload["policy_sequence"] != reply["policy_sequence"]
            or not np.isfinite(observation_time)
            or abs((stamp - observation_time) - age) > 1e-6
        ):
            raise RuntimeError("Polled policy metadata mismatch")

        policy = parse_policy_reply(payload)
    elif reply["policy"] is not None:
        raise RuntimeError("Unexpected policy without update flag")

    return policy, reply

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

def evaluate_policy_pair(
    process, stamp, future_stamp, current_state, expected_policy=None
):
    """Obtain current and future references in one transport round trip."""
    reply = request_transport(
        process,
        {
            "action": "evaluate_pair",
            "time": float(stamp),
            "future_time": float(future_stamp),
            "state": current_state.tolist(),
        },
    )

    if expected_policy is not None and (
        reply["reset_epoch"] != expected_policy["reset_epoch"]
        or reply["policy_sequence"] != expected_policy["policy_sequence"]
    ):
        raise RuntimeError("MRT pair differs from the selected policy")

    state = np.asarray(reply["state"], dtype=float)
    control = np.asarray(reply["input"], dtype=float)
    future_state = np.asarray(reply["future_state"], dtype=float)
    future_control = np.asarray(reply["future_input"], dtype=float)
    mode = reply["mode"]
    future_mode = reply["future_mode"]

    if (
        mode not in OCS2_MODE_CONTACTS
        or future_mode not in OCS2_MODE_CONTACTS
        or any(
            values.shape != (24,) or not np.isfinite(values).all()
            for values in (state, control, future_state, future_control)
        )
    ):
        raise RuntimeError("Invalid pair references returned by MRT")

    return state, control, mode, future_state, future_control, future_mode


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

def turn_yaw_goal(time, angle_deg=10.0):
    """Return desired yaw and yaw rate for a signed turn."""
    rate = np.deg2rad(angle_deg / 3.0)

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
    parser.add_argument(
        "--turn-angle-deg",
        type=float,
        default=10.0,
        help="Signed turn angle in degrees: positive left, negative right.",
    )
    parser.add_argument(
        "--joint-pd-scale",
        type=float,
        default=1.0,
        help="Scale the extra joint PD torque between 0 and 1.",
    )
    parser.add_argument(
        "--swing-reference",
        choices=("planner", "mpc"),
        default="planner",
        help="Select the source of WBC swing-foot references.",
    )
    parser.add_argument(
        "--async-mpc",
        action="store_true",
        help="Publish observations without waiting for each MPC solve.",
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
    if not np.isfinite(args.turn_angle_deg):
        parser.error("--turn-angle-deg must be finite")
    if not 0.0 <= args.joint_pd_scale <= 1.0:
        parser.error("--joint-pd-scale must be in [0, 1]")
    if args.swing_reference == "mpc" and not args.allow_trot:
        parser.error("--swing-reference mpc requires --allow-trot")
    return args


def main():
    args = parse_args()

    def yaw_goal(stamp):
        return turn_yaw_goal(stamp, angle_deg=args.turn_angle_deg)
    
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
        yaw_goal=yaw_goal if args.turn else None,                
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
    pd_reference_data = model.createData()
    mpc_foot_data = model.createData()

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
            yaw_goal(stamp) if args.turn else (0.0, 0.0)
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
    "source /home/bot/Project/ocs2_ws/install/local_setup.bash && "
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
    steps_per_poll = max(1, round(0.004 / dt))
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
    wall_start = None
    policy_info = None
    policy_observation_time = None
    policy_updates = 0
    max_policy_age = 0.1
    rows = []
    step_timing_rows = []
    timings = {
        "plan": [],
        "observe": [],
        "poll": [],
        "mrt_pair": [],
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

            control_start_lateness = float("nan")
            policy_changed = False

            if args.async_mpc and wall_start is not None:
                scheduled_start = wall_start + float(mj_data.time)
                remaining = scheduled_start - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)

                control_start_lateness = max(
                    0.0, time.perf_counter() - scheduled_start
                )

            loop_start = time.perf_counter()
            phase_marks = [loop_start]

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

            phase_marks.append(time.perf_counter())  # state
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

                stamp = float(mj_data.time)
                next_policy_time = stamp + steps_per_policy * dt

                # 覆盖下一次更新之前的控制与未来 5 ms 加速度查询。
                required_until = next_policy_time + 0.005

                observation_message = {
                    "time": stamp,
                    "state": state.tolist(),
                    "input": observation_input.tolist(),
                }
                if gait_command is not None:
                    observation_message["gait"] = gait_command

                if not args.async_mpc or step == 0:
                    # 初始化仍需等待第一条策略。
                    # 同步模式下，每次更新也继续走原来的等待流程。
                    (
                        times,
                        height_trajectory,
                        yaw_trajectory,
                        mode_event_times,
                        mode_sequence,
                    ) = timed_call(
                        timings["plan"],
                        request_policy,
                        process,
                        {
                            **observation_message,
                            "until": required_until,
                            "target": target.tolist(),
                            "target_times": target_times.tolist(),
                            "target_states": target_states.tolist(),
                        },
                    )
                else:
                    acknowledgement = timed_call(
                        timings["observe"],
                        request_transport,
                        process,
                        {
                            **observation_message,
                            "action": "observe",
                        },
                    )
                    if (
                        acknowledgement.get("observation_published") is not True
                        or acknowledgement.get("observation_time") != stamp
                    ):
                        raise RuntimeError(
                            "Invalid observation acknowledgement"
                        )

                if send_gait:
                    gait_command_sent = True
                if send_stance:
                    stance_command_sent = True

            # 策略检查独立于每 20 ms 的观测发布。
            if args.async_mpc and step % steps_per_poll == 0:
                poll_stamp = float(mj_data.time)
                poll_until = poll_stamp + steps_per_poll * dt + 0.005

                new_policy, new_info = timed_call(
                    timings["poll"],
                    poll_policy,
                    process,
                    poll_stamp,
                    poll_until,
                    max_policy_age,
                )

                if policy_info is not None:
                    sequence_advanced = (
                        new_info["policy_sequence"]
                        > policy_info["policy_sequence"]
                    )
                    if (
                        new_info["reset_epoch"]
                        != policy_info["reset_epoch"]
                        or new_info["policy_sequence"]
                        < policy_info["policy_sequence"]
                        or new_info["updated"] != sequence_advanced
                    ):
                        raise RuntimeError(
                            "Unexpected MPC policy identity change"
                        )

                if new_policy is not None:
                    (
                        times,
                        height_trajectory,
                        yaw_trajectory,
                        mode_event_times,
                        mode_sequence,
                    ) = new_policy
                    policy_updates += 1

                policy_info = new_info
                policy_changed = new_info["updated"]
                policy_observation_time = (
                    poll_stamp - new_info["policy_age_s"]
                )

                # 初始化等待结束后，再建立仿真的真实时间起点。
                if wall_start is None:
                    wall_start = time.perf_counter() - poll_stamp
                    control_start_lateness = 0.0

            # 保留每 20 ms 的高度预测诊断口径。
            if step % steps_per_policy == 0:
                predicted_next_z = float(np.interp(
                    next_policy_time,
                    times,
                    height_trajectory,
                ))

            policy_age_s = float("nan")
            if args.async_mpc:
                policy_age_s = (
                    float(mj_data.time) - policy_observation_time
                )
                if (
                    not -1e-6 <= policy_age_s <= max_policy_age
                    or times[0] > mj_data.time + 1e-6
                    or times[-1] < mj_data.time + 0.005
                ):
                    raise RuntimeError(
                        "Active MPC policy is stale or does not "
                        "cover the control queries"
                    )
            # B. 用当前仿真时间取得 MPC 参考。
            current_mpc_state = state_from_pinocchio(
                model,
                mapping_data,
                q,
                v,
            )

            # 一次通信取得同一条策略上的当前和未来参考。
            acceleration_interval = 0.005
            future_time = mj_data.time + acceleration_interval
            if future_time > times[-1]:
                raise RuntimeError(
                    "Policy does not cover acceleration estimation"
                )

            phase_marks.append(time.perf_counter())  # policy
            (
                state_ref,
                input_ref,
                mode,
                future_state_ref,
                future_input_ref,
                future_mode,
            ) = timed_call(
                timings["mrt_pair"],
                evaluate_policy_pair,
                process,
                mj_data.time,
                future_time,
                current_mpc_state,
                policy_info,
            )

            phase_marks.append(time.perf_counter())  # mrt_pair
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
                if args.swing_reference == "planner":
                    wbc, swing_targets = trot_wbc.prepare(
                        mj_data.time,
                        mode,
                        mode_event_times,
                        mode_sequence,
                        feet_by_name,
                        base_position=q[:3],
                    )
                else:
                    wbc = trot_wbc.controllers[mode]

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

            # 未来参考已随当前参考返回，仍单独检查未来接触模式。
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
            angular_acceleration_ff_local = (
                future_reference["desired_base_angular_velocity_local"]
                - reference["desired_base_angular_velocity_local"]
            ) / acceleration_interval

            if not np.isfinite(angular_acceleration_ff_local).all():
                raise RuntimeError(
                    "Non-finite base angular acceleration feedforward"
                )

            reference["desired_base_angular_acceleration_local"] = (
                angular_acceleration_ff_local
            )
            if args.swing_reference == "mpc" and mode != 15:
                current_feet = mpc_foot_references(
                    model, mpc_foot_data, state_ref, input_ref
                )
                future_feet = mpc_foot_references(
                    model, mpc_foot_data,
                    future_state_ref, future_input_ref,
                )

                swing_targets = {}
                for name in OCS2_CONTACT_ORDER:
                    if name in wbc.contact_frame_names:
                        continue

                    position, velocity = current_feet[name]
                    acceleration = np.zeros(3)

                    # 在同一摆动阶段内估计前馈，避免跨触地跳变求导。
                    if name not in future_wbc.contact_frame_names:
                        acceleration = (
                            future_feet[name][1] - velocity
                        ) / acceleration_interval

                    if not (
                        np.isfinite(position).all()
                        and np.isfinite(velocity).all()
                        and np.isfinite(acceleration).all()
                    ):
                        raise RuntimeError("Non-finite MPC swing reference")

                    swing_targets[name] = (
                        position, velocity, acceleration
                    )
            phase_marks.append(time.perf_counter())  # reference
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

            phase_marks.append(time.perf_counter())  # wbc, including checks
            # D. 关节 PD 的参考也来自当前 MPC 轨迹点。
            q_ref, v_ref = pinocchio_from_state(
                model, mapping_data, state_ref, input_ref[12:]
            )
            pd_swing_deltas = joint_pd_swing_reference_deltas(
                model, pd_reference_data,
                q, v, q_ref, v_ref, swing_targets,
            )
            feedback = args.joint_pd_scale * joint_pd_feedback(
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

            phase_marks.append(time.perf_counter())  # command
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
                    *(
                        [float(feet_by_name[name][1]) for name in OCS2_CONTACT_ORDER]
                        if trot_wbc is not None else [float("nan")] * 4
                    ),
                    *(
                        float(swing_targets[name][0][1])
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
                    height_trajectory,
                ))
                future_yaw = float(np.interp(
                    lookahead_time,
                    times,
                    yaw_trajectory,
                ))
                yaw_change = np.arctan2(
                    np.sin(future_yaw - state_ref[9]),
                    np.cos(future_yaw - state_ref[9]),
                )
                planned_yaw_rate_deg_s = np.rad2deg(yaw_change) / 0.2

                mpc_fz = float(np.sum(mpc_forces[:, 2]))
                wbc_fz = float(np.sum(wbc_forces_ocs2_order[:, 2]))

                print(
                    f"t={mj_data.time:5.2f} "
                    f"mode={mode} "
                    f"policy_age_ms={policy_age_s * 1000.0:.1f} "
                    f"yaw_goal={np.rad2deg(target[9]):.2f} "
                    f"yaw_mpc={np.rad2deg(state_ref[9]):.2f} "
                    f"yaw={np.rad2deg(measured_yaw):.2f} "
                    f"yaw+0.2={np.rad2deg(future_yaw):.2f} "
                    f"yaw_end={np.rad2deg(yaw_trajectory[-1]):.2f} "
                    f"yaw_rate_plan={planned_yaw_rate_deg_s:+.2f} "
                    f"x_goal={target[6]:.4f} "
                    f"x_mpc={state_ref[6]:.4f} "
                    f"x={q[0]:.4f} "
                    f"goal={target[8]:.4f} "
                    f"z={q[2]:.4f} "
                    f"plan+0.2={future_z:.4f} "
                    f"plan_end={height_trajectory[-1]:.4f} "
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

            phase_marks.append(time.perf_counter())  # logging
            # E. 执行力矩，推进真实的仿真动力学。
            mujoco.mj_step(mj_model, mj_data)
            phase_marks.append(time.perf_counter())  # simulation
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
                (
                    contact_tangent_speeds,
                    contact_translation_speeds,
                    contact_rotation_speeds,
                ) = floor_contact_tangent_speeds(
                    mj_model, mj_data, floor_id, foot_name_by_geom
                )
                row.extend(floor_forces[name] for name in OCS2_CONTACT_ORDER)
                row.extend(stance_speeds[name] for name in OCS2_CONTACT_ORDER)
                row.extend(
                    contact_tangent_speeds[name]
                    for name in OCS2_CONTACT_ORDER
                )
                row.extend(
                    contact_translation_speeds[name]
                    for name in OCS2_CONTACT_ORDER
                )
                row.extend(
                    contact_rotation_speeds[name]
                    for name in OCS2_CONTACT_ORDER
                )
                previous_feet_by_name = {
                    name: position.copy()
                    for name, position in feet_by_name.items()
                }
                previous_stance = set(wbc.contact_frame_names)
                previous_floor_forces = floor_forces
            else:
                row.extend([float("nan")] * 20)
            row.extend(pd_swing_deltas)
            row.extend([
                (
                    policy_info["reset_epoch"]
                    if policy_info is not None else float("nan")
                ),
                (
                    policy_info["policy_sequence"]
                    if policy_info is not None else float("nan")
                ),
                policy_age_s,
                int(policy_changed),
                control_start_lateness,
            ])
            rows.append(row)
            phase_marks.append(time.perf_counter())  # contact and row completion
            if viewer is not None:
                viewer.sync()
            phase_marks.append(time.perf_counter())  # viewer

            phase_ms = [
                (end - start) * 1000.0
                for start, end in zip(phase_marks[:-1], phase_marks[1:])
            ]
            observation_due = step % steps_per_policy == 0
            step_timing_row = [
                step,
                row[0],  # mj_step 已推进时间；沿用本步积分前的日志时间。
                mode,
                int(step == 0),
                int(observation_due and (not args.async_mpc or step == 0)),
                int(observation_due and args.async_mpc and step != 0),
                int(args.async_mpc and step % steps_per_poll == 0),
                int(policy_changed),
                policy_age_s * 1000.0,
                control_start_lateness * 1000.0,
                *phase_ms,
                0.0,
                0.0,
            ]
            step_timing_rows.append(step_timing_row)
            loop_end = time.perf_counter()
            loop_seconds = loop_end - loop_start
            timings["loop"].append(loop_seconds)
            step_timing_row[-2] = loop_seconds * 1000.0
            step_timing_row[-1] = (loop_end - phase_marks[-1]) * 1000.0

        print("\nFinal base position:", mj_data.qpos[:3])
        print("Clipped steps:", clipped_steps)
        controllers = (
            trot_wbc.controllers
            if trot_wbc is not None
            else {15: wbc}
        )

        osqp_workspace_stats = {}
        print("\nOSQP workspace statistics:")

        for contact_mode, controller in sorted(controllers.items()):
            workspace = controller.osqp_workspace
            osqp_workspace_stats[str(contact_mode)] = {
                "setup_count": workspace.setup_count,
                "update_count": workspace.update_count,
            }
            print(
                f"mode={contact_mode}: "
                f"setup={workspace.setup_count}, "
                f"update={workspace.update_count}"
            )
        # Inclusive wall-clock measurements; loop already contains all sections.
        # Each MRT pair entry is one round trip with two policy evaluations.
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

        timing_records = np.asarray(step_timing_rows, dtype=float)
        if (
            timing_records.ndim != 2
            or timing_records.shape[0] != records.shape[0]
            or timing_records.shape[1] != len(CONTROL_TIMING_COLUMNS)
            or not np.array_equal(timing_records[:, 1], records[:, column["time_s"]])
            or not np.array_equal(timing_records[:, 2], records[:, column["mode"]])
        ):
            raise RuntimeError("Control timing rows do not align with motion rows")
        control_timing_summary = summarize_control_timings(
            timing_records, dt, evaluation_start_s
        )
        control_timing_path = (
            args.csv.with_name(f"{args.csv.stem}_timing.csv")
            if args.csv is not None else None
        )
        control_timing_summary["csv_file"] = (
            str(control_timing_path) if control_timing_path is not None else None
        )

        print(
            f"\nControl phases [ms], t >= {evaluation_start_s:g} s "
            f"({control_timing_summary['samples']} samples):"
        )
        print(f"{'phase':<20} {'mean':>10} {'p95':>10} {'p99':>10} {'max':>10}")
        for name, stats in control_timing_summary["phase_ms"].items():
            print(
                f"{name:<20} {stats['mean']:>10.3f} {stats['p95']:>10.3f} "
                f"{stats['p99']:>10.3f} {stats['maximum']:>10.3f}"
            )

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
            "joint_pd_scale": args.joint_pd_scale,
            "swing_reference": args.swing_reference,
            "async_mpc": args.async_mpc,
            "observation_period_s": float(steps_per_policy * dt),
            "policy_poll_period_s": (
                float(steps_per_poll * dt) if args.async_mpc else None
            ),
            "mrt_query_mode": "pair",
            "policy_updates": policy_updates,
            "maximum_policy_age_s": (
                float(np.max(records[:, column["policy_age_s"]]))
                if args.async_mpc else None
            ),
            "maximum_control_start_lateness_ms": (
                float(
                    1000.0 * np.max(
                        records[:, column["control_start_lateness_s"]]
                    )
                )
                if args.async_mpc else None
            ),
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
            "control_timing": control_timing_summary,
            "osqp_workspace_stats": osqp_workspace_stats,
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

            np.savetxt(
                control_timing_path,
                timing_records,
                delimiter=",",
                comments="",
                header=",".join(CONTROL_TIMING_COLUMNS),
                fmt="%.10g",
            )

            summary_path = args.csv.with_suffix(".json")
            summary_path.write_text(
                json.dumps(summary, indent=2) + "\n",
                encoding="utf-8",
            )

            print(f"\nSaved {len(records)} samples to {args.csv}")
            print(f"Saved {len(timing_records)} timing samples to {control_timing_path}")
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
