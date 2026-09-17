"""Go2 load shift, right-front foot unloading and single-leg lift."""

import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import GO2_FOOT_FRAMES, build_go2_simulation_model
from go2_control.simulation.mujoco_bridge import MujocoPinocchioBridge
from go2_control.wbc import StandingWBC
from go2_control.wbc.reference import (
    FixedFootReference, joint_pd_feedback, 
)
from go2_control.wbc.swing_trajectory import vertical_lift_reference

def read_foot_normal_force(
    model,
    data,
    foot_geom_id,
    floor_geom_id,
):
    total_normal_force = 0.0
    contact_wrench = np.zeros(6)

    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]

        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)

        is_foot_floor_contact = (
            (geom1 == foot_geom_id and geom2 == floor_geom_id)
            or
            (geom2 == foot_geom_id and geom1 == floor_geom_id)
        )

        if not is_foot_floor_contact:
            continue

        if contact.efc_address < 0:
            continue

        mujoco.mj_contactForce(
            model,
            data,
            contact_id,
            contact_wrench,
        )

        total_normal_force += contact_wrench[0]

    return float(total_normal_force)


def run(args):
    # 1. 同源模型；q[:3] 是世界系基座位置。
    project_root = Path(__file__).resolve().parents[2]
    scene_path = project_root / "third_party/unitree_mujoco/unitree_robots/go2/scene.xml"
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    pin_model = build_go2_simulation_model()
    bridge = MujocoPinocchioBridge(model, pin_model)
    four_contact_controller = StandingWBC(pin_model)

    three_contact_controller = StandingWBC(
        pin_model,
        contact_frame_names=(
            "FL_foot",
            "RR_foot",
            "RL_foot",
        ),
    )
    fr_index = GO2_FOOT_FRAMES.index("FR_foot")
    fr_frame_id = pin_model.getFrameId("FR_foot")

    lift_start_time = 9.0
    lift_duration = 2.0
    lift_height = 0.03
    lift_started = False

    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key < 0:
        raise KeyError("MuJoCo home keyframe not found")
    mujoco.mj_resetDataKeyframe(model, data, key)
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    # 2. 固定足参考：初始 x/y，平地 z 取球形脚半径。
    # 这是点足参考的近似，不等同于球面上随滚动变化的接触点。
    initial_q, _ = bridge.state(data)
    reference_data = pin_model.createData()
    pin.forwardKinematics(pin_model, reference_data, initial_q)
    pin.updateFramePlacements(pin_model, reference_data)
    feet = np.array([
        reference_data.oMf[pin_model.getFrameId(name)].translation.copy()
        for name in GO2_FOOT_FRAMES
    ])
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    foot_geom_ids = []
    for i, name in enumerate(GO2_FOOT_FRAMES):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name.split("_")[0])
        if geom_id < 0 or floor_id < 0:
            raise KeyError("Expected named foot spheres and flat floor")
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_SPHERE:
            raise ValueError("This squat reference expects spherical feet")
        feet[i, 2] = data.geom_xpos[floor_id, 2] + model.geom_size[geom_id, 0]
        foot_geom_ids.append(geom_id)
    fr_geom_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "FR",
    )

    if fr_geom_id < 0:
        raise KeyError("FR foot geom not found")
    reference = FixedFootReference(pin_model, initial_q, feet)
    kinematics_data = pin_model.createData()

    # 3. 图形和无窗口模式使用同一个控制循环。
    if args.headless:
        viewer_context = nullcontext(None)
    else:
        from mujoco import viewer as mujoco_viewer
        viewer_context = mujoco_viewer.launch_passive(model, data)
    rows = []
    next_print_time = 0.0
    failures = 0
    max_dynamics_residual = 0.0
    max_contact_residual = 0.0
    print(f"mode={args.mode}, joint_kp={args.joint_kp if args.mode == 'tracking' else 0:g}, joint_kd={args.joint_kd:g}")
    print(
        " time  phase   FR_ref_z    FR_z"
        "   FR_QP   FR_MJ  feet  max_tau"
    )

    with viewer_context as viewer:
        if viewer is not None:
            with viewer.lock():
                viewer.cam.lookat[:] = [0.0, 0.0, 0.2]
                viewer.cam.distance = 1.5
                viewer.cam.azimuth = 135.0
                viewer.cam.elevation = -20.0
        try:
            while data.time < args.duration:
                if viewer is not None and not viewer.is_running():
                    break
                wall_start = time.perf_counter()
                t = float(data.time)
                #0～2秒：保持；2～4秒：移重；4秒后：保持终点。
                shift_start_time = 2.0
                shift_duration = 2.0

                start_position = np.array([0.0, 0.0, 0.28])
                displacement = np.array([-0.03, 0.03, 0.0])

                s = np.clip(
                    (t - shift_start_time) / shift_duration ,
                    0.0,
                    1.0,
                )

                progress = (
                    10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
                )

                progress_velocity = (
                    30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4
                ) / shift_duration

                progress_acceleration = (
                    60.0 * s - 180.0 * s**2 + 120.0 * s**3
                ) / shift_duration**2

                desired_position = (
                    start_position + progress * displacement
                )

                desired_velocity = (
                    progress_velocity * displacement
                )

                desired_acceleration = (
                    progress_acceleration * displacement
                )

                # 保留原脚本的高度误差记录。
                z_ref = desired_position[2]

                q, v = bridge.state(data)

                # 本周期四只脚的位置、速度参考
                foot_positions_ref = feet.copy()
                foot_velocities_ref = np.zeros((4, 3))

                lifting = t >= lift_start_time

                # 用于观测实际足端位置，不参与修改仿真状态
                pin.forwardKinematics(
                    pin_model, kinematics_data,q
                )
                pin.updateFramePlacements(
                    pin_model, kinematics_data,
                )
                fr_position = (
                    kinematics_data.oMf[fr_frame_id].translation.copy()
                )

                # 共用的身体跟踪任务
                solve_arguments = {
                    "q": q,
                    "v": v,
                    "desired_base_position": desired_position,
                    "desired_base_rotation": np.eye(3),
                    "desired_base_linear_velocity_world": desired_velocity,
                    "desired_base_linear_acceleration_world": desired_acceleration,
                }

                if not lifting:
                    phase = "unload" if t >= 6.0 else "shift"
                    controller = four_contact_controller

                    unload_s = np.clip(
                        (t - 6.0) / 2.0, 0.0, 1.0,
                    )
                    unload_progress = (
                        10.0 * unload_s**3 - 15.0 * unload_s**4 + 6.0 * unload_s**5
                    )
                    normal_force_limits = np.full(
                        4, controller.maximum_normal_force,
                    )
                    normal_force_limits[fr_index] *= (
                        1.0 - unload_progress
                    )
                    result = controller.solve(
                        **solve_arguments,
                        normal_force_upper_bounds=normal_force_limits,
                    )
                    fr_qp_force = result.contact_forces[
                        fr_index, 2
                    ]
                else:
                    phase = (
                        "lift"
                        if t < lift_start_time + lift_duration
                        else "hold"
                    )
                    controller = three_contact_controller

                    # 仅在切换时检查一次。
                    # 这些是本仿真实验的检查阈值，不是真机安全标准。
                    if not lift_started:
                        fr_force_before_lift = read_foot_normal_force(
                            model, data, fr_geom_id, floor_id,
                        )
                        xy_error_before_lift = np.linalg.norm(
                            desired_position[:2] - q[:2]
                        )

                        if (
                            fr_force_before_lift > 5.0
                            or xy_error_before_lift > 0.01
                        ):
                            raise RuntimeError(
                                "Not ready to lift: "
                                f"FR force={fr_force_before_lift:.3f} N, "
                                f"xy error={xy_error_before_lift:.4f} m"
                            )

                        lift_started = True

                    swing_position, swing_velocity, swing_acceleration = (
                        vertical_lift_reference(
                            time=t - lift_start_time,
                            initial_position=feet[fr_index],
                            lift_height=lift_height,
                            duration=lift_duration,
                        )
                    )

                    foot_positions_ref[fr_index] = swing_position
                    foot_velocities_ref[fr_index] = swing_velocity

                    result = controller.solve(
                        **solve_arguments,
                        swing_foot_name="FR_foot",
                        desired_swing_position=swing_position,
                        desired_swing_velocity=swing_velocity,
                        desired_swing_acceleration=swing_acceleration,
                    )

                    # 三足 QP 不再包含 FR 接触力变量。
                    # 这里的零表示模型中不允许 FR 提供支撑力。
                    fr_qp_force = 0.0

                if result.solver_status.lower() != "solved":
                    failures += 1
                    raise RuntimeError(
                        f"WBC status: {result.solver_status}, "
                        f"time={t:.4f} s, phase={phase}"
                    )


                # 5. WBC 和关节 PD 使用相同的足端参考。
                q_ref, v_ref = reference.solve(
                    base_position=desired_position,
                    base_rotation=np.eye(3),
                    linear_velocity_world=desired_velocity,
                    foot_positions_world=foot_positions_ref,
                    foot_velocities_world=foot_velocities_ref,
                )

                feedback = joint_pd_feedback(
                    pin_model,
                    q,
                    v,
                    q_ref,
                    v_ref,
                    args.joint_kp,
                    args.joint_kd,
                )

                # 6. 力矩前馈 + 关节 PD，最终命令统一限幅。
                raw_controls = bridge.pinocchio_torques_to_mujoco_controls(
                    result.motor_torques + feedback
                )
                if not np.isfinite(raw_controls).all():
                    raise RuntimeError("Non-finite motor command")
                controls = np.clip(raw_controls, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
                data.ctrl[:] = controls
                mujoco.mj_step(model, data)

                # q 和下面的接触结果对应本步积分前状态，避免错位统计。
                touching = set()
                for contact in data.contact:
                    g1, g2 = int(contact.geom1), int(contact.geom2)
                    if g1 == floor_id and g2 in foot_geom_ids:
                        touching.add(g2)
                    elif g2 == floor_id and g1 in foot_geom_ids:
                        touching.add(g1)
                rotation = pin.XYZQUATToSE3(q[:7]).rotation
                tilt_deg = np.rad2deg(np.linalg.norm(pin.rpy.matrixToRpy(rotation)[:2]))
                rows.append([
                    t, z_ref, q[2], z_ref - q[2], tilt_deg,
                    np.linalg.norm(q[:2]), np.max(np.abs(controls)),
                    np.max(np.abs(feedback)), len(touching),
                    np.max(np.abs(raw_controls - controls)),
                ])
                max_dynamics_residual = max(max_dynamics_residual, result.dynamics_residual_norm)
                max_contact_residual = max(max_contact_residual, result.contact_acceleration_residual_norm)
                if t >= next_print_time:
                    fr_actual_force = read_foot_normal_force(
                        model,
                        data,
                        fr_geom_id,
                        floor_id,
                    )

                    print(
                        f"{t:5.2f}"
                        f" {phase:>6s}"
                        f" {foot_positions_ref[fr_index, 2]:10.4f}"
                        f" {fr_position[2]:8.4f}"
                        f" {fr_qp_force:7.3f}"
                        f" {fr_actual_force:7.3f}"
                        f" {len(touching):5d}"
                        f" {np.max(np.abs(controls)):8.3f}"
                    )

                    next_print_time += 1.0
                if viewer is not None:
                    viewer.sync()
                    time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - wall_start)))
        finally:
            data.ctrl[:] = 0.0

    # 7. 按每个控制周期统计，误差指标排除前 2 s 初始化阶段。
    records = np.asarray(rows)
    if len(records) == 0 or not np.any(records[:, 0] >= 2.0):
        raise RuntimeError("Simulation ended before the squat evaluation window")
    active = records[records[:, 0] >= 2.0]
    summary = {
        "mode": args.mode,
        "duration_s": float(data.time),
        "base_shift_m": [-0.03, 0.03, 0.0],
        "shift_duration_s": 2.0,
        "joint_kp": args.joint_kp if args.mode == "tracking" else 0.0,
        "joint_kd": args.joint_kd,
        "evaluation_start_s": 2.0,
        "height_rmse_mm": float(1000 * np.sqrt(np.mean(active[:, 3]**2))),
        "height_max_error_mm": float(1000 * np.max(np.abs(active[:, 3]))),
        "maximum_tilt_deg": float(np.max(active[:, 4])),
        "maximum_xy_drift_mm": float(1000 * np.max(active[:, 5])),
        "maximum_motor_torque_nm": float(np.max(records[:, 6])),
        "maximum_feedback_torque_nm": float(np.max(records[:, 7])),
        "minimum_stance_feet": int(np.min(active[:, 8])),
        "clipped_steps": int(np.count_nonzero(records[:, 9] > 1e-8)),
        "qp_failures": failures,
        "maximum_dynamics_residual": float(max_dynamics_residual),
        "maximum_contact_residual": float(max_contact_residual),
        "final_base_position": data.qpos[:3].tolist(),
    }
    print("\nSummary (height/pose errors: t >= 2 s):")
    print(json.dumps(summary, indent=2))
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(args.csv, records, delimiter=",", comments="", header=(
            "time_s,z_ref_m,z_m,z_error_m,tilt_deg,xy_drift_m,"
            "max_torque_nm,max_feedback_nm,stance_feet,clip_nm"
        ))
        args.csv.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--mode", choices=("tracking",), default="tracking",
    )
    parser.add_argument("--duration", type=float, default=14.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=1.0)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    values = [
        args.duration,
        args.joint_kp,
        args.joint_kd,
    ]

    if not np.isfinite(values).all() or args.duration <= 11.0:
        parser.error(
            "Expected finite arguments and duration > 11 s"
        )

    if min(args.joint_kp, args.joint_kd) < 0.0:
        parser.error("Joint gains must be nonnegative")
    run(args)


if __name__ == "__main__":
    main()
