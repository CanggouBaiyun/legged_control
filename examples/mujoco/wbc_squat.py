"""Go2 fixed-foot squat: base tracking WBC plus joint-reference PD."""

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
from go2_control.wbc.kinematics import compute_contact_kinematics
from go2_control.wbc.reference import (
    FixedFootReference, joint_pd_feedback, squat_height_reference,
)


def run(args):
    # 1. 同源模型；q[:3] 是世界系基座位置。
    project_root = Path(__file__).resolve().parents[2]
    scene_path = project_root / "third_party/unitree_mujoco/unitree_robots/go2/scene.xml"
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    pin_model = build_go2_simulation_model()
    bridge = MujocoPinocchioBridge(model, pin_model)
    controller = StandingWBC(pin_model)
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
    print(" time   z_ref       z    err_mm  feet  max_tau  max_fb")

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
                z_ref, vz_ref, az_ref = squat_height_reference(
                    t, depth=args.depth, period=args.period,
                )
                desired_position = np.array([0.0, 0.0, z_ref])
                desired_velocity = np.array([0.0, 0.0, vz_ref])
                q, v = bridge.state(data)

                # 4. a_task = a_traj + Kp_b (p_des-p) + Kd_b (v_des-v)。
                result = controller.solve(
                    q=q, v=v,
                    desired_base_position=desired_position,
                    desired_base_rotation=np.eye(3),
                    desired_base_linear_velocity_world=desired_velocity,
                    desired_base_linear_acceleration_world=np.array([
                        0.0, 0.0, az_ref if args.mode == "tracking" else 0.0,
                    ]),
                )
                if result.solver_status.lower() != "solved":
                    failures += 1
                    raise RuntimeError(f"WBC status: {result.solver_status}")

                # 5. 同一身体轨迹对应的关节位置、速度参考。
                if args.mode == "tracking":
                    q_ref, v_ref = reference.solve(
                        desired_position, np.eye(3), desired_velocity,
                    )
                    feedback = joint_pd_feedback(
                        pin_model, q, v, q_ref, v_ref,
                        args.joint_kp, args.joint_kd,
                    )
                else:
                    # 可复现的旧基线：当前 q 上的速度参考，Kp_j=0。
                    jacobian, _ = compute_contact_kinematics(
                        pin_model, kinematics_data, q, v, GO2_FOOT_FRAMES,
                    )
                    base_velocity = np.zeros(6)
                    base_velocity[:3] = pin.XYZQUATToSE3(q[:7]).rotation.T @ desired_velocity
                    joint_velocity = np.linalg.solve(
                        jacobian[:, 6:], -jacobian[:, :6] @ base_velocity,
                    )
                    feedback = args.joint_kd * (joint_velocity - v[6:])

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
                    print(f"{t:5.2f} {z_ref:7.4f} {q[2]:7.4f} {(z_ref-q[2])*1000:9.3f}"
                          f" {len(touching):5d} {np.max(np.abs(controls)):8.3f} {np.max(np.abs(feedback)):7.3f}")
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
        "depth_m": args.depth,
        "period_s": args.period,
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
    parser.add_argument("--mode", choices=("tracking", "baseline"), default="tracking")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--depth", type=float, default=0.04)
    parser.add_argument("--period", type=float, default=4.0)
    parser.add_argument("--joint-kp", type=float, default=20.0)
    parser.add_argument("--joint-kd", type=float, default=1.0)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    values = [args.duration, args.depth, args.period, args.joint_kp, args.joint_kd]
    if not np.isfinite(values).all() or args.duration <= 2 or args.period <= 0:
        parser.error("Expected finite arguments, duration > 2 s and period > 0")
    if args.depth < 0 or min(args.joint_kp, args.joint_kd) < 0:
        parser.error("Depth and joint gains must be nonnegative")
    run(args)


if __name__ == "__main__":
    main()
