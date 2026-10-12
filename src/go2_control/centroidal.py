"""State mapping for the Go2 full centroidal model."""

import numpy as np
import pinocchio as pin

def get_joint_indices(model):
    """按构型顺序,返回十二个关节的q、v索引"""
    entries = []

    for joint_id in range(1, model.njoints):
        joint = model.joints[joint_id]

        if joint.nq == 1 and joint.nv == 1:
            entries.append((joint.idx_q, joint.idx_v))

    entries.sort()

    if model.nq != 19 or model.nv != 18 or len(entries) != 12:
        raise ValueError("Expected a free-flyer Go2 model")

    q_indices = [q_index for q_index, _ in entries]
    v_indices = [v_index for _, v_index in entries]

    return q_indices, v_indices

def state_from_pinocchio(model, data, q, v):
    """将Pinocchio的q、v转换成24维质心状态"""
    q_indices, _ = get_joint_indices(model)
    mass = pin.computeTotalMass(model)

    centroidal_map = pin.computeCentroidalMap(
        model, data, q
    ).copy()

    rotation = pin.XYZQUATToSE3(q[:7]).rotation
    rpy = pin.rpy.matrixToRpy(rotation)

    state = np.zeros(24)
    state[:6] = centroidal_map @ v / mass
    state[6:9] = q[:3]
    state[9:12] = rpy[::-1]
    state[12:] = q[q_indices]

    return state

def pinocchio_from_state(model ,data, state, joint_velocities):
    """由质心状态和十二个关节速度,恢复Pinocchio的q、v """
    q_indices, v_indices = get_joint_indices(model)
    mass = pin.computeTotalMass(model)

    # 1. 恢复构型q
    q = pin.neutral(model)
    q[:3] = state[6:9]

    # state中是yaw、pitch、roll；这里需要 roll、pitch、yaw
    rpy = state[9:12][::-1].copy()
    rotation = pin.rpy.rpyToMatrix(rpy)

    #Pinocchio的四元数系数顺序为x、y、z、w
    q[3:7] = pin.Quaternion(rotation).coeffs()
    q[q_indices] = state[12:]

    # 2. 必须使用恢复后的q重新计算动量矩阵
    centroidal_map = pin.computeCentroidalMap(
        model, data, q
    ).copy()

    # 3. 输入给出关节速度，总动量决定剩下的基座速度
    v = np.zeros(model.nv)
    v[v_indices] = joint_velocities

    momentum = mass * state[:6]

    v[:6] = np.linalg.solve(
        centroidal_map[:, :6],
        momentum - centroidal_map[:, 6:] @ v[6:] ,
    )

    return q, v

def centroidal_dynamics(model, data, state, control):
    """计算完整质心模型的状态导数， 接触力采用世界坐标系"""
    state = np.asarray(state, dtype=float)
    control = np.asarray(control, dtype=float)

    if state.shape != (24,) or control.shape != (24,):
        raise ValueError("Expected state and control with shape (24,)")

    if not np.all(np.isfinite(state)) or not np.all(np.isfinite(control)):
        raise ValueError("State and control must contain finite values")

    # 1.输入前12维四足三维接触力，后12维是关节速度
    contact_names = (
        "FR_foot",
        "FL_foot",
        "RR_foot",
        "RL_foot",
    )
    contact_forces = control[:12].reshape(4,3)
    joint_velocities = control[12:]

    # 2. 从状态和关节速度恢复完整的q、v
    q, v = pinocchio_from_state(model, data, state, joint_velocities)
    mass = pin.computeTotalMass(model)

    # 3. 在当前构型下，重新计算质心和足端位置
    com_position = pin.centerOfMass(model, data, q).copy()

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # 4. 外力决定线动量变化率，绕质心的力矩决定角动量变化率
    linear_momentum_rate = (
        np.sum(contact_forces, axis=0) + mass * model.gravity.linear
    )

    angular_momentum_rate = np.zeros(3)

    for name, force in zip(contact_names, contact_forces):
        frame_id = model.getFrameId(name)

        if frame_id >= model.nframes:
            raise ValueError(f"Unknown contact frame: {name}")

        foot_position = data.oMf[frame_id].translation
        lever_arm = foot_position - com_position
        angular_momentum_rate += np.cross(lever_arm, force)

    # 5. 身体系基座线速度转换为世界系位置导数
    rotation = pin.XYZQUATToSE3(q[:7]).rotation
    base_position_rate = rotation @ v[:3]

    # 6. 身体系角速度转换为ZYX欧拉角导数
    yaw, pitch, roll = state[9:12]

    if abs(np.cos(pitch)) < 1e-6:
        raise ValueError("ZYX Euler angles are near a singular configuration")

    euler_rate_to_body_omega = np.array([
        [-np.sin(pitch), 0.0, 1.0],
        [np.sin(roll) * np.cos(pitch), np.cos(roll), 0.0],
        [np.cos(roll) * np.cos(pitch), -np.sin(roll), 0.0],
    ])

    orientation_rate = np.linalg.solve(
        euler_rate_to_body_omega,
        v[3:6],
    )

    # 7. 按状态的排列顺序，组装状态导数。
    state_rate = np.zeros(24)
    state_rate[:3] = linear_momentum_rate / mass
    state_rate[3:6] = angular_momentum_rate / mass
    state_rate[6:9] = base_position_rate
    state_rate[9:12] = orientation_rate
    state_rate[12:] = joint_velocities


    return state_rate