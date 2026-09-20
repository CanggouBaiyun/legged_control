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