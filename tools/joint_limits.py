from go2_control.model import build_go2_model

model = build_go2_model()

print(
    f"{'joint':20s} "
    f"{'q_idx':>5s} "
    f"{'v_idx':>5s} "
    f"{'lower':>10s} "
    f"{'upper':>10s} "
    f"{'velocity':>10s} "
    f"{'effort':>10s}"
)

for joint_id in range(2, model.njoints):
    joint = model.joints[joint_id]

    if joint.nq !=1 or joint.nv !=1 :
        continue

    joint_name = model.names[joint_id]

    q_index = joint.idx_q
    v_index = joint.idx_v

    lower_limit = model.lowerPositionLimit[q_index]
    upper_limit = model.upperPositionLimit[q_index]
    velocity_limit = model.velocityLimit[v_index]
    effort_limit = model.effortLimit[v_index]

    print(
        f"{joint_name:20s} "
        f"{q_index:5d} "
        f"{v_index:5d} "
        f"{lower_limit:10.4f} "
        f"{upper_limit:10.4f} "
        f"{velocity_limit:10.4f} "
        f"{effort_limit:10.4f}"
    )