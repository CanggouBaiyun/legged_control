import pinocchio as pin

from go2_control.model import(
    build_go2_model,
    nominal_configuration,
)

model = build_go2_model()

data = model.createData()

q = nominal_configuration(model)

frame_name = "FR_foot"
frame_id = model.getFrameId(frame_name)

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

world_to_foot = data.oMf[frame_id]

print("model name:", model.name)
print("nq:", model.nq)
print("nv:", model.nv)
print("frame name:", frame_name)
print("frame id:", frame_id)

print("\nFR foot SE(3) placement:")
print(world_to_foot)

print("\nFR foot position in world [m]:")
print(world_to_foot.translation)

print("\nFR foot rotation in world:")
print(world_to_foot.rotation)