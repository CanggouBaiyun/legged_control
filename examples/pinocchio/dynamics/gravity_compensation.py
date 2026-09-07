import numpy as np
import pinocchio as pin

from go2_control.model import(
    build_go2_model,
    nominal_configuration,
    sdk_joint_mapping,
)

np.set_printoptions(
    precision=8,
    suppress=True,
)

model = build_go2_model()
q = nominal_configuration(model)

zero_velocity = np.zeros(model.nv)
zero_acceleration = np.zeros(model.nv)


#当前姿态下重力在所有广义坐标方向上产生的影响
gravity = pin.computeGeneralizedGravity(
    model,
    model.createData(),
    q,
).copy()

inverse_dynamics_static = pin.rnea(
    model,
    model.createData(),
    q,
    zero_velocity,
    zero_acceleration
).copy()

total_mass = pin.computeTotalMass(model)
gravity_acceleration = abs(model.gravity.linear[2])
robot_weight = total_mass * gravity_acceleration

print("Model nv:")
print(model.nv)

print("\nTotal mass [kg]:")
print(total_mass)

print("\nRobot weight m*g [N]:")
print(robot_weight)

print("\nGeneralized gravity vector g(q):")
print(gravity)

print("\nFloating-base part g(q)[:6]:")
print(gravity[:6])

print("\nActuated joint part in SDK order [Nm]:")

for entry in sdk_joint_mapping(model):
    joint_name = entry["name"]
    v_index = entry["v_index"]

    print(
        f"{joint_name:20s} "
        f"{gravity[v_index]: .8f}"
    )

print("\nRNEA(q, 0, 0):")
print(inverse_dynamics_static)

print("\nDifference between RNEA and gravity:")
print(inverse_dynamics_static - gravity)

print("\nDifference norm:")
print(
    np.linalg.norm(
        inverse_dynamics_static - gravity
    )
)