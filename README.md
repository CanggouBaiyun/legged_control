# Go2 Traditional Locomotion

[简体中文](README_zh-CN.md)

A model-based locomotion control stack for the Unitree Go2, built with Pinocchio, MuJoCo, whole-body control, OCS2, and ROS 2.

The project covers robot modeling, floating-base dynamics, contact-force optimization, whole-body control, trajectory optimization, simulation, and hardware interfaces. Its NMPC-WBC architecture is based on `legged_control` and adapted to the Go2 model, joint layout, ROS 2 communication, and Unitree SDK2.

## Control Architecture

```text
Velocity and posture commands
            ↓
Gait schedule and reference trajectories
            ↓
OCS2 NMPC
            ↓
State, foot, and contact-force references
            ↓
WBC-QP
            ↓
Feedforward joint torques + PD feedback
            ↓
MuJoCo / Unitree SDK2
```

## Current Status

### M1: Go2 Model Baseline — Complete

- Imported the official Go2 URDF from Unitree;
- built a floating-base model with `JointModelFreeFlyer()`;
- confirmed `nq=19` and `nv=18`;
- inspected joints, frames, inertial parameters, and joint limits;
- mapped the SDK2 motor order to the Pinocchio `q/v` indices;
- added basic model and dynamics tests.

### M2: Pinocchio Kinematics — Complete

- Forward kinematics and four-foot positions;
- foot Jacobians with finite-difference verification;
- verification of `v_foot = J @ v`;
- velocity IK and iterative position IK;
- damped least squares;
- Jacobian singular-value and damping analysis;
- joint velocity and position limits;
- unreachable-target tests.

The nominal FR-foot 5 cm upward motion converges to a final position error of approximately `1e-5 m`. The finite-difference velocity check has an error of approximately `2.4e-9`.

Milestone tags:

```text
v0.1.0-pinocchio-fk-baseline
v0.2.0-pinocchio-kinematics
```

### M3: Pinocchio Dynamics and Contacts — Complete

- Gravity compensation and static RNEA;
- CRBA mass matrix with symmetry, positive-definiteness, and kinetic-energy checks;
- verification of RNEA against `M @ a + h`;
- ABA forward dynamics and free-fall checks;
- `J.T @ force` contact-force mapping and power consistency;
- static distribution of vertical forces over four feet;
- explicit actuation selection matrix `S`;
- full floating-base dynamics with nonzero desired acceleration.

The model mass is `16.087 kg`. At the nominal configuration, the four vertical contact forces sum to `157.81347 N`, with a full static dynamics residual norm of approximately `9.12e-14`. For a vertical base acceleration of `0.5 m/s²`, the required support force is `165.85697 N` and the full dynamics residual is approximately `1.71e-13`.

Milestone tag:

```text
v0.3.0-pinocchio-dynamics
```

### M4: Contact-Force QP — Complete

- Four-variable vertical-force QP with normal-force bounds;
- active-bound load redistribution tests;
- 12-variable three-dimensional contact forces;
- linearized friction pyramids and normal-force limits;
- explicit infeasibility test under insufficient friction;
- equivalent solutions from OSQP and ProxQP;
- reusable solver interfaces under `src/go2_control/optimization/`.

For the current three-dimensional test, the required resultant force is `[8.0435, 0.0, 165.85697] N`. The full dynamics residual is approximately `2.96e-15`, and the OSQP–ProxQP solution difference is approximately `3.64e-6 N`.

### M5: Standing WBC — In Progress

- Verified the frame bias acceleration `J_dot @ v` against Pinocchio classical acceleration;
- verified `J @ dv + J_dot @ v` numerically for the FR foot;
- separated QP construction from solver-specific code;
- started development on `feature/wbc-standing`.

The next step is to stack all stance-foot constraints and formulate the joint decision vector `[dv, lambda, tau]`.

## Roadmap

- M5: Implement Go2 WBC with dynamics, stance-foot, friction-cone, and torque constraints;
- M6: Map states and indices between Pinocchio and MuJoCo;
- M7: Implement standing, squatting, leg lifting, and stepping in place in MuJoCo;
- M8: Add trot gait scheduling and swing-foot trajectories;
- M9: Run OCS2 examples and build the Go2 NMPC formulation;
- M10: Add ROS 2, SDK2, state-estimation, and safety interfaces.

The MuJoCo closed loop, WBC, OCS2 integration, and hardware-control interfaces are still under development.

## Repository Layout

```text
configs/                         Model and nominal-configuration files
docs/                            Roadmap, setup notes, and derivations
examples/pinocchio/kinematics/   Kinematics examples and numerical checks
examples/pinocchio/dynamics/     Dynamics and contact-force checks
examples/optimization/            Contact-force QP examples
scripts/                         Environment and model-version utilities
src/go2_control/                 Reusable model, optimization, and control modules
tests/                           Automated numerical tests
tools/                           Model inspection and maintenance tools
third_party/                     Official Unitree model repository
artifacts/                       Generated inspection results
```

## Environment

The current development environment is:

```text
Ubuntu 24.04
ROS 2 Jazzy
Python 3.10
Pinocchio 3.9
OSQP 1.1
ProxSuite / ProxQP 0.7
```

Run examples from the repository root:

```bash
conda activate go2_control
python examples/pinocchio/kinematics/fk_jacobian.py
python examples/pinocchio/kinematics/contact_acceleration.py
python examples/pinocchio/dynamics/static_contact_balance.py
python examples/optimization/contact_force_qp_friction.py
```

Run the numerical tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

Current test result:

```text
3 passed
```

The `hppfcl` migration notice for `coal` is a dependency deprecation warning and does not affect the current numerical checks.

## Dynamics Model

The whole-body controller uses the full floating-base dynamics:

$$
M(q)\dot v+h(q,v)=S^T\tau+J_c^T\lambda
$$

The first six floating-base dimensions are unactuated. The required base force and moment must therefore be generated by foot contacts. The contact-force QP currently solves `lambda` from the base equations and recovers joint torque from the actuated equations. The standing WBC will optimize `dv`, `lambda`, and `tau` together while enforcing stance-foot acceleration constraints.
