# OCS2–MRT–WBC Standing Integration

## Scope

This experiment validates the first complete OCS2 control path for the Go2 simulation:

```text
height trajectory + fixed stance mode 15
                  ↓
      OCS2 full-centroidal DDP MPC
                  ↓ ROS 2 policy
        C++ MRT policy cache/service
                  ↓ evaluated reference
       Python MPC-to-WBC adapter
                  ↓
          WBC QP + joint PD
                  ↓ torque limits
                MuJoCo
```

It covers four-foot standing and smooth height changes. It does not yet provide a walking gait, hardware state estimation or a Unitree SDK2 actuator interface.

## Contact and state conventions

The OCS2 configuration uses this joint and contact order:

```text
Joints:   FL, FR, RL, RR
Contacts: FL, FR, RL, RR
```

The WBC currently uses `FR, FL, RR, RL`. `standing_reference_from_mpc()` maps contact forces by frame name instead of relying on array position. `tests/test_mpc_reference.py` assigns a different force to each foot and verifies the reordered output.

The OCS2 state has 24 elements: six normalized centroidal-momentum components, base pose and 12 joint positions. The 24-element input contains four world-frame contact forces followed by 12 joint velocities. The adapter recovers Pinocchio configuration and velocity references, converts the reference linear velocity to the world frame and expresses angular velocity in the current body frame expected by WBC.

## Processes

All three terminals must use the same ROS domain.

### 1. OCS2 MPC node

```bash
source /opt/ros/jazzy/setup.bash
source /home/bot/Project/ocs2_ws/install/setup.bash
export ROS_DOMAIN_ID=213

ros2 run ocs2_legged_robot_ros legged_robot_ddp_mpc \
  --ros-args \
  -p taskFile:=/home/bot/Project/go2_traditional_locomotion/config/ocs2/go2/task.info \
  -p referenceFile:=/home/bot/Project/go2_traditional_locomotion/config/ocs2/go2/reference.info \
  -p urdfFile:=/home/bot/Project/go2_traditional_locomotion/models/ocs2/go2_mujoco_inertial.urdf
```

### 2. C++ MRT query server

```bash
source /opt/ros/jazzy/setup.bash
source /home/bot/Project/ocs2_ws/install/setup.bash
export ROS_DOMAIN_ID=213

ros2 run go2_mpc_bridge mrt_policy_server
```

### 3. MuJoCo closed loop

```bash
cd /home/bot/Project/go2_traditional_locomotion
export ROS_DOMAIN_ID=213

env -u PYTHONPATH \
  /home/bot/miniconda3/envs/go2_control/bin/python \
  examples/mujoco/ocs2_wbc_standing.py \
  --headless --duration 20 \
  --csv artifacts/ocs2_wbc_standing_ground_feet.csv
```

`PYTHONPATH` is removed only for the Conda Python process because the sourced ROS Jazzy environment exposes Python 3.12 packages, while `go2_control` uses Python 3.10. The ROS subprocess separately loads the Jazzy and OCS2 environments.

## Gait command transport check

The `standing_trot` template in `config/ocs2/go2/gait.info` was sent through the ROS 2 gait command node. The MPC inserted it after the active policy horizon at simulation time 10.08 s in this run. The policy then contained mode 9 from 10.08 to 10.38 s, mode 15 from 10.38 to 10.43 s, mode 6 from 10.43 to 10.73 s, and another mode 15 from 10.73 to 10.78 s. The C++ MRT service returned mode 9; the standing simulation stopped at its explicit mode-15 guard before applying a two-foot-stance torque.

This checks gait-command delivery, policy mode scheduling and MRT transport. It does not validate trot motion. The C++ MRT cache accepts modes 15, 9 and 6 and checks that event times are finite, ordered and one fewer than the modes. The standing simulation continues to reject modes 9 and 6 until WBC contact switching and swing references are integrated.

## Verified 20 s result

The test used a 2 ms MuJoCo timestep and recorded 10,000 samples. The commanded base height held 0.28 m, transitioned to 0.27 m, held, and returned to 0.28 m. Fixed-foot IK supplied joint-position targets for the height trajectory. The target foot centers were placed 0.022 m above the ground, matching the foot collision-sphere radius in this scene.

| Metric | Result |
| --- | ---: |
| Goal-height RMSE / maximum error, after 2 s | 2.617 / 5.936 mm |
| Measured tracking of MPC height, RMSE / maximum | 0.023 / 0.097 mm |
| MRT one-step prediction RMSE / maximum error | 0.053 / 0.114 mm |
| Maximum motor command / feedback magnitude | 8.237 / 2.344 Nm |
| Torque-clipped steps | 0 |
| Maximum dynamics residual | 1.99e-13 |
| Maximum contact-acceleration residual | 2.02e-15 |
| Minimum policy horizon | 0.982 s |
| Mean MPC / WBC total vertical force | 148.721 / 148.514 N |
| Mean absolute total-force difference | 0.392 N |

During the lower hold from 8 to 12 s, the goal was 0.27000 m, the mean MPC reference was 0.27317 m and the measured mean was 0.27318 m. The measured system followed the MPC reference closely. The remaining lower-hold error of about 3.18 mm still needs investigation.

The original base-only target gave 6.013 mm goal-height RMSE. Adding fixed-foot IK while anchoring the feet at their initial positions made it 7.861 mm. At the initial 0.270 m base height, the foot frames were about 0.0052 m above the ground, while the collision spheres had 0.022 m radii. The resulting penetration made those initial positions unsuitable as fixed contact targets. With the contact-height targets, goal-height RMSE fell to 2.617 mm. At 10 s and 18 s, MPC terminal foot drift from the fixed target fell from about 18.1 mm to 1.3 mm, and the terminal base-height offset fell from about 8.4 mm to 1.6–1.7 mm.

The CSV contains one expected non-finite pattern: `prediction_error_mm` is `nan` for the first 10 control steps, before the first 20 ms policy interval has elapsed and no previous prediction exists for comparison. The JSON summary filters those samples.

## Timing result and limitation

| Section | Calls | Mean | p95 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| MPC policy request | 1,000 | 18.377 ms | 19.742 ms | 861.782 ms |
| One MRT evaluation | 20,000 | 0.246 ms | 0.719 ms | 3.109 ms |
| WBC solve | 10,000 | 0.634 ms | 0.689 ms | 1.264 ms |
| Complete loop | 10,000 | 3.229 ms | 19.491 ms | 864.948 ms |

The first MPC request includes generated-library initialization. Later 50 Hz policy updates remain close to the 20 ms update period. The simulation requests these policies synchronously inside the 500 Hz loop, so 1,011 of 10,000 iterations exceeded the 2 ms timestep. This does not affect the deterministic headless simulation result, but it is unsuitable for a hardware torque loop.

Before hardware use, the 50 Hz MPC update must run asynchronously from the 500 Hz WBC and actuator loop. A real-time loop should read the latest complete policy without waiting, reject stale policies, pre-initialize generated libraries and enter a safe state on timeout.

## Next work

1. Investigate the remaining roughly 3.18 mm lower-hold height offset.
2. Decouple policy updates from the WBC loop and measure deadline misses again.
3. Select WBC stance feet from the transmitted trot mode schedule.
4. Connect swing-foot references and align them with the MPC policy.
5. Validate stepping in place before translation, stopping and turning.
6. Add state estimation and the Unitree SDK2 interface only after simulation timing and safety gates are established.
