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
  --csv artifacts/ocs2_wbc_standing.csv
```

`PYTHONPATH` is removed only for the Conda Python process because the sourced ROS Jazzy environment exposes Python 3.12 packages, while `go2_control` uses Python 3.10. The ROS subprocess separately loads the Jazzy and OCS2 environments.

## Verified 20 s result

The test used a 2 ms MuJoCo timestep and recorded 10,000 samples. The commanded base height held 0.28 m, transitioned to 0.27 m, held, and returned to 0.28 m.

| Metric | Result |
| --- | ---: |
| Goal-height RMSE / maximum error, after 2 s | 6.013 / 10.776 mm |
| Measured tracking of MPC height, RMSE / maximum | 0.015 / 0.065 mm |
| MRT one-step prediction RMSE / maximum error | 0.035 / 0.079 mm |
| Maximum motor command / feedback magnitude | 8.200 / 2.340 Nm |
| Torque-clipped steps | 0 |
| Maximum dynamics residual | 1.71e-13 |
| Maximum contact-acceleration residual | 2.43e-15 |
| Minimum policy horizon | 0.982 s |
| Mean MPC / WBC total vertical force | 148.967 / 148.792 N |
| Mean absolute total-force difference | 0.266 N |

During the lower hold from 8 to 12 s, the goal was 0.27000 m, the mean MPC reference was 0.27914 m and the measured mean was 0.27916 m. The measured system therefore followed the MPC reference closely; tuning or correcting the MPC target behavior is the next height-control task.

The CSV contains one expected non-finite pattern: `prediction_error_mm` is `nan` for the first 10 control steps, before the first 20 ms policy interval has elapsed and no previous prediction exists for comparison. The JSON summary filters those samples.

## Timing result and limitation

| Section | Calls | Mean | p95 | Maximum |
| --- | ---: | ---: | ---: | ---: |
| MPC policy request | 1,000 | 17.830 ms | 19.405 ms | 611.041 ms |
| One MRT evaluation | 20,000 | 0.255 ms | 0.719 ms | 1.494 ms |
| WBC solve | 10,000 | 0.640 ms | 0.688 ms | 2.879 ms |
| Complete loop | 10,000 | 3.203 ms | 19.211 ms | 616.252 ms |

The first MPC request includes generated-library initialization. Later 50 Hz policy updates remain close to the 20 ms update period. The simulation requests these policies synchronously inside the 500 Hz loop, so 1,006 of 10,000 iterations exceeded the 2 ms timestep. This does not affect the deterministic headless simulation result, but it is unsuitable for a hardware torque loop.

Before hardware use, the 50 Hz MPC update must run asynchronously from the 500 Hz WBC and actuator loop. A real-time loop should read the latest complete policy without waiting, reject stale policies, pre-initialize generated libraries and enter a safe state on timeout.

## Next work

1. Trace and correct the MPC height-target offset.
2. Decouple policy updates from the WBC loop and measure deadline misses again.
3. Replace fixed mode 15 with a trot mode schedule.
4. Generate swing-foot position, velocity and acceleration references from the schedule.
5. Validate stepping in place before translation, stopping and turning.
6. Add state estimation and the Unitree SDK2 interface only after simulation timing and safety gates are established.
