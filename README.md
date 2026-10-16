# Go2 Traditional Locomotion

[简体中文](README_zh-CN.md)

Model-based locomotion control for the Unitree Go2, using Pinocchio, MuJoCo and QP-based whole-body control. The target architecture follows [legged_control](https://github.com/qiayuanl/legged_control): OCS2 NMPC, WBC, state estimation and a robot hardware interface.

**Current scope:** closed-loop standing, squatting, load transfer, a complete right-front single-leg cycle and in-place trot in MuJoCo, plus initial low-speed forward, ±10° yaw turns and return-to-stance validation. Go2 full-centroidal OCS2 MPC drives mode-dependent WBC through ROS 2 and a C++ MRT policy-query service. Turning foot motion and contact-point tangent speeds are recorded; transient tracking lag and touchdown slip remain to be investigated. State estimation, the Unitree SDK2 interface and hardware safety remain to be implemented.

## Status

| Component | Status |
| --- | --- |
| Floating-base model, joint indexing and limits | Implemented |
| FK, Jacobians, velocity/position IK, damped least squares | Numerically verified |
| CRBA, RNEA, ABA and contact-force mapping | Numerically verified |
| Contact-force QP with OSQP / ProxQP | Verified in offline examples |
| Pinocchio–MuJoCo state and actuator mapping | Verified |
| Four-foot WBC standing and periodic squat | Closed-loop simulation verified |
| Load transfer, foot unloading and configurable stance contacts | Offline and closed-loop simulation verified |
| Swing-foot tracking with quintic position/velocity/acceleration references | Closed-loop simulation verified |
| FR lift, touchdown detection and four-foot support recovery | Complete cycle verified; automated regression test |
| OCS2 full-centroidal NMPC → MRT → WBC, fixed four-foot stance | 20 s closed-loop simulation verified |
| ROS 2 MPC transport and C++ MRT policy query | Implemented and verified |
| Trot gait command, MPC mode schedule and C++ MRT query | Modes 15/9/6 verified in a 20 s closed-loop simulation |
| Mode-aware WBC and latched in-place swing targets | 20 s closed-loop trot; all four feet lifted; vertical swing reference aligned with OCS2 parameters and floor contacts measured |
| Low-speed forward trot | 20 s headless run at 0.03 m/s with a 0.09 m displacement target; forward motion measured for all four feet, contact-point slip still to be checked |
| Trot to four-foot stance | Stance command at 15 s; mode 15 sustained from 16.020 s in a 20 s headless run |
| In-place yaw turn | Initial 20 s headless runs for ±10° targets; final measured +9.77° / -9.79°; left-turn foot XY direction verified, with endpoint errors and touchdown slip transients recorded |
| MPC-derived swing references | Optional `--swing-reference=mpc`; 20 s left/right turns and forward/stop runs verified: +9.91° / −9.86° for ±10° goals, final x 0.09090 m for a 0.09 m goal, actual forward swing for all feet |
| Asynchronous MPC updates | 20 s standing and trot/stop runs completed. Compact poll payloads reduced mean poll time from 2.075 to 0.292 ms in the first comparison; maximum start lateness fell from 3.47 to 1.52 s, with 1.30 s still remaining at the final step. Real-time execution remains pending. See [execution and timing notes](docs/ocs2_async_mpc.md) |
| Paired MRT queries | 20 s trot/stop verified: mean MRT time per step 0.689 → 0.499 ms, mean loop 1.963 ms; maximum start lateness 321 ms, near zero at the final step. Fixed 500 Hz execution remains unverified |
| Explicit OSQP builtin backend | Subsequent 20 s trot/stop: mean WBC 0.743 ms, mean loop 1.906 ms, maximum start lateness 34 ms; 2117/10000 loops still exceeded 2 ms. Solver creation and setup still occur each step. See [timing comparison](docs/ocs2_async_mpc.md) |
| OSQP workspace reuse | Latest 20 s trot/stop: 4 setups and 9996 updates across 10000 steps; mean WBC 0.628 ms, mean loop 1.733 ms, maximum start lateness 7.36 ms. 2118/10000 loops still exceeded 2 ms; fixed 500 Hz remains unverified |
| Asynchronous forward/stop | 20 s with 4 ms policy polling completed: median policy age 28 → 22 ms and moving-phase x RMSE 13.770 → 12.218 mm; synchronous comparison 9.793 mm. Final x 0.090449 m for a 0.09 m goal; mean loop 1.700 ms, maximum start lateness 8.38 ms. Tracking lag and fixed 500 Hz execution remain unresolved. See [forward results](docs/ocs2_forward_trot.md) |
| Asynchronous left/right turns and stop | Separate 20 s runs at ±10° completed with 4 ms polling: final yaw +9.829° / −9.794°, moving-phase target RMSE 2.171° / 2.174°, no torque clipping. Right-turn touchdown contact tangent-speed peak reached 103.88 mm/s; transients remain unresolved. See [turning results](docs/ocs2_trot_yaw.md) |
| Disturbance recovery | Planned |
| SDK2, state estimation and hardware safety | Planned |

The OCS2 controller has completed a 20 s in-place trot and a headless low-speed forward run with measured forward swing for all four feet and a return to four-foot stance. Adding angular-acceleration feedforward improved a 10° yaw-turn experiment from 4.26° to 9.77° at the last recorded sample; a -10° run measured -9.79°. Transient lag remains, with approximately 2.31° goal-tracking RMSE during 8–12 s in both directions. Left-turn foot XY tracking RMSE is 2.87–4.39 mm; loaded contact tangent-speed p95 during 8–12 s is 2.34–2.78 mm/s, with an FR touchdown peak of 149.01 mm/s later in the run. These measurements do not establish slip-free locomotion. The single-leg cycle still uses scripted contact phases; hardware readiness remains unverified. See the [forward trot notes](docs/ocs2_forward_trot.md) and [yaw-turn results](docs/ocs2_trot_yaw.md).

## Architecture

The OCS2 simulation offers two swing-reference sources. The default `planner` uses latched foot trajectories. With `--swing-reference=mpc`, WBC foot position and velocity targets come from the same MRT policy as the joint PD references; foot acceleration feedforward is estimated using the existing future policy query within the same swing phase. This optional path has completed left/right turns and low-speed forward/stop validation, with measured forward swing for all feet. Current-policy tracking errors cannot be compared directly with fixed planner targets because MPC replans from measured states. Synchronous loop timing still exceeds the 2 ms timestep on many steps; real-time execution remains pending. See the [turning comparison](docs/ocs2_trot_yaw.md) and [forward results](docs/ocs2_forward_trot.md).

Implemented scripted-motion simulation path:

```text
Base/foot trajectories + contact-phase logic
       ├──→ Base and swing tasks + stance set ──→ WBC-QP ──→ torque
       └──→ Joint reference IK ──→ Joint PD ───────────────→ feedback
MuJoCo state ──→ State bridge ──→ Pinocchio ──→ WBC-QP / Joint PD
                                                           ↓
                                    torque + feedback → actuator limits
                                                           ↓
                                                         MuJoCo
```

WBC and joint-reference IK use the same base and foot trajectories. Joint PD feedback is added outside the QP, followed by actuator mapping and final torque limits. Gains are selected for this simulation, not copied verbatim from the upstream controller.

Implemented OCS2 standing path:

```text
Height trajectory + fixed stance mode 15
                    ↓
        OCS2 full-centroidal NMPC node
                    ↓ policy topic
        C++ MRT cache and query service
                    ↓
       State, velocity and force references
                    ↓
             WBC + joint PD
                    ↓
                  MuJoCo
```

OCS2 orders the contacts as FL, FR, RL, RR; the default four-foot WBC uses FR, FL, RR, RL. The adapter maps forces by frame name, and tests use distinct per-foot forces to detect ordering mistakes. The adapter, ROS transport and C++ MRT service accept stance modes 15 (four feet), 9 (FL/RR) and 6 (FR/RL). With `--allow-trot`, the simulation selects a WBC for the active stance feet and gives both swing feet latched trajectories. `--trot-at 3` sends the `standing_trot` schedule through the same ROS 2 topic used by the keyboard gait tool. The ROS observation reports the active policy mode. See [in-place trot results](docs/ocs2_trot.md).

## Verified Results

### OCS2–MRT–WBC standing

[ocs2_wbc_standing.py](examples/mujoco/ocs2_wbc_standing.py) evaluates a full-centroidal Go2 MPC policy through the C++ MRT service, converts it to WBC references and applies the resulting torques in MuJoCo. The 20 s reference holds 0.28 m, lowers smoothly to 0.27 m, then returns to 0.28 m. Fixed-foot IK generates matching joint-position targets. The foot-center targets sit 22 mm above the ground, matching the foot collision-sphere radius in this MuJoCo scene. Metrics exclude the first 2 s where stated.

| Metric | Result |
| --- | ---: |
| Simulation steps / timestep | 10,000 / 2 ms |
| Goal-height RMSE / maximum error | 2.617 / 5.936 mm |
| WBC/MuJoCo tracking of MPC height, RMSE / maximum | 0.023 / 0.097 mm |
| MRT one-step prediction RMSE / maximum error | 0.053 / 0.114 mm |
| Maximum motor command / clipped steps | 8.237 Nm / 0 |
| Maximum dynamics / contact residual | 1.99e-13 / 2.02e-15 |
| Mean MPC / WBC total vertical force | 148.721 / 148.514 N |
| Mean total vertical-force difference | 0.392 N |

During the 8–12 s lower hold, the goal was 0.27000 m, the mean MPC height was 0.27317 m and MuJoCo measured 0.27318 m. The WBC tracks the supplied MPC reference closely; the remaining lower-hold offset of about 3.18 mm needs further investigation. For comparison, goal-height RMSE was 6.013 mm with the original base-only target and 7.861 mm when fixed-foot IK used the initially penetrating foot positions.

WBC took 0.634 ms on average and 0.689 ms at p95. One MRT query took 0.246 ms on average; the experiment makes two queries per control step. MPC policy updates took 18.377 ms on average and 19.742 ms at p95, including an 862 ms first initialization. Because policy updates are requested synchronously, 1,011 of 10,000 loops exceeded the 2 ms simulation timestep. The hardware control path must move policy updates off the 500 Hz torque loop and pre-initialize generated libraries before enabling actuation.

See [OCS2 standing integration notes](docs/ocs2_standing.md) for the process layout, commands, data fields and remaining work.

### Standing and squat

The squat uses a 4 cm vertical excursion and a 4 s period. Metrics below are from a 10 s simulation, sampled every control step; height errors exclude the initial 2 s settling interval.

| Metric | Velocity-feedback baseline | Joint-reference PD |
| --- | ---: | ---: |
| Height RMSE | 9.196 mm | 3.350 mm |
| Maximum height error | 14.302 mm | 4.983 mm |
| Maximum motor command magnitude | 7.454 Nm | 7.489 Nm |
| Torque-clipped steps | 0 | 0 |

Joint damping, friction loss and contact settings were retained. Additional runs covered 30 s operation, a 3 s period and a 6 cm excursion. These tests do not establish disturbance robustness.

See [squat tracking notes](docs/squat_tracking.md) for the changes, test conditions and remaining limitations.

### Three-foot support and swing-foot task

With FR excluded from support, the nominal static load on RL is only about 0.84 N. Moving the base 3 cm backward and 3 cm left, while keeping foot positions fixed through IK, changes the vertical-force-only static solution to approximately:

```text
FL: 63.07 N    RR: 60.04 N    RL: 26.06 N
```

The three-foot WBC optimizes 39 variables: 18 generalized accelerations, 9 contact-force components and 12 motor torques. Adding a swing-foot holding task removes the large free-leg accelerations observed without that task.

In the latest offline upward-response check, a 3 cm FR position offset with foot Kp=100 produces a 3 m/s² acceleration target:

```text
Achieved FR vertical acceleration: 2.99986 m/s²
Dynamics residual norm:           2.31e-14
Stance acceleration residual:     1.31e-15
Maximum motor torque utilization: 27.35%
```

This offline result checks acceleration at one state. Closed-loop lift and touchdown are evaluated separately below.

### Single-leg lift and touchdown

[wbc_single_leg_cycle.py](examples/mujoco/wbc_single_leg_cycle.py) runs a 20 s sequence on flat ground:

```text
Stand → shift base backward/left → unload FR → lift → hold
      → lower → confirm touchdown → restore four-foot support
```

The base target shifts by [-0.03, 0.03, 0] m. FR follows a 3 cm upward reference over 2 s, holds, then lowers over 2 s. Touchdown requires normal force above 1 N for about 20 ms and a foot-frame height within 6 mm of the ground reference. Once confirmed, the reference is anchored at the measured foot position and the FR normal-force upper bound increases over 2 s. These are simulation-specific transition settings, not hardware safety thresholds.

Representative 20 s run (base height/tilt metrics exclude the first 2 s):

| Metric | Result |
| --- | ---: |
| Base height RMSE / maximum error | 1.969 / 3.425 mm |
| Maximum base tilt | 0.775° |
| Maximum motor command magnitude | 11.419 Nm |
| Confirmed touchdown time | 14.584 s |
| FR normal force at 19 s | 24.725 N |
| QP failures / torque-clipped steps | 0 / 0 |
| Four-foot support recovery | Completed |

At 13 s, FR's frame height was about 0.050 m against a 0.052 m target. The 3 cm value describes the commanded displacement, not exact measured lift or sole clearance. FR contact force dropped to zero during swing, and contact count changed from four to three and back to four. The base remained at the shifted target after landing.

The [cycle regression test](tests/test_single_leg_cycle.py) checks actual lift-off, swing height error, touchdown timing, sustained final FR loading, base tracking and command limits. Repeated walking, uneven terrain and external-push recovery are not covered.

## Models and Dynamics

Two model sources are retained:

| Model | Use | Total mass |
| --- | --- | ---: |
| Unitree Go2 URDF | Original kinematics/dynamics examples and joint limits | 16.087 kg |
| Unitree Go2 MJCF | MuJoCo simulation and matched Pinocchio simulation model | 15.206408 kg |

The simulation model includes the MJCF inertial parameters and armature. Its free-root placement is normalized so that Pinocchio q[:7] represents the absolute world base pose. Joint limits missing from the parser are supplied by joint-name mapping from the URDF. Joint friction and damping remain in MuJoCo; matching mass and bias terms does not make the complete contact/passive dynamics identical.

WBC uses:

$$
M(q)\dot v+h(q,v)=S^T\tau+J_c^T\lambda
$$

Stance-foot acceleration equalities, friction-pyramid bounds, unilateral contact, normal-force limits and motor-torque limits constrain the solution. Base tracking and swing-foot tracking are weighted objectives. The swing foot is omitted from the stance-force variables; its mass and actuators remain in the robot model. Four-foot and three-foot configurations have 42 and 39 decision variables, respectively.

The stance constraint is acceleration-level; it does not itself correct accumulated foot-position drift. QP residuals describe the optimization model, not MuJoCo tracking accuracy. Real-time execution and hardware safety have not been established.

## Run

Examples assume an installed project environment and model repositories at the paths in [config/go2_model.json](config/go2_model.json). See [setup notes](docs/setup_guide.md) for the existing environment setup.

Recorded numerical environment:

```text
Ubuntu 24.04
Python 3.10
Pinocchio 3.9.0
MuJoCo 3.4.0
OSQP 1.1.3
ProxSuite / ProxQP 0.7
```

The standalone WBC examples do not require ROS. The OCS2 standing experiment uses ROS 2 Jazzy for the MPC policy, reset service and C++ MRT query service. It is a simulation transport, not a robot hardware interface.

From the repository root:

```bash
conda activate go2_control

# MuJoCo demonstrations
python examples/mujoco/wbc_standing.py
python examples/mujoco/wbc_squat.py

# Load transfer, unloading and single-leg demonstrations
python examples/mujoco/wbc_load_shift.py
python examples/mujoco/wbc_unload.py --duration 12
python examples/mujoco/wbc_single_leg_lift.py --duration 14
python examples/mujoco/wbc_single_leg_cycle.py --duration 20

# Headless full cycle with per-step CSV and JSON summary
python examples/mujoco/wbc_single_leg_cycle.py --headless --duration 20 --csv artifacts/single_leg_cycle.csv

# Headless squat comparison and logs
python examples/mujoco/wbc_squat.py --headless --mode baseline --csv artifacts/squat_baseline.csv
python examples/mujoco/wbc_squat.py --headless --csv artifacts/squat_tracking.csv

# OCS2 standing requires the MPC and MRT nodes described in
# docs/ocs2_standing.md, with the same ROS_DOMAIN_ID in every terminal.
export ROS_DOMAIN_ID=213
env -u PYTHONPATH \
  /home/bot/miniconda3/envs/go2_control/bin/python \
  examples/mujoco/ocs2_wbc_standing.py \
  --headless --duration 20 \
  --csv artifacts/ocs2_wbc_standing_ground_feet.csv

# Offline three-foot checks
python examples/wbc/three_contact_kinematics.py
python examples/wbc/three_contact_load_shift.py
python examples/wbc/three_contact_wbc.py

# Regression tests
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_single_leg_cycle.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

Latest full regression after repository cleanup and configuration-path consolidation (2026-10-15): **38 passed, 6 warnings in 11.06 s**. Tests include MPC/WBC contact-order conversion, MRT transport boundaries, quintic and OCS2 vertical swing trajectory checks, and a headless 20 s single-leg cycle. Live ROS 2 MPC/MRT integration is verified through separate runs; asynchronous in-place trot/stop and forward/stop runs completed 20 s each. A comparison using the current synchronous code found greater moving-phase lag with 20 ms asynchronous polling (x RMSE 13.770 versus 9.793 mm); a subsequent 20 s run with 4 ms polling reduced asynchronous RMSE to 12.218 mm. Asynchronous left/right turn and stop runs additionally completed 20 s each (final yaw +9.829° / −9.794°). Touchdown transients and fixed 500 Hz execution remain unresolved. The MPC swing-reference path previously completed separate synchronous 20 s left-turn, right-turn and forward/stop runs. The synchronous OCS2 experiment does not intentionally sleep to match wall time; optional `--async-mpc` execution uses wall-clock pacing even in headless mode. Dependency deprecation warnings for hppfcl/coal and OSQP are separate from test failures.

Cycle CSV logs include FR desired/actual frame height and actual normal contact force. Summary height errors refer to the base; `maximum_xy_drift_mm` measures displacement from the world origin, including the intentional base shift, not target-tracking error.

## Roadmap

Per-step phase timing and nested MRT timing have both completed 20 s forward/stop runs. In the latest run, excluding the first 2 s, mean/p99 loop duration was 1.762/2.971 ms and 2008/9000 steps exceeded 2 ms. The two C++ evaluations averaged 4.52 μs combined; activation averaged 0.539 ms on steps activating a new policy. The ROS round trip minus the C++ callback peaked at 7.056 ms; this residual includes queuing, serialization and scheduling and does not measure pure network latency. See [control-loop timing analysis](docs/ocs2_control_timing.md).

After reviewing upstream and local implementations, migration will proceed through model/coordinate and build checks, offline C++ WBC comparison, local MRT/controller integration, and a C++ MuJoCo loop. Initially retain the external DDP/SLQ MPC process and the validated WBC formulation. See the [C++ control migration plan](docs/roadmap.md) for differences, acceptance criteria and estimates.

The shared C++ model/dynamics implementation is integrated into the workspace and passed three Python/C++ comparisons including tilted poses and nonzero velocities: maximum absolute M/h/J/Jdot·v errors were approximately 5.27e-17/8.88e-16/5.55e-17/7.81e-18; motor limits and armature matched. See the [C++ model baseline](docs/cpp_model_baseline.md).

The native C++ WBC is merged and passed seven Python comparisons covering four-foot and both diagonal contact sets, nonzero motion and normal-force limits, with maximum torque difference about 6.56e-12 N·m. The `feature/cpp-control-loop` branch now connects local MRT, centroidal/reference conversion, WBC, joint PD and MuJoCo C API in one control process. Python prepares the existing IK target before the run and summarizes results afterward; there are no per-step Python pipe or MRT service calls. The MPC remains an external DDP/SLQ process. See the [C++ implementation and run instructions](docs/cpp_model_baseline.md#本地-mrt-与-c-mujoco-闭环2026-10-15).

```bash
bash scripts/build_cpp_control.sh
bash scripts/run_cpp_simulation.sh \
  --headless --duration 20 --trot-at 3 --stop-trot-at 15 --forward \
  --csv artifacts/cpp_control_forward.csv
```

The launcher manages its own MPC on ROS domain 216 by default and writes CSV/JSON results. Remove `--headless` to open the official MuJoCo 3.4.0 Simulate passive viewer with both sidebars, mouse camera controls and visualization menus. The main thread handles the UI, while a C++ worker advances the controlled simulation and synchronizes snapshots. The window is created after MPC reset and the first policy are ready; service-wait progress is printed every 5 s. The latest local official-viewer forward/stop run completed 20 s (2026-10-16, final x=0.090628 m, no torque clips). UI synchronization adds timing overhead; see the baseline report for separate measurements. Reset/keyframe loading requires restarting the run to reset MPC consistently.

The current SQP alignment branch adds `--mpc-solver ddp|sqp` to the launcher (default: `ddp`), checks the selected executable and prints the solver at startup. The controller reports the active policy type and records it in CSV/JSON. WBC motion references and acceleration differences use the nominal policy trajectory; current feedback inputs supply contact forces and joint-PD velocity references. SQP standing and in-place trot/stop each completed 20 s / 10,000 steps with linear policies and no policy rejections or torque clips. MPC-height tracking RMSE was 0.049/0.104 mm and loop p99 was 0.402/0.429 ms, respectively, with 0/9000 loops over 2 ms in each run after excluding the first 2 s. Trot returned to sustained mode 15 at 16.020 s; all four feet lifted. SQP forward/stop also completed 20 s / 10,000 steps: final x=0.090041 m for a 0.09 m goal, no rejections or torque clips, loop p99=0.563 ms and 0/9000 loops over 2 ms. Moving-phase goal-x RMSE remains 11.730 mm; SQP turning validation is pending. The earlier measurements below use DDP/SLQ. See the [SQP results](docs/cpp_model_baseline.md#sqp-线性反馈策略站立验证2026-10-16) and [current plan](docs/roadmap.md).

The WBC alignment branch adds `--joint-kp KP --joint-kd KD` (defaults: 20 and 1) for controlled gain comparisons. Actual gains are printed and recorded in CSV/JSON; a 20 s SQP standing run with Kp=0, Kd=3 completed, but startup height peaked at 0.3367 m and maximum roll/pitch reached 6.32°. A subsequent 20 s standing run at 0/1 reduced peak height to 0.2832 m and maximum roll/pitch to 1.23°, with no torque clips or policy rejections. The default remains 20/1. The optional `--centroidal-base-accel` task now computes its reference from momentum-rate dynamics and converts it to measured free-flyer coordinates. CSV/JSON record the active task source. Its 20 s SQP standing run at 0/1 completed with no policy rejections or torque clips: maximum height 0.2808 m, maximum roll/pitch 0.593°, and maximum torque 7.557 N·m. The first trot run stopped on a WBC constraint check. After explicitly tightening OSQP absolute/relative tolerances to 1e-6, a full 20 s trot/stop run completed with no rejections or torque clips; loop p99/max was 0.655/1.228 ms. A subsequent 20 s forward/stop run completed with final x=0.090773 m for a 0.09 m goal, no rejections or torque clips, and loop p99/max=0.596/1.502 ms. Moving-phase goal-x RMSE remains 9.377 mm. Turning, remaining upstream task differences and extreme-QP solution sensitivity remain unresolved.

Native standing, forward/stop and left/right turn runs each completed 20 s / 10,000 steps. Final forward x was 0.090654 m (target 0.09 m), and final yaw was +9.827° / −9.788°. Excluding the first 2 s, the forward loop averaged 0.199 ms with p99 0.470 ms and 0/9000 steps over 2 ms; maximum start lateness was 1.140 ms. No run clipped torques. These short headless runs do not establish hard real-time or hardware readiness.

Nested timing passed a bridge build, 12 transport tests, isolated service checks and the full closed-loop run above. There were no cache retries or torque clips; final x was 0.090636 m against a 0.09 m target. These timing intervals are nested and must not be added together. The service response type changed; use a rebuilt and restarted MRT node as documented above.

Hardware context (2026-10-15, user reported): a modified Go2 Air with existing SDK2 low-level state-reading and joint-command capability. This project has not yet integrated or independently verified that interface. Sensor availability, joint mapping, control modes, communication-failure behavior and control ownership switching require validation; the deployment compute platform remains to be confirmed.

1. Finish visual acceptance and merge the native C++ loop; validate timing over longer runs and under load, including start lateness and rendering overhead.
2. Compare upstream SQP and WBC task definitions against the retained DDP/SLQ and validated WBC formulation, changing one major component at a time.
3. Extend scripted motions to online ROS 2 velocity, heading and gait commands. Validate forward/backward, lateral and combined motion and repeated starts/stops; consolidate launch entry points and dependency/configuration versions. Compare the current DDP/SLQ and upstream SQP paths separately, keeping changes to major modules isolated.
4. Improve dynamic target lag and stopping touchdown transients by checking landing position, velocity and contact-switch consistency. Test disturbance recovery with defined force pulses and record application point, magnitude, duration, recovery time and constraint violations.
5. Add IMU, joint-state and contact inputs and validate state estimation against simulation truth. Gradually replace ground-truth base position/velocity inputs and evaluate noise, bias and incorrect contact detection.
6. Confirm Go2 low-level interface access, compute platform and SDK2 communication. Complete read-only acquisition, joint direction/unit checks, torque/attitude limits, watchdogs, control ownership switching and emergency-stop handling before staged protected hardware standing, in-place trot and low-speed motion.

Additional gaits follow a stable basic trot. Neither arbitrary-push recovery nor immediate sim-to-real deployment is claimed.

## Repository Layout

```text
config/                        Model JSON and OCS2 task/reference/gait files
docs/                          Setup, derivations and experiment reports
examples/pinocchio/             Kinematics and dynamics examples
examples/optimization/          Contact-force optimization
examples/wbc/                   Offline standing and three-foot WBC
examples/mujoco/                Mapping, standing, squat and single-leg cycles
examples/ocs2/                  MPC transport and policy diagnostics
ros2/go2_control_cpp/           C++ model, WBC, local MRT and MuJoCo control entry
ros2/go2_mpc_bridge/            C++ MRT policy query service
launch/                        OCS2 dummy/RViz launch entry
src/go2_control/wbc/            Dynamics, kinematics, references and WBC
src/go2_control/simulation/     MuJoCo–Pinocchio bridge
src/go2_control/optimization/   QP solver wrappers
tests/                         Numerical and closed-loop regression tests
scripts/                       Environment and model-version utilities
tools/                         Inspection tools
third_party/                   Unitree models and pinned official MuJoCo Simulate sources
artifacts/                     Generated logs and reports
```

The development plan is maintained in [docs/roadmap.md](docs/roadmap.md). Open [go2_control.code-workspace](go2_control.code-workspace) for C++ editing with per-package compilation databases. Learning examples retain the complete variants: `foot_position_ik_joint_limits.py` for IK and `contact_force_qp_friction.py` for contact-force QP; controllers and tests use the shared `src/go2_control` modules.

## Milestones

```text
v0.1.0-pinocchio-fk-baseline
v0.2.0-pinocchio-kinematics
v0.3.0-pinocchio-dynamics
v0.4.0-wbc-standing-offline
v0.5.0-mujoco-standing-squat
v0.6.0-mujoco-single-leg-cycle
```

`v0.6.0-mujoco-single-leg-cycle` contains the verified lift–touchdown cycle and its regression test. Fixed-stance OCS2–MRT–WBC integration was merged from `feature/ocs2-integration`; 20 s in-place trot is verified on `feature/ocs2-trot`.

## License

Original code and documentation in this project are licensed under the [MIT License](LICENSE). Copyright (c) 2026 CanggouBaiyun.

Third-party code, dependencies, robot models, meshes and other assets remain subject to their respective licenses and copyright notices. This includes content under `third_party/` and files derived from upstream sources, such as the exported Unitree URDF under `models/ocs2/`. The project license does not replace upstream license terms or attribution requirements.

The vendored official MuJoCo Simulate sources use Apache-2.0; their LodePNG dependency retains its zlib-style license. See [source versions and notices](third_party/mujoco_simulate/UPSTREAM.md).
