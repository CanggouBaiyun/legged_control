# OCS2 原地 trot 闭环验证

## 控制链

MuJoCo 实测状态经 ROS 2 送入 OCS2 NMPC。C++ MRT 服务对当前策略求值，返回参考状态、接触力和模式。模式 `15` 表示四脚支撑，`9` 表示 FL/RR 支撑，`6` 表示 FR/RL 支撑。仿真按当前模式选择对应的 WBC QP；摆动脚使用起摆时锁定的原地落脚位置和 0.1 m 抬脚轨迹。关节 PD 使用 MPC 关节参考，WBC 与 PD 力矩相加后经过 MuJoCo 执行器限幅。

`--allow-trot` 才启用双足支撑控制。默认运行仍要求模式 `15`，遇到其他模式会停止。`--trot-at 3` 在仿真时间约 3 s 时，通过 ROS 2 `/legged_robot_mpc_mode_schedule` 发送与 `config/ocs2/go2/gait.info` 中 `standing_trot` 相同的时序：`9` 支撑 0.30 s、`15` 支撑 0.05 s、`6` 支撑 0.30 s、`15` 支撑 0.05 s。MPC 在当前策略的终点之后插入新步态，因此实际首次起摆晚于 3 s。ROS 观测消息使用当前策略对应的接触模式。

## 运行

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source /home/bot/Project/ocs2_ws/install/setup.bash
export ROS_DOMAIN_ID=214
ros2 run ocs2_legged_robot_ros legged_robot_ddp_mpc --ros-args \
  -p taskFile:=/home/bot/Project/go2_traditional_locomotion/config/ocs2/go2/task.info \
  -p referenceFile:=/home/bot/Project/go2_traditional_locomotion/config/ocs2/go2/reference.info \
  -p urdfFile:=/home/bot/Project/go2_traditional_locomotion/models/ocs2/go2_mujoco_inertial.urdf
```

等 MPC 提示 `MPC node is ready` 后，终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source /home/bot/Project/ocs2_ws/install/setup.bash
export ROS_DOMAIN_ID=214
ros2 run go2_mpc_bridge mrt_policy_server
```

终端 3：

```bash
cd /home/bot/Project/go2_traditional_locomotion
export ROS_DOMAIN_ID=214
env -u PYTHONPATH /home/bot/miniconda3/envs/go2_control/bin/python \
  examples/mujoco/ocs2_wbc_standing.py \
  --headless --duration 20 --allow-trot --trot-at 3 \
  --csv artifacts/ocs2_wbc_trot_contact.csv
```

## 2026-10-12 结果

无窗口 MuJoCo 完成 20 s、10,000 个 2 ms 步长，仿真没有异常退出。模式 `15/9/6` 分别占 3137/3449/3414 个控制样本。四只脚在摆动阶段的最高足端中心高度为 0.1140–0.1146 m；足端高度相对本地摆动目标的 RMSE 为 6.50–6.61 mm。最大电机力矩 15.02 Nm，限幅步数 0。基座目标高度 RMSE 3.20 mm，MPC 高度跟踪 RMSE 0.039 mm，末端水平位置约为 (-1.14, 1.33) mm。WBC 动力学和接触加速度残差最大值分别为 9.32e-6 和 1.51e-6。

`artifacts/ocs2_wbc_trot_contact.csv` 包含每步模式、基座高度、MPC/WBC 接触力、四脚实际高度、摆动目标高度、MuJoCo 实际地面法向力以及支撑足端中心的水平速度；同名 JSON 保存汇总。当时的完整回归测试为 37 passed、6 warnings。

带窗口检查中两组对角腿规律交替，没有观察到明显擦地、打滑或持续倾斜。该机器当时加载的 NVIDIA 内核模块为 580.173.02，图形库为 580.178.04；用 `__GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1` 可以临时打开窗口，但软件渲染使 9,720/10,000 步的墙钟耗时超过 2 ms。因此带窗口播放只用于观察动作，下面的接触统计来自独立的无窗口运行。

## 实际触地诊断

2026-10-12 的独立 20 s 无窗口运行再次完成 10,000 步，力矩限幅 0 次。每只脚在计划支撑期间，MuJoCo 地面法向力大于 1 N 的样本比例为 99.35%–99.46%。其余样本全部落在接触模式切换前后 2 ms 内；最长连续不足 1 N 为两个采样步，即 4 ms。足端摆动目标高度高于 0.07 m 时，四脚均没有与地面发生接触。单脚最大实际地面法向力为 85.2–85.5 N。

在已经实际触地且连续处于支撑的采样中，足端**中心**水平速度的 95 分位为 6.7–10.9 mm/s。这是用相邻两步足端中心位置估计的运动速度；球形足端可以滚动，因此该数值不是接触点的严格滑移速度。下一阶段如需定量判断摩擦滑移，应计算接触点相对地面的切向速度。

## 2026-10-14 摆腿竖直轨迹对齐

原地 trot 的 WBC 摆腿竖直参考改为两段三次样条：起摆速度 0.2 m/s，中点速度 0，落脚速度 -0.4 m/s，抬脚高度 0.1 m，与当前 OCS2 配置的竖直轨迹参数一致。水平方向仍沿用原有轨迹，落脚点仍锁定在起摆时的位置；这一步没有接入 OCS2 的落脚位置规划。

在终端 1、2 分别启动 MPC 和 MRT 服务后，终端 3 用以下命令运行；三个终端的 `ROS_DOMAIN_ID` 均为 `214`，否则 reset 服务可能不可见：

```bash
cd /home/bot/Project/go2_traditional_locomotion
env -u PYTHONPATH ROS_DOMAIN_ID=214 \
  /home/bot/miniconda3/envs/go2_control/bin/python \
  examples/mujoco/ocs2_wbc_standing.py \
  --headless --duration 20 --allow-trot --trot-at 3 \
  --csv artifacts/ocs2_wbc_trot_swing_consistency.csv
```

该次运行完成 10,000 个 2 ms 仿真步，模式 `15/9/6` 分别占 3147/3449/3404 步，力矩限幅 0 次。与上述 2026-10-12 的独立无窗口运行相比：

| 指标 | 原轨迹 | 竖直轨迹对齐后 |
| --- | ---: | ---: |
| 四脚摆动高度跟踪 RMSE | 6.50–6.61 mm | 5.76–6.36 mm |
| 计划支撑期间实际地面法向力 > 1 N 的样本比例 | 99.35%–99.46% | 四脚均 100% |
| 摆动中段实际触地样本 | 四脚均 0 | 四脚均 0 |
| 已触地连续支撑时足端中心水平速度 p95 | 6.7–10.9 mm/s | 3.1–4.1 mm/s |
| 基座目标高度 RMSE | 3.20 mm | 3.51 mm |
| 最大电机力矩 | 15.02 Nm | 15.41 Nm |
| 超过 2 ms 的控制步数 | 2599 / 10000 | 2415 / 10000 |

摆动高度跟踪和接触指标改善，但基座目标高度 RMSE 增加约 0.31 mm。足端中心速度只是滚动球足的运动代理量，不能直接解释为触地点滑移。控制步超时计数来自两次单独运行，不能据此判定时延性能有稳定改善；本次完整循环平均耗时 3.31 ms，MPC 请求仍在仿真循环中同步等待。完整回归测试为 38 passed、6 warnings，其中包含摆腿轨迹边界条件。汇总见 `artifacts/ocs2_wbc_trot_swing_consistency.json`；`artifacts/` 是本地生成数据目录，不随 Git 提交。

## 已知限制与下一步

- WBC 摆腿竖直参考已按当前 OCS2 配置对齐；水平方向和落脚位置规划尚未对齐。下一步验证低速前进时的摆腿目标、触地和机身跟踪。
- 这次验证的是原地踏步。低速前进、停步、转向、外力恢复、不同地面及接触状态估计尚未验证。
- MPC 策略请求在 500 Hz 仿真循环中同步等待；本次接触诊断运行的 10,000 步里有 2599 步的墙钟耗时超过 2 ms。实机控制前必须把 50 Hz MPC 与高频 WBC 解耦，并加入策略过期和通信丢失保护。
