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
  --csv artifacts/ocs2_wbc_trot_first_run.csv
```

## 2026-10-12 结果

无窗口 MuJoCo 完成 20 s、10,000 个 2 ms 步长，仿真没有异常退出。模式 `15/9/6` 分别占 3137/3449/3414 个控制样本。四只脚在摆动阶段的最高足端中心高度为 0.1140–0.1146 m；足端高度相对本地摆动目标的 RMSE 为 6.50–6.61 mm。最大电机力矩 15.02 Nm，限幅步数 0。基座目标高度 RMSE 3.20 mm，MPC 高度跟踪 RMSE 0.039 mm，末端水平位置约为 (-1.14, 1.33) mm。WBC 动力学和接触加速度残差最大值分别为 9.32e-6 和 1.51e-6。

`artifacts/ocs2_wbc_trot_first_run.csv` 包含每步模式、基座高度、MPC/WBC 接触力、四脚实际高度和摆动目标高度；同名 JSON 保存汇总。完整回归测试为 37 passed、6 warnings。

## 已知限制与下一步

- 当前 WBC 用零端点速度的五次轨迹，OCS2 在 `task.info` 中使用起摆 0.2 m/s、落脚 -0.4 m/s 的样条；两者尚未精确对齐。下一步先核对摆动末端和触地瞬间的实际接触力，再统一轨迹定义。
- 这次验证的是原地踏步。低速前进、停步、转向、外力恢复、不同地面及接触状态估计尚未验证。
- MPC 策略请求在 500 Hz 仿真循环中同步等待；10,000 步里有 2378 步的墙钟耗时超过 2 ms。实机控制前必须把 50 Hz MPC 与高频 WBC 解耦，并加入策略过期和通信丢失保护。
