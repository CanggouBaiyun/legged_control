# Go2 Traditional Locomotion

[English](README.md)

面向 Unitree Go2 的模型运动控制项目，使用 Pinocchio、MuJoCo 和基于 QP 的全身控制。目标架构参考 [legged_control](https://github.com/qiayuanl/legged_control)，包含 OCS2 NMPC、WBC、状态估计和机器人硬件接口。

**当前范围：** 已完成 MuJoCo 站立、蹲起、移重、右前腿完整抬落脚循环和原地 trot 闭环，并完成低速前进、±10° 偏航转向及切回四脚站立的初步仿真验证。Go2 全质心 OCS2 MPC 通过 ROS 2 和 C++ MRT 策略查询服务驱动按接触模式切换的 WBC。已记录转向二维足端运动及接触点切向速度，动态滞后和触地滑移瞬态待分析；状态估计、Unitree SDK2 接口及实机安全待实现。

## 当前状态

| 模块 | 状态 |
| --- | --- |
| 浮动基模型、关节索引和限制 | 已实现 |
| 正运动学、Jacobian、速度/位置 IK、阻尼最小二乘 | 已完成数值验证 |
| CRBA、RNEA、ABA 和接触力映射 | 已完成数值验证 |
| OSQP / ProxQP 接触力优化 | 已完成离线示例验证 |
| Pinocchio–MuJoCo 状态与执行器映射 | 已验证 |
| 四足 WBC 站立和周期蹲起 | 已完成闭环仿真验证 |
| 移重、足端卸载与可配置支撑脚 | 已完成离线及闭环仿真验证 |
| 五次多项式位置、速度、加速度参考与摆动脚跟踪 | 已完成闭环仿真验证 |
| FR 抬升、触地确认与四足承重恢复 | 完整循环已验证，已加入自动回归测试 |
| OCS2 全质心 NMPC → MRT → WBC 固定四足站立 | 已完成 20 s 闭环仿真验证 |
| ROS 2 MPC 通信与 C++ MRT 策略查询 | 已实现并验证 |
| trot 命令、MPC 模式时间表与 C++ MRT 查询 | 20 s 闭环仿真已运行模式 15/9/6 |
| 按模式切换 WBC 和锁定原地摆动目标 | 20 s 原地 trot 闭环验证；四脚均抬升，摆腿竖直参考已按 OCS2 参数对齐并核对实际触地 |
| 低速前进 trot | 0.03 m/s、目标位移 0.09 m 的 20 s 无窗口验证；四脚实际向前摆动已量化，触地点滑移待检查 |
| 从 trot 切回四脚站立 | 15 s 发指令，16.020 s 起持续模式 15；20 s 无窗口验证 |
| 原地 trot 偏航转向 | ±10° 各完成初步 20 s 无窗口验证，最后实测 +9.77° / -9.79°；左转四脚二维方向已核对，末端误差和触地滑移瞬态已记录 |
| MPC 派生摆腿参考 | 可选 `--swing-reference=mpc`；左转、右转及前进停步各完成 20 s，±10° 目标实测 +9.91° / −9.86°，前进目标 0.09 m 最终 x 0.09090 m，四脚实际前摆已量化 |
| MPC 异步更新 | 站立及 trot 停步各完成 20 s；精简传输首次对比中 poll 平均耗时由 2.075 降至 0.292 ms，最大累计迟到由 3.47 降至 1.52 s，末步仍迟到 1.30 s，实时执行仍待解决，详见[执行与计时说明](docs/ocs2_async_mpc.md) |
| MRT 双时刻查询 | 20 s trot 停步验证完成，每步 MRT 平均 0.689 → 0.499 ms，循环平均 1.963 ms；最大累计迟到 321 ms，结束时基本追回，仍未达到稳定 500 Hz |
| 显式指定 OSQP builtin 后端 | 后续 20 s trot 停步运行：WBC 平均 0.743 ms，循环平均 1.906 ms，最大启动迟到 34 ms；仍有 2117/10000 步超过 2 ms，每步仍创建和初始化求解器，详见[计时对比](docs/ocs2_async_mpc.md) |
| OSQP 工作区复用 | 最新 20 s trot 停步运行：10000 步共 4 次 setup、9996 次 update；WBC 平均 0.628 ms，循环平均 1.733 ms，最大启动迟到 7.36 ms；仍有 2118/10000 步超过 2 ms，尚未达到稳定 500 Hz |
| 异步前进与停步 | 4 ms 策略检查完成 20 s：策略年龄中位数 28 → 22 ms，移动阶段 x RMSE 13.770 → 12.218 mm，同步对照 9.793 mm；最终 x 为 0.090449 m（目标 0.09 m），循环平均 1.700 ms、最大启动迟到 8.38 ms，动态滞后及稳定 500 Hz 仍待解决，详见[前进记录](docs/ocs2_forward_trot.md) |
| 异步左右转与停步 | 4 ms 策略检查完成 ±10° 各 20 s：最终偏航 +9.829° / −9.794°，动态目标 RMSE 2.171° / 2.174°，无力矩限幅；右转停步触地切向速度峰值 103.88 mm/s，瞬态仍待改进，详见[转向记录](docs/ocs2_trot_yaw.md) |
| 抗扰恢复 | 计划中 |
| SDK2、状态估计和实机安全 | 计划中 |

OCS2 控制器已完成 MuJoCo 无窗口 20 s 原地 trot、低速前进、逐足前摆和切回四脚站立的验证。加入角加速度前馈后，目标 10° 转向的最后实测由 4.26° 提升至 9.77°；目标 -10° 的右转最后实测 -9.79°。动态滞后仍存在，左右转 8–12 s 的目标跟踪 RMSE 均约 2.31°。左转四脚二维跟踪 RMSE 为 2.87–4.39 mm；8–12 s 承重接触点切向速度 p95 为 2.34–2.78 mm/s，后续一次 FR 触地峰值为 149.01 mm/s，不能宣称全程无滑移。单腿循环仍使用脚本接触阶段，实机部署能力尚未验证。详见[前进 trot 记录](docs/ocs2_forward_trot.md)和[转向记录](docs/ocs2_trot_yaw.md)。

## 控制架构

OCS2 仿真支持两种摆腿参考来源：默认 `planner` 使用锁定足端轨迹；`--swing-reference=mpc` 则从与关节 PD 相同的 MRT 策略生成 WBC 足端位置、速度参考，并利用已有未来策略查询估计同一摆动阶段内的足端加速度前馈。可选 MPC 路径已完成左转、右转及低速前进停步，并量化四脚实际前摆。由于 MPC 根据实测状态重新规划，当前策略跟踪误差不能与固定规划器目标直接等价比较。同步循环仍有较多步骤超过 2 ms，实时执行待解决。详见[转向对照](docs/ocs2_trot_yaw.md)和[前进记录](docs/ocs2_forward_trot.md)。

已实现的脚本动作仿真路径：

```text
基座/足端轨迹 + 接触阶段管理
       ├──→ 基座、摆动脚任务与支撑脚集合 ──→ WBC-QP ──→ 前馈力矩
       └──→ 关节参考 IK ──→ 关节 PD ─────────────────→ 反馈力矩
MuJoCo 状态 ──→ 状态映射 ──→ Pinocchio ──→ WBC-QP / 关节 PD
                                                        ↓
                                      力矩叠加 → 执行器映射与限幅
                                                        ↓
                                                      MuJoCo
```

WBC 和关节参考 IK 使用同一组基座、足端轨迹。关节 PD 在 QP 外叠加，之后执行电机映射和最终力矩限幅。增益针对当前仿真选取，并非照搬原工程。

已实现的 OCS2 站立路径：

```text
高度轨迹 + 固定四足模式 15
             ↓
      OCS2 全质心 NMPC 节点
             ↓ ROS 2 策略
      C++ MRT 缓存与查询服务
             ↓
     状态、速度和接触力参考
             ↓
         WBC + 关节 PD
             ↓
           MuJoCo
```

OCS2 足端顺序为 FL、FR、RL、RR，默认四足 WBC 顺序为 FR、FL、RR、RL。适配器按足端名称转换接触力，测试使用每只脚不同的力检查顺序错误。适配器、ROS 传输层和 C++ MRT 服务支持模式 15（四脚支撑）、9（FL/RR）和 6（FR/RL）。启用 `--allow-trot` 后，仿真按当前支撑脚选择 WBC，并向两只摆动脚提供锁定的轨迹目标。`--trot-at 3` 通过与键盘步态工具相同的 ROS 2 话题发送 `standing_trot` 时序；ROS 观测消息也会报告当前策略模式。详见[原地 trot 验证记录](docs/ocs2_trot.md)。

## 验证结果

### OCS2–MRT–WBC 站立

[ocs2_wbc_standing.py](examples/mujoco/ocs2_wbc_standing.py) 通过 C++ MRT 服务查询 Go2 全质心 MPC 策略，将其转换为 WBC 参考，并在 MuJoCo 中施加求解力矩。20 s 参考轨迹从 0.28 m 平滑下降至 0.27 m，再回到 0.28 m。固定足端 IK 同时生成匹配的关节位置目标；足端中心目标位于地面上方 22 mm，与当前 MuJoCo 场景的足端碰撞球半径一致。注明的误差统计排除最初 2 s。

| 指标 | 结果 |
| --- | ---: |
| 仿真步数 / 步长 | 10,000 / 2 ms |
| 目标高度 RMSE / 最大误差 | 2.617 / 5.936 mm |
| WBC/MuJoCo 跟踪 MPC 高度 RMSE / 最大误差 | 0.023 / 0.097 mm |
| MRT 单步预测 RMSE / 最大误差 | 0.053 / 0.114 mm |
| 最大电机命令 / 力矩限幅次数 | 8.237 Nm / 0 |
| 最大动力学 / 接触约束残差 | 1.99e-13 / 2.02e-15 |
| MPC / WBC 平均竖直合力 | 148.721 / 148.514 N |
| 平均竖直合力差 | 0.392 N |

在 8–12 s 的低位保持阶段，目标为 0.27000 m，MPC 平均高度为 0.27317 m，MuJoCo 实测平均高度为 0.27318 m。WBC 能紧密跟踪 MPC 参考；剩余约 3.18 mm 的低位偏差仍需分析。作为对比，原先只有基座高度目标时，目标高度 RMSE 为 6.013 mm；直接固定初始穿地足端位置的 IK 版本为 7.861 mm。

WBC 平均耗时 0.634 ms，p95 为 0.689 ms；单次 MRT 查询平均 0.246 ms，每个控制周期查询两次。MPC 策略更新平均 18.377 ms、p95 为 19.742 ms，首次初始化约 862 ms。当前策略更新同步阻塞控制循环，10,000 次循环中有 1,011 次超过 2 ms 仿真步长。实机前必须将 MPC 更新移出 500 Hz 力矩循环，并在使能力矩前预热生成库。

进程关系、运行命令、数据字段和剩余工作见 [OCS2 站立集成记录](docs/ocs2_standing.md)。

### 站立与蹲起

蹲起深度为 4 cm、周期为 4 s。下表来自 10 s 仿真，每个控制周期采样；高度误差统计排除最初 2 s 的初始化阶段。

| 指标 | 原速度反馈基线 | 关节参考 PD |
| --- | ---: | ---: |
| 高度 RMSE | 9.196 mm | 3.350 mm |
| 最大高度误差 | 14.302 mm | 4.983 mm |
| 最大电机命令力矩 | 7.454 Nm | 7.489 Nm |
| 力矩限幅次数 | 0 | 0 |

保留了原有的关节阻尼、干摩擦和接触设置。补充验证覆盖 30 s 连续运行、3 s 周期和 6 cm 深度；这些测试不等于已经验证抗扰能力。

修改内容、测试条件和剩余问题见 [蹲起跟踪记录](docs/squat_tracking.md)。

### 三足支撑与摆动脚任务

右前脚 FR 不参与支撑时，名义姿态下左后脚 RL 的静态承重只有约 0.84 N。通过 IK 保持足端位置不变，将基座向后、向左各移动 3 cm 后，仅考虑竖直力的静态解约为：

```text
FL：63.07 N    RR：60.04 N    RL：26.06 N
```

三足 WBC 联合优化 39 个变量：18 个广义加速度、9 个接触力分量和 12 个电机力矩。加入摆动脚保持任务后，消除了未指定自由腿任务时出现的大幅关节加速现象。

最近一次离线向上响应检查中，FR 目标上移 3 cm，足端 Kp=100，对应期望加速度 3 m/s²：

```text
实际 FR 竖直加速度：      2.99986 m/s²
动力学残差范数：         2.31e-14
支撑脚加速度残差：       1.31e-15
最大电机力矩利用率：     27.35%
```

这项离线结果只检查单个状态下的加速度，闭环抬落脚结果见下节。

### 单腿抬升与落脚

[wbc_single_leg_cycle.py](examples/mujoco/wbc_single_leg_cycle.py) 在平地完成 20 s 动作：

```text
站立 → 身体向左后移重 → FR 卸载 → 抬升 → 保持
     → 下降 → 确认触地 → 恢复四足承重
```

基座目标平移 [-0.03, 0.03, 0] m。FR 期望轨迹在 2 s 内抬高 3 cm，保持后再用 2 s 下降。触地判据为法向力超过 1 N、持续约 20 ms，且足端坐标系高度距离地面参考不足 6 mm。确认后将期望位置固定在实测触地点附近，并在 2 s 内逐步增加 FR 法向力上限。这些是仿真实验的切换参数，不是真机安全阈值。

一次 20 s 运行结果如下；身体高度和倾斜统计排除最初 2 s：

| 指标 | 结果 |
| --- | ---: |
| 身体高度 RMSE / 最大误差 | 1.969 / 3.425 mm |
| 最大身体倾斜 | 0.775° |
| 最大电机命令力矩 | 11.419 Nm |
| 确认触地时间 | 14.584 s |
| 19 s 时 FR 实际法向力 | 24.725 N |
| QP 失败 / 力矩限幅次数 | 0 / 0 |
| 四足承重恢复 | 已完成 |

13 s 时 FR 坐标系高度约为 0.050 m，目标为 0.052 m。3 cm 指期望抬升量，不是精确实测抬升量，也不直接等于脚底离地间隙。摆动期间 FR 接触力降为零，接触脚数由四变三、落地后恢复为四。落脚后身体仍保持在移重后的目标位置。

[完整循环回归测试](tests/test_single_leg_cycle.py) 检查实际离地、摆动高度误差、触地时刻、最终持续承重、身体跟踪和命令限幅情况。尚未覆盖连续行走、不平地面或外部推力恢复。

## 模型与动力学

项目保留两套模型来源：

| 模型 | 用途 | 总质量 |
| --- | --- | ---: |
| Unitree Go2 URDF | 原有运动学、动力学示例及关节限制 | 16.087 kg |
| Unitree Go2 MJCF | MuJoCo 仿真及同源 Pinocchio 仿真模型 | 15.206408 kg |

仿真控制模型使用 MJCF 的惯性参数和 armature，并归一化自由根关节固定变换，使 Pinocchio 的 q[:7] 表示世界系绝对基座位姿。解析器未提供的关节限制通过名称映射从 URDF 补入。MuJoCo 中仍保留关节摩擦和阻尼；质量矩阵、偏置力一致不代表全部接触与被动力学完全一致。

WBC 使用完整浮动基动力学：

$$
M(q)\dot v+h(q,v)=S^T\tau+J_c^T\lambda
$$

支撑足加速度等式、摩擦金字塔、单边接触、法向力范围和电机力矩限制约束求解结果。基座跟踪与摆动脚跟踪采用加权目标。摆动脚不进入支撑力变量，但其质量和电机仍保留在机器人模型中。四足和三足配置分别使用 42 和 39 个决策变量。

当前支撑约束处于加速度层，不会自行消除累计足端位置漂移。QP 残差反映优化模型约束满足程度，不等于 MuJoCo 实际跟踪精度。实时运行能力和真机安全尚未验证。

## 运行

示例依赖已安装的项目环境，以及 [config/go2_model.json](config/go2_model.json) 指定位置下的模型仓库。现有环境安装记录见 [安装说明](docs/setup_guide.md)。

已记录的数值计算环境：

```text
Ubuntu 24.04
Python 3.10
Pinocchio 3.9.0
MuJoCo 3.4.0
OSQP 1.1.3
ProxSuite / ProxQP 0.7
```

独立 WBC 示例不依赖 ROS。OCS2 站立实验使用 ROS 2 Jazzy 传输 MPC 策略、reset 服务和 C++ MRT 查询服务；这仍是仿真通信链路，不是机器人硬件接口。

在项目根目录执行：

```bash
conda activate go2_control

# MuJoCo 演示
python examples/mujoco/wbc_standing.py
python examples/mujoco/wbc_squat.py

# 移重、卸载和单腿动作
python examples/mujoco/wbc_load_shift.py
python examples/mujoco/wbc_unload.py --duration 12
python examples/mujoco/wbc_single_leg_lift.py --duration 14
python examples/mujoco/wbc_single_leg_cycle.py --duration 20

# 无窗口完整循环，保存逐周期 CSV 和 JSON 汇总
python examples/mujoco/wbc_single_leg_cycle.py --headless --duration 20 --csv artifacts/single_leg_cycle.csv

# 无窗口蹲起对照与日志
python examples/mujoco/wbc_squat.py --headless --mode baseline --csv artifacts/squat_baseline.csv
python examples/mujoco/wbc_squat.py --headless --csv artifacts/squat_tracking.csv

# OCS2 站立还需按 docs/ocs2_standing.md 启动 MPC 与 MRT 节点，
# 所有终端必须使用相同的 ROS_DOMAIN_ID。
export ROS_DOMAIN_ID=213
env -u PYTHONPATH \
  /home/bot/miniconda3/envs/go2_control/bin/python \
  examples/mujoco/ocs2_wbc_standing.py \
  --headless --duration 20 \
  --csv artifacts/ocs2_wbc_standing_ground_feet.csv

# 离线三足检查
python examples/wbc/three_contact_kinematics.py
python examples/wbc/three_contact_load_shift.py
python examples/wbc/three_contact_wbc.py

# 回归测试
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_single_leg_cycle.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

最近一次完整回归在项目清理和配置路径统一后重新运行（2026-10-15）：**38 passed，6 warnings，11.06 s**，包含 MPC/WBC 足端顺序、MRT 通信边界、五次轨迹、OCS2 摆腿竖直轨迹边界条件及 20 s 无窗口单腿循环检查。真实 ROS 2 MPC/MRT 链路由独立运行提供集成验证；异步原地 trot 停步及前进停步均已完成 20 s；同版本同步对照中，移动阶段 x RMSE 为同步 9.793 mm、20 ms poll 异步 13.770 mm；随后 4 ms poll 的独立 20 s 验证将异步 RMSE 降至 12.218 mm。异步左右转停步另已各完成 20 s（最终 +9.829° / −9.794°），触地瞬态及稳定 500 Hz 仍待解决。MPC 摆腿参考此前已完成同步左转、右转和前进停步各 20 s 验证。同步 OCS2 实验不会主动等待以匹配真实时间；可选 `--async-mpc` 即使无窗口运行也使用真实时间节拍。hppfcl/coal 和 OSQP 的依赖弃用提示与测试失败是两回事。

循环 CSV 保存 FR 期望/实际坐标系高度和实际法向接触力。汇总中的高度误差指身体高度；`maximum_xy_drift_mm` 是相对世界原点的水平位移，包含主动移重，不是目标跟踪误差。

## 开发路线

逐步分段及 MRT 内部计时均已完成 20 s 前进停步验证。最新运行排除前 2 s 后，循环平均/p99 为 1.762/2.971 ms，2008/9000 步超过 2 ms。C++ 两次求值平均合计约 4.52 μs，新策略激活步骤的激活耗时平均 0.539 ms；ROS 往返扣除 C++ 回调后的剩余耗时最大 7.056 ms，其中还包含排队、序列化及调度，不能直接当成纯网络延迟。见[控制循环耗时分析](docs/ocs2_control_timing.md)。

对照上游及本地实现后，下一阶段按“模型/坐标及编译对照 → C++ WBC 离线对照 → 本地 MRT 与控制入口 → C++ MuJoCo 闭环”推进；首阶段保留当前独立进程的 DDP/SLQ MPC 和已验证的 WBC 数学问题。具体差异、验收及预计用时见[C++ 控制循环迁移计划](docs/roadmap.md)。

C++ 公共模型/动力学已接入工作空间并完成三组含倾斜和非零速度的 Python/C++ 对照，M/h/J/Jdot·v 最大绝对误差分别约 5.27e-17/8.88e-16/5.55e-17/7.81e-18，电机限制及 armature 一致。操作与范围见[C++ 模型基线](docs/cpp_model_baseline.md)。

C++ WBC 已合并，完成原生 OSQP 求解和 7 组 Python 对照，覆盖四足、两组对角支撑、非零运动和法向力上限；最大力矩差约 6.56e-12 N·m。`feature/cpp-control-loop` 已将本地 MRT、质心/参考转换、WBC、关节 PD 与 MuJoCo C API 接入同一控制进程。Python 只在启动前生成既有 IK 目标、结束后汇总，逐步控制不再经过 Python 管道或 MRT 查询服务；MPC 暂时保留外部 DDP/SLQ 进程。操作和结果见[C++ 闭环记录](docs/cpp_model_baseline.md#本地-mrt-与-c-mujoco-闭环2026-10-15)。

```bash
bash scripts/build_cpp_control.sh
bash scripts/run_cpp_simulation.sh \
  --headless --duration 20 --trot-at 3 --stop-trot-at 15 --forward \
  --csv artifacts/cpp_control_forward.csv
```

脚本默认在 ROS 域 216 管理自己启动的 MPC，并输出 CSV/JSON。删掉 `--headless` 打开官方 MuJoCo 3.4.0 Simulate 被动查看器，包含左右侧栏、鼠标相机操作及可视化菜单；主线程处理界面，C++ 控制线程推进仿真并同步状态。窗口在 MPC 和首条策略就绪后创建，等待服务时每 5 s 输出进度。本地最新官方界面复跑已完整完成 20 s 前进停步（2026-10-16，最终 x=0.090628 m，无力矩截断）；界面同步有额外开销，计时单独记录。Reset/加载关键帧需要重新启动本轮，使 MPC 和仿真一致重置。操作、验证范围及剩余工作见同一记录。

当前 SQP 对齐分支已给启动脚本增加 `--mpc-solver ddp|sqp`（默认 `ddp`），检查对应可执行文件并在启动时打印选择。控制端打印实际激活的策略类型，并写入 CSV/JSON。WBC 运动参考和差分加速度来自规划轨迹；当前反馈输入用于接触力及关节 PD 速度参考。SQP 站立及原地 trot/停步均完成 20 s / 10,000 步，全程线性策略、无拒收和力矩截断；两轮对 MPC 高度参考 RMSE 分别为 0.049/0.104 mm，完整循环 p99 为 0.402/0.429 ms，排除前 2 s 后均为 0/9000 步超过 2 ms。trot 四足均抬起，16.020 s 起持续恢复 mode 15。SQP 前进停步也已完成 20 s / 10,000 步，最终 x=0.090041 m（目标 0.09 m），无拒收和力矩截断；完整循环 p99 为 0.563 ms，0/9000 步超过 2 ms。运动段目标 x RMSE 仍为 11.730 mm，SQP 转向尚待验收。下面的早期结果使用 DDP/SLQ。见 [SQP 实验结果](docs/cpp_model_baseline.md#sqp-线性反馈策略站立验证2026-10-16)和[开发路线](docs/roadmap.md)。

WBC 对照分支已增加 `--joint-kp KP --joint-kd KD`（默认 20、1），便于逐项比较关节增益；实际参数写入启动输出及 CSV/JSON。Kp=0、Kd=3 已完整运行 20 s，但启动高度峰值达到 0.3367 m、最大倾角 6.32°；随后仅把 Kd 改为 1 完成 20 s 站立，高度峰值降为 0.2832 m、最大倾角 1.23°，无截断和策略拒收。默认保留 20/1。已增加可选 `--centroidal-base-accel` 任务，从质心动量变化率生成参考，并转换至实测 free-flyer 坐标；实际任务来源记录于 CSV/JSON。该任务以 0/1 增益完成 20 s SQP 站立，无拒收/力矩截断，全程高度峰值 0.2808 m、最大倾角 0.593°、最大力矩 7.557 N·m；首次 trot 在 WBC 约束检查处停止；将 OSQP 绝对/相对容差明确设为 1e-6 后，已完整完成 20 s trot/停步，无拒收和力矩截断，循环 p99/最大为 0.655/1.228 ms。该任务前进停步也已完整完成 20 s，最终 x=0.090773 m（目标 0.09 m），无拒收和力矩截断，循环 p99/最大为 0.596/1.502 ms；运动段目标 x RMSE 仍为 9.377 mm。转向、剩余上游任务差异及极端 QP 解的数值敏感性仍待处理。

原生站立、前进停步和左右转各完成 20 s / 10,000 步。前进最终 x=0.090654 m（目标 0.09 m），左右转最终 +9.827° / −9.788°。排除前 2 s 后，前进轮完整循环平均/p99 为 0.199/0.470 ms，0/9000 步计算超过 2 ms，最大开始迟到 1.140 ms；四轮均无力矩截断。这些短时无窗口结果尚不证明硬实时或实机可部署。

内部计时已通过 bridge 构建、12 项通信测试、隔离服务检查及上述完整闭环；本轮没有缓存重试，无力矩限幅，最终 x 为 0.090636 m（目标 0.09 m）。计时子项相互嵌套，不能重复相加。服务响应类型已扩展，需使用重新构建并重启的 MRT 节点，操作见同一文档。

实机条件（2026-10-15，用户反馈）：Go2 Air，经现有改装已具备 SDK2 底层状态读取和关节指令发送能力；本项目尚未接入或独立验证该接口。后续需核对实际传感器数据、关节映射、控制模式、通信异常处理及控制权切换行为，部署计算平台待确认。

1. 完成 C++ 闭环窗口观察和当前分支合并，验证更长时间及负载下的时序，包括开始迟到与渲染开销。
2. 逐项对照上游 SQP 和 WBC 任务定义，与当前保留的 DDP/SLQ、已验证 WBC 比较，每次只改变一个主要模块。
3. 将脚本化运动扩展为 ROS 2 在线速度、朝向及步态命令，验证前后、横向、组合运动和多次启停；整理启动入口、依赖及配置版本。当前 DDP/SLQ 与上游 SQP 路径的配置和行为差异需另行对照，不同时混改多个关键模块。
4. 改进动态目标滞后及停步触地瞬态，核对落脚位置、速度和接触切换的一致性；用明确的外力脉冲逐步验证抗扰恢复，记录作用点、大小、持续时间、恢复时间和约束违反情况。
5. 接入 IMU、关节状态和接触信息，实现在仿真中可与真值比较的状态估计，逐步将基座位置/速度等真值输入替换为估计值；验证噪声、偏置及错误接触判断的影响。
6. 确认 Go2 底层接口权限、计算平台和 SDK2 通信，完成只读采集、关节方向/单位核对、力矩和姿态限制、超时保护、控制权切换及急停，再分阶段验证实机受保护站立、原地 trot 和低速运动。

其他步态在基本 trot 稳定之后再扩展。目前不宣称任意推力下都能恢复，也不宣称可直接从仿真部署到真机。

## 目录

```text
config/                        模型配置 JSON、OCS2 task/reference/gait
docs/                          安装、推导与实验记录
examples/pinocchio/             运动学和动力学示例
examples/optimization/          接触力优化
examples/wbc/                   离线站立与三足 WBC
examples/mujoco/                映射、站立、蹲起与单腿抬落脚仿真
examples/ocs2/                  MPC 通信与策略诊断
ros2/go2_control_cpp/           C++ 模型、WBC、本地 MRT 与 MuJoCo 控制入口
ros2/go2_mpc_bridge/            C++ MRT 策略查询服务
launch/                        OCS2 dummy/RViz 启动入口
src/go2_control/wbc/            动力学、运动学、参考与 WBC
src/go2_control/simulation/     MuJoCo–Pinocchio 映射
src/go2_control/optimization/   QP 求解器封装
tests/                         数值与闭环回归测试
scripts/                       环境和模型版本记录
tools/                         检查工具
third_party/                   Unitree 模型与固定版本的官方 MuJoCo Simulate 源码
artifacts/                     生成的日志和报告
```

开发路线统一见 [docs/roadmap.md](docs/roadmap.md)。编辑器可打开 [go2_control.code-workspace](go2_control.code-workspace)，C++ 依赖由各包构建生成的编译数据库提供。学习示例保留完整版本：IK 使用 `foot_position_ik_joint_limits.py`，接触力 QP 使用 `contact_force_qp_friction.py`；控制入口和测试使用 `src/go2_control` 公共模块。

## 阶段标签

```text
v0.1.0-pinocchio-fk-baseline
v0.2.0-pinocchio-kinematics
v0.3.0-pinocchio-dynamics
v0.4.0-wbc-standing-offline
v0.5.0-mujoco-standing-squat
v0.6.0-mujoco-single-leg-cycle
```

`v0.6.0-mujoco-single-leg-cycle` 包含已验证的单腿抬落脚循环及回归测试。固定站立 OCS2–MRT–WBC 验证已从 `feature/ocs2-integration` 合并；20 s 原地 trot 已在 `feature/ocs2-trot` 验证。

## 许可证

本项目的原创代码与文档采用 [MIT 许可证](LICENSE)。Copyright (c) 2026 CanggouBaiyun。

第三方代码、依赖库、机器人模型、网格及其他资源仍遵循各自的许可证和版权声明，包括 `third_party/` 中的内容，以及从上游资源派生的文件，例如 `models/ocs2/` 下导出的 Unitree URDF。本项目的许可证不替代上游许可条款或署名要求。

随仓库保存的官方 MuJoCo Simulate 源码采用 Apache-2.0，LodePNG 依赖保留其 zlib 风格许可。版本及声明见[源码来源记录](third_party/mujoco_simulate/UPSTREAM.md)。
