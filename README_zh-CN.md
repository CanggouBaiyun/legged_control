# Go2 Traditional Locomotion

[English](README.md)

面向 Unitree Go2 的模型运动控制项目，使用 Pinocchio、MuJoCo 和基于 QP 的全身控制。目标架构参考 [legged_control](https://github.com/qiayuanl/legged_control)，包含 OCS2 NMPC、WBC、状态估计和机器人硬件接口。

**当前范围：** 已完成 MuJoCo 站立、蹲起、移重和右前腿完整抬落脚闭环，包含足端卸载、三足支撑、平滑摆动跟踪和四足承重恢复。OCS2、trot、ROS 2 控制接口和 Unitree SDK2 实机执行尚未实现。

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
| OCS2 NMPC、trot 和抗扰恢复 | 计划中 |
| ROS 2 / SDK2、状态估计和实机安全 | 计划中 |

当前验证的是平地单腿动作，不是连续行走步态，也不代表已经具备真机部署条件。

## 控制架构

已实现的仿真路径：

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

WBC 和关节参考 IK 使用同一组基座、足端轨迹。关节 PD 在 QP 外叠加，之后执行电机映射和最终力矩限幅。增益针对当前仿真选取，并非照搬原工程。当前接触阶段由脚本管理，尚未由 MPC 或步态规划器生成。

计划接入的运动控制路径：

```text
位置/速度指令 + 步态时序
           ↓
       OCS2 NMPC
           ↓
状态、关节速度和接触力参考
           ↓
          WBC
           ↓
 MuJoCo / Unitree SDK2
```

## 验证结果

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

示例依赖已安装的项目环境，以及 [configs/go2_model.json](configs/go2_model.json) 指定位置下的模型仓库。现有环境安装记录见 [安装说明](docs/setup_guide.md)。

已记录的数值计算环境：

```text
Ubuntu 24.04
Python 3.10
Pinocchio 3.9.0
MuJoCo 3.4.0
OSQP 1.1.3
ProxSuite / ProxQP 0.7
```

开发环境安装了 ROS 2 Jazzy，但这些 Python 示例不需要启动 ROS 通信图。ROS 2 控制接口尚未接入。

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

# 离线三足检查
python examples/wbc/three_contact_kinematics.py
python examples/wbc/three_contact_load_shift.py
python examples/wbc/three_contact_wbc.py

# 回归测试
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_single_leg_cycle.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

最近一次已验证的回归结果为 **28 passed，2 warnings**，包含五次轨迹检查及 20 s 无窗口单腿循环。无窗口模式不会主动等待以匹配真实时间，耗时取决于计算速度。hppfcl/coal 和 OSQP 的依赖弃用提示与测试失败是两回事。

循环 CSV 保存 FR 期望/实际坐标系高度和实际法向接触力。汇总中的高度误差指身体高度；`maximum_xy_drift_mm` 是相对世界原点的水平位移，包含主动移重，不是目标跟踪误差。

## 开发路线

1. 确定目标环境下的 OCS2 构建方案，以及 MPC 状态、输入和 WBC 接口。
2. 接入 Go2 质心模型和参考管理，先验证固定四足接触下的 MPC–WBC。
3. 加入 trot 接触时序与摆动轨迹，验证踏步、低速前进、停止和转向。
4. 使用可重复的外力脉冲验证抗扰恢复。区分“不摔倒”“恢复姿态”和“回到固定世界位置”，记录外力、持续时间、作用点、恢复时间及约束违反情况。
5. 完成性能分析和必要的 C++ 实现，加入 ROS 2 / SDK2、状态估计、接触判断、超时保护和急停处理。

其他步态在基本 trot 稳定之后再扩展。目前不宣称任意推力下都能恢复，也不宣称可直接从仿真部署到真机。

## 目录

```text
configs/                       模型路径与名义姿态
docs/                          安装、推导与实验记录
examples/pinocchio/             运动学和动力学示例
examples/optimization/          接触力优化
examples/wbc/                   离线站立与三足 WBC
examples/mujoco/                映射、站立、蹲起与单腿抬落脚仿真
src/go2_control/wbc/            动力学、运动学、参考与 WBC
src/go2_control/simulation/     MuJoCo–Pinocchio 映射
src/go2_control/optimization/   QP 求解器封装
tests/                         数值与闭环回归测试
scripts/                       环境和模型版本记录
tools/                         检查工具
third_party/                   Unitree 模型仓库
artifacts/                     生成的日志和报告
```

## 阶段标签

```text
v0.1.0-pinocchio-fk-baseline
v0.2.0-pinocchio-kinematics
v0.3.0-pinocchio-dynamics
v0.4.0-wbc-standing-offline
v0.5.0-mujoco-standing-squat
v0.6.0-mujoco-single-leg-cycle
```

`v0.6.0-mujoco-single-leg-cycle` 包含已验证的单腿抬落脚循环及回归测试。下一阶段接入 OCS2。

## 许可证

本项目的原创代码与文档采用 [MIT 许可证](LICENSE)。Copyright (c) 2026 CanggouBaiyun。

第三方代码、依赖库、机器人模型、网格及其他资源仍遵循各自的许可证和版权声明，包括 `third_party/` 中的内容，以及从上游资源派生的文件，例如 `models/ocs2/` 下导出的 Unitree URDF。本项目的许可证不替代上游许可条款或署名要求。
