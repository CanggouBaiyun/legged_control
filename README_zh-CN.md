# Go2 Traditional Locomotion

[English](README.md)

面向 Unitree Go2 的传统模型运动控制栈，使用 Pinocchio、MuJoCo、全身控制、OCS2 和 ROS 2 构建。

项目涵盖机器人建模、浮动基动力学、接触力优化、全身控制、轨迹优化、仿真验证和实机接口。整体 NMPC-WBC 架构参考 `legged_control`，并针对 Go2 的模型参数、关节映射、ROS 2 通信和 Unitree SDK2 接口进行适配。

## 控制架构

```text
速度和姿态指令
       ↓
步态与参考轨迹
       ↓
OCS2 NMPC
       ↓
状态、足端和接触力参考
       ↓
WBC-QP
       ↓
关节前馈力矩 + PD 反馈
       ↓
MuJoCo / Unitree SDK2
```

## 当前进度

### M1：Go2 模型基线——完成

- 从 Unitree 官方仓库导入 Go2 URDF；
- 使用 `JointModelFreeFlyer()` 建立浮动基模型；
- 确认 `nq=19`、`nv=18`；
- 盘点 joint、frame、质量、惯量和关节限制；
- 建立 SDK2 电机顺序到 Pinocchio `q/v` 索引的映射；
- 加入模型和动力学基础测试。

### M2：Pinocchio 运动学模块——完成

- 正运动学和四足位置；
- 足端 Jacobian 及有限差分验证；
- 验证足端速度关系 `v_foot = J @ v`；
- 速度 IK 和迭代位置 IK；
- 阻尼最小二乘；
- Jacobian 奇异值与阻尼分析；
- 关节速度和位置限制；
- 不可达目标测试。

正常的 FR 足端上移 5 cm 实验最终位置误差约为 `1e-5 m`。Jacobian 速度有限差分误差约为 `2.4e-9`。

阶段标签：

```text
v0.1.0-pinocchio-fk-baseline
v0.2.0-pinocchio-kinematics
```

### M3：Pinocchio 动力学与接触——完成

- 重力补偿和静态 RNEA；
- CRBA 质量矩阵、对称性、正定性和动能检查；
- RNEA 与 `M @ a + h` 一致性验证；
- ABA 正动力学和自由落体检查；
- `J.T @ force` 接触力映射及功率一致性验证；
- 四足竖直接触力静态分配；
- 显式执行器选择矩阵 `S`；
- 非零期望加速度下的完整浮动基动力学。

名义姿态下模型质量为 `16.087 kg`，四足竖直力总和为 `157.81347 N`，完整静态动力学残差范数约为 `9.12e-14`。当基座竖直加速度为 `0.5 m/s²` 时，所需支撑力为 `165.85697 N`，完整动力学残差约为 `1.71e-13`。

阶段标签：

```text
v0.3.0-pinocchio-dynamics
```

### M4：接触力 QP——完成

- 带法向力上下限的四维竖直力 QP；
- 主动约束触发后的载荷重新分配；
- 四足 12 维三维接触力；
- 线性化摩擦锥和法向力范围；
- 摩擦能力不足时的不可行性验证；
- OSQP 与 ProxQP 同问题求解对比；
- `src/go2_control/optimization/` 下的可复用求解器接口。

当前三维算例的目标合力为 `[8.0435, 0.0, 165.85697] N`，完整动力学残差约为 `2.96e-15`，OSQP 与 ProxQP 的解差约为 `3.64e-6 N`。

### M5：离线站立 WBC——完成

- 将四个足端的线性 Jacobian 堆叠成 `J_c`，并验证支撑足加速度约束；
- 建立 42 维联合决策变量 `[dv, lambda, tau]`；
- 加入完整浮动基动力学和四足加速度等式；
- 加入单边接触、线性摩擦锥、法向力范围和电机力矩限制；
- 加入三维基座位置 PD 与 SO(3) 姿态 PD 任务；
- 使用 roll 扰动验证左右足载荷重新分配；
- 抽取可复用的动力学、接触运动学、基座任务和 QP 求解器函数；
- 为模型、基座任务、不同接触足集合、质量矩阵和 RNEA 一致性加入自动测试。

在名义状态下，期望基座向上加速度为 `0.5 m/s²` 时，WBC 求得 `0.49971572 m/s²`。完整动力学残差约为 `2.97e-14`，四足加速度残差约为 `1.69e-16`。人为加入 `+5°` roll 扰动后，期望恢复角加速度为 `-3.4906585 rad/s²`，左右足接触力按预期重新分配。

阶段标签：

```text
v0.4.0-wbc-standing-offline
```

## 开发路线

- M5：完成带动力学、固定接触、摩擦、力矩限制和基座位姿任务的离线站立 WBC；
- M6：完成 Pinocchio 与 MuJoCo 状态和索引映射——当前阶段；
- M7：实现 MuJoCo 站立、蹲起、抬腿和原地踏步；
- M8：加入 Trot 步态调度和摆动足轨迹；
- M9：运行 OCS2 示例并建立 Go2 NMPC；
- M10：加入 ROS 2、SDK2、状态估计和安全接口。

离线四足站立 WBC 已完成。MuJoCo 闭环、动态接触模式、OCS2、状态估计和实机控制接口仍在开发中。

## 目录

```text
configs/                         模型和名义姿态配置
docs/                            路线、安装记录和公式推导
examples/pinocchio/kinematics/   运动学算法示例与数值验证
examples/pinocchio/dynamics/     动力学和接触力算法验证
examples/optimization/            接触力 QP 示例
examples/wbc/                     接触运动学和离线 WBC 示例
scripts/                         环境和模型版本记录工具
src/go2_control/wbc/             可复用的动力学、接触和基座任务函数
src/go2_control/optimization/    可复用的 QP 求解器接口
tests/                           自动化数值测试
tools/                           模型检查和维护工具
third_party/                     Unitree 官方模型仓库
artifacts/                       自动生成的检查结果
```

## 运行环境

当前开发环境：

```text
Ubuntu 24.04
ROS 2 Jazzy
Python 3.10
Pinocchio 3.9
OSQP 1.1
ProxSuite / ProxQP 0.7
```

在项目根目录运行示例：

```bash
conda activate go2_control
python examples/pinocchio/kinematics/fk_jacobian.py
python examples/pinocchio/kinematics/contact_acceleration.py
python examples/pinocchio/dynamics/static_contact_balance.py
python examples/optimization/contact_force_qp_friction.py
python examples/wbc/standing_wbc_qp.py
```

运行数值测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

当前测试结果：

```text
12 passed
```

`hppfcl` 关于迁移到 `coal` 的提示是依赖库弃用警告，目前不影响现有数值检查。

## 动力学模型

全身控制器使用完整浮动基动力学：

$$
M(q)\dot v+h(q,v)=S^T\tau+J_c^T\lambda
$$

浮动基前 6 维没有直接驱动，所需的基座合力和力矩必须由足端接触产生。当前离线站立 WBC 已经联合优化 `dv`、`lambda` 和 `tau`，并同时满足完整动力学、支撑足加速度、摩擦、法向力和电机力矩约束。下一步将其作为 MuJoCo 站立闭环的力矩计算核心。
