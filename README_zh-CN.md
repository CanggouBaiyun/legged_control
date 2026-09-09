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

### M5：站立 WBC——进行中

- 使用 Pinocchio 足端经典加速度验证 `J_dot @ v`；
- 在 FR 足端上数值验证 `J @ dv + J_dot @ v`；
- 将 QP 构造与具体求解器调用分开；
- 在 `feature/wbc-standing` 分支开发。

下一步是堆叠全部支撑脚约束，并建立联合决策变量 `[dv, lambda, tau]`。

## 开发路线

- M5：实现 Go2 WBC，加入动力学、支撑足、摩擦锥和力矩约束；
- M6：完成 Pinocchio 与 MuJoCo 状态和索引映射；
- M7：实现 MuJoCo 站立、蹲起、抬腿和原地踏步；
- M8：加入 Trot 步态调度和摆动足轨迹；
- M9：运行 OCS2 示例并建立 Go2 NMPC；
- M10：加入 ROS 2、SDK2、状态估计和安全接口。

MuJoCo 闭环、WBC、OCS2 和实机控制接口仍在开发中。

## 目录

```text
configs/                         模型和名义姿态配置
docs/                            路线、安装记录和公式推导
examples/pinocchio/kinematics/   运动学算法示例与数值验证
examples/pinocchio/dynamics/     动力学和接触力算法验证
examples/optimization/            接触力 QP 示例
scripts/                         环境和模型版本记录工具
src/go2_control/                 可复用的模型、优化与控制模块
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
```

运行数值测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

当前测试结果：

```text
3 passed
```

`hppfcl` 关于迁移到 `coal` 的提示是依赖库弃用警告，目前不影响现有数值检查。

## 动力学模型

全身控制器使用完整浮动基动力学：

$$
M(q)\dot v+h(q,v)=S^T\tau+J_c^T\lambda
$$

浮动基前 6 维没有直接驱动，所需的基座合力和力矩必须由足端接触产生。当前接触力 QP 先根据基座方程求 `lambda`，再根据驱动关节方程恢复关节力矩。站立 WBC 将同时优化 `dv`、`lambda` 和 `tau`，并满足支撑足加速度约束。
