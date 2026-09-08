# Go2 Traditional Locomotion

Traditional model-based locomotion control stack for the Unitree Go2, built with Pinocchio, MuJoCo, WBC, OCS2 and ROS 2.

本项目面向 Unitree Go2 构建传统模型驱动的四足运动控制栈，涵盖机器人建模、浮动基动力学、接触力优化、全身控制、轨迹优化、仿真验证和实机接口。

整体控制架构参考 `legged_control` 的 NMPC-WBC 设计，并针对 Go2 的模型参数、关节映射、ROS 2 通信和 Unitree SDK2 接口进行适配。

## 目标控制链路

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
关节前馈力矩 + PD
       ↓
MuJoCo / Unitree SDK2
```

## 当前进度

### M1：Go2 模型基线——完成

- 从 Unitree 官方仓库固定 Go2 URDF；
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
- Jacobian 奇异值与阻尼对比；
- 关节速度、位置限制和不可达目标测试。

正常的 FR 足端上移 5 cm 实验最终位置误差约为 `1e-5 m`。Jacobian 速度有限差分误差约为 `2.4e-9`。

阶段标签：

```text
v0.1.0-pinocchio-fk-baseline
v0.2.0-pinocchio-kinematics
```

### M3：Pinocchio 动力学与接触——进行中

目前完成：

- 重力补偿和静态 RNEA；
- CRBA 质量矩阵、对称性、正定性和动能检查；
- RNEA 与 `M @ a + h` 一致性验证；
- ABA 正动力学和自由落体检查；
- `J.T @ force` 接触力映射及功率验证；
- 四足竖直接触力静态分配；
- 完整静态动力学残差检查。

名义姿态下模型质量为 `16.087 kg`，四足竖直力总和为 `157.81347 N`。静态动力学残差范数约为 `9.12e-14`。

正在做：

- 非零期望加速度下的浮动基动力学；
- 执行器选择矩阵 `S`；
- 从静态最小二乘过渡到带接触约束的 QP。

## 开发路线

- M4：使用两个求解器完成接触力分配 QP；
- M5：实现 Go2 WBC，加入动力学、支撑足、摩擦锥和力矩约束；
- M6：完成 Pinocchio 与 MuJoCo 状态和索引映射；
- M7：实现 MuJoCo 站立、蹲起、抬腿和原地踏步；
- M8：加入 Trot 步态和摆动足轨迹；
- M9：运行 OCS2 示例并建立 Go2 NMPC；
- M10：整理 ROS 2、SDK2、状态估计和安全接口。

MuJoCo 闭环、WBC、OCS2 和实机控制接口仍在开发中。

## 目录

```text
configs/                         模型和名义姿态配置
docs/                            路线、安装步骤和推导记录
examples/pinocchio/kinematics/   运动学算法示例与数值验证
examples/pinocchio/dynamics/     动力学和接触力算法验证
scripts/                         环境和模型版本记录
src/go2_control/                 可复用的模型与控制代码
tests/                           自动化数值检查
tools/                           模型盘点和维护工具
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
```

在项目根目录运行示例：

```bash
conda activate go2_control
python examples/pinocchio/kinematics/fk_jacobian.py
python examples/pinocchio/dynamics/static_contact_balance.py
```

运行基础测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

当前测试结果：

```text
3 passed
```

`hppfcl` 关于迁移到 `coal` 的提示是依赖库弃用警告，目前不影响这些数值检查。

## 动力学模型

项目后续 WBC 使用完整浮动基动力学：

$$
M(q)\dot v+h(q,v)=S^T\tau+J_c^T\lambda
$$

其中前 6 维浮动基没有直接驱动，必须依靠足端接触力产生基座所需的合力和力矩。当前静态实验先分步计算接触力与关节力矩，后面会把 `dv`、`lambda` 和 `tau` 放入带约束 QP。

