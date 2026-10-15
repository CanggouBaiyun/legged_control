# Go2 复现进度与开发路线

更新：2026-10-15。本文件是开发路线的统一入口；具体实验数据保存在各阶段记录中。

## 目标与当前进度

在 Go2 上复现原项目相关的控制功能与运动效果，先完成仿真验收，再尽快部署实机。现有 Python 实现作为算法、数值和动作基线。

| 模块 | 已验证 | 尚需完成 |
| --- | --- | --- |
| 模型与映射 | Go2 free-flyer、关节/足名映射、MuJoCo 动力学与 FK 对照 | 后续实机参数与传感器映射 |
| Python WBC | 站立、蹲起、移重、单腿抬落脚、按 mode 切换支撑足 | C++ WBC 和上游任务对照 |
| OCS2 闭环 | 全质心 DDP/SLQ、站立、trot、低速前进、±10° 转向与停步 | 与上游 SQP/任务配置对照、扩展运动及抗扰验收 |
| 执行时序 | 异步 MPC、双时刻 MRT、OSQP 复用、逐步计时 | 本地 C++ 控制链与稳定周期验证 |
| C++ 模型 | ROS 工具链构建、限制/armature、三组状态的 M/h/J/Jdot·v/FK 对照 | C++ WBC 求解及完整闭环 |
| 状态估计 | 使用 MuJoCo 真值完成控制基线 | IMU/关节/接触估计及与真值比较 |
| 实机 | 用户反馈 Go2 Air 已能 SDK2 底层读写 | 项目接口、控制机确认、异常处理及受保护闭环 |

最新模型结果见[C++ 模型基线](cpp_model_baseline.md)，执行时序见[耗时记录](ocs2_control_timing.md)。完成语言迁移不等于原项目效果已经达到。

## 原项目结构与采用方式

上游 [`LeggedController.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_controllers/src/LeggedController.cpp) 在同一进程中使用 MPC 独立线程和 `MPC_MRT_Interface`，控制侧直接调用 MRT 和 WBC。[`LeggedHWLoop.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_hw/src/LeggedHWLoop.cpp) 按 read → update → write 执行控制循环。

[`WeightedWbc.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_wbc/src/WeightedWbc.cpp) / [`WbcBase.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_wbc/src/WbcBase.cpp) 提供任务与约束组织参考。上游使用 qpOASES，当前 Python 使用 OSQP；基座加速度任务、摆腿前馈、关节命令增益和变量布局也有差异。直接复用源码时保留来源、版本及许可证。

当前采用 ROS 2 Jazzy。先保留外部 DDP/SLQ MPC 进程，通过策略消息更新控制进程内的 MRT；将逐步管道/服务往返移出控制路径。上游 SQP、任务定义与求解器差异单独比较，便于定位行为变化。

## 接下来的实施顺序

### 1. C++ WBC

- 基于已验证模型，迁移动力学、支撑足加速度、摩擦/单边力和力矩约束及任务代价。
- 先保持现有 mode 15/9/6、参数、变量布局和 OSQP 数学问题。
- 比较同状态/参考下的 QP 矩阵、解、关节力矩及残差。双足及接触切换样本从有效轨迹采集；现有 103 列运动 CSV 不包含完整回放输入。
- 模型/动力学和 WBC 核心尽量独立于 ROS 消息，供控制入口直接调用。

### 2. 本地 MRT 与 C++ 控制循环

```text
外部 MPC → ROS 接收线程：校验、解码、准备策略缓冲
                         ↓
控制线程：读状态 → 采用有效策略 → 本地 MRT → 参考转换 → WBC → PD/限制 → 写命令
```

优先复用 OCS2 的 `MRT_ROS_Interface` / `MRT_BASE`。接收侧准备策略，控制侧只操作 active policy；核对 try-lock/指针交换、reset 顺序和元数据归属。保留 epoch/sequence、年龄、时间覆盖及有限性检查；同一步当前/未来求值使用同一条策略。reset 不与正常求值并发修改 MRT。

MuJoCo 用 C API 在控制进程内接入。关节 PD 与 WBC 力矩分开保存，防止 SDK2 接入时重复叠加。分别记录控制、仿真和诊断耗时，再用完整循环的 p95/p99、峰值和启动迟到判断执行能力。

### 3. 完整仿真验收

- 恢复站立、原地 trot、前进停步、左右转，之后扩展前后/横向/组合运动和重复启停。
- 单独对照上游 SQP、基座/摆腿任务、权重和关节命令增益。
- 改进动态目标滞后、触地瞬态，按明确外力工况测试恢复能力。
- 实现 IMU + 关节 + 接触状态估计，在仿真中与真值比较，再替换真值控制输入。

### 4. 实机部署

SDK2 数据采集、关节/IMU/接触映射和时间戳核对可提前准备。部署需要明确控制机、控制权切换、看门狗、限幅、异常停止和急停行为。完成相应仿真验收后，依次进行受保护站立、原地 trot 和低速运动。

## 关键数据约定

- WBC：四元数 free-flyer，q=19、v=18，四元数 xyzw；基座速度为局部表达。
- OCS2：24 维质心状态，内部模型使用 Translation + SphericalZYX。它与 WBC q/v 不能直接复制混用。
- OCS2 足序 FL/FR/RL/RR；默认 WBC/SDK 顺序 FR/FL/RR/RL，按名称显式映射。
- ROS 侧 Pinocchio 4.1.0，Conda 侧 3.9.0；各使用明确工具链，通过数值对照衔接。

## 工作方式

每组修改交付完整代码、运行命令和验收结果；测试针对具体风险。每个实质阶段完成后更新 README、提交并推送 Git。阶段用时根据已完成的接口、数值对照和闭环结果更新，避免把演示扩展或准备工作当成实机部署完成。
