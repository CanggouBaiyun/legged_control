# C++ 控制循环迁移计划

## 状态与本次决定

2026-10-15，重新检查上游 `qiayuanl/legged_control` 的控制器、硬件循环、WBC，以及本地 OCS2 ROS 2 接口和 Python WBC。本文是待实施计划，不代表已经实现 C++ WBC、本地控制循环或实机接口。

目标是建立同一 C++ 控制进程内的“读取状态 → 本地 MRT → 参考转换 → WBC → 输出命令”。先完成模型/数据对照和 C++ WBC，再接入策略接收与 MuJoCo 闭环。单独提取现有 `PolicyEvaluator` 可作为实现中的小改动，不再将它作为独立性能里程碑。

现有 Python 实验继续作为动作、数值和计时基线。MPC 暂时保留独立 ROS 2 进程、DDP/SLQ 和现有任务配置；先完成控制路径迁移，再单独比较 SQP 和上游 WBC 任务定义。

## 上游实际执行结构

本次检查的是上游 master 页面，后续若直接引入代码，应记录具体提交并保留来源及其许可证。

- [`LeggedController.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_controllers/src/LeggedController.cpp)：`setupMrt()` 创建本进程的 `MPC_MRT_Interface`，MPC 在线程中调用 `advanceMpc()`；控制更新中直接调用 `updatePolicy()`、`evaluatePolicy()` 和 WBC。这里 MPC 和控制线程属于同一进程。
- [`LeggedHWLoop.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_hw/src/LeggedHWLoop.cpp)：独立循环线程按顺序执行硬件 read、controller update、hardware write，再等待节拍，并检查周期超时。
- [`WeightedWbc.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_wbc/src/WeightedWbc.cpp)：组合约束和加权任务，用 qpOASES 求解。当前上游实现每次 update 创建 QProblem；不把所有实现细节都视作必须照搬的优化方法。
- [`WbcBase.cpp`](https://github.com/qiayuanl/legged_control/blob/master/legged_wbc/src/WbcBase.cpp)：组织实测/期望模型、动力学、接触及任务。其基座加速度任务由质心动力学构造；摆腿任务使用期望/实测位置速度反馈。

这些代码属于 ROS 1/ros_control。我们复用线程分工、数据流和任务组织思路，在 ROS 2 Jazzy 上实现接口。首阶段保留外部 MPC，通过较低频率的策略消息更新本地 MRT；这是有意保留的架构差异。后续是否改成同进程 `MPC_MRT_Interface`，依据控制负载和功能对照决定。

## 本地必须处理的差异

| 项目 | 当前实现 | 对迁移的影响 |
| --- | --- | --- |
| WBC 模型 | `build_go2_simulation_model()` 从 MJCF 构造 free-flyer，nq=19、nv=18 | C++ 对照先使用同一模型来源，并保留根关节 placement 修正、armature 和名称映射 |
| OCS2 模型 | `createPinocchioInterface()` 使用 Translation + SphericalZYX，nq=18、nv=18 | 与 WBC q/v 的表示不同，必须经显式适配，不能直接复制数组 |
| 坐标与索引 | WBC free-flyer 基座速度为局部表达，OCS2 使用质心状态；足序也有差异 | 明确四元数 xyzw、MuJoCo wxyz、角速度/线速度表达及 FL/FR/RL/RR 与内部/SDK 顺序 |
| WBC 决策变量 | 18 维加速度 + 当前支撑足力 + 12 维力矩 | 四足模式 42 维、双足模式 36 维；上游保留所有足力，不能默认矩阵尺寸一致 |
| WBC 任务 | 基座姿态/位置 PD 和前馈、摆腿任务、力跟踪及正则；OSQP 工作区复用 | 先迁移已验证的数学问题和参数；不能把更换任务/求解器后的差异归因于语言 |
| 求值路径 | 每控制步 Python 管道 → ROS 服务 → C++ MRT | 需要让 MRT 与 WBC 在同一控制进程内直接调用，单独新建服务仍保留逐步往返 |
| 环境 | ROS 侧 Pinocchio 安装头文件为 4.1.0，Conda 为 3.9.0 | C++ 使用 OCS2 对应的 ROS 工具链，核对链接；不将 Conda 前缀整体混入 ROS 编译环境 |

本地依据：`src/go2_control/model.py`、`centroidal.py`、`mpc_reference.py`、`wbc/{standing,tasks,kinematics,dynamics}.py`、`optimization/qp_solvers.py`；OCS2 的 `ocs2_centroidal_model/src/FactoryFunctions.cpp`。

最新计时基线见[耗时记录](ocs2_control_timing.md#2026-10-15-内部计时结果)：持续区间 2008/9000 步超过 2 ms，两次 C++ 求值平均合计 4.52 μs，新策略激活时平均 0.539 ms，回调外 ROS 往返剩余时间最大 7.056 ms；WBC 阶段也有 6.157 ms 峰值。这支持减少逐步跨进程往返，同时保留对 WBC 长尾的测量。

## 拟采用的执行结构

```text
独立 OCS2 MPC 进程（先保留当前配置）
       ↑ observation / 目标 / 步态
       ↓ 策略消息
C++ 控制进程
  ROS 接收线程：校验代次/序号 → 解码准备 → 策略缓冲
  控制线程：读取状态 → 采用有效新策略 → 本地双时刻 MRT
                         → 参考转换 → WBC → PD/限制 → 输出命令
  诊断：控制线程记录固定数据，循环外统计/写文件
       ↕
  MuJoCo C API 适配；后续另行接入 SDK2
```

ROS 策略消息仍有传输与解码成本，但控制线程不等待每步服务响应。ROS 回调线程不操作控制线程正在求值的 active policy。优先检查现有 `MRT_ROS_Interface` / `MRT_BASE` 的解码、缓冲和交换机制，而非重新实现 OCS2 求值。

本地 `MRT_BASE::updatePolicy()` 已有 try-lock 和指针交换，`mpcPolicyCallback()` 已有解码及 `moveToBuffer()`。接入前需核对回调由哪个线程执行、reset 初始化顺序，以及 epoch/sequence 与实际采用策略的元数据绑定。不能在每步关键路径中调用阻塞 spin，也不能只因使用缓冲就声称没有锁、分配或实时抖动。

策略保留年龄、时间覆盖、代次/序号及有限性检查。当前/未来查询在同一步使用同一策略，接触 mode 分别按各自时刻检查。reset 期间应停止正常控制更新并清理旧策略，不能与求值并发修改同一个 MRT。

## 实施阶段与验收

### A：模型、接口和编译基线（约 1–2 个工作日）

拟新增 ROS 2 包 `go2_control_cpp`，与 `go2_mpc_bridge` 分开；模型和 WBC 核心尽量不依赖 ROS 消息。第一项可执行程序只加载模型、计算并输出结果，不发送机器人命令。

1. 确认 ROS 工具链、Pinocchio/Eigen、MuJoCo C API 和 OSQP C API 的版本与链接来源。OSQP Python 包版本不能替代对 C 库 ABI、编译选项及默认参数的核对。
2. 固定 State、Reference、Command 的维度、坐标系、单位、关节名和足名；OCS2 24 维状态不是 WBC 的 q。
3. 对同一组有效 q/v，比较 Python/C++ 的质量、惯量/armature、足端 FK、质量矩阵 M、非线性项 h、接触 Jacobian 和 Jdot·v。
4. 样本包含非零基座旋转/速度，不能只用零姿态掩盖坐标系错误。记录最大绝对/相对误差，并为不同物理量定义容差。

验收：能够解释并通过模型/坐标对照，C++ 二进制使用确定的依赖来源。完成后提交源码及说明；不重复运行与本阶段无关的 20 s 动作。

### B：C++ WBC 离线对照（约 3–5 个工作日）

按模型计算 → 硬约束 → 任务/代价 → OSQP 求解 → 结果检查的结构迁移现有数学问题。保留 mode 15/9/6、力矩和摩擦限制、现有增益、PD 及加速度前馈定义。第一版保留当前模式对应的变量尺寸，避免同时引入固定足力布局。

使用同一 q/v、基座/摆腿参考、接触力参考和参数，对照 QP 的 H/g/A/l/u、加速度、接触力、力矩和约束残差。双足模式使用从有效 trot 状态取得的样本，加入接触切换附近样本；验收以一致的数学问题和求解精度为依据，不要求不同求解结果逐位相同。

现有 103 列运动 CSV 没有完整 q/v、所有参考和 QP 数据，不能直接充当完整回放输入。需要单独导出少量完整样本和元数据；采集与下一次有目的的集成实验合并，循环内不逐步写磁盘。

验收：定位并解释差异后，所有选定模式的 QP/力矩对照和约束检查通过。提交 C++ WBC 及回放工具。暂不同时切换到上游 qpOASES/基座任务。

### C：本地 MRT 与控制入口（约 2–3 个工作日）

复用官方接收/解码/缓冲接口，在 ROS 接收侧准备策略，控制侧采用有效策略并直接调用 MRT。将 metadata 与解码后的策略绑定，保留 reset、过期、覆盖和同策略双时刻规则。对相同策略、状态和时刻，与现有服务参考比较，包括事件附近查询。

把参考转换与 C++ WBC 组合成一个 step 入口，接收实测状态和时间，返回关节命令与状态。关节 PD 和 WBC 力矩分开保留，防止后续 SDK2 接入重复叠加。通信层、控制数学和设备读写分别组织。

验收：普通控制步没有 Python 管道或 ROS 服务往返；双时刻引用同一策略；迟到、旧代次和非法输入行为明确。通过后提交。

### D：C++ MuJoCo 闭环（约 2–4 个工作日）

通过 MuJoCo C API 在控制进程内完成状态读取、力矩设置和仿真推进。先无窗口站立，再依次恢复原地 trot、前进停步及左右转；GUI 在闭环正确后接入。保留现有 Python 实验作为独立对照，不以每步服务调用 Python WBC 代替完成迁移。

每个新控制能力执行有目的的 20 s 验证，记录目标跟踪、足端接触、约束、限幅、策略年龄和时序；再选择代表性工况做更长稳定性运行。控制计算、仿真推进与诊断分别计时，并保留含完整循环的指标，比较 p95/p99、最大值和启动迟到。主动节拍等待、初始化和循环外统计分别说明。

验收：C++ 闭环功能达到既有仿真基线，且实测时序有可解释的变化；不能由 C++ 或平均小于 2 ms 自动推断稳定 500 Hz。根据剩余长尾再决定日志迁移、预分配、线程设置或求解器处理。

## 此后推进顺序

1. C++ 基础闭环稳定后，单独对照原项目的 SQP、基座加速度构造、摆腿任务、任务权重与关节命令增益；语言迁移完成不等于上游算法和效果完全复现。
2. 整理 ROS 2 在线速度/朝向/步态命令，验证前后、横向、组合运动和多次启停，处理已有动态滞后和触地瞬态，再定义并测试外力扰动工况。
3. 接入 IMU/关节/接触信息，先在仿真中将估计值与真值比较，再替换真值控制输入。
4. SDK2 只读采集、映射及时间戳核对可并行准备；发力阶段需完成控制权切换、异常停止与超时行为验证，再开展受保护站立、原地 trot、低速运动。

Go2 Air 接口能力来自用户反馈，控制电脑及具体数据通路尚待确认。原项目视频中的完整效果还依赖模型、状态估计、参数、执行时序及硬件条件，不能承诺迁移 C++ 后自然达到。

## 工作方式与用时

A–D 预计约 8–14 个专注工作日；按每天 8–9 小时并穿插 C++/ROS 2 讲解，预留约 2–3 周。依赖兼容和数值差异可能延长工期；这是 C++ 仿真阶段估计，不含后续状态估计、算法对照和实机调试。

每个阶段完成可 review 的结果后更新 README、记录命令/结果/限制并提交；相关分支合并后推进下一里程碑。指导代码时给出实际文件、最新行号或唯一定位块、完整修改片段、运行命令及预期结果。测试用于检查明确风险，不因格式调整反复全量回归。
