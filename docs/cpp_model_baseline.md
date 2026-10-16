# C++ 模型、动力学与 WBC 基线

## 当前状态（2026-10-15）

模型/动力学与原生 C++ WBC 已分别通过 PR #12、#13 合并到 `main`，通过工作空间符号链接构建。模型完成三组 Python/C++ 对照，WBC 完成七组 QP/力矩对照。`feature/cpp-control-loop` 已完成本地 MRT 和 MuJoCo C API 闭环及四轮 headless 验收，待窗口观察和 PR 收尾。模型配置统一为 `config/go2_model.json`。

## 本组修改

| 文件 | 作用 |
| --- | --- |
| `include/go2_control_cpp/model.hpp` | 声明模型加载结果和 WBC 所需动力学结果 |
| `src/model.cpp` | 读取现有 JSON/MJCF，保留根 placement 修正，按名称复制 URDF 电机 effort/velocity 限制 |
| `src/dynamics.cpp` | 计算对称质量矩阵 M、非线性项 h、指定接触足的 J 和 Jdot·v |
| `src/model_check.cpp` | 改为调用公共模型加载函数，并输出限制值 |
| `src/dynamics_check.cpp` | 生成三组状态及动力学结果 JSON |
| `CMakeLists.txt` | 构建可复用 `go2_model` 静态库及两个检查入口 |
| `examples/pinocchio/dynamics/check_cpp_dynamics.py` | 调用已有 Python 模型/动力学函数，与 C++ 输出对照 |

`package.xml` 沿用上一组。模型和动力学核心只依赖 Pinocchio 等计算库，不依赖 ROS 消息或仿真进程，后续 WBC 直接链接 `go2_model`。

## 与 WBC 的对应

完整动力学等式为 `M * a + h = Sᵀ * tau + Jᵀ * f`，其中 a 是广义速度 v 的导数。支撑足加速度约束为 `J * a + Jdot * v = 0`。

`computeDynamics()` 使用四元数 free-flyer 的 q/v，接触 Jacobian 为 `LOCAL_WORLD_ALIGNED` 的线速度三行。基座 v 仍为局部表达，不能将前三维直接当作世界系速度；接触力与足端速度使用世界对齐坐标表达。CRBA 只使用上三角并显式对称化，动力学数据和接触运动学数据分别保存。

本实现保留现有 Python 模型中的 MJCF armature，并从 URDF 按关节名称取力矩/速度限制。没有将该模型当作 OCS2 的欧拉角基座模型；后续质心状态转换仍需显式适配。当前方法返回动态尺寸数组，尚未完成控制循环预分配及性能优化。

## 已完成的验证

工作空间 CMake Release 构建成功，运行 `model_check` 和 `dynamics_check` 成功；使用现有 Conda Python/Pinocchio 3.9.0 与 C++/Pinocchio 4.1.0 比较：

- 总质量 15.206408 kg、armature、URDF effort/velocity 限制逐项一致。
- 三组状态包含名义静止、正向倾斜运动、反向倾斜运动；运动状态使用非零基座/关节速度及变化后的关节角。
- 对照脚本按关节名称映射坐标索引，避免依赖两版解析器恰好相同的关节排列。
- 各状态的质量矩阵为正定，运动样本的 Jdot·v 非零。

| 对照量 | 三组状态的最大绝对误差 |
| --- | ---: |
| M | 5.27e-17 |
| h | 8.88e-16 |
| J | 5.55e-17 |
| Jdot·v | 7.81e-18 |
| 足端位置（m） | 5.55e-17 |

对照容差为 atol=1e-10、rtol=1e-9。三组完整 q/v 与结果保存在 `artifacts/cpp_model_dynamics.json`，对照摘要为同名 `_comparison.json`。这些是离线计算对照，不代表接触力求解、动态稳定性、实时性能或硬件闭环验证。

## 用户接入与运行

源码已由 Git 管理，无需复制临时目录。首次设置工作空间时按[安装说明](setup_guide.md)建立符号链接。构建并导出 C++ 结果：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/bot/Project/ocs2_ws
env PATH=/usr/bin:/bin:/usr/sbin:/sbin CMAKE_BUILD_PARALLEL_LEVEL=1 \
  /usr/bin/python3 -m colcon build \
  --packages-select go2_control_cpp --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/ros/jazzy \
    -Dpinocchio_DIR=/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/pinocchio \
    -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
ros2 run go2_control_cpp dynamics_check \
  /home/bot/Project/go2_traditional_locomotion \
  /home/bot/Project/go2_traditional_locomotion/artifacts/cpp_model_dynamics.json
```

使用原 Python 函数完成比较：

```bash
cd /home/bot/Project/go2_traditional_locomotion
env -u PYTHONPATH /home/bot/miniconda3/envs/go2_control/bin/python \
  examples/pinocchio/dynamics/check_cpp_dynamics.py \
  artifacts/cpp_model_dynamics.json
```

末尾预期：`PASS: model limits, armature, FK, M, h, J and Jdot*v match in all 3 cases.` 程序保存 `artifacts/cpp_model_dynamics_comparison.json`。无需启动 MPC/MRT；本步骤核对计算模块，没有连接机器人。

两个 C++ 包默认导出各自的 `compile_commands.json`，用于编辑器准确解析依赖；可打开根目录的 `go2_control.code-workspace`。随后进入 C++ WBC 的任务/约束组装和 OSQP 求解接口，沿用已验证的 Python 参数与数学问题。总体路线见[开发路线](roadmap.md)。

## C++ WBC 离线实现（2026-10-15）

源码已接入 `ros2/go2_control_cpp` 并通过 PR #13 合并，工作空间构建和下述离线对照均已通过，无需使用临时补丁。

### 实现与范围

- `include/go2_control_cpp/wbc.hpp`：基座/摆腿参考、QP、求解结果及控制器接口。
- `src/wbc.cpp`：完整浮动基动力学、支撑足加速度、摩擦/单边力/法向力上限、电机力矩限制；基座位置/姿态任务、摆腿前馈和反馈、接触力跟踪及正则。
- `src/osqp_solver.cpp`：原生 OSQP C API，采用 builtin 后端，polishing=true、max_iter=20000、warm_starting=false；稀疏结构相同则更新，变化时重新 setup。只接受 solved 状态并检查输出/约束。
- `src/wbc_replay.cpp`：从相同输入离线组装 QP、求解、导出矩阵和力矩；检查更新后的工作区与新建求解器结果一致，以及恢复原问题后的解一致。
- `examples/wbc/check_cpp_wbc.py`：调用现有 Python WBC 生成参考 QP 和结果，按关节名称对照 C++ 输出。
- `scripts/check_cpp_wbc.sh`：一条命令完成参考生成、定向构建、C++ 回放和 Python 比较。

保留现有 Python 数学问题和参数。四足为 42 个变量，两足为 36 个变量；接触力遵循传入的足名顺序。摆腿使用同一实测状态下的完整 Jacobian 数据，不重复计算质量矩阵。

C++ WBC 核心输出 QP 力矩；关节 PD、MRT 参考转换和 MuJoCo 控制循环由下述独立入口负责。默认的 Python/MuJoCo 控制入口没有改动，SDK2 尚未接入。上游任务定义和 SQP 对照仍按开发路线进行。

### OSQP 依赖

本机 Python `osqp` 包为 1.1.3，供 C++ 使用的 `libosqp` 为 1.0.0；它们是不同的版本指标。`environment.yml` 中显式声明 `libosqp>=1,<2`，本机已安装，无需重新安装环境。

CMake 通过 `GO2_OSQP_ROOT` 指定成对的 `include/osqp` 与 `lib/libosqp.so`。仅将 OSQP C 头文件目录加入编译，通过绝对路径在启动时动态装载 C API；不将整个 Conda 前缀加入 CMake/Pinocchio 搜索路径。当前 C 库仅依赖 qdldl 和系统 C 库，已核对并测试。更换部署机器时需配置并验证对应平台的 C 库和 ABI，不直接复制本机二进制。

未设置 `GO2_OSQP_ROOT` 时仍可构建模型检查；设置后才启用 WBC 库和 `wbc_replay`。构建脚本会传入本机已验证的前缀，可通过 `GO2_PYTHON` / `GO2_OSQP_ROOT` / `GO2_WORKSPACE` 覆盖路径。

### 已验证结果

Release 构建通过，7 组离线输入全部求解并通过 Python/C++ 对照：四足、mode 9、mode 6 的静态和非零运动状态，以及四足法向力上限工况。双足样本包含两只脚的非零摆腿位置、速度、加速度参考。它们是合成的数学对照样本，与下述闭环实验分开记录。

| 指标 | 最大值 |
| --- | ---: |
| H / g 元素绝对差 | 1.46e-11 / 2.91e-10 |
| 动力学/接触等式矩阵绝对差 | 5.55e-17 |
| 不等式矩阵、上下界绝对差 | 0 |
| 整个 QP 解的绝对差 | 2.24e-11 |
| 电机力矩绝对差 | 6.56e-12 N·m |
| C++ 动力学/接触等式残差范数 | 5.81e-14 |
| 不等式违反量 | 0 |

矩阵比较采用 atol=1e-8、rtol=1e-9；解比较采用 atol=1e-4、rtol=1e-5。另确认 7 类非法输入显式拒绝，缺失 C 库、不可行 QP、非有限 QP 也抛出错误；失败时不返回关节命令。这些离线结果本身不证明控制循环的执行时序。

### 接入与运行

在包含 PR #13 的仓库中执行：

```bash
cd /home/bot/Project/go2_traditional_locomotion
bash scripts/check_cpp_wbc.sh
```

预期最后输出：

```text
PASS: C++ WBC QP matrices, solutions and torque outputs match Python in all 7 cases.
```

生成数据为 `artifacts/cpp_wbc_cases.json`、`cpp_wbc_output.json` 和 `cpp_wbc_output_comparison.json`。不需要 MPC/MRT 节点或机器人连接。

## 本地 MRT 与 C++ MuJoCo 闭环（2026-10-15）

### 控制链

```text
启动前：Python 生成既有 IK 目标时间表及参考转换对照样本 → JSON

外部 OCS2 DDP/SLQ MPC ← 每 20 ms 的实测观测
           ↓ 策略 ROS 消息
C++ ROS 接收线程：校验、解码、准备最多 8 条策略
           ↓ 每 4 ms 尝试采用有效的新策略
C++ 控制线程：MuJoCo 状态 → 质心状态 → 本地 MRT 双时刻求值
           → 基座/足端参考 → WBC → 关节 PD → 力矩映射与限幅 → mj_step
```

逐步控制不调用 Python 管道或 MRT ROS 查询服务。Python 只在启动前生成目标，在运行结束后汇总 CSV；MPC 暂时保持外部进程，与控制进程通过观测/策略消息连接。没有将 MPC 改为同进程求解线程。

| 文件 | 职责 |
| --- | --- |
| `include/go2_control_cpp/reference.hpp` / `src/reference.cpp` | 显式关节名映射；质心状态与四元数 q/v 双向转换；基座、接触力和摆腿参考 |
| `include/go2_control_cpp/local_mrt.hpp` / `src/local_mrt.cpp` | 一次 reset、策略校验与接收线程、策略选择和本地 MRT |
| `include/go2_control_cpp/simulation.hpp` / `src/simulation.cpp` | MuJoCo C API、状态与力矩映射、地面接触力、官方 Simulate 被动查看器 |
| `src/control_simulation.cpp` | 2 ms 仿真步、观测/策略时序、WBC、PD、异常停止与 CSV |
| `examples/mujoco/prepare_cpp_target.py` | 复用既有 Python IK/目标函数，生成 401 个目标点和三组转换对照 |
| `scripts/build_cpp_control.sh` | 使用 ROS Pinocchio 和明确的 OSQP/MuJoCo 路径构建 |
| `scripts/run_cpp_simulation.sh` | 准备目标、启动配套 MPC、运行、汇总，退出时关闭自己启动的 MPC |
| `tools/summarize_cpp_control.py` | 检查完整采样、身份/年龄及残差，汇总运动、接触和时序 |

### 坐标和策略约定

- OCS2 关节顺序从目标 JSON 读取并与 `task.info` 完全核对，再按名称映射到 Pinocchio。力按 FL/FR/RL/RR 及 mode 的实际支撑足映射。MuJoCo 电机也按名称匹配，并验证为直接力矩驱动。
- 当前和未来 5 ms 参考在同一次激活的策略上求值，期间不切换 active policy。未来 mode 独立检查；只有该足在两个时刻都处于摆动时，才差分足端速度得到加速度。
- 摆腿采用现有 Python `--swing-reference mpc` 路径：位置和速度来自 MPC 参考的 FK。关节 PD 采用同一时刻的关节参考，kp=20、kd=1，单独叠加到 QP 力矩后限幅。
- ROS 线程先完成解码，再用短互斥区传递准备好的对象。控制侧 try-lock 选择策略；激活和两次求值均由控制线程执行，epoch/sequence 元数据随同一对象切换。运行中禁止再次 reset。
- 仅接受 feedforward、24 维状态/输入和 mode 15/9/6。检查 epoch、递增 sequence、观测时间不倒退、有限值、轨迹尺寸和时间表。采用策略时要求覆盖下一次检查与未来查询，逐步检查年龄不超过 100 ms 和当前/未来查询覆盖。
- `MRT_ROS_Interface` 作为官方解码/求值的基类；本地封装自行接收消息和调用 reset 服务，没有调用 `launchNodes()` 或另起查询服务。

### 构建和运行

系统需要已有 ROS 2/OCS2 工作空间、Conda 控制环境、MuJoCo 3.4.0、OSQP 1.x C 库，以及 GLFW/OpenGL 开发文件。本机这些依赖已具备；新机器缺少图形开发文件时安装 `libglfw3-dev`、`libgl1-mesa-dev`。构建不会自动安装依赖或修改 OCS2 fork。

```bash
cd /home/bot/Project/go2_traditional_locomotion
bash scripts/build_cpp_control.sh

# 无窗口：站立、trot、前进 90 mm、停步，20 秒完整运行
bash scripts/run_cpp_simulation.sh \
  --headless --duration 20 --trot-at 3 --stop-trot-at 15 --forward \
  --csv artifacts/cpp_control_forward.csv
```

删掉 `--headless` 打开官方 MuJoCo Simulate 查看器。启动时先完成 MPC reset 和首条策略接收，再创建界面；主线程运行官方事件/渲染循环，C++ 控制线程等待模型加载、同步初始状态后建立真实时间起点。等待服务每 5 s 输出进度。原先在阻塞初始化前创建窗口、未处理事件导致的“无响应”已修正。下表四轮性能数据仍是此前的 headless 基线，官方界面的同步开销另行记录。

纯站立去掉 `--trot-at`、`--stop-trot-at` 和 `--forward`。左右转用 `--turn-angle-deg 10` / `--turn-angle-deg -10` 替代 `--forward`，保留 trot/stop 时间。当前运动仍是预生成的有限时间表。

默认 `ROS_DOMAIN_ID=216`，无需另外启动 MPC、MRT 服务或步态终端；请勿在该域同时运行另一套同名节点。脚本用独占锁避免重叠运行，默认 MPC 日志为 `artifacts/cpp_control_mpc.log`。首次启动可能重新编译 CppAD，reset 服务等待上限 90 s；服务调用与首策略等待也有超时。只关闭脚本自己启动的 MPC。`GO2_WORKSPACE`、`GO2_PYTHON`、`GO2_OSQP_ROOT`、`GO2_MUJOCO_ROOT` 可覆盖构建路径。

每次成功运行自动生成指定 CSV 和同名 JSON。若控制失败，先将仿真力矩清零，保存已完成的部分 CSV 并返回非零退出码。汇总工具拒绝不完整或非连续采样，不将半程运行写成通过。

### 验证与计时口径

Release 编译通过。启动时执行三组 Python/C++ 转换对照，包含倾斜姿态、非零基座/关节速度、未来姿态变化、mode 9/6 及跨触地摆腿加速度清零，最大绝对误差 1.11e-14，容差 1e-8。另确认目标关节顺序错误、非法 JSON NaN、非单位四元数、负运行时间均返回非零退出码。七组 WBC 数学对照沿用上一阶段结果；本阶段没有重新运行完整 pytest。

`control_ms` 从读取实测状态前计时到力矩写入后，包含观测发布、策略选择/激活、两次 MRT、参考转换、WBC 与 PD/限幅。`loop_ms` 还包含 `mj_step`、诊断、日志入内存、每秒终端输出和查看器状态同步；排除等待下个周期以及结束后的 CSV 写盘/JSON 汇总。接入官方 Simulate 后，`viewer_ms` 为控制线程调用 `Sync()` 的耗时（含互斥等待），主线程实际绘制不计入控制线程 `loop_ms`；此前自建窗口版本的该项包含同线程绘制，两者不能直接当成相同口径的渲染性能比较。`control_start_lateness_ms` 单独记录相对计划开始时刻的迟到。汇总排除前 2 s，共 9,000 步；这些是当前机器的短时测量，不是硬实时保证。

四轮最终版本均完成 20 s / 10,000 步，MPC 策略拒收数为 0；全部无力矩截断、支撑期地面力超过 1 N 的比例为 100%，摆腿目标高度超过 40 mm 时没有超过 1 N 的地面接触样本。CSV 的机身/足端位置是本步施加力矩前的测量值，地面接触力来自本次 `mj_step`，按执行步骤记录。

| 工况 | 最终 x（m） | 最终 yaw（°） | 高度跟踪 MPC RMSE（mm） | 最大执行力矩（N·m） |
| --- | ---: | ---: | ---: | ---: |
| 纯站立/高度变化 | 0.000233 | 0.000 | 0.079 | 7.699 |
| trot 前进停步，目标 90 mm | 0.090654 | 0.026 | 0.124 | 16.687 |
| trot 左转停步，目标 +10° | 0.001099 | 9.827 | 0.125 | 16.531 |
| trot 右转停步，目标 −10° | 0.001083 | −9.788 | 0.123 | 15.985 |

| 工况 | 控制平均/p99（ms） | 完整循环平均/p99/最大（ms） | 完整循环 >2 ms | 最大开始迟到（ms） |
| --- | ---: | ---: | ---: | ---: |
| 站立 | 0.125 / 0.265 | 0.156 / 0.343 / 0.486 | 0/9000 | 0.138 |
| 前进 | 0.155 / 0.376 | 0.199 / 0.470 / 0.707 | 0/9000 | 1.140 |
| 左转 | 0.161 / 0.388 | 0.207 / 0.485 / 0.651 | 0/9000 | 0.094 |
| 右转 | 0.156 / 0.402 | 0.202 / 0.499 / 0.650 | 0/9000 | 0.118 |

前进轮运动段 8–12 s 的目标 x RMSE 为 12.02 mm；最终位置超出目标 0.654 mm。姿态与接触残差均通过运行期检查：四轮最大动力学残差约 1.58e-5、最大支撑足加速度残差约 4.80e-6；策略最大年龄站立 38 ms、行走 34 ms。3 s 发送 trot，首次非四足支撑约 4.002–4.022 s；15 s 发送 stance，最后非四足支撑为 16.018 s，与 MPC 在预测时域末端插入步态的机制一致。

前进轮循环平均/p99 为 0.199/0.470 ms；此前 Python 阶段记录为 1.762/2.971 ms，2008/9000 步超过 2 ms。两轮的控制链、摆腿参考路径和诊断量并非完全相同，该比较用于阶段进度，不能解释为只改变语言的独立性能实验。剩余约 12 mm 的运动段目标滞后仍需继续处理；本地计算加快并未消除 MPC 目标跟踪偏差。

完整数据为 `artifacts/cpp_control_{standing,forward,yaw_left,yaw_right}.csv` 与同名 `.json`。它们是生成文件，由 `.gitignore` 排除，结果摘要保存在本文档。

### 旧版窗口启动修复与复验

用户反馈窗口“无响应”后，日志显示 MPC 仍在编译 CppAD。旧入口提前创建 GLFW 窗口，随后阻塞等待 reset 服务/首策略，期间未处理窗口事件。已把窗口创建和首次渲染移到首策略就绪之后，真实时间起点放在首次渲染完成后；等待服务每 5 s 打印进度，reset 和首策略也有明确状态输出。

修复后重新构建，实际带窗口运行前进停步 20 s / 10,000 步完成，退出码 0：最终 x=0.090672 m、最大执行力矩 16.709 N·m、截断 0 步、策略拒收 0 条。排除前 2 s 后，包含渲染的完整循环平均/p99/最大为 0.219/0.645/1.028 ms，0/9000 步超过 2 ms，最大开始迟到 0.079 ms。日志与结果为 `/tmp/go2_cpp_viewer_fix_run.log`、`artifacts/cpp_control_forward_viewer.csv` 和同名 JSON。这验证图形入口可完成运行，动作质量仍由人工观察补充。

### 官方 Simulate 查看器接入

当前默认图形入口已替换上述简易窗口，直接编译官方 MuJoCo 3.4.0 Simulate 源码，采用与 Python `launch_passive()` 相同的被动模式。左右侧栏、菜单、鼠标旋转/平移/缩放、相机选择、接触可视化、Profiler 和截图均来自官方实现。源码未修改，版本为 `e55fff5dea6f1d5dd7963ca52eecc41d05ad0922`；原许可证、PNG 依赖版本和校验和见 [third_party/mujoco_simulate/UPSTREAM.md](../third_party/mujoco_simulate/UPSTREAM.md)。构建使用已下载的源码，无需联网重新获取。

线程分工如下：

- 主线程创建 `GlfwAdapter` / `Simulate`，运行 `RenderLoop()` 并处理官方 UI。
- 控制线程独占实际仿真数据和 MPC/WBC 计算，每 20 ms 调用官方 `Sync()`，向查看器内部副本同步状态并接收界面操作；GUI 不执行额外 `mj_step()`。
- 接收线程仍负责 MPC 策略解码。当前与未来参考、PD、足序/坐标约定保持原有实现。
- 正常完成时控制线程请求关闭界面；窗口提前关闭时停止仿真力矩，保存部分 CSV 并退出。主线程回收控制线程后再销毁查看器和模型，模型加载等待也响应窗口退出。
- 界面 Reset、Load key/history 或改变 timestep 会使仿真时间/周期与当前 MPC 策略失配，因此停止本轮并提示重新启动 MPC 与仿真。被动模式下官方 Run/Pause 等项目本来就不可用，仿真推进由 C++ 控制入口负责；关节控制滑条也不会替代每步控制器输出。

运行命令不变，只需去掉 `--headless`。常用操作：左键拖动旋转，右键拖动平移，滚轮缩放；Tab 切换左侧栏，Shift+Tab 切换右侧栏；F1 帮助、F2 信息、F3 Profiler、F4 Sensor，Ctrl+P 截图。截图默认保存到启动时的当前目录。

已完成 Release 构建和首次官方界面的 20 s / 10,000 步前进停步，退出码 0：最终 x=0.090573 m，最大执行力矩 16.699 N·m，无截断、无策略拒收。排除前 2 s 后，控制计算平均/p99/最大为 0.157/0.378/0.842 ms；控制线程循环（含界面同步）为 0.285/1.256/4.845 ms，6/9000 步超过 2 ms，最大开始迟到 2.867 ms。首次运行日志为 `/tmp/go2_official_viewer_run.log`。

2026-10-16 本地最新日志复核：再次完整完成 20 s / 10,000 步，最终 x=0.090628 m（目标 0.09 m），最大执行力矩 16.722 N·m、无截断，支撑期接触比例四足均为 100%，抬高摆腿的地面接触样本均为 0。排除前 2 s 后，控制计算平均/p99/最大为 0.155/0.367/0.575 ms；控制线程循环（含界面同步）为 0.259/1.033/2.535 ms，1/9000 步超过 2 ms，最大开始迟到 0.595 ms。数据为 `artifacts/cpp_control_official_viewer.csv` 与同名 JSON，当前文件已由这轮复跑覆盖。官方 UI 的同步/锁等待有额外开销，不能沿用 headless 的零超时结论；不同次运行的峰值也会变化。

另已实际截取官方左右侧栏界面，并通过 Tab 隐藏左侧栏；截图为 `artifacts/cpp_control_official_viewer.png`。2026-10-16 恢复后补充退出检查：发送鼠标拖动事件，并在 4.856 s 通过 `WM_DELETE_WINDOW` 提前关闭本次窗口，程序保存 2,428 步部分 CSV，返回预期的中断退出码 1，控制线程、界面和配套 MPC 正常结束，无卡死。数据为 `artifacts/cpp_control_official_ui_close.csv`，日志为 `/tmp/go2_official_ui_probe_close/run.log`。提前关闭不生成成功运行的 JSON 汇总。

自动脚本中 Profiler 的截图快捷键未触发，未将其记为验证通过；鼠标相机变化和 Profiler 画面仍需人工观察。前述完整 20 s 闭环结果与提前关闭检查分开记录，未重新运行完整 pytest。

仍需完成：窗口动作人工观察、更长时间/负载下的时序、在线运动指令、上游 SQP/任务对照、抗扰与状态估计、SDK2 接入及实机验收。现有状态输入仍为 MuJoCo 真值。
