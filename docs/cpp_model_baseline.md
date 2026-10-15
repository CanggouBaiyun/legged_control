# C++ 模型、动力学与 WBC 基线

## 当前状态（2026-10-15）

`feature/cpp-model-baseline` 已接入 `ros2/go2_control_cpp`，通过工作空间符号链接构建。模型/动力学实现已在项目中运行，并完成 C++/Python 三组状态对照。C++ WBC 和 C++ 闭环尚未实现。模型配置统一为 `config/go2_model.json`。

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

本组候选源码位于 `/tmp/go2_cpp_wbc_step`，已编译并完成下述对照；补丁 `/tmp/go2_cpp_wbc_step.patch` 等待用户应用到仓库。模型/动力学已经接入工作空间，WBC 的正式工作空间接入尚未完成。

### 实现与范围

- `include/go2_control_cpp/wbc.hpp`：基座/摆腿参考、QP、求解结果及控制器接口。
- `src/wbc.cpp`：完整浮动基动力学、支撑足加速度、摩擦/单边力/法向力上限、电机力矩限制；基座位置/姿态任务、摆腿前馈和反馈、接触力跟踪及正则。
- `src/osqp_solver.cpp`：原生 OSQP C API，采用 builtin 后端，polishing=true、max_iter=20000、warm_starting=false；稀疏结构相同则更新，变化时重新 setup。只接受 solved 状态并检查输出/约束。
- `src/wbc_replay.cpp`：从相同输入离线组装 QP、求解、导出矩阵和力矩；检查更新后的工作区与新建求解器结果一致，以及恢复原问题后的解一致。
- `examples/wbc/check_cpp_wbc.py`：调用现有 Python WBC 生成参考 QP 和结果，按关节名称对照 C++ 输出。
- `scripts/check_cpp_wbc.sh`：一条命令完成参考生成、定向构建、C++ 回放和 Python 比较。

保留现有 Python 数学问题和参数。四足为 42 个变量，两足为 36 个变量；接触力遵循传入的足名顺序。摆腿使用同一实测状态下的完整 Jacobian 数据，不重复计算质量矩阵。

C++ WBC 输出的是 QP 力矩，尚未接入关节 PD、MRT 参考转换、MuJoCo 控制循环或 SDK2。默认的 Python/MuJoCo 控制入口没有改动。上游任务定义和 SQP 对照仍按开发路线进行。

### OSQP 依赖

本机 Python `osqp` 包为 1.1.3，供 C++ 使用的 `libosqp` 为 1.0.0；它们是不同的版本指标。候选补丁在 `environment.yml` 中显式声明 `libosqp>=1,<2`，本机已安装，无需重新安装环境。

CMake 通过 `GO2_OSQP_ROOT` 指定成对的 `include/osqp` 与 `lib/libosqp.so`。仅将 OSQP C 头文件目录加入编译，通过绝对路径在启动时动态装载 C API；不将整个 Conda 前缀加入 CMake/Pinocchio 搜索路径。当前 C 库仅依赖 qdldl 和系统 C 库，已核对并测试。更换部署机器时需配置并验证对应平台的 C 库和 ABI，不直接复制本机二进制。

未设置 `GO2_OSQP_ROOT` 时仍可构建模型检查；设置后才启用 WBC 库和 `wbc_replay`。构建脚本会传入本机已验证的前缀，可通过 `GO2_PYTHON` / `GO2_OSQP_ROOT` / `GO2_WORKSPACE` 覆盖路径。

### 已验证结果

临时目录 Release 构建通过，7 组离线输入全部求解并通过 Python/C++ 对照：四足、mode 9、mode 6 的静态和非零运动状态，以及四足法向力上限工况。双足样本包含两只脚的非零摆腿位置、速度、加速度参考。它们是合成的数学对照样本，尚非真实 trot 轨迹回放或闭环稳定性证明。

| 指标 | 最大值 |
| --- | ---: |
| H / g 元素绝对差 | 1.46e-11 / 2.91e-10 |
| 动力学/接触等式矩阵绝对差 | 5.55e-17 |
| 不等式矩阵、上下界绝对差 | 0 |
| 整个 QP 解的绝对差 | 2.24e-11 |
| 电机力矩绝对差 | 6.56e-12 N·m |
| C++ 动力学/接触等式残差范数 | 5.81e-14 |
| 不等式违反量 | 0 |

矩阵比较采用 atol=1e-8、rtol=1e-9；解比较采用 atol=1e-4、rtol=1e-5。另确认 7 类非法输入显式拒绝，缺失 C 库、不可行 QP、非有限 QP 也抛出错误；失败时不返回关节命令。尚未重新运行原有完整 pytest，也未测量控制循环实时性能，本步骤针对新增候选模块做定向验证。

### 接入与运行

从当前已推送的模型基线创建 WBC 分支并应用完整补丁：

```bash
cd /home/bot/Project/go2_traditional_locomotion
git switch -c feature/cpp-wbc
git apply /tmp/go2_cpp_wbc_step.patch
bash scripts/check_cpp_wbc.sh
```

预期最后输出：

```text
PASS: C++ WBC QP matrices, solutions and torque outputs match Python in all 7 cases.
```

生成数据为 `artifacts/cpp_wbc_cases.json`、`cpp_wbc_output.json` 和 `cpp_wbc_output_comparison.json`。不需要 MPC/MRT 节点或机器人连接。通过后提交 WBC 源码与文档；下一阶段将本地 MRT、参考转换、关节 PD 和 C++ WBC 接入同一控制入口，再进行 MuJoCo 闭环验证。
