# C++ 模型与动力学基线

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
