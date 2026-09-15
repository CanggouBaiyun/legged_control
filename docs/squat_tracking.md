# 四足支撑蹲起：跟踪修复记录

## 结果

同一 MuJoCo 场景、同一初始状态、2 ms 步长。先保持 2 s，再执行 4 cm 深、4 s 周期的蹲起，总时长 10 s。
高度误差在每个控制周期记录，统计窗口为 t >= 2 s；力矩峰值和限幅次数统计整个运行过程。

| 指标 | 原速度反馈基线 | 关节参考 PD |
| --- | ---: | ---: |
| 高度 RMSE | 9.196 mm | 3.350 mm |
| 最大高度误差 | 14.302 mm | 4.983 mm |
| 最大 roll/pitch 合成角度 | 2.359° | 0.576° |
| 最大水平位置偏移 | 9.985 mm | 5.767 mm |
| 最大电机命令力矩 | 7.454 Nm | 7.489 Nm |
| 最少接触脚数（评估窗口） | 4 | 4 |
| 力矩限幅次数 | 0 | 0 |

补充验证：

| 工况 | 时长 | 高度 RMSE | 最大高度误差 |
| --- | ---: | ---: | ---: |
| 4 cm / 4 s 周期 | 30 s | 3.270 mm | 4.983 mm |
| 4 cm / 3 s 周期 | 14 s | 3.340 mm | 5.075 mm |
| 6 cm / 6 s 周期 | 14 s | 3.501 mm | 5.103 mm |

以上工况均未出现 QP 求解失败或力矩限幅，评估窗口内四只脚均有地面接触。
这不代表真机部署或扰动鲁棒性已经验证。

## 改了什么

### 1. 修正 MJCF 根节点坐标

Pinocchio 读取这个 MJCF 时保留了根节点的初始 placement：z = 0.445 m。
bridge 又把 MuJoCo 中的绝对基座位置直接写入 q[:3]，导致位置被重复偏移。
home 状态下基座的 Pinocchio FK 曾是 0.715 m，而 MuJoCo 是 0.270 m。

现在模型构建时将自由根关节 placement 设为单位变换，保证 q[:7] 表示绝对世界位姿。
质量矩阵与重力对比没有暴露这个固定平移问题；新增的世界系基座、足端和质心测试可以检测它。
这个修复保证位置参考可用，不能把之前的所有动力学跟踪误差都归因于它。

### 2. 关节参考不再只有速度

原来只有 Kd_j * (v_j_des - v_j)，运动变慢以后，速度误差可能很小，但位置仍然偏离目标。

现在由同一条基座轨迹生成：

- 固定足端世界位置，逆运动学求 q_j_des；
- 在参考构型上求解 J_b v_b_des + J_j v_j_des = 0，得到 v_j_des；
- 用参考关节位置和速度计算反馈力矩。

最终电机命令：

    tau_cmd = clip(tau_wbc + Kp_j * (q_j_des - q_j)
                           + Kd_j * (v_j_des - v_j))

默认 Kp_j = 20 Nm/rad，Kd_j = 1 Nm·s/rad。这里的数值是本项目仿真参数，不是照搬原工程或真机参数。
IK 用上一帧参考构型做初值，不从实际构型重新积分生成位置参考，以免把实际跟踪误差带进目标。

### 3. 基座任务补上轨迹加速度前馈

    a_task_world = a_traj_world + Kp_b * (p_des - p)
                               + Kd_b * (v_des - v)

再转换成 Pinocchio 的局部切空间加速度，保留原来的旋转坐标修正和加速度限幅。
单独加入该前馈的对照实验 RMSE 为 9.417 mm，没有改善原基线；主要改善来自一致的关节位置参考和非零 P 项。

### 4. 保留真实仿真设置和诊断入口

没有修改第三方 MJCF；关节 damping=0.1、frictionloss=0.2、armature=0.01，以及足端接触参数保持不变。
旧诊断脚本完整保存在 examples/mujoco/wbc_squat_diagnostics.py。
正常入口只保留每秒一行日志，支持无窗口验证与 CSV 导出。

## 运行

从项目根目录，在 go2_control 环境中执行：

```bash
# 图形展示
python examples/mujoco/wbc_squat.py

# 同一个控制循环的基线/修复对照
python examples/mujoco/wbc_squat.py --headless --mode baseline --csv artifacts/squat_baseline.csv
python examples/mujoco/wbc_squat.py --headless --csv artifacts/squat_tracking.csv

# 延长运行
python examples/mujoco/wbc_squat.py --headless --duration 30

# 自动测试
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

CSV 旁会生成同名 JSON 指标文件。baseline 模式关闭轨迹加速度前馈和关节位置反馈，保留旧速度反馈方式。
参考轨迹开始时位置、速度连续，但加速度有有限跳变；目前还不是 jerk 连续的轨迹。

## 验证范围和剩余问题

- 23 项自动测试通过，包含模型世界坐标、质量矩阵/重力一致性、轨迹导数、固定足 IK、关节 PD 和 6.1 s 闭环回归。
- 当前是四足支撑、平地、使用仿真完整状态的蹲起示例，不包含状态估计、MPC 或接触切换。
- 仍有约 5 mm 的最大高度误差，不能由 QP 内部残差很小推断实际跟踪误差很小。
- WBC 使用三维点接触力；仿真使用球形脚和 condim=6 接触。固定足参考用球半径确定高度，未精确描述球面滚动接触点。
- PD 反馈在 QP 外相加，因此 QP 中的力矩约束不能单独保证最终命令合规；代码对最终电机命令再次限幅，并记录次数。
- 下一阶段以此为基线实现三足支撑/单腿抬起，而不是继续无限追加蹲起诊断量。

验证版本：MuJoCo 3.4.0、Pinocchio 3.9.0、OSQP 1.1.3。
unitree_mujoco commit: 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d。
