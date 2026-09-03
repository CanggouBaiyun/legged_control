# Go2 Traditional Locomotion

面向 Unitree Go2 的传统四足运动控制学习与复现项目。

```text
Go2 URDF + Pinocchio
        ↓
MuJoCo 模型与状态映射
        ↓
浮动基状态估计
        ↓
WBC-QP
        ↓
OCS2 NMPC
        ↓
Unitree SDK2 实机部署
```

`legged_control` 只作为 NMPC-WBC 架构与代码组织参考，不直接复用其
A1/ROS 1 硬件层。

## 当前里程碑：M1 Go2 模型基线

在写控制器之前，先完成：

- 固定官方 Go2 URDF 与 MJCF 版本；
- 使用 Pinocchio 建立自由浮动模型；
- 盘点 link、joint、frame、质量、惯量和关节限制；
- 显式建立 SDK2 电机顺序到 Pinocchio 索引的映射；
- 检查名义站立姿态下四足位置和质量矩阵；
- 数值验证 CRBA、RNEA 和非线性项的一致性。

## 开始使用

本项目不会自动下载模型或安装依赖。请依次阅读并执行：

1. [`docs/setup_guide.md`](docs/setup_guide.md)
2. [`docs/roadmap.md`](docs/roadmap.md)

完成安装和模型下载后运行：

```bash
conda activate go2_control
python -m pip install -e .
python scripts/check_environment.py
python tools/urdf_inventory.py
python tools/model_inspect.py
pytest
```

生成结果保存在 `artifacts/`，上游模型 commit 记录在
`third_party/model_versions.txt`。

## 目录

```text
configs/                  模型与名义姿态配置
docs/                     路线、安装步骤和推导笔记
scripts/                  环境与版本检查工具
src/go2_control/          可复用 Python 模块
tests/                    数值验收测试
tools/                    学习和模型检查程序
third_party/              手动下载的 Unitree 官方仓库
artifacts/                自动生成的检查报告
```

## 安全边界

当前阶段没有任何程序连接或控制真实 Go2。只有在模型映射、MuJoCo WBC、
状态估计、命令限幅、通信看门狗和悬吊测试全部通过后，才会加入实机
`LowCmd` 输出。

