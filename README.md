# Go2 Legged_Control

面向 Unitree Go2 的legged_control复现。

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


## 当前进度：M1 Go2 模型基线

在写控制器之前，已完成：

- 固定官方 Go2 URDF 与 MJCF 版本；
- 使用 Pinocchio 建立自由浮动模型；
- 盘点 link、joint、frame、质量、惯量和关节限制；
- 显式建立 SDK2 电机顺序到 Pinocchio 索引的映射；
- 检查名义站立姿态下四足位置和质量矩阵；
- 数值验证 CRBA、RNEA 和非线性项的一致性。



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



