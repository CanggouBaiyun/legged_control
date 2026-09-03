# M1 环境与官方模型准备

以下命令由使用者手动执行；项目中的程序不会自动安装软件或下载仓库。

## 1. 进入项目

```bash
cd /home/bot/Project/go2_traditional_locomotion
```

## 2. 建立独立 Conda 环境

不要安装到现有 `freefall` 或 `MJX` 环境，以免 Pinocchio 的 C++ 依赖和
NumPy 版本影响已有强化学习项目。

```bash
conda env create -f environment.yml
conda activate go2_control
python -m pip install -e .
```

检查环境：

```bash
python scripts/check_environment.py
```

预期至少看到：

```text
pinocchio: OK
mujoco: OK
numpy: OK
scipy: OK
osqp: OK
```

## 3. 手动下载官方 Go2 模型

### 3.1 URDF

使用稀疏克隆，只获取 `unitree_ros` 中的 Go2 描述：

```bash
mkdir -p third_party
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/unitreerobotics/unitree_ros.git \
  third_party/unitree_ros
git -C third_party/unitree_ros sparse-checkout set robots/go2_description
```

预期 URDF：

```text
third_party/unitree_ros/robots/go2_description/urdf/go2_description.urdf
```

### 3.2 MuJoCo MJCF

```bash
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/unitreerobotics/unitree_mujoco.git \
  third_party/unitree_mujoco
git -C third_party/unitree_mujoco sparse-checkout set unitree_robots/go2
```

预期 MJCF：

```text
third_party/unitree_mujoco/unitree_robots/go2/go2.xml
```

现阶段不需要下载或编译 SDK2；等进入仿真接口和实机通信阶段再做。

## 4. 固定上游版本

模型下载完成后执行：

```bash
./scripts/record_model_versions.sh
cat third_party/model_versions.txt
```

不要只记录“最新版”。必须保留 commit，这样之后上游模型变化时仍能解释
质量、惯量、关节限制或仿真表现为何不同。

## 5. 第一组检查

不依赖 Pinocchio 的原始 URDF 盘点：

```bash
python tools/urdf_inventory.py
```

Pinocchio 自由浮动模型检查：

```bash
python tools/model_inspect.py
pytest
```

预期生成：

```text
artifacts/go2_urdf_inventory.json
artifacts/go2_model_summary.json
```

## 6. 遇到问题时提供这些信息

```bash
conda env list
python scripts/check_environment.py
cat third_party/model_versions.txt
python tools/urdf_inventory.py
python tools/model_inspect.py
pytest -q
```

把完整输出发回学习记录，不要只截取最后一行报错。

