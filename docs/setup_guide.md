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

## 7. OCS2 ROS 2 工作区

当前 OCS2 集成基于 ROS 2 Jazzy。Go2 所需的模型名称配置和 RViz
关节名称修正保存在项目维护的 OCS2 fork 中：

```text
Repository: https://github.com/CanggouBaiyun/ocs2
Branch: feature/go2-model-config
Verified commit: 083c222ee7876d449099fb750a79292291a57050

Robotic assets: https://github.com/leggedrobotics/ocs2_robotic_assets
Branch: ros2
Verified commit: 9feab34
```

建立工作区：

```bash
mkdir -p /home/bot/Project/ocs2_ws/src
cd /home/bot/Project/ocs2_ws/src

git clone \
  --branch feature/go2-model-config \
  https://github.com/CanggouBaiyun/ocs2.git \
  ocs2

git -C ocs2 remote add \
  upstream https://github.com/leggedrobotics/ocs2.git

git clone \
  --branch ros2 \
  https://github.com/leggedrobotics/ocs2_robotic_assets.git
```

构建 Go2 MPC 所需的 OCS2 包及依赖：

```bash
cd /home/bot/Project/ocs2_ws
source /opt/ros/jazzy/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=2 \
colcon build \
  --packages-up-to \
    ocs2_legged_robot_ros \
    ocs2_double_integrator \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`go2_mpc_bridge` 的源码由主项目 Git 管理，通过符号链接加入 OCS2
工作区，避免维护两份代码：

```bash
cd /home/bot/Project/ocs2_ws/src

ln -s \
  /home/bot/Project/go2_traditional_locomotion/ros2/go2_mpc_bridge \
  go2_mpc_bridge
```

构建桥接包：

```bash
cd /home/bot/Project/ocs2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=2 \
colcon build \
  --packages-select go2_mpc_bridge \
  --symlink-install
```

验证安装结果：

```bash
source /home/bot/Project/ocs2_ws/install/setup.bash

ros2 pkg prefix go2_mpc_bridge
ros2 pkg executables go2_mpc_bridge
ros2 run go2_mpc_bridge mrt_interface_check
```

预期能发现以下三个可执行程序：

```text
mrt_interface_check
mrt_policy_check
mrt_policy_server
```

Python 控制环境与 ROS 2 Jazzy 使用不同的 Python 版本。运行 Conda
环境下的测试时，应避免 ROS 2 的 Python 3.12 路径覆盖 Conda 的
Python 3.10 包：

```bash
env -u PYTHONPATH \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
/home/bot/miniconda3/envs/go2_control/bin/python \
-m pytest -q
```
