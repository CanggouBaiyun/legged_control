#!/usr/bin/env bash
set -eo pipefail
GO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_WORKSPACE="${GO2_WORKSPACE:-${GO2_PROJECT_ROOT}/../ocs2_ws}"
GO2_PYTHON="${GO2_PYTHON:-/home/bot/miniconda3/envs/go2_control/bin/python}"
GO2_OSQP_ROOT="${GO2_OSQP_ROOT:-$(dirname -- "$(dirname -- "${GO2_PYTHON}")")}"
GO2_MUJOCO_ROOT="${GO2_MUJOCO_ROOT:-$(env -u PYTHONPATH "${GO2_PYTHON}" -c 'import pathlib, mujoco; print(pathlib.Path(mujoco.__file__).parent)')}"
source /opt/ros/jazzy/setup.bash
source "${GO2_WORKSPACE}/install/local_setup.bash"
cd "${GO2_WORKSPACE}"
env PATH=/usr/bin:/bin:/usr/sbin:/sbin CMAKE_BUILD_PARALLEL_LEVEL=1 \
  /usr/bin/python3 -m colcon build --packages-select go2_control_cpp --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/jazzy \
    -Dpinocchio_DIR=/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/pinocchio \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DGO2_OSQP_ROOT="${GO2_OSQP_ROOT}" -DGO2_MUJOCO_ROOT="${GO2_MUJOCO_ROOT}"
