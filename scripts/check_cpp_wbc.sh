#!/usr/bin/env bash
set -eo pipefail

GO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_WORKSPACE="${GO2_WORKSPACE:-${GO2_PROJECT_ROOT}/../ocs2_ws}"
GO2_PYTHON="${GO2_PYTHON:-/home/bot/miniconda3/envs/go2_control/bin/python}"
GO2_OSQP_ROOT="${GO2_OSQP_ROOT:-$(dirname -- "$(dirname -- "${GO2_PYTHON}")")}"
GO2_INPUT="${GO2_PROJECT_ROOT}/artifacts/cpp_wbc_cases.json"
GO2_OUTPUT="${GO2_PROJECT_ROOT}/artifacts/cpp_wbc_output.json"

printf '%s\n' 'Preparing reference QPs with the existing Python WBC...'
env -u PYTHONPATH "${GO2_PYTHON}" \
  "${GO2_PROJECT_ROOT}/examples/wbc/check_cpp_wbc.py" --prepare "${GO2_INPUT}"

(
  source /opt/ros/jazzy/setup.bash
  source "${GO2_WORKSPACE}/install/local_setup.bash"
  cd "${GO2_WORKSPACE}"
  env PATH=/usr/bin:/bin:/usr/sbin:/sbin CMAKE_BUILD_PARALLEL_LEVEL=1 \
    /usr/bin/python3 -m colcon build \
    --packages-select go2_control_cpp --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/opt/ros/jazzy \
      -Dpinocchio_DIR=/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/pinocchio \
      -DPython3_EXECUTABLE=/usr/bin/python3 \
      -DGO2_OSQP_ROOT="${GO2_OSQP_ROOT}"
  source "${GO2_WORKSPACE}/install/local_setup.bash"
  ros2 run go2_control_cpp wbc_replay "${GO2_PROJECT_ROOT}" "${GO2_INPUT}" "${GO2_OUTPUT}"
)

printf '%s\n' 'Comparing native C++ matrices, solutions and torques with Python...'
env -u PYTHONPATH "${GO2_PYTHON}" \
  "${GO2_PROJECT_ROOT}/examples/wbc/check_cpp_wbc.py" --check "${GO2_INPUT}" "${GO2_OUTPUT}"
