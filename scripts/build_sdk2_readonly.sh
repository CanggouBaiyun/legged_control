#!/usr/bin/env bash
set -eo pipefail
GO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_SDK2_PREFIX="${GO2_SDK2_PREFIX:-${GO2_PROJECT_ROOT}/../unitree_sdk2/install}"
if [[ ! -f "${GO2_SDK2_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]]; then
  printf 'SDK2 CMake package not found under %s\n' "${GO2_SDK2_PREFIX}" >&2
  exit 1
fi
env PATH=/usr/bin:/bin /usr/bin/cmake \
  -S "${GO2_PROJECT_ROOT}/hardware/sdk2_readonly" \
  -B "${GO2_PROJECT_ROOT}/build/sdk2_readonly" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_PREFIX_PATH="${GO2_SDK2_PREFIX}"
env PATH=/usr/bin:/bin /usr/bin/cmake \
  --build "${GO2_PROJECT_ROOT}/build/sdk2_readonly" --parallel 2
