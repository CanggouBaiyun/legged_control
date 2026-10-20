#!/usr/bin/env bash
# Build and run offline command conversion checks; no DDS participant is created.
set -eo pipefail
GO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_SDK2_PREFIX="${GO2_SDK2_PREFIX:-${GO2_PROJECT_ROOT}/../unitree_sdk2/install}"
if [[ ! -f "${GO2_SDK2_PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]]; then
  printf 'SDK2 CMake package not found under %s\n' "${GO2_SDK2_PREFIX}" >&2
  exit 1
fi
env PATH=/usr/bin:/bin /usr/bin/cmake \
  -S "${GO2_PROJECT_ROOT}/hardware/sdk2_adapter" -B "${GO2_PROJECT_ROOT}/build/sdk2_adapter" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_PREFIX_PATH="${GO2_SDK2_PREFIX}"
env PATH=/usr/bin:/bin /usr/bin/cmake --build "${GO2_PROJECT_ROOT}/build/sdk2_adapter" --parallel 2
exec env LD_LIBRARY_PATH="${GO2_SDK2_PREFIX}/lib" \
  "${GO2_PROJECT_ROOT}/build/sdk2_adapter/go2_sdk2_adapter_check" \
  --project-root "${GO2_PROJECT_ROOT}" "$@"
