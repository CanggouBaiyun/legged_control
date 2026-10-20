#!/usr/bin/env bash
set -eo pipefail
GO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_SDK2_PREFIX="${GO2_SDK2_PREFIX:-${GO2_PROJECT_ROOT}/../unitree_sdk2/install}"
GO2_SDK2_BINARY="${GO2_PROJECT_ROOT}/build/sdk2_readonly/go2_read_lowstate"
if [[ ! -x "${GO2_SDK2_BINARY}" ]]; then
  printf '%s\n' 'Build first: bash scripts/build_sdk2_readonly.sh' >&2
  exit 1
fi
# Use SDK2's matching DDS libraries even if this shell previously sourced ROS.
exec env LD_LIBRARY_PATH="${GO2_SDK2_PREFIX}/lib" "${GO2_SDK2_BINARY}" "$@"
