#!/usr/bin/env bash
# Owns one external MPC process; no MRT server or Python transport is needed.
set -eo pipefail
if [[ "${1:-}" == --help ]]; then
  printf '%s\n' 'Usage: bash scripts/run_cpp_simulation.sh [--headless] [--duration 20]' \
    '  [--trot-at 3] [--stop-trot-at 15] [--forward] [--turn-angle-deg -10] [--csv PATH]' \
    'Runs one external DDP MPC and the native C++ simulation; closes its own MPC on exit.'
  exit 0
fi
exec 200>/tmp/go2_cpp_simulation.lock
if ! flock -n 200; then
  printf '%s\n' 'Another native simulation is running; finish it before starting another.' >&2; exit 1
fi
GO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
GO2_WORKSPACE="${GO2_WORKSPACE:-${GO2_PROJECT_ROOT}/../ocs2_ws}"
GO2_PYTHON="${GO2_PYTHON:-/home/bot/miniconda3/envs/go2_control/bin/python}"
GO2_TARGET="${GO2_TARGET:-${GO2_PROJECT_ROOT}/artifacts/cpp_control_target.json}"
GO2_MPC_LOG="${GO2_MPC_LOG:-${GO2_PROJECT_ROOT}/artifacts/cpp_control_mpc.log}"
GO2_DURATION=20
GO2_CSV="${GO2_PROJECT_ROOT}/artifacts/cpp_control.csv"
GO2_TARGET_ARGS=()
GO2_CONTROL_ARGS=()
while (($#)); do
  case "$1" in
    --duration|--csv)
      if (($#<2)); then printf '%s\n' "$1 requires a value" >&2; exit 2; fi
      if [[ "$1" == --duration ]]; then GO2_DURATION="$2"; else GO2_CSV="$2"; fi
      GO2_CONTROL_ARGS+=("$1" "$2"); shift 2 ;;
    --forward) GO2_TARGET_ARGS+=("$1"); shift ;;
    --turn-angle-deg)
      if (($#<2)); then printf '%s\n' '--turn-angle-deg requires a value' >&2; exit 2; fi
      GO2_TARGET_ARGS+=("$1" "$2"); shift 2 ;;
    --turn-angle-deg=*) GO2_TARGET_ARGS+=("$1"); shift ;;
    *) GO2_CONTROL_ARGS+=("$1"); shift ;;
  esac
done
env -u PYTHONPATH "${GO2_PYTHON}" "${GO2_PROJECT_ROOT}/examples/mujoco/prepare_cpp_target.py" \
  "${GO2_TARGET}" "${GO2_TARGET_ARGS[@]}"
source /opt/ros/jazzy/setup.bash
source "${GO2_WORKSPACE}/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-216}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/go2_cpp_control_ros_logs}"
GO2_BINARY="${GO2_WORKSPACE}/install/go2_control_cpp/lib/go2_control_cpp/control_simulation"
GO2_MPC_BINARY="${GO2_WORKSPACE}/install/ocs2_legged_robot_ros/lib/ocs2_legged_robot_ros/legged_robot_ddp_mpc"
if [[ ! -x "${GO2_BINARY}" ]]; then
  printf '%s\n' 'Build first: bash scripts/build_cpp_control.sh' >&2; exit 1
fi
"${GO2_BINARY}" --project-root "${GO2_PROJECT_ROOT}" --target "${GO2_TARGET}" --check-only
mkdir -p -- "$(dirname -- "${GO2_MPC_LOG}")"
"${GO2_MPC_BINARY}" --ros-args \
  -p taskFile:="${GO2_PROJECT_ROOT}/config/ocs2/go2/task.info" \
  -p referenceFile:="${GO2_PROJECT_ROOT}/config/ocs2/go2/reference.info" \
  -p urdfFile:="${GO2_PROJECT_ROOT}/models/ocs2/go2_mujoco_inertial.urdf" \
  >"${GO2_MPC_LOG}" 2>&1 &
GO2_MPC_PID=$!
go2_stop_mpc() {
  kill -INT "${GO2_MPC_PID}" 2>/dev/null || true
  for ((go2_wait=0;go2_wait<30;go2_wait++)); do
    if ! kill -0 "${GO2_MPC_PID}" 2>/dev/null; then break; fi
    sleep 0.1
  done
  kill -TERM "${GO2_MPC_PID}" 2>/dev/null || true
  wait "${GO2_MPC_PID}" 2>/dev/null || true
}
trap go2_stop_mpc EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'Native simulation ROS_DOMAIN_ID=%s; MPC log: %s\n' "${ROS_DOMAIN_ID}" "${GO2_MPC_LOG}"
"${GO2_BINARY}" --project-root "${GO2_PROJECT_ROOT}" --target "${GO2_TARGET}" "${GO2_CONTROL_ARGS[@]}"
env -u PYTHONPATH "${GO2_PYTHON}" "${GO2_PROJECT_ROOT}/tools/summarize_cpp_control.py" \
  "${GO2_CSV}" --duration "${GO2_DURATION}"
