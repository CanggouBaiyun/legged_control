#!/usr/bin/env bash
# Owns the MPC, native control and optional online command processes.
set -eo pipefail
if [[ "${1:-}" == --help ]]; then
  printf '%s\n' 'Usage: bash scripts/run_cpp_simulation.sh [--headless] [--duration 20]' \
    '  [--mpc-solver ddp|sqp] (default: ddp)' \
    '  [--joint-kp KP] [--joint-kd KD] (defaults: 20, 1)' \
    '  [--centroidal-base-accel] (default: existing tracking task)' \
    '  [--estimator-shadow] (log Kalman estimates; control uses simulation truth)' \
    '  [--estimator-control] (simulation truth startup, then estimated state after readiness checks)' \
    '  [--sensor-startup] (nominal joint preparation -> estimated-state MPC/WBC; no base p/v truth in control)' \
    '  [--sensor-errors config/sensors/moderate_errors.json] (seeded measurement errors; requires --sensor-startup)' \
    '  [--sensor-event delay|dropout|contact-on|contact-off --sensor-event-at 8 --sensor-event-duration 0.01]' \
    '  [--sensor-delay-ms 2] [--sensor-contact-foot fl]; same-step freshness guard stays active.' \
    '  [--request-stop-at SECONDS] (zero target + stance, settle 0.3 s, end session; timeout 5 s)' \
    '  [--push-at 8 --push-duration 0.2 --push-fy 20] (world-frame N at base_link COM; --push-fx/--push-fz also supported)' \
    '  Stop topic: /go2_control/stop (std_msgs/msg/Empty); this ends the simulation session.' \
    '  Fault checks: --inject-fault sensor-stale|estimator-invalid|policy-stale|wbc-invalid --inject-at SECONDS' \
    '  [--wbc-config JSON] (relative to project root; default: built-in WBC settings)' \
    '  [--online] (starts /cmd_vel targets; permits duration >20 s; no --forward/--turn-angle-deg)' \
    '  Online mode without --duration (or --duration 0) runs until you close the window or press Ctrl+C.' \
    '  [--online-reference measured|integrated] (default: integrated; requires --online)' \
    '  [--trot-at 3] [--stop-trot-at 15] [--forward] [--turn-angle-deg -10] [--csv PATH]' \
    'Online velocities are body-heading vx/vy and yaw rate; publish repeatedly (timeout: 0.5 s).' \
    'Zero velocity holds position; it does not switch the gait to stance.' \
    'Online body pose: /go2_control/body_pose (Pose z/roll/pitch); keyboard R/F, I/K, J/L, U.' \
    'Closes its own processes on exit; stops control if an owned worker exits.'
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
GO2_DURATION=
GO2_MPC_SOLVER=ddp
GO2_ONLINE=0
GO2_ONLINE_REFERENCE=integrated
GO2_REFERENCE_SPECIFIED=0
GO2_CSV="${GO2_PROJECT_ROOT}/artifacts/cpp_control.csv"
GO2_TARGET_ARGS=()
GO2_CONTROL_ARGS=()
while (($#)); do
  case "$1" in
    --mpc-solver)
      if (($#<2)); then
        printf '%s\n' '--mpc-solver requires ddp or sqp' >&2
        exit 2
      fi
      GO2_MPC_SOLVER="$2"
      shift 2 ;;
    --duration|--csv)
      if (($#<2)); then printf '%s\n' "$1 requires a value" >&2; exit 2; fi
      if [[ "$1" == --duration ]]; then GO2_DURATION="$2"; else GO2_CSV="$2"; fi
      if [[ "$1" == --csv ]]; then GO2_CONTROL_ARGS+=("$1" "$2"); fi
      shift 2 ;;
    --online) GO2_ONLINE=1; GO2_CONTROL_ARGS+=("$1"); shift ;;
    --online-reference)
      if (($#<2)); then printf '%s\n' '--online-reference requires measured or integrated' >&2; exit 2; fi
      GO2_ONLINE_REFERENCE="$2"; GO2_REFERENCE_SPECIFIED=1; shift 2 ;;
    --forward) GO2_TARGET_ARGS+=("$1"); shift ;;
    --turn-angle-deg)
      if (($#<2)); then printf '%s\n' '--turn-angle-deg requires a value' >&2; exit 2; fi
      GO2_TARGET_ARGS+=("$1" "$2"); shift 2 ;;
    --turn-angle-deg=*) GO2_TARGET_ARGS+=("$1"); shift ;;
    *) GO2_CONTROL_ARGS+=("$1"); shift ;;
  esac
done
if [[ -z "${GO2_DURATION}" ]]; then
  if [[ "${GO2_ONLINE}" == 1 ]]; then GO2_DURATION=0; else GO2_DURATION=20; fi
fi
GO2_CONTROL_ARGS+=(--duration "${GO2_DURATION}")
if [[ "${GO2_REFERENCE_SPECIFIED}" == 1 && "${GO2_ONLINE}" != 1 ]]; then
  printf '%s\n' '--online-reference requires --online' >&2; exit 2
fi
case "${GO2_ONLINE_REFERENCE}" in
  measured) GO2_INTEGRATE_REFERENCE=false ;;
  integrated) GO2_INTEGRATE_REFERENCE=true ;;
  *) printf '%s\n' '--online-reference requires measured or integrated' >&2; exit 2 ;;
esac
if [[ "${GO2_ONLINE}" == 1 && ${#GO2_TARGET_ARGS[@]} -gt 0 ]]; then
  printf '%s\n' '--online cannot use --forward or --turn-angle-deg; publish /cmd_vel instead.' >&2
  exit 2
fi
case "${GO2_MPC_SOLVER}" in
  ddp|sqp) ;;
  *)
    printf 'Unsupported MPC solver: %s; choose ddp or sqp\n' \
      "${GO2_MPC_SOLVER}" >&2
    exit 2 ;;
esac
env -u PYTHONPATH "${GO2_PYTHON}" "${GO2_PROJECT_ROOT}/examples/mujoco/prepare_cpp_target.py" \
  "${GO2_TARGET}" "${GO2_TARGET_ARGS[@]}"
source /opt/ros/jazzy/setup.bash
source "${GO2_WORKSPACE}/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-216}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/go2_cpp_control_ros_logs}"
GO2_BINARY="${GO2_WORKSPACE}/install/go2_control_cpp/lib/go2_control_cpp/control_simulation"
GO2_MPC_BINARY="${GO2_WORKSPACE}/install/ocs2_legged_robot_ros/lib/ocs2_legged_robot_ros/legged_robot_${GO2_MPC_SOLVER}_mpc"
GO2_COMMAND_BINARY="${GO2_WORKSPACE}/install/go2_control_cpp/lib/go2_control_cpp/online_command"
GO2_COMMAND_CSV="${GO2_COMMAND_CSV:-${GO2_CSV%.csv}.commands.csv}"
GO2_COMMAND_LOG="${GO2_COMMAND_LOG:-${GO2_CSV%.csv}_command.log}"
if [[ ! -x "${GO2_MPC_BINARY}" ]]; then
  printf 'MPC executable not found: %s\n' "${GO2_MPC_BINARY}" >&2
  exit 1
fi
if [[ ! -x "${GO2_BINARY}" ]]; then
  printf '%s\n' 'Build first: bash scripts/build_cpp_control.sh' >&2; exit 1
fi
if [[ "${GO2_ONLINE}" == 1 && ! -x "${GO2_COMMAND_BINARY}" ]]; then
  printf '%s\n' 'Online command node not found; run bash scripts/build_cpp_control.sh' >&2
  exit 1
fi
"${GO2_BINARY}" --project-root "${GO2_PROJECT_ROOT}" --target "${GO2_TARGET}" \
  "${GO2_CONTROL_ARGS[@]}" --check-only
GO2_UNLIMITED=0
if /usr/bin/python3 -c 'import sys; sys.exit(float(sys.argv[1]) != 0)' "${GO2_DURATION}"; then
  GO2_UNLIMITED=1
fi
mkdir -p -- "$(dirname -- "${GO2_MPC_LOG}")"
GO2_MPC_PID=
GO2_COMMAND_PID=
GO2_CONTROL_PID=
GO2_USER_EXIT=0
go2_stop_processes() {
  local go2_pid go2_wait go2_alive
  local go2_pids=()
  for go2_pid in "${GO2_CONTROL_PID}" "${GO2_COMMAND_PID}" "${GO2_MPC_PID}"; do
    if [[ -n "${go2_pid}" ]]; then go2_pids+=("${go2_pid}"); fi
  done
  if ((${#go2_pids[@]}==0)); then return; fi
  kill -INT "${go2_pids[@]}" 2>/dev/null || true
  for ((go2_wait=0;go2_wait<30;go2_wait++)); do
    go2_alive=0
    for go2_pid in "${go2_pids[@]}"; do
      if kill -0 "${go2_pid}" 2>/dev/null; then go2_alive=1; fi
    done
    if [[ "${go2_alive}" == 0 ]]; then break; fi
    sleep 0.1
  done
  kill -TERM "${go2_pids[@]}" 2>/dev/null || true
  for go2_pid in "${go2_pids[@]}"; do wait "${go2_pid}" 2>/dev/null || true; done
  GO2_CONTROL_PID= GO2_COMMAND_PID= GO2_MPC_PID=
}
trap go2_stop_processes EXIT
go2_interrupt() {
  if [[ "${GO2_UNLIMITED}" == 1 && -n "${GO2_CONTROL_PID}" ]]; then
    GO2_USER_EXIT=1
    kill -INT "${GO2_CONTROL_PID}" 2>/dev/null || true
  else
    exit 130
  fi
}
trap go2_interrupt INT
trap 'exit 143' TERM
"${GO2_MPC_BINARY}" --ros-args \
  -p taskFile:="${GO2_PROJECT_ROOT}/config/ocs2/go2/task.info" \
  -p referenceFile:="${GO2_PROJECT_ROOT}/config/ocs2/go2/reference.info" \
  -p urdfFile:="${GO2_PROJECT_ROOT}/models/ocs2/go2_mujoco_inertial.urdf" \
  >"${GO2_MPC_LOG}" 2>&1 200>&- &
GO2_MPC_PID=$!
if [[ "${GO2_ONLINE}" == 1 ]]; then
  mkdir -p -- "$(dirname -- "${GO2_COMMAND_LOG}")"
  "${GO2_COMMAND_BINARY}" --ros-args \
    -p taskFile:="${GO2_PROJECT_ROOT}/config/ocs2/go2/task.info" \
    -p referenceFile:="${GO2_PROJECT_ROOT}/config/ocs2/go2/reference.info" \
    -p commandLog:="${GO2_COMMAND_CSV}" \
    -p integrateReference:="${GO2_INTEGRATE_REFERENCE}" \
    >"${GO2_COMMAND_LOG}" 2>&1 200>&- &
  GO2_COMMAND_PID=$!
  printf 'Online commands: /cmd_vel, /go2_control/body_pose; command CSV: %s; node log: %s\n' \
    "${GO2_COMMAND_CSV}" "${GO2_COMMAND_LOG}"
  printf 'Online position reference: %s\n' "${GO2_ONLINE_REFERENCE}"
fi
printf 'Native simulation MPC=%s; ROS_DOMAIN_ID=%s; MPC log: %s\n' \
  "${GO2_MPC_SOLVER}" "${ROS_DOMAIN_ID}" "${GO2_MPC_LOG}"
"${GO2_BINARY}" --project-root "${GO2_PROJECT_ROOT}" --target "${GO2_TARGET}" "${GO2_CONTROL_ARGS[@]}" 200>&- &
GO2_CONTROL_PID=$!
GO2_WORKER_FAILED=0
while kill -0 "${GO2_CONTROL_PID}" 2>/dev/null; do
  if [[ "${GO2_USER_EXIT}" == 0 ]] && ! kill -0 "${GO2_MPC_PID}" 2>/dev/null; then
    printf 'MPC exited; stopping control. See %s\n' "${GO2_MPC_LOG}" >&2
    # Give the native policy-age guard up to 0.3 s to latch/report this loss.
    # If native control is unresponsive, EXIT cleanup still terminates it.
    for ((go2_watchdog_wait=0;go2_watchdog_wait<15;go2_watchdog_wait++)); do
      if ! kill -0 "${GO2_CONTROL_PID}" 2>/dev/null; then break; fi
      sleep 0.02
    done
    GO2_WORKER_FAILED=1; break
  fi
  if [[ "${GO2_USER_EXIT}" == 0 && -n "${GO2_COMMAND_PID}" ]] && ! kill -0 "${GO2_COMMAND_PID}" 2>/dev/null; then
    printf 'Online command node exited; stopping control. See %s\n' "${GO2_COMMAND_LOG}" >&2
    kill -USR1 "${GO2_CONTROL_PID}" 2>/dev/null || true
    for ((go2_watchdog_wait=0;go2_watchdog_wait<15;go2_watchdog_wait++)); do
      if ! kill -0 "${GO2_CONTROL_PID}" 2>/dev/null; then break; fi
      sleep 0.02
    done
    GO2_WORKER_FAILED=1; break
  fi
  sleep 0.1
done
if [[ "${GO2_WORKER_FAILED}" == 1 ]]; then exit 1; fi
GO2_CONTROL_STATUS=0
wait "${GO2_CONTROL_PID}" || GO2_CONTROL_STATUS=$?
GO2_CONTROL_PID=
if [[ "${GO2_CONTROL_STATUS}" != 0 ]]; then exit "${GO2_CONTROL_STATUS}"; fi
go2_stop_processes
GO2_SUMMARY_ARGS=()
if [[ "${GO2_ONLINE}" == 1 ]]; then GO2_SUMMARY_ARGS+=(--commands "${GO2_COMMAND_CSV}"); fi
env -u PYTHONPATH "${GO2_PYTHON}" "${GO2_PROJECT_ROOT}/tools/summarize_cpp_control.py" \
  "${GO2_CSV}" --duration "${GO2_DURATION}" "${GO2_SUMMARY_ARGS[@]}"
