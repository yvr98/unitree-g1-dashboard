#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS_DIR="${ROOT_DIR}/ros2_ws"
MUJOCO_DIR="${ROOT_DIR}/deps/unitree_mujoco/simulate_python"
RUN_DIR="${ROOT_DIR}/scripts/.run"
LOG_DIR="${ROOT_DIR}/scripts/logs"
PID_FILE="${RUN_DIR}/g1_dev_stack.pids"
MUJOCO_LOG="${LOG_DIR}/mujoco.log"
ROS_LOG="${LOG_DIR}/ros_bringup.log"
API_PORT=8000
ROSBRIDGE_PORT=9090
STARTUP_TIMEOUT_SEC=25

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

log_info() {
  printf '[INFO] %s\n' "$1"
}

log_ok() {
  printf '[ OK ] %s\n' "$1"
}

log_warn() {
  printf '[WARN] %s\n' "$1"
}

log_error() {
  printf '[FAIL] %s\n' "$1" >&2
}

source_ros_env() {
  set +u
  source /opt/ros/*/setup.bash
  source "${ROS_WS_DIR}/install/setup.bash"
  set -u
}

port_is_busy() {
  local port="$1"
  ss -ltnp 2>/dev/null | grep -qE ":${port}[[:space:]]"
}

print_port_owner() {
  local port="$1"
  ss -ltnp 2>/dev/null | grep -E ":${port}[[:space:]]" || true
}

kill_if_running() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
  fi
}

cleanup_started_processes() {
  kill_if_running "${ROS_PID:-}"
  kill_if_running "${MUJOCO_PID:-}"
  sleep 1
  if [[ -n "${ROS_PID:-}" ]] && kill -0 "${ROS_PID}" 2>/dev/null; then
    kill -9 "${ROS_PID}" 2>/dev/null || true
  fi
  if [[ -n "${MUJOCO_PID:-}" ]] && kill -0 "${MUJOCO_PID}" 2>/dev/null; then
    kill -9 "${MUJOCO_PID}" 2>/dev/null || true
  fi
  rm -f "${PID_FILE}"
}

print_log_excerpt() {
  local label="$1"
  local file="$2"
  if [[ ! -f "${file}" ]]; then
    return
  fi

  echo
  echo "${label} log excerpt (${file}):"
  python3 - <<'PY' "${file}"
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
lines = path.read_text(errors='replace').splitlines()
interesting = [
    line for line in lines
    if re.search(r'ERROR|Traceback|address already in use|failed|exception|unavailable', line, re.IGNORECASE)
]
snippet = interesting[-12:] if interesting else lines[-12:]
for line in snippet:
    print(line)
PY
}

fail_startup() {
  local message="$1"
  log_error "${message}"
  print_log_excerpt "MuJoCo" "${MUJOCO_LOG}"
  print_log_excerpt "ROS bringup" "${ROS_LOG}"
  cleanup_started_processes
  exit 1
}

wait_for_port() {
  local port="$1"
  local timeout="$2"
  local elapsed=0
  while (( elapsed < timeout )); do
    if port_is_busy "${port}"; then
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done
  return 1
}

wait_for_service() {
  local service_name="$1"
  local timeout="$2"
  local elapsed=0
  while (( elapsed < timeout )); do
    if timeout 5 ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done
  return 1
}

if [[ -f "${PID_FILE}" ]]; then
  log_error "PID file already exists: ${PID_FILE}"
  echo "Run scripts/stop_g1_golden_path.sh first if the stack is already running."
  exit 1
fi

if [[ ! -d "${MUJOCO_DIR}" ]]; then
  log_error "MuJoCo directory not found: ${MUJOCO_DIR}"
  exit 1
fi

if [[ ! -f "${ROS_WS_DIR}/install/setup.bash" ]]; then
  log_error "ROS workspace is not built: ${ROS_WS_DIR}/install/setup.bash missing"
  echo "Run: cd ${ROS_WS_DIR} && colcon build"
  exit 1
fi

source_ros_env

for port in "${API_PORT}" "${ROSBRIDGE_PORT}"; do
  if port_is_busy "${port}"; then
    log_error "Required port ${port} is already in use."
    print_port_owner "${port}"
    exit 1
  fi
done

: >"${MUJOCO_LOG}"
: >"${ROS_LOG}"

log_info "Starting MuJoCo with elastic band enabled..."
(
  cd "${MUJOCO_DIR}"
  UNITREE_MUJOCO_ELASTIC_BAND=1 UNITREE_MUJOCO_EMBEDDED_CONTROLLER=0 python3 unitree_mujoco.py
) >"${MUJOCO_LOG}" 2>&1 &
MUJOCO_PID=$!

sleep 2
if ! kill -0 "${MUJOCO_PID}" 2>/dev/null; then
  fail_startup "MuJoCo exited immediately after launch."
fi
log_ok "MuJoCo started (PID ${MUJOCO_PID})."

log_info "Starting ROS full system bringup..."
(
  cd "${ROOT_DIR}"
  set +u
  source /opt/ros/*/setup.bash
  source "${ROS_WS_DIR}/install/setup.bash"
  set -u
  ros2 launch g1_bringup full_system.launch.py
) >"${ROS_LOG}" 2>&1 &
ROS_PID=$!

cat >"${PID_FILE}" <<EOF
MUJOCO_PID=${MUJOCO_PID}
ROS_PID=${ROS_PID}
EOF

sleep 2
if ! kill -0 "${ROS_PID}" 2>/dev/null; then
  fail_startup "ROS bringup exited immediately after launch."
fi

if wait_for_port "${ROSBRIDGE_PORT}" "${STARTUP_TIMEOUT_SEC}"; then
  log_ok "rosbridge listening on port ${ROSBRIDGE_PORT}."
else
  fail_startup "rosbridge did not start on port ${ROSBRIDGE_PORT} within ${STARTUP_TIMEOUT_SEC}s."
fi

if wait_for_port "${API_PORT}" "${STARTUP_TIMEOUT_SEC}"; then
  log_ok "API listening on port ${API_PORT}."
else
  fail_startup "API server did not start on port ${API_PORT} within ${STARTUP_TIMEOUT_SEC}s."
fi

if wait_for_service "/set_locomotion_mode" "${STARTUP_TIMEOUT_SEC}"; then
  log_ok "Service /set_locomotion_mode is available."
else
  fail_startup "Service /set_locomotion_mode did not become available within ${STARTUP_TIMEOUT_SEC}s."
fi

cat <<EOF
Golden path started successfully.

Launch summary:
  [OK] MuJoCo PID: ${MUJOCO_PID}
  [OK] ROS launch PID: ${ROS_PID}
  [OK] rosbridge port: ${ROSBRIDGE_PORT}
  [OK] API port: ${API_PORT}
  [OK] Service: /set_locomotion_mode

Logs:
  ${MUJOCO_LOG}
  ${ROS_LOG}

Next steps:
  1. In the MuJoCo window: press 9 to toggle the band, 8 to lift, 7 to lower.
  2. Release the band with 9 before stand-up.
  3. Check safety:
     source /opt/ros/*/setup.bash && source ~/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 topic echo /safety_status --once
  4. If safe, reset and stand:
     source /opt/ros/*/setup.bash && source ~/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 service call /set_locomotion_mode g1_msgs/srv/SetLocomotionMode "{requested_mode: 'reset'}"
     source /opt/ros/*/setup.bash && source ~/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 service call /set_locomotion_mode g1_msgs/srv/SetLocomotionMode "{requested_mode: 'stand_up'}"
  5. Walk:
     source /opt/ros/*/setup.bash && source ~/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --rate 10
EOF
