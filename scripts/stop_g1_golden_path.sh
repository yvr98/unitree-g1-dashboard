#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/scripts/.run"
PID_FILE="${RUN_DIR}/g1_dev_stack.pids"

declare -a PATTERNS=(
  "ros2 launch g1_bringup full_system.launch.py"
  "install/g1_bridge/lib/g1_bridge/sdk_bridge_node"
  "install/g1_safety/lib/g1_safety/safety_monitor_node"
  "install/g1_orchestrator/lib/g1_orchestrator/orchestrator_node"
  "install/g1_locomotion/lib/g1_locomotion/locomotion_node"
  "install/g1_api/lib/g1_api/api_server"
  "rosbridge_websocket"
  "simulate_python/unitree_mujoco.py"
  "unitree_mujoco.py"
)

kill_if_running() {
  local pid="$1"
  local signal_name="$2"
  if kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
  fi
}

if [[ -f "${PID_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${PID_FILE}"
  [[ -n "${MUJOCO_PID:-}" ]] && kill_if_running "${MUJOCO_PID}" TERM
  [[ -n "${ROS_PID:-}" ]] && kill_if_running "${ROS_PID}" TERM
  rm -f "${PID_FILE}"
fi

sleep 1

python3 - <<'PY'
import os
import signal
import subprocess
import time

patterns = [
    'ros2 launch g1_bringup full_system.launch.py',
    'install/g1_bridge/lib/g1_bridge/sdk_bridge_node',
    'install/g1_safety/lib/g1_safety/safety_monitor_node',
    'install/g1_orchestrator/lib/g1_orchestrator/orchestrator_node',
    'install/g1_locomotion/lib/g1_locomotion/locomotion_node',
    'install/g1_api/lib/g1_api/api_server',
    'rosbridge_websocket',
    'simulate_python/unitree_mujoco.py',
    'unitree_mujoco.py',
]

def matched_processes():
    out = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True)
    current_pid = os.getpid()
    matches = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_text, args = line.split(' ', 1)
        pid = int(pid_text)
        if pid == current_pid:
            continue
        if any(pattern in args for pattern in patterns):
            matches.append((pid, args))
    return matches

initial = matched_processes()
for pid, _ in initial:
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass

time.sleep(1.0)

remaining = matched_processes()
for pid, _ in remaining:
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass

final_remaining = matched_processes()

print(f'Terminated {len(initial)} matching process(es).')
if initial:
    for pid, args in initial:
        print(f'  {pid} {args}')

if final_remaining:
    print('WARNING: some matching processes remain:')
    for pid, args in final_remaining:
        print(f'  {pid} {args}')
else:
    print('All golden-path processes stopped.')
PY

echo "Golden path stack stopped."
