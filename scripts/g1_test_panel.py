#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]
ROS_WS_DIR = ROOT_DIR / "ros2_ws"
ROS_SETUP = "/opt/ros/*/setup.bash"
WORKSPACE_SETUP = ROS_WS_DIR / "install" / "setup.bash"
HOST = "127.0.0.1"
PORT = 8765


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>G1 Test Panel</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 0; background: #0f172a; color: #e2e8f0; }
    .wrap { max-width: 1100px; margin: 0 auto; padding: 24px; }
    h1 { margin: 0 0 8px; font-size: 28px; }
    p { color: #94a3b8; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin: 20px 0; }
    button { width: 100%; padding: 14px 16px; border: 0; border-radius: 12px; cursor: pointer; font-size: 15px; font-weight: 700; color: #0f172a; background: #38bdf8; }
    button.secondary { background: #cbd5e1; }
    button.warn { background: #fbbf24; }
    button.danger { background: #f87171; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    .cards { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin: 20px 0; }
    .card { background: #111827; border: 1px solid #1f2937; border-radius: 14px; padding: 16px; }
    .label { color: #94a3b8; font-size: 13px; margin-bottom: 8px; }
    .value { white-space: pre-wrap; font-family: ui-monospace, monospace; font-size: 13px; line-height: 1.45; }
    .output { background: #020617; border: 1px solid #1e293b; border-radius: 14px; padding: 16px; min-height: 320px; white-space: pre-wrap; font-family: ui-monospace, monospace; font-size: 13px; line-height: 1.45; }
    .row { display: flex; gap: 12px; flex-wrap: wrap; margin-bottom: 16px; }
    input[type="text"] { flex: 1; min-width: 220px; background: #0b1220; color: #e2e8f0; border: 1px solid #334155; border-radius: 10px; padding: 12px; }
    @media (max-width: 800px) { .cards { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>G1 Test Panel</h1>
    <p>Click buttons instead of retyping ROS commands. Output shows the exact response from the command that ran.</p>

    <div class="grid">
      <button onclick="runAction('get_safety')">Get Safety</button>
      <button onclick="runAction('get_state')">Get State</button>
      <button class="secondary" onclick="runAction('reset')">Reset</button>
      <button class="secondary" onclick="runAction('stand_up')">Stand Up</button>
      <button class="secondary" onclick="runAction('sit_down')">Sit Down</button>
      <button class="warn" onclick="runAction('walk_forward_start')">Start Walk Forward</button>
      <button class="warn" onclick="runAction('turn_start')">Start Turn</button>
      <button class="danger" onclick="runAction('cmd_stop')">Stop /cmd_vel</button>
      <button class="danger" onclick="runAction('get_estop_snapshot')">Get Estop Snapshot</button>
      <button class="danger" onclick="runAction('kill_stack')">Kill Stack</button>
    </div>

    <div class="row">
      <input id="customMode" type="text" placeholder="Custom requested_mode, e.g. stand_up">
      <button class="secondary" onclick="runCustomMode()">Call Custom Mode</button>
    </div>

    <div class="cards">
      <div class="card">
        <div class="label">Last known locomotion state</div>
        <div id="stateBox" class="value">(not loaded yet)</div>
      </div>
      <div class="card">
        <div class="label">Last known safety status</div>
        <div id="safetyBox" class="value">(not loaded yet)</div>
      </div>
    </div>

    <div class="label">Command output</div>
    <div id="output" class="output">Ready.</div>
  </div>

  <script>
    async function runAction(action, payload={}) {
      const output = document.getElementById('output');
      output.textContent = 'Running ' + action + '...';
      try {
        const response = await fetch('/api/action', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action, ...payload }),
        });
        const data = await response.json();
        output.textContent = data.output;
        if (data.state) document.getElementById('stateBox').textContent = data.state;
        if (data.safety) document.getElementById('safetyBox').textContent = data.safety;
      } catch (error) {
        output.textContent = 'Request failed: ' + error;
      }
    }

    function runCustomMode() {
      const mode = document.getElementById('customMode').value.trim();
      if (!mode) return;
      runAction('custom_mode', { mode });
    }

    runAction('refresh');
  </script>
</body>
</html>
"""


class CommandRunner:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cmd_vel_process: subprocess.Popen[str] | None = None

    def _shell_prefix(self) -> str:
        return (
            f'cd "{ROOT_DIR}" && '
            f'set +u; source {ROS_SETUP}; source "{WORKSPACE_SETUP}"; set -u; '
        )

    def run(self, shell_command: str, timeout: float = 20.0) -> str:
        full_command = self._shell_prefix() + shell_command
        completed = subprocess.run(
            ["bash", "-lc", full_command],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = []
        output.append(f"$ {shell_command}")
        if completed.stdout.strip():
            output.append(completed.stdout.strip())
        if completed.stderr.strip():
            output.append(completed.stderr.strip())
        output.append(f"exit_code={completed.returncode}")
        return "\n\n".join(output)

    def start_cmd_vel(self, twist_text: str) -> str:
        with self._lock:
            self.stop_cmd_vel_locked()
            shell_command = (
                self._shell_prefix()
                + f'exec ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{twist_text}" --rate 10'
            )
            self._cmd_vel_process = subprocess.Popen(
                ["bash", "-lc", shell_command],
                cwd=ROOT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
        return f"Started /cmd_vel publisher with payload: {twist_text}"

    def stop_cmd_vel_locked(self) -> None:
        if self._cmd_vel_process is None:
            return
        process = self._cmd_vel_process
        self._cmd_vel_process = None
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)

    def stop_cmd_vel(self) -> str:
        with self._lock:
            was_running = (
                self._cmd_vel_process is not None
                and self._cmd_vel_process.poll() is None
            )
            self.stop_cmd_vel_locked()
        return (
            "Stopped /cmd_vel publisher."
            if was_running
            else "No /cmd_vel publisher was running."
        )

    def close(self) -> None:
        with self._lock:
            self.stop_cmd_vel_locked()


RUNNER = CommandRunner()


def safe_text_output(value: str) -> str:
    return value.strip() if value.strip() else "(no output)"


def get_state_snapshot() -> str:
    return safe_text_output(
        RUNNER.run("ros2 topic echo /locomotion_state --once", timeout=20.0)
    )


def get_safety_snapshot() -> str:
    return safe_text_output(
        RUNNER.run("ros2 topic echo /safety_status --once", timeout=20.0)
    )


def do_action(action: str, payload: dict[str, Any]) -> dict[str, str]:
    output = ""
    if action == "refresh":
        output = "Refreshed state and safety."
    elif action == "get_state":
        output = get_state_snapshot()
    elif action == "get_safety":
        output = get_safety_snapshot()
    elif action == "reset":
        output = RUNNER.run(
            "ros2 service call /set_locomotion_mode g1_msgs/srv/SetLocomotionMode \"{requested_mode: 'reset'}\""
        )
    elif action == "stand_up":
        output = RUNNER.run(
            "ros2 service call /set_locomotion_mode g1_msgs/srv/SetLocomotionMode \"{requested_mode: 'stand_up'}\""
        )
    elif action == "sit_down":
        output = RUNNER.run(
            "ros2 service call /set_locomotion_mode g1_msgs/srv/SetLocomotionMode \"{requested_mode: 'sit_down'}\""
        )
    elif action == "custom_mode":
        mode = str(payload.get("mode", "")).strip()
        if not mode:
            output = "Custom mode is empty."
        else:
            output = RUNNER.run(
                f"ros2 service call /set_locomotion_mode g1_msgs/srv/SetLocomotionMode \"{{requested_mode: '{mode}'}}\""
            )
    elif action == "walk_forward_start":
        output = RUNNER.start_cmd_vel("{linear: {x: 0.3}}")
    elif action == "turn_start":
        output = RUNNER.start_cmd_vel("{angular: {z: 0.5}}")
    elif action == "cmd_stop":
        output = RUNNER.stop_cmd_vel()
    elif action == "kill_stack":
        RUNNER.stop_cmd_vel()
        output = RUNNER.run(
            'pkill -9 -f "unitree_mujoco.py|sdk_bridge_node|safety_monitor_node|orchestrator_node|locomotion_node|ros2 launch g1_bringup"',
            timeout=10.0,
        )
    elif action == "get_estop_snapshot":
        output = RUNNER.run(
            'grep "Safety estop snapshot:" ~/unitree-g1-dashboard/scripts/logs/ros_bringup.log'
        )
    else:
        output = f"Unknown action: {action}"

    state = get_state_snapshot()
    safety = get_safety_snapshot()
    return {"output": safe_text_output(output), "state": state, "safety": safety}


class PanelHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            body = HTML.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        if self.path != "/api/action":
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length)
        payload = json.loads(raw_body.decode("utf-8") or "{}")
        action = str(payload.get("action", ""))
        result = do_action(action, payload)
        body = json.dumps(result).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: Any) -> None:
        del format, args


def main() -> None:
    server = ThreadingHTTPServer((HOST, PORT), PanelHandler)
    print(f"G1 test panel running at http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        RUNNER.close()
        server.server_close()


if __name__ == "__main__":
    main()
