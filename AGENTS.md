# PROJECT KNOWLEDGE BASE
Unitree G1 control workspace spanning ROS 2 runtime packages, a React dashboard, and scripted bringup around the MuJoCo simulator.
**Generated:** 2026-04-22
**Commit:** n/a (no commits yet)
**Branch:** master

## OVERVIEW
Unitree G1 control workspace. First-party code lives in the ROS 2 stack under `ros2_ws/src`, the operator dashboard under `dashboard/`, and operational bringup/test helpers under `scripts/`. `deps/` remains vendored upstream code and robot assets.

## STRUCTURE
```text
unitree-g1-dashboard/
├── dashboard/      # React/Vite operator UI over rosbridge + REST API
├── ros2_ws/        # authored ROS 2 workspace; build here
├── deps/           # vendored Unitree/MuJoCo/RL repos; avoid local edits
├── scripts/        # golden-path start/stop helpers and local operator test panel
├── simulation/     # placeholder; currently not the active sim entrypoint
├── README.md
└── pyrightconfig.json
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Dashboard UI | `dashboard/` | Has child AGENTS; rosbridge + REST frontend |
| ROS package map | `ros2_ws/src/` | Main authored codebase |
| Golden-path operations | `scripts/` | Has child AGENTS; start/stop stack and panel |
| Start full stack | `scripts/start_g1_golden_path.sh` | Starts MuJoCo + ROS bringup |
| Stop full stack | `scripts/stop_g1_golden_path.sh` | Kills residual stack processes |
| HTTP/API behavior | `ros2_ws/src/g1_api/` | FastAPI + rosbridge |
| DDS / Unitree bridge | `ros2_ws/src/g1_bridge/` | Low-level SDK bridge |
| Safety logic | `ros2_ws/src/g1_safety/` | Joint limits + safety monitor |
| Motion orchestration | `ros2_ws/src/g1_orchestrator/` | State machine / mode transitions |
| Locomotion control | `ros2_ws/src/g1_locomotion/` | RL policy + PD fallback |

## CODE MAP
| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `G1ApiNode` | class | `ros2_ws/src/g1_api/g1_api/api_node.py` | REST façade over ROS state and commands |
| `SdkBridgeNode` | class | `ros2_ws/src/g1_bridge/g1_bridge/sdk_bridge_node.py` | DDS ↔ ROS command/state bridge |
| `G1SafetyNode` | class | `ros2_ws/src/g1_safety/g1_safety/safety_monitor_node.py` | Publishes safety state / estop conditions |
| `G1OrchestratorNode` | class | `ros2_ws/src/g1_orchestrator/g1_orchestrator/orchestrator_node.py` | Coordinates standing, walking, stop modes |
| `PolicyRunner` | class | `ros2_ws/src/g1_locomotion/g1_locomotion/policy_runner.py` | RL policy load + PD fallback command generation |
| `App` | React component | `dashboard/src/App.tsx` | rosbridge dashboard, joystick, and REST command UI |

## CONVENTIONS
- Treat `ros2_ws/src` as source of truth; `ros2_ws/build` and `ros2_ws/install` are generated colcon outputs.
- ROS Python packages use `ament_python`, `setup.py`, per-package `config/*.yaml`, and per-package `launch/*.launch.py`.
- Nodes validate parameters eagerly with explicit `ValueError` messages.
- Runtime ROS message imports are often lazy via `import_module(...)`; match that pattern when package build order matters.
- G1 assumptions are fixed around 29 joints; do not change joint order casually.
- Dashboard runtime currently assumes rosbridge at `ws://localhost:9090` and API at `http://localhost:8000`; keep frontend/backend changes in sync.

## ANTI-PATTERNS (THIS PROJECT)
- Do not edit `deps/` unless you intentionally want to patch vendored upstream code.
- Do not edit `ros2_ws/build/`, `ros2_ws/install/`, `.ruff_cache/`, or `.pytest_cache/` outputs.
- Do not edit `dashboard/node_modules/`, `dashboard/dist/`, `scripts/.run/`, or `scripts/logs/` outputs.
- Do not infer behavior from the empty `simulation/` dir; active sim flow lives under `deps/unitree_mujoco` and the golden-path scripts.
- Do not bypass package config YAMLs with scattered magic numbers if a parameter already exists.

## UNIQUE STYLES
- Package responsibilities are cleanly split by domain: API, bridge, safety, orchestrator, locomotion.
- `g1_bringup` composes launches; it is not where runtime logic lives.
- `g1_msgs` is a shared contract package; behavior changes usually belong in consuming nodes, not the message package.
- `dashboard/` and `scripts/` are real first-party domains now; use their child AGENTS files for local workflow details instead of overloading the root doc.

## COMMANDS
```bash
# build workspace
cd ros2_ws && colcon build

# run dashboard locally
cd dashboard && npm install && npm run dev

# run workspace tests
cd ros2_ws && colcon test && colcon test-result --verbose

# source ROS + workspace
source /opt/ros/*/setup.bash && source ros2_ws/install/setup.bash

# start / stop golden path
./scripts/start_g1_golden_path.sh
./scripts/stop_g1_golden_path.sh
```

## NOTES
- Commands assume you are running from the repository root unless noted otherwise.
- Most repository file count comes from vendored robot meshes and upstream code, not first-party logic.
- For package-level specifics, follow the nearest child `AGENTS.md`.
