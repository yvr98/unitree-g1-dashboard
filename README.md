# unitree-g1-dashboard

Unitree G1 control workspace combining a ROS 2 runtime stack, a React operator dashboard, and scripted bringup around the MuJoCo simulator.

This repo is organized for local development first. The fastest path is:

1. build the ROS 2 workspace
2. start the golden path scripts
3. open the dashboard against the local API + rosbridge services

## What is in this repo?

- `ros2_ws/` - first-party ROS 2 packages for API, bridge, safety, orchestrator, locomotion, bringup, and messages
- `dashboard/` - React + TypeScript + Vite frontend that talks to rosbridge and the local API server
- `scripts/` - start/stop helpers plus local test tools
- `deps/` - vendored upstream dependencies such as Unitree MuJoCo, SDK bindings, and RL assets
- `simulation/` - currently a placeholder, not the active simulator entrypoint

## Architecture at a glance

The main runtime pieces are:

- `g1_bridge` - DDS / Unitree SDK bridge
- `g1_safety` - safety monitoring and estop-related checks
- `g1_orchestrator` - standing / walking / reset state transitions
- `g1_locomotion` - RL policy execution with PD fallback behavior
- `g1_api` - HTTP API and rosbridge launch surface
- `dashboard/` - browser UI for commands, telemetry, and joystick-driven `/cmd_vel`

The dashboard assumes:

- rosbridge on `ws://localhost:9090`
- API on `http://localhost:8000`

Those defaults currently match the golden-path startup flow.

## Locomotion backend: RL first, PD fallback second

`g1_locomotion` prefers a packaged TorchScript policy and falls back to a built-in PD gait generator if that policy cannot be loaded.

In practice:

- normal walking tries to use `ros2_ws/src/g1_locomotion/policies/g1_motion.pt`
- if the policy file is missing, `torch` is unavailable, or the policy load fails, the node stays on `pd_fallback`
- zero command still settles into a standing-hold path rather than forcing gait output

This means the full RL training stack is **not** required for normal runtime use.

### What users need for intended RL walking

- ROS 2 + the normal workspace dependencies
- Python 3
- **PyTorch** available in the Python environment used by `g1_locomotion`
- the packaged policy artifact at `ros2_ws/src/g1_locomotion/policies/g1_motion.pt`
- the vendored Unitree SDK path expected by the runtime

### What users do not need for normal use

Users do **not** need the full `unitree_rl_gym` training environment just to run the packaged walking policy.

That vendored RL content is mainly relevant for:

- policy regeneration or experimentation
- deeper RL validation
- helper scripts such as `scripts/test_balance_dds.py`

### What happens without PyTorch

Walking does not disappear entirely. Instead, the locomotion node falls back to the built-in PD gait path.

So the behavior is:

- PyTorch installed -> packaged RL walking path available
- no PyTorch -> PD fallback only

## Repository layout

```text
unitree-g1-dashboard/
├── dashboard/      # React/Vite operator UI
├── ros2_ws/        # ROS 2 workspace and authored packages
├── deps/           # vendored upstream repos and assets
├── scripts/        # golden-path bringup / teardown / test helpers
├── simulation/     # placeholder, not the active sim entrypoint
├── README.md
└── pyrightconfig.json
```

## Prerequisites

This project is currently aimed at Linux development with a local ROS 2 installation.

You should expect to have:

- ROS 2 installed under `/opt/ros/...`
- Python 3 available as `python3`
- Node.js + npm for the dashboard
- a working colcon build environment
- the vendored simulator dependencies present under `deps/`

Because the stack includes ROS 2, MuJoCo, rosbridge, and vendored Unitree dependencies, this repo is **not** yet a one-command portable environment for every machine. Documented local setup is the intended path right now.

## Quick start

### 1. Build the ROS 2 workspace

```bash
cd /home/plate/unitree-g1-dashboard/ros2_ws
colcon build
```

### 2. Start the golden path

This launches MuJoCo first, then ROS bringup, then waits for:

- rosbridge on port `9090`
- API on port `8000`
- ROS service `/set_locomotion_mode`

```bash
/home/plate/unitree-g1-dashboard/scripts/start_g1_golden_path.sh
```

Logs and runtime state land under:

- `scripts/logs/`
- `scripts/.run/`

### 3. Run the dashboard

In a separate shell:

```bash
cd /home/plate/unitree-g1-dashboard/dashboard
npm install
npm run dev
```

Then open the local Vite URL in your browser.

### 4. Stop the stack

```bash
/home/plate/unitree-g1-dashboard/scripts/stop_g1_golden_path.sh
```

## Common development commands

### ROS 2 workspace

```bash
cd /home/plate/unitree-g1-dashboard/ros2_ws
colcon build
colcon test
colcon test-result --verbose
```

### Source ROS + workspace

```bash
source /opt/ros/*/setup.bash
source /home/plate/unitree-g1-dashboard/ros2_ws/install/setup.bash
```

### Dashboard

```bash
cd /home/plate/unitree-g1-dashboard/dashboard
npm install
npm run dev
npm run build
npm run lint
```

## Testing

Automated tests currently live mostly in package-level `test/` directories under `ros2_ws/src/`.

Examples:

- `ros2_ws/src/g1_safety/test/test_joint_limits.py`
- `ros2_ws/src/g1_orchestrator/test/test_orchestrator_helpers.py`
- `ros2_ws/src/g1_locomotion/test/test_policy_runner.py`
- `ros2_ws/src/g1_locomotion/test/test_rl_live.py`
- `ros2_ws/src/g1_locomotion/test/test_locomotion_command_selection.py`

Package-level test runs are typically done with `pytest`, while workspace-level runs use `colcon test`.

There are also manual / operational helpers in `scripts/`, especially:

- `scripts/g1_test_panel.py` - small local operator panel on `127.0.0.1:8765`
- `scripts/test_balance_dds.py` - lower-level DDS / balance harness

## Notes about `deps/`

`deps/` contains vendored upstream code and assets. It is part of the working tree, but it is **not** the best place to make casual first-party edits.

In general:

- treat `deps/` as vendored input
- make first-party behavior changes in `ros2_ws/src/`, `dashboard/`, or `scripts/`
- only patch vendored code deliberately and document that decision

## What not to edit directly

Generated outputs should not be treated as source:

- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`
- `dashboard/node_modules/`
- `dashboard/dist/`
- `scripts/.run/`
- `scripts/logs/`

## Current limitations

- `simulation/` is still a placeholder; active simulator flow currently lives under `deps/unitree_mujoco` and the golden-path scripts
- the dashboard still relies on hard-coded local endpoints for rosbridge and the API
- the root environment is documented for local Linux use, not yet fully containerized for every host setup
- dashboard testing is still mostly manual; there is no dedicated frontend test suite yet

## Suggested first places to read

- root project guidance: `AGENTS.md`
- dashboard-specific guidance: `dashboard/AGENTS.md`
- scripts / operator workflow guidance: `scripts/AGENTS.md`
- ROS workspace guidance: `ros2_ws/AGENTS.md`

## Publishing notes

If you plan to publish this repo for others to clone:

- prefer clear setup docs over forcing Docker immediately
- document the supported host environment explicitly
- keep the quick-start path centered on `colcon build`, `start_g1_golden_path.sh`, and `npm run dev`
- decide intentionally whether vendored `deps/` content belongs in the public history as-is
