# Unitree G1 Dashboard

ROS 2 control workspace for the Unitree G1 humanoid, combining:

- a ROS 2 runtime stack for bridge, safety, orchestration, and locomotion
- a React-based operator dashboard over rosbridge + HTTP
- scripted bringup around the MuJoCo simulator

This repository is structured for local development and simulation-driven bringup on Linux.

---

## Overview

The project is organized around a small set of first-party domains:

- **`g1_bridge`** — Unitree SDK / DDS bridge into ROS 2
- **`g1_safety`** — safety checks, joint-limit monitoring, estop-related state
- **`g1_orchestrator`** — locomotion state transitions such as idle, standing, walking, and reset
- **`g1_locomotion`** — walking command generation with RL-first control and PD fallback
- **`g1_api`** — HTTP API and rosbridge launch surface
- **`dashboard/`** — browser-based operator UI for commands, telemetry, and joystick input

The default local runtime assumes:

- rosbridge on `ws://localhost:9090`
- API on `http://localhost:8000`

---

## Repository Layout

```text
unitree-g1-dashboard/
├── dashboard/      # React + TypeScript + Vite operator UI
├── ros2_ws/        # ROS 2 workspace and authored packages
├── deps/           # vendored upstream dependencies and robot assets
├── scripts/        # startup, shutdown, and local test helpers
├── simulation/     # placeholder; not the active simulator entrypoint
├── README.md
└── pyrightconfig.json
```

### First-party code

The primary authored code lives in:

- `ros2_ws/src/`
- `dashboard/`
- `scripts/`

### Vendored code

`deps/` contains upstream content such as Unitree MuJoCo assets, SDK bindings, and RL-related assets. It is tracked in this repository for reproducible local setup, but it should be treated as vendored input rather than the default place for first-party feature work.

---

## Prerequisites

This repository currently targets **Linux-based local development**.

You should have:

- ROS 2 installed under `/opt/ros/...`
- Python 3 available as `python3`
- Node.js and npm for the dashboard
- `colcon` available for building the ROS 2 workspace

You should also expect that this project depends on local simulator / graphics / ROS networking assumptions. It is not yet packaged as a fully portable, host-agnostic environment.

---

## Quick Start

### 1. Build the ROS 2 workspace

From the repository root:

```bash
cd ros2_ws
colcon build
```

### 2. Start the golden path

From the repository root:

```bash
./scripts/start_g1_golden_path.sh
```

The startup script:

- launches MuJoCo first
- launches the full ROS stack
- waits for:
  - rosbridge on port `9090`
  - API on port `8000`
  - `/set_locomotion_mode` service availability

Runtime artifacts are written under:

- `scripts/logs/`
- `scripts/.run/`

### 3. Run the dashboard

In a separate shell, from the repository root:

```bash
cd dashboard
npm install
npm run dev
```

Then open the Vite development URL shown in the terminal.

### 4. Stop the stack

From the repository root:

```bash
./scripts/stop_g1_golden_path.sh
```

---

## Locomotion Backend

`g1_locomotion` uses a **hybrid backend**:

- **preferred path:** packaged TorchScript RL policy
- **fallback path:** built-in PD gait generation

By default, runtime walking tries to load:

- `ros2_ws/src/g1_locomotion/policies/g1_motion.pt`

If the policy file is missing, `torch` is unavailable, or the policy fails to load, the node stays on the PD fallback path.

### For intended RL walking

Users should have:

- the normal ROS 2 workspace dependencies
- Python 3
- **PyTorch** available in the Python environment used by `g1_locomotion`
- the packaged policy artifact `g1_motion.pt`

### What is not required for normal runtime use

Users do **not** need the full training stack just to run the packaged walking policy.

The vendored RL-related content in `deps/unitree_rl_gym` is mainly relevant for:

- policy regeneration or experimentation
- deeper RL validation
- helper scripts such as `scripts/test_balance_dds.py`

### Behavior without PyTorch

If PyTorch is unavailable, locomotion does not disappear entirely. The system falls back to the built-in PD gait path.

---

## Common Development Commands

### ROS 2 workspace

From the repository root:

```bash
cd ros2_ws
colcon build
colcon test
colcon test-result --verbose
```

### Source ROS + workspace overlay

From the repository root:

```bash
source /opt/ros/*/setup.bash
source ros2_ws/install/setup.bash
```

### Dashboard

From the repository root:

```bash
cd dashboard
npm install
npm run dev
npm run build
npm run lint
```

---

## Testing

Automated tests currently live primarily in package-level `test/` directories under `ros2_ws/src/`.

Representative examples:

- `ros2_ws/src/g1_safety/test/test_joint_limits.py`
- `ros2_ws/src/g1_orchestrator/test/test_orchestrator_helpers.py`
- `ros2_ws/src/g1_locomotion/test/test_policy_runner.py`
- `ros2_ws/src/g1_locomotion/test/test_rl_live.py`
- `ros2_ws/src/g1_locomotion/test/test_locomotion_command_selection.py`

Package-local tests are generally run with `pytest`, while workspace-wide runs use `colcon test`.

The repository also includes operational/manual helpers under `scripts/`, including:

- `scripts/g1_test_panel.py` — lightweight local operator panel on `127.0.0.1:8765`
- `scripts/test_balance_dds.py` — lower-level DDS / locomotion harness

---

## Notes on Vendored Dependencies

`deps/` is intentionally tracked in this repository because the simulator and runtime stack depend on upstream assets and SDK content.

In practice:

- treat `deps/` as vendored content
- make first-party behavior changes in `ros2_ws/src/`, `dashboard/`, or `scripts/`
- only patch vendored code deliberately and document that decision

---

## Generated Output Directories

The following should be treated as generated artifacts rather than source:

- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`
- `dashboard/node_modules/`
- `dashboard/dist/`
- `scripts/.run/`
- `scripts/logs/`

---

## Current Status / Limitations

- `simulation/` is still a placeholder; active simulation flow currently lives under `deps/unitree_mujoco` and the startup scripts
- the dashboard still uses local default endpoints for rosbridge and the API
- the project is documented for local Linux use, not as a fully containerized environment for every host setup
- dashboard testing is still primarily manual; there is no dedicated frontend test suite yet

---

## Additional Documentation

For contributor- and subsystem-specific guidance, see:

- `AGENTS.md`
- `dashboard/AGENTS.md`
- `scripts/AGENTS.md`
- `ros2_ws/AGENTS.md`

These files are intended for deeper project navigation and implementation details. The README is the public-facing starting point.
