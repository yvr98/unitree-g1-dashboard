# G1 BRIDGE PACKAGE

## OVERVIEW
Bridges Unitree SDK DDS topics to ROS 2 `RobotState` and `JointCommand` flows. This is the hardware/sim edge of the first-party stack.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Main node | `g1_bridge/sdk_bridge_node.py` | Channel setup, low-state ingest, low-cmd publish |
| Launch file | `launch/bridge.launch.py` | Loads `bridge_config.yaml` |
| Runtime params | `config/bridge_config.yaml` | sim vs real channel config |

## CONVENTIONS
- `mode` must be `sim` or `real`; network/domain settings come from parameters.
- SDK import path is discovered via `UNITREE_SDK2PY_PATH` first, then repo-relative fallback.
- Joint count is fixed to 29 for G1.
- When commands or state go stale, the node disables motors instead of publishing stale actuation.

## ANTI-PATTERNS
- Do not edit installed SDK copies under `ros2_ws/install`; source changes belong here or in vendored SDK code deliberately.
- Do not relax stale-command / stale-state guards for convenience.
- Do not change DDS topic names (`rt/lowcmd`, `rt/lowstate`) without understanding Unitree SDK expectations.

## COMMANDS
```bash
source /opt/ros/*/setup.bash && source ros2_ws/install/setup.bash
ros2 launch g1_bridge bridge.launch.py
```
