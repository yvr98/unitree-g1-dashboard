# ROS2 PACKAGE MAP

## OVERVIEW
`src/` holds the first-party package graph. Most work lands in a domain package; `g1_msgs` and `g1_bringup` stay thin.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Shared ROS contracts | `g1_msgs/` | Message + service definitions only |
| Compose full launch stack | `g1_bringup/` | Aggregates package launch files |
| External API surface | `g1_api/` | Has child AGENTS |
| Unitree DDS bridge | `g1_bridge/` | Has child AGENTS |
| Safety monitoring | `g1_safety/` | Has child AGENTS |
| State coordination | `g1_orchestrator/` | Has child AGENTS |
| Locomotion backend | `g1_locomotion/` | Has child AGENTS |

## CONVENTIONS
- Package layout is consistent: `package.xml`, `setup.py` or `CMakeLists.txt`, `launch/`, `config/`, optional `test/`.
- Python packages expose ROS entrypoints via `console_scripts` in `setup.py`.
- Package configs are YAML files consumed by launch files; update both when adding parameters.

## PACKAGE ROLES
- `g1_msgs`: `JointCommand`, `RobotState`, `SafetyStatus`, `LocomotionState`, `SetLocomotionMode`.
- `g1_bringup`: `full_system.launch.py` includes bridge, safety, orchestrator, locomotion, API.
- `g1_api`: HTTP / rosbridge layer.
- `g1_bridge`: Unitree SDK / DDS integration.
- `g1_safety`: safety status from robot state + joint limits.
- `g1_orchestrator`: mode transitions, cmd_vel gating, locomotion enable/disable.
- `g1_locomotion`: RL policy runner and fallback gait logic.

## ANTI-PATTERNS
- Do not add behavior into `g1_msgs`; extend consuming packages unless the wire contract must change.
- Do not duplicate launch composition inside domain packages if `g1_bringup` already owns the stack assembly.
- Do not document package-specific rules here when a nearer child `AGENTS.md` exists.

## COMMANDS
```bash
# launch full authored stack
source /opt/ros/*/setup.bash && source /home/plate/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 launch g1_bringup full_system.launch.py

# run targeted package tests
cd /home/plate/unitree-g1-dashboard/ros2_ws && colcon test --packages-select g1_safety g1_orchestrator g1_locomotion
```
