# G1 SAFETY PACKAGE

## OVERVIEW
Safety-monitoring domain. Converts `RobotState` into `SafetyStatus` using upright checks, stale-data checks, and joint-limit enforcement.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Main node | `g1_safety/safety_monitor_node.py` | periodic safety evaluation |
| Joint limit logic | `g1_safety/joint_limits.py` | expected joint order, YAML loading, validation |
| Params | `config/safety_params.yaml` | joint bounds + thresholds |
| Launch file | `launch/safety.launch.py` | loads config YAML |
| Tests | `test/test_joint_limits.py` | core helper and config assumptions |

## CONVENTIONS
- `JOINT_NAME_ORDER` is the canonical 29-joint ordering; config must match exactly.
- A missing / stale robot state should publish unsafe status, not silence.
- Safety defaults conservative: stale data or tilt violations trigger estop.
- Joint limits belong in YAML and are loaded/validated before runtime.

## ANTI-PATTERNS
- Do not weaken “unsafe by default” behavior to make bringup look healthy.
- Do not reorder joints casually; every consumer assumes the same index mapping.
- Do not accept all-zero joint limits; that explicitly signals misloaded config here.

## COMMANDS
```bash
cd /home/plate/unitree-g1-dashboard/ros2_ws/src/g1_safety && python3 -m pytest test/
source /opt/ros/*/setup.bash && source /home/plate/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 launch g1_safety safety.launch.py
```
