# G1 ORCHESTRATOR PACKAGE

## OVERVIEW
High-level motion coordinator. Owns the locomotion state machine, service-driven mode transitions, hold poses, and cmd_vel gating.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Main node | `g1_orchestrator/orchestrator_node.py` | transition logic, safety gating, locomotion toggles |
| State constants | `g1_orchestrator/state_machine.py` | `IDLE`, `STANDING`, `WALKING`, `EMERGENCY_STOP`, `ERROR` |
| Runtime params | `config/orchestrator_params.yaml` | poses, gains, topic names, timeouts |
| Launch files | `launch/orchestrator.launch.py`, `launch/phase2_stack.launch.py` | standard + phase-2 composition |
| Tests | `test/test_orchestrator_helpers.py` | helper behavior coverage |

## CONVENTIONS
- Treat this package as the owner of locomotion mode semantics (`reset`, `stand_up`, `sit_down`, walking transitions).
- Safety status is authoritative; unsafe input should reject or interrupt motion.
- State strings come from `state_machine.py`, not scattered literals.
- Pose/gain vectors are 29-element G1 arrays; validate lengths early.

## ANTI-PATTERNS
- Do not publish ungated `/cmd_vel` straight into locomotion when orchestrator safety or state logic should arbitrate it.
- Do not add new mode names in one file only; update service handling, state machine semantics, and tests together.
- Do not hide transition failures; this package is the right place to emit explicit rejection reasons.

## COMMANDS
```bash
cd ros2_ws/src/g1_orchestrator && python3 -m pytest test/
source /opt/ros/*/setup.bash && source ros2_ws/install/setup.bash && ros2 launch g1_orchestrator orchestrator.launch.py
```
