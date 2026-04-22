# G1 LOCOMOTION PACKAGE

## OVERVIEW
Locomotion execution layer. Prefers a TorchScript RL policy, falls back to PD gait generation, and publishes `JointCommand` for standing/walking.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Node entry | `g1_locomotion/locomotion_node.py` | ROS plumbing, enable/disable services, tick loop |
| Control backend | `g1_locomotion/policy_runner.py` | RL load path, observation build, fallback gait |
| Policy artifact | `policies/g1_motion.pt` | Packaged TorchScript asset |
| Package params | `config/locomotion_params.yaml` | gains, limits, gait tuning |
| Tests | `test/` | policy runner, command selection, RL live path |

## CONVENTIONS
- Joint count is fixed at 29; RL outputs are generated for joints 0-14, while the package still emits full 29-joint commands using configured waist and arm targets.
- `PolicyRunner` owns backend choice (`rl_policy` vs `pd_fallback`) and exposes why.
- Parameter tuning lives in YAML, not ad hoc code edits.
- Zero command should settle into standing behavior rather than forcing gait output.

## ANTI-PATTERNS
- Do not assume Torch is available; fallback mode is a first-class path.
- Do not move or rename `g1_motion.pt` without updating packaged paths and default resolution logic.
- Do not widen control beyond the intended controlled joint mask without checking orchestrator/safety expectations.

## COMMANDS
```bash
cd /home/plate/unitree-g1-dashboard/ros2_ws/src/g1_locomotion && python3 -m pytest test/
source /opt/ros/*/setup.bash && source /home/plate/unitree-g1-dashboard/ros2_ws/install/setup.bash && ros2 launch g1_locomotion locomotion.launch.py
```
