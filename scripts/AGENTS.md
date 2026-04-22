# SCRIPTED OPERATIONS

## OVERVIEW
`scripts/` owns local operator workflows: booting the golden path, tearing it down cleanly, and running the lightweight test panel / DDS helpers.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Start full stack | `start_g1_golden_path.sh` | boots MuJoCo, ROS bringup, waits for ports/services |
| Stop full stack | `stop_g1_golden_path.sh` | PID-aware cleanup plus process-pattern fallback |
| Local operator panel | `g1_test_panel.py` | HTTP panel for common ROS actions and snapshots |
| DDS/RL balance harness | `test_balance_dds.py` | lower-level runtime experiment script |
| Runtime artifacts | `.run/`, `logs/` | generated PID and log outputs |

## CONVENTIONS
- Start flow is opinionated: MuJoCo first, then `ros2 launch g1_bringup full_system.launch.py`, then readiness checks on rosbridge, API, and `/set_locomotion_mode`.
- Scripts source `/opt/ros/*/setup.bash` and `ros2_ws/install/setup.bash` themselves; keep those assumptions valid when moving files or renaming workspace paths.
- `start_g1_golden_path.sh` owns PID tracking in `scripts/.run/g1_dev_stack.pids`; `stop_g1_golden_path.sh` is the matching teardown path.
- `g1_test_panel.py` is intentionally local-only on `127.0.0.1:8765` and shells out to ROS commands after sourcing the environment.
- `test_balance_dds.py` is not a normal unit test; it pulls directly from vendored SDK / RL assets and assumes the simulator-side runtime layout.

## ANTI-PATTERNS
- Do not hand-kill random stack processes first; use `stop_g1_golden_path.sh` so PID cleanup and pattern cleanup stay consistent.
- Do not edit files under `.run/` or `logs/`; they are generated runtime artifacts.
- Do not change hard-coded ports, service names, or workspace paths in one script only; the start script, panel, and frontend assumptions must stay aligned.
- Do not expose `g1_test_panel.py` beyond localhost without adding real auth / safety controls.
- Do not treat `test_balance_dds.py` as a package pytest test; it is an operational harness with heavier external dependencies.

## COMMANDS
```bash
/home/plate/unitree-g1-dashboard/scripts/start_g1_golden_path.sh
/home/plate/unitree-g1-dashboard/scripts/stop_g1_golden_path.sh
python3 /home/plate/unitree-g1-dashboard/scripts/g1_test_panel.py
python3 /home/plate/unitree-g1-dashboard/scripts/test_balance_dds.py
```

## NOTES
- `start_g1_golden_path.sh` fails fast if the workspace is not built or the required ports are already occupied.
- The panel is for fast manual checks; package-level automated tests still live under `ros2_ws/src/*/test/`.
