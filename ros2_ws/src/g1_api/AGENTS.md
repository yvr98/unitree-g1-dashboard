# G1 API PACKAGE

## OVERVIEW
FastAPI wrapper over ROS state and mode-control services. Also launches `rosbridge_websocket` alongside the API server.

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Main node | `g1_api/api_node.py` | FastAPI app + ROS executor thread |
| REST routes | `g1_api/routes/` | `commands.py`, `state.py` |
| Launch wiring | `launch/api.launch.py` | Starts rosbridge + API and ties shutdowns together |
| Runtime params | `config/api_config.yaml` | host, port, topics, command list |

## CONVENTIONS
- Keep API route handlers thin; node methods own ROS interaction.
- State is read from subscriptions and normalized into JSON-friendly dicts.
- Command dispatch goes through `/set_locomotion_mode`; unsupported commands return `HTTPException(400)`.
- Launch behavior expects rosbridge and API lifecycle to stay coupled.

## ANTI-PATTERNS
- Do not hardcode command names in routes if `command_list` parameter already defines them.
- Do not bypass `set_api_node()` / `get_api_node()` app-state wiring.
- Do not block the ROS executor thread with HTTP-only work.

## COMMANDS
```bash
source /opt/ros/*/setup.bash && source /home/plate/unitree-g1-dashboard/ros2_ws/install/setup.bash
ros2 launch g1_api api.launch.py
```
