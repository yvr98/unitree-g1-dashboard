import os


ROBOT = "g1"  # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_flat.xml"  # Robot scene
DOMAIN_ID = 1  # Domain id
INTERFACE = "lo"  # Interface

USE_JOYSTICK = 0  # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox"  # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0  # Joystick number

PRINT_SCENE_INFORMATION = True  # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = (
    os.environ.get("UNITREE_MUJOCO_ELASTIC_BAND", "1") == "1"
)  # Virtual spring band, used for lifting humanoids

SIMULATE_DT = float(
    os.environ.get("UNITREE_MUJOCO_SIMULATE_DT", "0.002")
)  # Matches deploy_mujoco.py by default; override for slower viewers if needed.
VIEWER_DT = 0.04

EMBEDDED_CONTROLLER_ENABLED = (
    os.environ.get("UNITREE_MUJOCO_EMBEDDED_CONTROLLER", "0") == "1"
)
EMBEDDED_RL_ENABLED = os.environ.get("UNITREE_MUJOCO_EMBEDDED_RL", "0") == "1"
EMBEDDED_RL_CMD_EPSILON = float(
    os.environ.get("UNITREE_MUJOCO_EMBEDDED_RL_CMD_EPSILON", "0.001")
)
EMBEDDED_RL_BLEND_SEC = float(
    os.environ.get("UNITREE_MUJOCO_EMBEDDED_RL_BLEND_SEC", "0.25")
)
EMBEDDED_STAND_RAMP_SEC = float(
    os.environ.get("UNITREE_MUJOCO_EMBEDDED_STAND_RAMP_SEC", "2.0")
)
RL_CMD_TOPIC = os.environ.get("UNITREE_MUJOCO_RL_CMD_TOPIC", "rt/locomotion/cmd_vel")
RL_ENABLE_TOPIC = os.environ.get(
    "UNITREE_MUJOCO_RL_ENABLE_TOPIC", "rt/locomotion/enabled"
)
