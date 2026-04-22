import time
import sys
from importlib import import_module
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import mujoco
import mujoco.viewer
from threading import Thread
import threading

UNITREE_SDK_PATH = Path(__file__).resolve().parents[1].parent / "unitree_sdk2_python"
if str(UNITREE_SDK_PATH) not in sys.path:
    sys.path.insert(0, str(UNITREE_SDK_PATH))

ChannelFactoryInitialize = import_module(
    "unitree_sdk2py.core.channel"
).ChannelFactoryInitialize
bridge_module = import_module("unitree_sdk2py_bridge")
UnitreeSdk2Bridge = bridge_module.UnitreeSdk2Bridge
ElasticBand = bridge_module.ElasticBand

config_spec = spec_from_file_location(
    "unitree_mujoco_sim_config", Path(__file__).with_name("config.py")
)
if config_spec is None or config_spec.loader is None:
    raise RuntimeError("Unable to load simulate_python/config.py")
config = module_from_spec(config_spec)
config_spec.loader.exec_module(config)


locker = threading.Lock()
sim_bridge = None

mj_model = getattr(mujoco, "MjModel").from_xml_path(getattr(config, "ROBOT_SCENE"))
mj_data = getattr(mujoco, "MjData")(mj_model)


if getattr(config, "ENABLE_ELASTIC_BAND"):
    elastic_band = ElasticBand()
    if getattr(config, "ROBOT") == "h1" or getattr(config, "ROBOT") == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
else:
    elastic_band = None


def ViewerKeyCallback(key):
    if elastic_band is not None:
        elastic_band.MujuocoKeyCallback(key)
    if sim_bridge is not None and sim_bridge.embedded_rl is not None:
        sim_bridge.embedded_rl.MujocoKeyCallback(key)


viewer = mujoco.viewer.launch_passive(mj_model, mj_data, key_callback=ViewerKeyCallback)

mj_model.opt.timestep = getattr(config, "SIMULATE_DT")
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model, sim_bridge

    ChannelFactoryInitialize(getattr(config, "DOMAIN_ID"), getattr(config, "INTERFACE"))
    unitree = UnitreeSdk2Bridge(mj_model, mj_data, mj_data_lock=locker)
    sim_bridge = unitree

    if getattr(config, "USE_JOYSTICK"):
        unitree.SetupJoystick(device_id=0, js_type=getattr(config, "JOYSTICK_TYPE"))
    if getattr(config, "PRINT_SCENE_INFORMATION"):
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if getattr(config, "ENABLE_ELASTIC_BAND"):
            if elastic_band is not None and elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        unitree.Advance()
        getattr(mujoco, "mj_step")(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(getattr(config, "VIEWER_DT"))


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
