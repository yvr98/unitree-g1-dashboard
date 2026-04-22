import mujoco
import numpy as np
import pygame
import sys
import struct
import time
import os
from importlib import import_module
from importlib.util import module_from_spec, spec_from_file_location
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from threading import Lock

import torch
import yaml
from numpy.typing import NDArray

UNITREE_SDK_PATH = Path(__file__).resolve().parents[1].parent / "unitree_sdk2_python"
if str(UNITREE_SDK_PATH) not in sys.path:
    sys.path.insert(0, str(UNITREE_SDK_PATH))

config_spec = spec_from_file_location(
    "unitree_mujoco_sim_config", Path(__file__).with_name("config.py")
)
if config_spec is None or config_spec.loader is None:
    raise RuntimeError("Unable to load simulate_python/config.py")
config = module_from_spec(config_spec)
config_spec.loader.exec_module(config)

channel_module = import_module("unitree_sdk2py.core.channel")
default_module = import_module("unitree_sdk2py.idl.default")
geometry_module = import_module("unitree_sdk2py.idl.geometry_msgs.msg.dds_")
std_module = import_module("unitree_sdk2py.idl.std_msgs.msg.dds_")
go_module = import_module("unitree_sdk2py.idl.unitree_go.msg.dds_")
thread_module = import_module("unitree_sdk2py.utils.thread")

ChannelSubscriber = channel_module.ChannelSubscriber
ChannelPublisher = channel_module.ChannelPublisher
geometry_msgs_msg_dds__Twist_ = default_module.geometry_msgs_msg_dds__Twist_
std_msgs_msg_dds__String_ = default_module.std_msgs_msg_dds__String_
Twist_ = geometry_module.Twist_
String_ = std_module.String_
SportModeState_ = go_module.SportModeState_
WirelessController_ = go_module.WirelessController_
unitree_go_msg_dds__SportModeState_ = default_module.unitree_go_msg_dds__SportModeState_
unitree_go_msg_dds__WirelessController_ = (
    default_module.unitree_go_msg_dds__WirelessController_
)
RecurrentThread = thread_module.RecurrentThread
ROBOT = getattr(config, "ROBOT", "g1")

if ROBOT == "g1":
    hg_module = import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    LowCmd_ = hg_module.LowCmd_
    LowState_ = hg_module.LowState_
    LowState_default = default_module.unitree_hg_msg_dds__LowState_
else:
    LowCmd_ = go_module.LowCmd_
    LowState_ = go_module.LowState_
    LowState_default = default_module.unitree_go_msg_dds__LowState_

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"
TOPIC_RL_CMD = getattr(config, "RL_CMD_TOPIC", "rt/locomotion/cmd_vel")
TOPIC_RL_ENABLE = getattr(config, "RL_ENABLE_TOPIC", "rt/locomotion/enabled")

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20
NUM_MOTOR_IDL_HG = 35
REPO_ROOT = Path(__file__).resolve().parents[3]
DEPLOY_CONFIG_PATH = (
    REPO_ROOT
    / "deps"
    / "unitree_rl_gym"
    / "deploy"
    / "deploy_mujoco"
    / "configs"
    / "g1.yaml"
)
LOCOMOTION_CONFIG_PATH = (
    REPO_ROOT
    / "ros2_ws"
    / "src"
    / "g1_locomotion"
    / "config"
    / "locomotion_params.yaml"
)
POLICY_PATH = (
    REPO_ROOT / "ros2_ws" / "src" / "g1_locomotion" / "policies" / "g1_motion.pt"
)


@dataclass(frozen=True)
class EmbeddedPolicyConfig:
    default_angles: NDArray[np.float32]
    default_pose: NDArray[np.float32]
    kps: NDArray[np.float32]
    kds: NDArray[np.float32]
    waist_joint_kp: NDArray[np.float32]
    waist_joint_kd: NDArray[np.float32]
    arm_joint_kp: NDArray[np.float32]
    arm_joint_kd: NDArray[np.float32]
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float
    cmd_scale: NDArray[np.float32]
    num_actions: int
    num_obs: int
    control_decimation: int
    simulation_dt: float


def load_embedded_policy_config() -> EmbeddedPolicyConfig:
    deploy_raw = yaml.safe_load(DEPLOY_CONFIG_PATH.read_text(encoding="utf-8"))
    locomotion_raw = yaml.safe_load(LOCOMOTION_CONFIG_PATH.read_text(encoding="utf-8"))[
        "/locomotion_node"
    ]["ros__parameters"]
    return EmbeddedPolicyConfig(
        default_angles=np.asarray(deploy_raw["default_angles"], dtype=np.float32),
        default_pose=np.asarray(locomotion_raw["default_pose"], dtype=np.float32),
        kps=np.asarray(deploy_raw["kps"], dtype=np.float32),
        kds=np.asarray(deploy_raw["kds"], dtype=np.float32),
        waist_joint_kp=np.asarray(locomotion_raw["waist_joint_kp"], dtype=np.float32),
        waist_joint_kd=np.asarray(locomotion_raw["waist_joint_kd"], dtype=np.float32),
        arm_joint_kp=np.asarray(locomotion_raw["arm_joint_kp"], dtype=np.float32),
        arm_joint_kd=np.asarray(locomotion_raw["arm_joint_kd"], dtype=np.float32),
        ang_vel_scale=float(deploy_raw["ang_vel_scale"]),
        dof_pos_scale=float(deploy_raw["dof_pos_scale"]),
        dof_vel_scale=float(deploy_raw["dof_vel_scale"]),
        action_scale=float(deploy_raw["action_scale"]),
        cmd_scale=np.asarray(deploy_raw["cmd_scale"], dtype=np.float32),
        num_actions=int(deploy_raw["num_actions"]),
        num_obs=int(deploy_raw["num_obs"]),
        control_decimation=int(deploy_raw["control_decimation"]),
        simulation_dt=float(deploy_raw["simulation_dt"]),
    )


def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3, dtype=np.float32)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def _glfw_module():
    return getattr(mujoco, "glfw").glfw


def _mjt_obj(name):
    return getattr(getattr(mujoco, "_enums").mjtObj, name)


def _mj_id2name(model, obj_type, index):
    return getattr(mujoco, "mj_id2name")(model, obj_type, index)


class EmbeddedRLController:
    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.config = load_embedded_policy_config()
        self.policy = torch.jit.load(str(POLICY_PATH), map_location="cpu")
        self.policy.eval()
        self.enabled = bool(getattr(config, "EMBEDDED_RL_ENABLED", True))
        self.blend_duration_sec = max(
            float(getattr(config, "EMBEDDED_RL_BLEND_SEC", 0.25)),
            self.config.simulation_dt,
        )
        self.stand_ramp_duration_sec = max(
            float(getattr(config, "EMBEDDED_STAND_RAMP_SEC", 2.0)),
            self.config.simulation_dt,
        )
        self.cmd_vel = np.zeros(3, dtype=np.float32)
        self.action = np.zeros(self.config.num_actions, dtype=np.float32)
        self.rl_target_dof_pos = self.config.default_angles.copy()
        self.applied_leg_targets = self.config.default_angles.copy()
        self.initial_full_pose = np.asarray(
            self.mj_data.qpos[7 : 7 + self.mj_model.nu], dtype=np.float32
        ).copy()
        self.obs = np.zeros(self.config.num_obs, dtype=np.float32)
        self.counter = 0
        self.stand_blend = 0.0
        self.policy_blend = 0.0
        self.status_lock = Lock()

        self.cmd_vel_suber = ChannelSubscriber(TOPIC_RL_CMD, Twist_)
        self.cmd_vel_suber.Init(self.CmdVelHandler, 10)
        self.enable_suber = ChannelSubscriber(TOPIC_RL_ENABLE, String_)
        self.enable_suber.Init(self.EnableHandler, 10)
        print(
            f"[EmbeddedRL] walk_rl={'enabled' if self.enabled else 'disabled'} policy_path={POLICY_PATH} "
            f"cmd_topic={TOPIC_RL_CMD} enable_topic={TOPIC_RL_ENABLE}"
        )

    def CmdVelHandler(self, msg):
        with self.status_lock:
            self.cmd_vel[0] = float(msg.linear.x)
            self.cmd_vel[1] = float(msg.linear.y)
            self.cmd_vel[2] = float(msg.angular.z)

    def EnableHandler(self, msg):
        command = str(msg.data).strip().lower()
        if command in {"1", "true", "enable", "enabled", "on"}:
            self.SetEnabled(True)
        elif command in {"0", "false", "disable", "disabled", "off"}:
            self.SetEnabled(False)

    def SetEnabled(self, enabled: bool):
        with self.status_lock:
            if self.enabled == enabled:
                return
            self.enabled = enabled
            self.cmd_vel.fill(0.0)
            self._reset_policy_state()
        print(f"[EmbeddedRL] walk_rl {'enabled' if enabled else 'disabled'}")

    def MujocoKeyCallback(self, key):
        glfw = _glfw_module()
        with self.status_lock:
            if key == glfw.KEY_R:
                next_enabled = not self.enabled
                self.SetEnabled(next_enabled)
                return
            if key == glfw.KEY_SPACE:
                self.cmd_vel.fill(0.0)
            elif key == glfw.KEY_UP:
                self.cmd_vel[0] = min(self.cmd_vel[0] + 0.1, 0.8)
            elif key == glfw.KEY_DOWN:
                self.cmd_vel[0] = max(self.cmd_vel[0] - 0.1, -0.8)
            elif key == glfw.KEY_LEFT:
                self.cmd_vel[2] = min(self.cmd_vel[2] + 0.2, 1.57)
            elif key == glfw.KEY_RIGHT:
                self.cmd_vel[2] = max(self.cmd_vel[2] - 0.2, -1.57)
            else:
                return
            print(
                f"[EmbeddedRL] cmd_vel=({self.cmd_vel[0]:.2f}, {self.cmd_vel[1]:.2f}, {self.cmd_vel[2]:.2f})"
            )

    def Advance(self):
        with self.status_lock:
            enabled = self.enabled
            cmd_vel = self.cmd_vel.copy()

        rl_active = enabled
        self.stand_blend = min(
            1.0,
            self.stand_blend + self.config.simulation_dt / self.stand_ramp_duration_sec,
        )

        if rl_active:
            self.counter += 1
            if self.counter % self.config.control_decimation == 0:
                self.rl_target_dof_pos = self._infer_rl_target(cmd_vel)
        else:
            self.counter = 0
            self.action.fill(0.0)
            self.rl_target_dof_pos = self.config.default_angles.copy()

        self.policy_blend = self._next_blend(self.policy_blend, rl_active)
        self.applied_leg_targets = (
            1.0 - self.policy_blend
        ) * self.config.default_angles + self.policy_blend * self.rl_target_dof_pos

        tau = pd_control(
            self._full_target_pose(),
            self.mj_data.qpos[7 : 7 + self.mj_model.nu],
            self._full_kp(),
            np.zeros(self.mj_model.nu, dtype=np.float32),
            self.mj_data.qvel[6 : 6 + self.mj_model.nu],
            self._full_kd(),
        )
        self.mj_data.ctrl[: self.mj_model.nu] = tau
        return True

    def _reset_policy_state(self):
        self.action.fill(0.0)
        self.rl_target_dof_pos = self.config.default_angles.copy()
        self.applied_leg_targets = self.config.default_angles.copy()
        self.counter = 0
        self.stand_blend = 0.0
        self.policy_blend = 0.0
        for state_name in ("hidden_state", "cell_state"):
            state = getattr(self.policy, state_name, None)
            if state is None:
                continue
            try:
                state.zero_()
            except Exception:
                continue

    def _infer_rl_target(self, cmd_vel):
        qj = self.mj_data.qpos[7 : 7 + self.config.num_actions]
        dqj = self.mj_data.qvel[6 : 6 + self.config.num_actions]
        quat = self.mj_data.qpos[3:7]
        omega = self.mj_data.qvel[3:6]

        qj = (qj - self.config.default_angles) * self.config.dof_pos_scale
        dqj = dqj * self.config.dof_vel_scale
        gravity_orientation = get_gravity_orientation(quat)
        omega = omega * self.config.ang_vel_scale

        period = 0.8
        count = self.counter * self.config.simulation_dt
        phase = count % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        self.obs[:3] = omega
        self.obs[3:6] = gravity_orientation
        self.obs[6:9] = cmd_vel * self.config.cmd_scale
        self.obs[9 : 9 + self.config.num_actions] = qj
        self.obs[9 + self.config.num_actions : 9 + 2 * self.config.num_actions] = dqj
        self.obs[9 + 2 * self.config.num_actions : 9 + 3 * self.config.num_actions] = (
            self.action
        )
        self.obs[
            9 + 3 * self.config.num_actions : 9 + 3 * self.config.num_actions + 2
        ] = np.array([sin_phase, cos_phase], dtype=np.float32)
        with torch.inference_mode():
            obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
            action = (
                self.policy(obs_tensor)
                .detach()
                .cpu()
                .numpy()
                .squeeze()
                .astype(np.float32)
            )
        self.action = action
        return action * self.config.action_scale + self.config.default_angles

    def _next_blend(self, current_blend, walk_active):
        blend_step = self.config.simulation_dt / self.blend_duration_sec
        if walk_active:
            return min(1.0, current_blend + blend_step)
        return max(0.0, current_blend - blend_step)

    def _full_target_pose(self):
        full_target = (
            (1.0 - self.stand_blend) * self.initial_full_pose
            + self.stand_blend * self.config.default_pose
        ).astype(np.float32)
        full_target[: self.config.num_actions] = self.applied_leg_targets
        return full_target

    def _full_kp(self):
        kp = np.zeros(self.mj_model.nu, dtype=np.float32)
        kp[: self.config.num_actions] = self.config.kps
        kp[12:15] = self.config.waist_joint_kp
        kp[15:29] = self.config.arm_joint_kp
        return kp

    def _full_kd(self):
        kd = np.zeros(self.mj_model.nu, dtype=np.float32)
        kd[: self.config.num_actions] = self.config.kds
        kd[12:15] = self.config.waist_joint_kd
        kd[15:29] = self.config.arm_joint_kd
        return kd


class UnitreeSdk2Bridge:
    def __init__(self, mj_model, mj_data, mj_data_lock=None):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.mj_data_lock = mj_data_lock
        self.low_cmd_lock = Lock()
        self.latest_low_cmd = None

        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep
        self.idl_type = (
            self.num_motor > NUM_MOTOR_IDL_GO
        )  # 0: unitree_go, 1: unitree_hg

        self.joystick = None
        self.axis_id = {}
        self.button_id = {}

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = _mj_id2name(self.mj_model, _mjt_obj("mjOBJ_SENSOR"), i)
            if name == "imu_quat":
                self.have_imu = True
            if name == "frame_pos":
                self.have_frame_sensor = True

        # Unitree sdk2 message
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishLowState, name="sim_lowstate"
        )
        self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishHighState, name="sim_highstate"
        )
        self.HighStateThread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(
            TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_puber.Init()
        self.WirelessControllerThread = RecurrentThread(
            interval=0.01,
            target=self.PublishWirelessController,
            name="sim_wireless_controller",
        )
        self.WirelessControllerThread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        self.embedded_rl = None
        if ROBOT == "g1" and bool(getattr(config, "EMBEDDED_CONTROLLER_ENABLED", True)):
            self.embedded_rl = EmbeddedRLController(self.mj_model, self.mj_data)

        # joystick
        self.key_map = {
            "R1": 0,
            "L1": 1,
            "start": 2,
            "select": 3,
            "R2": 4,
            "L2": 5,
            "F1": 6,
            "F2": 7,
            "A": 8,
            "B": 9,
            "X": 10,
            "Y": 11,
            "up": 12,
            "right": 13,
            "down": 14,
            "left": 15,
        }

    def LowCmdHandler(self, msg):
        with self.low_cmd_lock:
            self.latest_low_cmd = msg

    def Advance(self):
        if self.embedded_rl is not None and self.embedded_rl.Advance():
            return
        self.ApplyLatestLowCmd()

    def ApplyLatestLowCmd(self):
        with self.low_cmd_lock:
            low_cmd = self.latest_low_cmd

        if self.mj_data is None:
            return

        if low_cmd is None:
            self.mj_data.ctrl[: self.num_motor] = 0.0
            return

        for i in range(self.num_motor):
            self.mj_data.ctrl[i] = (
                low_cmd.motor_cmd[i].tau
                + low_cmd.motor_cmd[i].kp
                * (low_cmd.motor_cmd[i].q - self.mj_data.sensordata[i])
                + low_cmd.motor_cmd[i].kd
                * (
                    low_cmd.motor_cmd[i].dq
                    - self.mj_data.sensordata[i + self.num_motor]
                )
            )

    def PublishLowState(self):
        if self.mj_data != None:
            with self.mj_data_lock or nullcontext():
                for i in range(self.num_motor):
                    self.low_state.motor_state[i].q = self.mj_data.qpos[7 + i]
                    self.low_state.motor_state[i].dq = self.mj_data.qvel[6 + i]
                    self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[
                        i + 2 * self.num_motor
                    ]

                if self.have_imu:
                    self.low_state.imu_state.quaternion[0] = self.mj_data.qpos[3]
                    self.low_state.imu_state.quaternion[1] = self.mj_data.qpos[4]
                    self.low_state.imu_state.quaternion[2] = self.mj_data.qpos[5]
                    self.low_state.imu_state.quaternion[3] = self.mj_data.qpos[6]

                    self.low_state.imu_state.gyroscope[0] = self.mj_data.qvel[3]
                    self.low_state.imu_state.gyroscope[1] = self.mj_data.qvel[4]
                    self.low_state.imu_state.gyroscope[2] = self.mj_data.qvel[5]

                if self.have_frame_sensor:
                    self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[
                        self.dim_motor_sensor + 7
                    ]
                    self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[
                        self.dim_motor_sensor + 8
                    ]
                    self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[
                        self.dim_motor_sensor + 9
                    ]

            if self.joystick != None:
                pygame.event.get()
                # Buttons
                self.low_state.wireless_remote[2] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                0,
                                0,
                                int(self.joystick.get_axis(self.axis_id["LT"]) > 0),
                                int(self.joystick.get_axis(self.axis_id["RT"]) > 0),
                                int(self.joystick.get_button(self.button_id["SELECT"])),
                                int(self.joystick.get_button(self.button_id["START"])),
                                int(self.joystick.get_button(self.button_id["LB"])),
                                int(self.joystick.get_button(self.button_id["RB"])),
                            ]
                        ]
                    ),
                    2,
                )
                self.low_state.wireless_remote[3] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                int(self.joystick.get_hat(0)[0] < 0),  # left
                                int(self.joystick.get_hat(0)[1] < 0),  # down
                                int(self.joystick.get_hat(0)[0] > 0),  # right
                                int(self.joystick.get_hat(0)[1] > 0),  # up
                                int(self.joystick.get_button(self.button_id["Y"])),  # Y
                                int(self.joystick.get_button(self.button_id["X"])),  # X
                                int(self.joystick.get_button(self.button_id["B"])),  # B
                                int(self.joystick.get_button(self.button_id["A"])),  # A
                            ]
                        ]
                    ),
                    2,
                )
                # Axes
                sticks = [
                    self.joystick.get_axis(self.axis_id["LX"]),
                    self.joystick.get_axis(self.axis_id["RX"]),
                    -self.joystick.get_axis(self.axis_id["RY"]),
                    -self.joystick.get_axis(self.axis_id["LY"]),
                ]
                packs = list(map(lambda x: struct.pack("f", x), sticks))
                self.low_state.wireless_remote[4:8] = packs[0]
                self.low_state.wireless_remote[8:12] = packs[1]
                self.low_state.wireless_remote[12:16] = packs[2]
                self.low_state.wireless_remote[20:24] = packs[3]

            self.low_state_puber.Write(self.low_state)

    def PublishHighState(self):
        if self.mj_data != None:
            with self.mj_data_lock or nullcontext():
                self.high_state.position[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 10
                ]
                self.high_state.position[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 11
                ]
                self.high_state.position[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 12
                ]

                self.high_state.velocity[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 13
                ]
                self.high_state.velocity[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 14
                ]
                self.high_state.velocity[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 15
                ]

        self.high_state_puber.Write(self.high_state)

    def PublishWirelessController(self):
        if self.joystick != None:
            pygame.event.get()
            key_state = [0] * 16
            key_state[self.key_map["R1"]] = self.joystick.get_button(
                self.button_id["RB"]
            )
            key_state[self.key_map["L1"]] = self.joystick.get_button(
                self.button_id["LB"]
            )
            key_state[self.key_map["start"]] = self.joystick.get_button(
                self.button_id["START"]
            )
            key_state[self.key_map["select"]] = self.joystick.get_button(
                self.button_id["SELECT"]
            )
            key_state[self.key_map["R2"]] = (
                self.joystick.get_axis(self.axis_id["RT"]) > 0
            )
            key_state[self.key_map["L2"]] = (
                self.joystick.get_axis(self.axis_id["LT"]) > 0
            )
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = self.joystick.get_button(self.button_id["A"])
            key_state[self.key_map["B"]] = self.joystick.get_button(self.button_id["B"])
            key_state[self.key_map["X"]] = self.joystick.get_button(self.button_id["X"])
            key_state[self.key_map["Y"]] = self.joystick.get_button(self.button_id["Y"])
            key_state[self.key_map["up"]] = self.joystick.get_hat(0)[1] > 0
            key_state[self.key_map["right"]] = self.joystick.get_hat(0)[0] > 0
            key_state[self.key_map["down"]] = self.joystick.get_hat(0)[1] < 0
            key_state[self.key_map["left"]] = self.joystick.get_hat(0)[0] < 0

            key_value = 0
            for i in range(16):
                key_value += key_state[i] << i

            self.wireless_controller.keys = key_value
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"])
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"])

            self.wireless_controller_puber.Write(self.wireless_controller)

    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("No gamepad detected.")
            sys.exit()

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
            }
        else:
            print("Unsupported gamepad. ")

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = _mj_id2name(self.mj_model, _mjt_obj("mjOBJ_BODY"), i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = _mj_id2name(self.mj_model, _mjt_obj("mjOBJ_JOINT"), i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = _mj_id2name(self.mj_model, _mjt_obj("mjOBJ_ACTUATOR"), i)
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = _mj_id2name(self.mj_model, _mjt_obj("mjOBJ_SENSOR"), i)
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:
    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = _glfw_module()
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
