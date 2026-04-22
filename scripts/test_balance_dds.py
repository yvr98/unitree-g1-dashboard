#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from threading import Lock
from typing import Any

import numpy as np
from numpy.typing import NDArray
import torch
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
UNITREE_SDK_PATH = REPO_ROOT / "deps" / "unitree_sdk2_python"
if str(UNITREE_SDK_PATH) not in sys.path:
    sys.path.insert(0, str(UNITREE_SDK_PATH))


def _load_runtime_types() -> dict[str, Any]:
    channel_module = import_module("unitree_sdk2py.core.channel")
    default_module = import_module("unitree_sdk2py.idl.default")
    hg_module = import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    crc_module = import_module("unitree_sdk2py.utils.crc")
    return {
        "ChannelFactoryInitialize": channel_module.ChannelFactoryInitialize,
        "ChannelPublisher": channel_module.ChannelPublisher,
        "ChannelSubscriber": channel_module.ChannelSubscriber,
        "LowCmdFactory": default_module.unitree_hg_msg_dds__LowCmd_,
        "LowCmdType": hg_module.LowCmd_,
        "LowStateType": hg_module.LowState_,
        "CRC": crc_module.CRC,
    }


TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
MODE_PR = 0
CONTROL_DT = 0.005
JOINT_COUNT = 29
LEG_JOINT_COUNT = 12
WAIST_JOINT_COUNT = 3

DEPLOY_CONFIG_PATH = (
    REPO_ROOT
    / "deps"
    / "unitree_rl_gym"
    / "deploy"
    / "deploy_mujoco"
    / "configs"
    / "g1.yaml"
)
POSTURE_CONFIG_PATH = (
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
class DeployConfig:
    default_angles: NDArray[np.float32]
    kps: NDArray[np.float32]
    kds: NDArray[np.float32]
    simulation_dt: float
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float
    cmd_scale: NDArray[np.float32]
    num_actions: int
    num_obs: int


@dataclass(frozen=True)
class PostureConfig:
    waist_joint_target: list[float]
    waist_joint_kp: list[float]
    waist_joint_kd: list[float]
    arm_joint_target: list[float]
    arm_joint_kp: list[float]
    arm_joint_kd: list[float]


def load_deploy_config() -> DeployConfig:
    raw = yaml.safe_load(DEPLOY_CONFIG_PATH.read_text(encoding="utf-8"))
    return DeployConfig(
        default_angles=np.asarray(raw["default_angles"], dtype=np.float32),
        kps=np.asarray(raw["kps"], dtype=np.float32),
        kds=np.asarray(raw["kds"], dtype=np.float32),
        simulation_dt=float(raw["simulation_dt"]),
        ang_vel_scale=float(raw["ang_vel_scale"]),
        dof_pos_scale=float(raw["dof_pos_scale"]),
        dof_vel_scale=float(raw["dof_vel_scale"]),
        action_scale=float(raw["action_scale"]),
        cmd_scale=np.asarray(raw["cmd_scale"], dtype=np.float32),
        num_actions=int(raw["num_actions"]),
        num_obs=int(raw["num_obs"]),
    )


def load_posture_config() -> PostureConfig:
    raw = yaml.safe_load(POSTURE_CONFIG_PATH.read_text(encoding="utf-8"))[
        "/locomotion_node"
    ]["ros__parameters"]
    return PostureConfig(
        waist_joint_target=[float(value) for value in raw["waist_joint_target"]],
        waist_joint_kp=[float(value) for value in raw["waist_joint_kp"]],
        waist_joint_kd=[float(value) for value in raw["waist_joint_kd"]],
        arm_joint_target=[float(value) for value in raw["arm_joint_target"]],
        arm_joint_kp=[float(value) for value in raw["arm_joint_kp"]],
        arm_joint_kd=[float(value) for value in raw["arm_joint_kd"]],
    )


def get_gravity_orientation(
    quaternion: NDArray[np.float32],
) -> NDArray[np.float32]:
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3, dtype=np.float32)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


class LowStateBuffer:
    def __init__(self) -> None:
        self._lock = Lock()
        self._latest: object | None = None

    def update(self, msg: object) -> None:
        with self._lock:
            self._latest = msg

    def read_latest(self) -> object | None:
        with self._lock:
            return self._latest


def init_lowcmd(low_cmd: Any, mode_machine: int, mode_pr: int) -> None:
    low_cmd.mode_machine = int(mode_machine)
    low_cmd.mode_pr = int(mode_pr)
    for motor_cmd in low_cmd.motor_cmd:
        motor_cmd.mode = 1
        motor_cmd.q = 0.0
        motor_cmd.dq = 0.0
        motor_cmd.tau = 0.0
        motor_cmd.kp = 0.0
        motor_cmd.kd = 0.0
        motor_cmd.reserve = 0


def build_observation(
    state: Any,
    deploy_config: DeployConfig,
    previous_action: NDArray[np.float32],
    cmd_vel: NDArray[np.float32],
) -> NDArray[np.float32]:
    qj = np.asarray(
        [state.motor_state[i].q for i in range(deploy_config.num_actions)],
        dtype=np.float32,
    )
    dqj = np.asarray(
        [state.motor_state[i].dq for i in range(deploy_config.num_actions)],
        dtype=np.float32,
    )
    quat = np.asarray(state.imu_state.quaternion, dtype=np.float32)
    omega = np.asarray(state.imu_state.gyroscope, dtype=np.float32)

    qj = (qj - deploy_config.default_angles) * deploy_config.dof_pos_scale
    dqj = dqj * deploy_config.dof_vel_scale
    gravity_orientation = get_gravity_orientation(quat)
    omega = omega * deploy_config.ang_vel_scale

    period = 0.8
    count = int(state.tick) * deploy_config.simulation_dt
    phase = count % period / period
    sin_phase = np.sin(2 * np.pi * phase)
    cos_phase = np.cos(2 * np.pi * phase)

    obs = np.zeros(deploy_config.num_obs, dtype=np.float32)
    obs[:3] = omega
    obs[3:6] = gravity_orientation
    obs[6:9] = cmd_vel * deploy_config.cmd_scale
    obs[9 : 9 + deploy_config.num_actions] = qj
    obs[9 + deploy_config.num_actions : 9 + 2 * deploy_config.num_actions] = dqj
    obs[9 + 2 * deploy_config.num_actions : 9 + 3 * deploy_config.num_actions] = (
        previous_action
    )
    obs[9 + 3 * deploy_config.num_actions : 9 + 3 * deploy_config.num_actions + 2] = (
        np.array([sin_phase, cos_phase], dtype=np.float32)
    )
    return obs


def build_lowcmd(
    action: NDArray[np.float32],
    state: Any,
    deploy_config: DeployConfig,
    posture_config: PostureConfig,
    low_cmd: Any,
    crc: Any,
) -> Any:
    target_dof_pos = action * deploy_config.action_scale + deploy_config.default_angles

    low_cmd.mode_pr = MODE_PR
    low_cmd.mode_machine = int(state.mode_machine)

    for motor_index, motor_cmd in enumerate(low_cmd.motor_cmd):
        motor_cmd.mode = 1
        motor_cmd.q = 0.0
        motor_cmd.dq = 0.0
        motor_cmd.tau = 0.0
        motor_cmd.kp = 0.0
        motor_cmd.kd = 0.0
        motor_cmd.reserve = 0
        if motor_index >= JOINT_COUNT:
            continue

    for joint_index in range(LEG_JOINT_COUNT):
        motor_cmd = low_cmd.motor_cmd[joint_index]
        motor_cmd.q = float(target_dof_pos[joint_index])
        motor_cmd.dq = 0.0
        motor_cmd.tau = 0.0
        motor_cmd.kp = float(deploy_config.kps[joint_index])
        motor_cmd.kd = float(deploy_config.kds[joint_index])

    waist_start = LEG_JOINT_COUNT
    for offset, target in enumerate(posture_config.waist_joint_target):
        motor_cmd = low_cmd.motor_cmd[waist_start + offset]
        motor_cmd.q = float(target)
        motor_cmd.dq = 0.0
        motor_cmd.tau = 0.0
        motor_cmd.kp = float(posture_config.waist_joint_kp[offset])
        motor_cmd.kd = float(posture_config.waist_joint_kd[offset])

    arm_start = LEG_JOINT_COUNT + WAIST_JOINT_COUNT
    for offset, target in enumerate(posture_config.arm_joint_target):
        motor_cmd = low_cmd.motor_cmd[arm_start + offset]
        motor_cmd.q = float(target)
        motor_cmd.dq = 0.0
        motor_cmd.tau = 0.0
        motor_cmd.kp = float(posture_config.arm_joint_kp[offset])
        motor_cmd.kd = float(posture_config.arm_joint_kd[offset])

    low_cmd.crc = crc.Crc(low_cmd)
    return low_cmd


def wait_for_lowstate(buffer: LowStateBuffer) -> Any:
    while True:
        state: Any = buffer.read_latest()
        if state is not None and int(state.tick) != 0:
            return state
        time.sleep(CONTROL_DT)


def wait_for_next_lowstate(buffer: LowStateBuffer, last_tick: int) -> Any:
    while True:
        state: Any = buffer.read_latest()
        if state is not None and int(state.tick) > last_tick:
            return state
        time.sleep(CONTROL_DT)


def main() -> None:
    runtime_types = _load_runtime_types()
    channel_factory_initialize = runtime_types["ChannelFactoryInitialize"]
    channel_publisher_type = runtime_types["ChannelPublisher"]
    channel_subscriber_type = runtime_types["ChannelSubscriber"]
    low_cmd_factory = runtime_types["LowCmdFactory"]
    low_cmd_type = runtime_types["LowCmdType"]
    low_state_type = runtime_types["LowStateType"]
    crc_type = runtime_types["CRC"]

    deploy_config = load_deploy_config()
    posture_config = load_posture_config()
    policy = torch.jit.load(str(POLICY_PATH), map_location="cpu")
    policy.eval()

    channel_factory_initialize(1, "lo")

    low_state_buffer = LowStateBuffer()
    low_cmd = low_cmd_factory()
    init_lowcmd(low_cmd, mode_machine=0, mode_pr=MODE_PR)
    crc = crc_type()

    low_cmd_publisher = channel_publisher_type(TOPIC_LOWCMD, low_cmd_type)
    low_cmd_publisher.Init()

    low_state_subscriber = channel_subscriber_type(TOPIC_LOWSTATE, low_state_type)
    low_state_subscriber.Init(low_state_buffer.update, 10)

    print(f"Loading policy from {POLICY_PATH}")
    print(
        f"Subscribing to {TOPIC_LOWSTATE} and publishing to {TOPIC_LOWCMD} over DDS domain=1 interface=lo"
    )

    initial_state = wait_for_lowstate(low_state_buffer)
    init_lowcmd(low_cmd, mode_machine=int(initial_state.mode_machine), mode_pr=MODE_PR)

    previous_action = np.zeros(deploy_config.num_actions, dtype=np.float32)
    cmd_vel = np.zeros(3, dtype=np.float32)
    last_tick = int(initial_state.tick)

    while True:
        state = wait_for_next_lowstate(low_state_buffer, last_tick)
        last_tick = int(state.tick)
        obs = build_observation(state, deploy_config, previous_action, cmd_vel)
        with torch.inference_mode():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0)
            action = (
                policy(obs_tensor).detach().cpu().numpy().squeeze().astype(np.float32)
            )
        cmd = build_lowcmd(action, state, deploy_config, posture_config, low_cmd, crc)
        low_cmd_publisher.Write(cmd)
        previous_action = action.copy()
        time.sleep(CONTROL_DT)


if __name__ == "__main__":
    main()
