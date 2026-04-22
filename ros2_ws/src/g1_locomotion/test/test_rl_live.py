from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
import signal
import subprocess
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
import torch
import yaml


mujoco = import_module("mujoco")


def load_message_types() -> tuple[Any, Any]:
    g1_msgs_module = import_module("g1_msgs.msg")
    return g1_msgs_module.JointCommand, g1_msgs_module.RobotState


JointCommand, RobotState = load_message_types()


REPO_ROOT = Path(__file__).resolve().parents[4]
ROS_WS_DIR = REPO_ROOT / "ros2_ws"
MUJOCO_DIR = REPO_ROOT / "deps" / "unitree_mujoco" / "simulate_python"
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
SIM_DURATION_SEC = 10.0
OBS_PRINT_COUNT = 5
CONTROL_DECIMATION = 10
JOINT_COUNT = 29
LEG_JOINT_COUNT = 12


@dataclass(frozen=True)
class DeployConfig:
    policy_path: str
    xml_path: str
    simulation_dt: float
    control_decimation: int
    kps: NDArray[np.float32]
    kds: NDArray[np.float32]
    default_angles: NDArray[np.float32]
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


@dataclass(frozen=True)
class TraceSample:
    observation: list[float]
    action: list[float]


def load_deploy_config() -> DeployConfig:
    raw = yaml.safe_load(DEPLOY_CONFIG_PATH.read_text(encoding="utf-8"))
    root_dir = str(REPO_ROOT / "deps" / "unitree_rl_gym")
    return DeployConfig(
        policy_path=raw["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", root_dir),
        xml_path=raw["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", root_dir),
        simulation_dt=float(raw["simulation_dt"]),
        control_decimation=int(raw["control_decimation"]),
        kps=np.asarray(raw["kps"], dtype=np.float32),
        kds=np.asarray(raw["kds"], dtype=np.float32),
        default_angles=np.asarray(raw["default_angles"], dtype=np.float32),
        ang_vel_scale=float(raw["ang_vel_scale"]),
        dof_pos_scale=float(raw["dof_pos_scale"]),
        dof_vel_scale=float(raw["dof_vel_scale"]),
        action_scale=float(raw["action_scale"]),
        cmd_scale=np.asarray(raw["cmd_scale"], dtype=np.float32),
        num_actions=int(raw["num_actions"]),
        num_obs=int(raw["num_obs"]),
    )


def load_posture_config() -> PostureConfig:
    raw = yaml.safe_load(LOCOMOTION_CONFIG_PATH.read_text(encoding="utf-8"))[
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
    quaternion: list[float] | NDArray[np.float32],
) -> NDArray[np.float32]:
    qw = float(quaternion[0])
    qx = float(quaternion[1])
    qy = float(quaternion[2])
    qz = float(quaternion[3])
    gravity_orientation = np.zeros(3, dtype=np.float32)
    gravity_orientation[0] = 2.0 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2.0 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return gravity_orientation


def quaternion_wxyz_to_roll_pitch(
    quaternion: list[float] | NDArray[np.float32],
) -> tuple[float, float]:
    w = float(quaternion[0])
    x = float(quaternion[1])
    y = float(quaternion[2])
    z = float(quaternion[3])
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def build_observation(
    deploy_config: DeployConfig,
    joint_position: NDArray[np.float32],
    joint_velocity: NDArray[np.float32],
    quaternion: list[float] | NDArray[np.float32],
    imu_gyroscope: list[float] | NDArray[np.float32],
    previous_action: NDArray[np.float32],
    counter: int,
) -> NDArray[np.float32]:
    qj = (joint_position - deploy_config.default_angles) * deploy_config.dof_pos_scale
    dqj = joint_velocity * deploy_config.dof_vel_scale
    gravity_orientation = get_gravity_orientation(quaternion)
    omega = np.asarray(imu_gyroscope, dtype=np.float32) * deploy_config.ang_vel_scale
    cmd = np.zeros(3, dtype=np.float32)
    count_time = counter * deploy_config.simulation_dt
    phase = count_time % 0.8 / 0.8
    sin_phase = math.sin(2.0 * math.pi * phase)
    cos_phase = math.cos(2.0 * math.pi * phase)

    observation = np.zeros(deploy_config.num_obs, dtype=np.float32)
    observation[:3] = omega
    observation[3:6] = gravity_orientation
    observation[6:9] = cmd * deploy_config.cmd_scale
    observation[9 : 9 + deploy_config.num_actions] = qj
    observation[9 + deploy_config.num_actions : 9 + 2 * deploy_config.num_actions] = dqj
    observation[
        9 + 2 * deploy_config.num_actions : 9 + 3 * deploy_config.num_actions
    ] = previous_action
    observation[
        9 + 3 * deploy_config.num_actions : 9 + 3 * deploy_config.num_actions + 2
    ] = np.asarray(
        [sin_phase, cos_phase],
        dtype=np.float32,
    )
    return observation


def build_joint_command_message(
    target_leg_positions: NDArray[np.float32],
    deploy_config: DeployConfig,
    posture_config: PostureConfig,
    mode_machine: int,
) -> Any:
    positions = [0.0] * JOINT_COUNT
    kp = [0.0] * JOINT_COUNT
    kd = [0.0] * JOINT_COUNT

    for joint_index in range(LEG_JOINT_COUNT):
        positions[joint_index] = float(target_leg_positions[joint_index])
        kp[joint_index] = float(deploy_config.kps[joint_index])
        kd[joint_index] = float(deploy_config.kds[joint_index])

    waist_start = LEG_JOINT_COUNT
    for offset, target in enumerate(posture_config.waist_joint_target):
        positions[waist_start + offset] = target
        kp[waist_start + offset] = posture_config.waist_joint_kp[offset]
        kd[waist_start + offset] = posture_config.waist_joint_kd[offset]

    arm_start = waist_start + len(posture_config.waist_joint_target)
    for offset, target in enumerate(posture_config.arm_joint_target):
        positions[arm_start + offset] = target
        kp[arm_start + offset] = posture_config.arm_joint_kp[offset]
        kd[arm_start + offset] = posture_config.arm_joint_kd[offset]

    command = JointCommand()
    command.header.stamp.sec = 0
    command.header.stamp.nanosec = 0
    command.header.frame_id = "base"
    command.mode_pr = 0
    command.mode_machine = int(mode_machine)
    command.q = positions
    command.dq = [0.0] * JOINT_COUNT
    command.tau = [0.0] * JOINT_COUNT
    command.kp = kp
    command.kd = kd
    command.joint_mask = [True] * JOINT_COUNT
    return command


def load_policy_module(policy_path: str) -> Any:
    policy = torch.jit.load(policy_path, map_location="cpu")
    policy.eval()
    return policy


class RosRlLiveNode(Node):
    def __init__(
        self, deploy_config: DeployConfig, posture_config: PostureConfig, policy: Any
    ) -> None:
        super().__init__("test_rl_live")
        self.deploy_config = deploy_config
        self.posture_config = posture_config
        self.policy = policy
        self.robot_state_subscription = self.create_subscription(
            RobotState,
            "/robot_state",
            self._handle_robot_state,
            10,
        )
        self.command_publisher = self.create_publisher(
            JointCommand, "/joint_commands", 10
        )
        self.latest_robot_state: Any | None = None
        self.robot_state_sequence = 0
        self.last_processed_sequence = -1
        self.counter = 0
        self.previous_action = np.zeros(
            self.deploy_config.num_actions, dtype=np.float32
        )
        self.target_leg_positions = self.deploy_config.default_angles.copy()
        self.trace_samples: list[TraceSample] = []
        self.max_abs_roll = 0.0
        self.max_abs_pitch = 0.0
        self.first_state_time: float | None = None

    def _handle_robot_state(self, msg: Any) -> None:
        self.latest_robot_state = msg
        self.robot_state_sequence += 1
        if self.first_state_time is None:
            self.first_state_time = time.time()
        roll, pitch = quaternion_wxyz_to_roll_pitch(list(msg.imu_quaternion_wxyz))
        self.max_abs_roll = max(self.max_abs_roll, abs(roll))
        self.max_abs_pitch = max(self.max_abs_pitch, abs(pitch))

    def step(self) -> None:
        robot_state = self.latest_robot_state
        if robot_state is None:
            return
        if self.robot_state_sequence == self.last_processed_sequence:
            return
        self.last_processed_sequence = self.robot_state_sequence
        self.counter += 1
        self.command_publisher.publish(
            build_joint_command_message(
                self.target_leg_positions,
                self.deploy_config,
                self.posture_config,
                int(robot_state.mode_machine),
            )
        )
        if self.counter % self.deploy_config.control_decimation != 0:
            return

        observation = build_observation(
            self.deploy_config,
            np.asarray(robot_state.joint_position[:LEG_JOINT_COUNT], dtype=np.float32),
            np.asarray(robot_state.joint_velocity[:LEG_JOINT_COUNT], dtype=np.float32),
            list(robot_state.imu_quaternion_wxyz),
            list(robot_state.imu_gyroscope),
            self.previous_action,
            self.counter,
        )
        action = (
            self.policy(torch.from_numpy(observation).unsqueeze(0))
            .detach()
            .cpu()
            .numpy()
            .squeeze()
            .astype(np.float32)
        )
        self.target_leg_positions = (
            action * self.deploy_config.action_scale + self.deploy_config.default_angles
        )
        if len(self.trace_samples) < OBS_PRINT_COUNT:
            self.trace_samples.append(
                TraceSample(
                    observation=observation.tolist(),
                    action=action.tolist(),
                )
            )
        self.previous_action = action.copy()


def start_process(command: str, cwd: Path, log_path: Path) -> subprocess.Popen[str]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(cwd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    return process


def stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def run_ros_live_trial(
    deploy_config: DeployConfig, posture_config: PostureConfig
) -> dict[str, Any]:
    env_source = f"source /opt/ros/*/setup.bash && source '{ROS_WS_DIR / 'install' / 'setup.bash'}'"
    mujoco_command = (
        "xvfb-run -a env UNITREE_MUJOCO_ELASTIC_BAND=1 "
        f"UNITREE_MUJOCO_SIMULATE_DT={deploy_config.simulation_dt} python3 unitree_mujoco.py"
    )
    bridge_command = f"{env_source} && ros2 launch g1_bridge bridge.launch.py"

    mujoco_process = start_process(
        mujoco_command,
        MUJOCO_DIR,
        Path("/tmp/test_rl_live_mujoco.log"),
    )
    time.sleep(0.05)
    bridge_process = start_process(
        bridge_command,
        REPO_ROOT,
        Path("/tmp/test_rl_live_bridge.log"),
    )

    policy = load_policy_module(deploy_config.policy_path)
    rclpy.init()
    node = RosRlLiveNode(deploy_config, posture_config, policy)
    warmup_end = time.time() + (
        deploy_config.simulation_dt * deploy_config.control_decimation
    )
    try:
        while time.time() < warmup_end:
            if mujoco_process.poll() is not None:
                raise RuntimeError(
                    "MuJoCo process exited early; inspect /tmp/test_rl_live_mujoco.log"
                )
            if bridge_process.poll() is not None:
                raise RuntimeError(
                    "Bridge process exited early; inspect /tmp/test_rl_live_bridge.log"
                )
            rclpy.spin_once(node, timeout_sec=deploy_config.simulation_dt)
            node.command_publisher.publish(
                build_joint_command_message(
                    node.target_leg_positions,
                    deploy_config,
                    posture_config,
                    0,
                )
            )
        start_time = time.time()
        while time.time() - start_time < SIM_DURATION_SEC:
            if mujoco_process.poll() is not None:
                raise RuntimeError(
                    "MuJoCo process exited early; inspect /tmp/test_rl_live_mujoco.log"
                )
            if bridge_process.poll() is not None:
                raise RuntimeError(
                    "Bridge process exited early; inspect /tmp/test_rl_live_bridge.log"
                )
            rclpy.spin_once(node, timeout_sec=0.01)
            node.step()
        if node.latest_robot_state is None:
            raise RuntimeError(
                "No /robot_state messages received during ROS live RL test."
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_process(bridge_process)
        stop_process(mujoco_process)

    return {
        "trace_samples": node.trace_samples,
        "max_abs_roll": node.max_abs_roll,
        "max_abs_pitch": node.max_abs_pitch,
        "received_robot_state": node.latest_robot_state is not None,
        "mujoco_log": "/tmp/test_rl_live_mujoco.log",
        "bridge_log": "/tmp/test_rl_live_bridge.log",
    }


def pd_control(
    target_q: NDArray[np.float32],
    q: NDArray[np.float32],
    kp: NDArray[np.float32],
    target_dq: NDArray[np.float32],
    dq: NDArray[np.float32],
    kd: NDArray[np.float32],
) -> NDArray[np.float32]:
    return ((target_q - q) * kp + (target_dq - dq) * kd).astype(np.float32)


def run_headless_deploy_reference(deploy_config: DeployConfig) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(deploy_config.xml_path)
    data = mujoco.MjData(model)
    model.opt.timestep = deploy_config.simulation_dt
    policy = load_policy_module(deploy_config.policy_path)

    action = np.zeros(deploy_config.num_actions, dtype=np.float32)
    target_dof_pos = deploy_config.default_angles.copy()
    trace_samples: list[TraceSample] = []
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    step_count = 0
    total_steps = int(SIM_DURATION_SEC / deploy_config.simulation_dt)

    for _ in range(total_steps):
        tau = pd_control(
            target_dof_pos,
            np.asarray(data.qpos[7:], dtype=np.float32),
            deploy_config.kps,
            np.zeros_like(deploy_config.kds),
            np.asarray(data.qvel[6:], dtype=np.float32),
            deploy_config.kds,
        )
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        step_count += 1

        roll, pitch = quaternion_wxyz_to_roll_pitch(
            np.asarray(data.qpos[3:7], dtype=np.float32)
        )
        max_abs_roll = max(max_abs_roll, abs(roll))
        max_abs_pitch = max(max_abs_pitch, abs(pitch))

        if step_count % deploy_config.control_decimation != 0:
            continue

        observation = build_observation(
            deploy_config,
            np.asarray(data.qpos[7:], dtype=np.float32),
            np.asarray(data.qvel[6:], dtype=np.float32),
            np.asarray(data.qpos[3:7], dtype=np.float32),
            np.asarray(data.qvel[3:6], dtype=np.float32),
            action,
            step_count,
        )
        action = (
            policy(torch.from_numpy(observation).unsqueeze(0))
            .detach()
            .cpu()
            .numpy()
            .squeeze()
            .astype(np.float32)
        )
        target_dof_pos = (
            action * deploy_config.action_scale + deploy_config.default_angles
        )
        if len(trace_samples) < OBS_PRINT_COUNT:
            trace_samples.append(
                TraceSample(
                    observation=observation.tolist(),
                    action=action.tolist(),
                )
            )

    return {
        "trace_samples": trace_samples,
        "max_abs_roll": max_abs_roll,
        "max_abs_pitch": max_abs_pitch,
    }


def compare_trace_samples(
    ros_samples: list[TraceSample],
    deploy_samples: list[TraceSample],
) -> dict[str, Any]:
    observation_names = {
        "ang_vel": slice(0, 3),
        "gravity": slice(3, 6),
        "cmd": slice(6, 9),
        "qj": slice(9, 21),
        "dqj": slice(21, 33),
        "previous_action": slice(33, 45),
        "phase": slice(45, 47),
    }
    compared = min(len(ros_samples), len(deploy_samples))
    group_diffs: dict[str, float] = {}
    sample_diffs: list[dict[str, float]] = []
    if compared == 0:
        return {
            "compared_samples": 0,
            "group_max_abs_diff": {},
            "per_sample_max_abs_diff": [],
        }

    ros_obs = np.asarray(
        [sample.observation for sample in ros_samples[:compared]], dtype=np.float32
    )
    deploy_obs = np.asarray(
        [sample.observation for sample in deploy_samples[:compared]], dtype=np.float32
    )
    diff = np.abs(ros_obs - deploy_obs)

    for name, index_slice in observation_names.items():
        group_diffs[name] = float(np.max(diff[:, index_slice]))
    for sample_index in range(compared):
        sample_diffs.append(
            {
                name: float(np.max(diff[sample_index, index_slice]))
                for name, index_slice in observation_names.items()
            }
        )
    return {
        "compared_samples": compared,
        "group_max_abs_diff": group_diffs,
        "per_sample_max_abs_diff": sample_diffs,
    }


def print_trace(label: str, trace_samples: list[TraceSample]) -> None:
    print(f"{label} first_{len(trace_samples)}_observations_actions:")
    for index, sample in enumerate(trace_samples, start=1):
        print(f"  sample_{index}_observation={json.dumps(sample.observation)}")
        print(f"  sample_{index}_action={json.dumps(sample.action)}")


def main() -> None:
    deploy_config = load_deploy_config()
    posture_config = load_posture_config()

    ros_result = run_ros_live_trial(deploy_config, posture_config)
    deploy_result = run_headless_deploy_reference(deploy_config)
    comparison = compare_trace_samples(
        ros_result["trace_samples"],
        deploy_result["trace_samples"],
    )

    print_trace("ros_live", ros_result["trace_samples"])
    print_trace("deploy_headless", deploy_result["trace_samples"])
    print("summary:")
    print(
        json.dumps(
            {
                "ros_live": {
                    "received_robot_state": ros_result["received_robot_state"],
                    "max_abs_roll": ros_result["max_abs_roll"],
                    "max_abs_pitch": ros_result["max_abs_pitch"],
                    "stable_with_band": bool(
                        ros_result["max_abs_pitch"] < 0.5
                        and ros_result["max_abs_roll"] < 0.5
                    ),
                    "mujoco_log": ros_result["mujoco_log"],
                    "bridge_log": ros_result["bridge_log"],
                },
                "deploy_headless": {
                    "max_abs_roll": deploy_result["max_abs_roll"],
                    "max_abs_pitch": deploy_result["max_abs_pitch"],
                    "stable": bool(
                        deploy_result["max_abs_pitch"] < 0.5
                        and deploy_result["max_abs_roll"] < 0.5
                    ),
                },
                "comparison": comparison,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
