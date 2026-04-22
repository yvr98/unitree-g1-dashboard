from __future__ import annotations

from importlib import import_module
import os
from pathlib import Path
from threading import Lock
from typing import Any
import sys


def _ensure_unitree_sdk_path() -> None:
    env_path = Path(
        os.environ.get(
            "UNITREE_SDK2PY_PATH",
            "/home/plate/unitree-g1-dashboard/deps/unitree_sdk2_python",
        )
    )
    if env_path.exists():
        env_path_str = str(env_path)
        if env_path_str not in sys.path:
            sys.path.insert(0, env_path_str)
        return

    current = Path(__file__).resolve()
    for base in [current, *current.parents]:
        candidate = base / "deps" / "unitree_sdk2_python"
        if candidate.exists():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
            return


_ensure_unitree_sdk_path()

import rclpy
from rclpy.node import Node


TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
DEFAULT_JOINT_COUNT = 29


def build_robot_state_message(
    robot_state_type: Any,
    low_state: Any,
    joint_count: int,
    frame_id: str,
    stamp_msg: Any,
    connected: bool,
) -> Any:
    ros_msg = robot_state_type()
    ros_msg.header.stamp = stamp_msg
    ros_msg.header.frame_id = frame_id
    ros_msg.connected = connected
    ros_msg.version = list(low_state.version)
    ros_msg.mode_pr = int(low_state.mode_pr)
    ros_msg.mode_machine = int(low_state.mode_machine)
    ros_msg.tick = int(low_state.tick)

    ros_msg.joint_position = [
        float(low_state.motor_state[i].q) for i in range(joint_count)
    ]
    ros_msg.joint_velocity = [
        float(low_state.motor_state[i].dq) for i in range(joint_count)
    ]
    ros_msg.joint_acceleration = [
        float(low_state.motor_state[i].ddq) for i in range(joint_count)
    ]
    ros_msg.joint_torque_estimate = [
        float(low_state.motor_state[i].tau_est) for i in range(joint_count)
    ]
    ros_msg.joint_mode = [
        int(low_state.motor_state[i].mode) for i in range(joint_count)
    ]
    ros_msg.imu_quaternion_wxyz = [
        float(value) for value in low_state.imu_state.quaternion
    ]
    ros_msg.imu_gyroscope = [float(value) for value in low_state.imu_state.gyroscope]
    ros_msg.imu_accelerometer = [
        float(value) for value in low_state.imu_state.accelerometer
    ]
    ros_msg.imu_rpy = [float(value) for value in low_state.imu_state.rpy]
    ros_msg.imu_temperature = int(low_state.imu_state.temperature)
    ros_msg.wireless_remote = [int(value) for value in low_state.wireless_remote]
    ros_msg.crc = int(low_state.crc)
    return ros_msg


def reset_motor_command(motor_cmd: Any) -> None:
    motor_cmd.mode = 0
    motor_cmd.q = 0.0
    motor_cmd.dq = 0.0
    motor_cmd.tau = 0.0
    motor_cmd.kp = 0.0
    motor_cmd.kd = 0.0


def apply_joint_command_to_low_cmd(
    low_cmd: Any, joint_command: Any, joint_count: int
) -> None:
    low_cmd.mode_pr = int(joint_command.mode_pr)
    low_cmd.mode_machine = int(joint_command.mode_machine)
    for joint_index in range(joint_count):
        motor_cmd = low_cmd.motor_cmd[joint_index]
        if not bool(joint_command.joint_mask[joint_index]):
            reset_motor_command(motor_cmd)
            continue

        motor_cmd.mode = 1
        motor_cmd.q = float(joint_command.q[joint_index])
        motor_cmd.dq = float(joint_command.dq[joint_index])
        motor_cmd.tau = float(joint_command.tau[joint_index])
        motor_cmd.kp = float(joint_command.kp[joint_index])
        motor_cmd.kd = float(joint_command.kd[joint_index])


def _load_runtime_dependencies() -> dict[str, Any]:
    g1_msgs_module = import_module("g1_msgs.msg")
    channel_module = import_module("unitree_sdk2py.core.channel")
    default_module = import_module("unitree_sdk2py.idl.default")
    hg_module = import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    crc_module = import_module("unitree_sdk2py.utils.crc")

    return {
        "JointCommand": g1_msgs_module.JointCommand,
        "RobotState": g1_msgs_module.RobotState,
        "ChannelFactoryInitialize": channel_module.ChannelFactoryInitialize,
        "ChannelPublisher": channel_module.ChannelPublisher,
        "ChannelSubscriber": channel_module.ChannelSubscriber,
        "LowCmdFactory": default_module.unitree_hg_msg_dds__LowCmd_,
        "LowCmdType": hg_module.LowCmd_,
        "LowStateType": hg_module.LowState_,
        "CRC": crc_module.CRC,
    }


class SdkBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sdk_bridge")

        deps = _load_runtime_dependencies()
        self.joint_command_type = deps["JointCommand"]
        self.robot_state_type = deps["RobotState"]
        self.channel_factory_initialize = deps["ChannelFactoryInitialize"]
        self.channel_publisher_type = deps["ChannelPublisher"]
        self.channel_subscriber_type = deps["ChannelSubscriber"]
        self.low_cmd_factory = deps["LowCmdFactory"]
        self.low_cmd_type = deps["LowCmdType"]
        self.low_state_type = deps["LowStateType"]
        self.crc_type = deps["CRC"]

        self.declare_parameter("mode", "sim")
        self.declare_parameter("sim_domain_id", 1)
        self.declare_parameter("sim_interface", "lo")
        self.declare_parameter("real_domain_id", 0)
        self.declare_parameter("real_interface", "eth0")
        self.declare_parameter("state_topic", "/robot_state")
        self.declare_parameter("command_topic", "/joint_commands")
        self.declare_parameter("joint_count", DEFAULT_JOINT_COUNT)
        self.declare_parameter("control_rate_hz", 500.0)
        self.declare_parameter("command_timeout_sec", 0.1)
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("frame_id", "base")

        self.mode = self._string_param("mode")
        self.joint_count = self._int_param("joint_count")
        self.control_rate_hz = self._float_param("control_rate_hz")
        self.command_timeout_sec = self._float_param("command_timeout_sec")
        self.state_timeout_sec = self._float_param("state_timeout_sec")
        self.frame_id = self._string_param("frame_id")

        if self.joint_count != DEFAULT_JOINT_COUNT:
            raise ValueError(
                f"g1_bridge only supports {DEFAULT_JOINT_COUNT} joints right now."
            )
        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0.")

        domain_id, interface = self._resolve_channel_config()
        self.channel_factory_initialize(domain_id, interface)

        self.get_logger().info(
            f"Initialized Unitree channels in {self.mode!r} mode with domain_id={domain_id}, interface={interface!r}."
        )

        self.crc = self.crc_type()
        self.lock = Lock()
        self.latest_low_state: Any | None = None
        self.latest_joint_command: Any | None = None
        self.latest_state_time = self.get_clock().now()
        self.latest_command_time = self.get_clock().now()

        self.current_low_cmd = self.low_cmd_factory()

        state_topic = self._string_param("state_topic")
        command_topic = self._string_param("command_topic")

        self.robot_state_publisher = self.create_publisher(
            self.robot_state_type, state_topic, 10
        )
        self.joint_command_subscriber = self.create_subscription(
            self.joint_command_type,
            command_topic,
            self._handle_joint_command,
            10,
        )

        self.low_cmd_publisher = self.channel_publisher_type(
            TOPIC_LOWCMD, self.low_cmd_type
        )
        self.low_cmd_publisher.Init()
        self.low_state_subscriber = self.channel_subscriber_type(
            TOPIC_LOWSTATE, self.low_state_type
        )
        self.low_state_subscriber.Init(self._handle_low_state, 10)

        period = 1.0 / self.control_rate_hz
        self.state_timer = self.create_timer(period, self._publish_robot_state)
        self.command_timer = self.create_timer(period, self._write_low_cmd)

    def _resolve_channel_config(self) -> tuple[int, str]:
        if self.mode == "sim":
            return (
                self._int_param("sim_domain_id"),
                self._string_param("sim_interface"),
            )
        if self.mode == "real":
            return (
                self._int_param("real_domain_id"),
                self._string_param("real_interface"),
            )
        raise ValueError(f"Unsupported mode {self.mode!r}; expected 'sim' or 'real'.")

    def _string_param(self, name: str) -> str:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return str(value)

    def _int_param(self, name: str) -> int:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return int(value)

    def _float_param(self, name: str) -> float:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return float(value)

    def _handle_low_state(self, msg: Any) -> None:
        with self.lock:
            self.latest_low_state = msg
            self.latest_state_time = self.get_clock().now()

    def _handle_joint_command(self, msg: Any) -> None:
        with self.lock:
            self.latest_joint_command = msg
            self.latest_command_time = self.get_clock().now()

    def _publish_robot_state(self) -> None:
        with self.lock:
            low_state = self.latest_low_state
            state_age = (
                self.get_clock().now() - self.latest_state_time
            ).nanoseconds / 1e9

        if low_state is None:
            return

        ros_msg = build_robot_state_message(
            self.robot_state_type,
            low_state,
            self.joint_count,
            self.frame_id,
            self.get_clock().now().to_msg(),
            state_age <= self.state_timeout_sec,
        )

        self.robot_state_publisher.publish(ros_msg)

    def _write_low_cmd(self) -> None:
        with self.lock:
            low_state = self.latest_low_state
            joint_command = self.latest_joint_command
            command_age = (
                self.get_clock().now() - self.latest_command_time
            ).nanoseconds / 1e9

            low_cmd = self.current_low_cmd

            if joint_command is None or command_age > self.command_timeout_sec:
                self._disable_all_motors(low_cmd)
                if low_state is not None:
                    low_cmd.mode_machine = int(low_state.mode_machine)
            else:
                apply_joint_command_to_low_cmd(low_cmd, joint_command, self.joint_count)

            low_cmd.crc = self.crc.Crc(low_cmd)

        self.low_cmd_publisher.Write(low_cmd)

    def _disable_all_motors(self, low_cmd: Any) -> None:
        for joint_index in range(self.joint_count):
            reset_motor_command(low_cmd.motor_cmd[joint_index])

    def destroy_node(self) -> None:
        try:
            self.low_state_subscriber.Close()
        except Exception:
            pass

        try:
            self.low_cmd_publisher.Close()
        except Exception:
            pass

        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SdkBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
