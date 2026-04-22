from __future__ import annotations

from importlib import import_module
import os
from pathlib import Path
from threading import Lock
from types import SimpleNamespace
import sys
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from .policy_runner import (
    JOINT_COUNT,
    PolicyConfig,
    PolicyRunner,
    controlled_joint_mask,
    is_zero_command,
    commanded_velocity_triplet,
)


MODE_PR = 0
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_EMBEDDED_CMD_VEL = "rt/locomotion/cmd_vel"
TOPIC_EMBEDDED_ENABLE = "rt/locomotion/enabled"
DEFAULT_JOINT_COUNT = 29


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


def _load_runtime_types() -> dict[str, Any]:
    g1_msgs_module = import_module("g1_msgs.msg")
    channel_module = import_module("unitree_sdk2py.core.channel")
    default_module = import_module("unitree_sdk2py.idl.default")
    geometry_module = import_module("unitree_sdk2py.idl.geometry_msgs.msg.dds_")
    hg_module = import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    std_module = import_module("unitree_sdk2py.idl.std_msgs.msg.dds_")
    crc_module = import_module("unitree_sdk2py.utils.crc")
    return {
        "JointCommand": g1_msgs_module.JointCommand,
        "LocomotionState": g1_msgs_module.LocomotionState,
        "RobotState": g1_msgs_module.RobotState,
        "ChannelFactoryInitialize": channel_module.ChannelFactoryInitialize,
        "ChannelPublisher": channel_module.ChannelPublisher,
        "ChannelSubscriber": channel_module.ChannelSubscriber,
        "EmbeddedCmdFactory": default_module.geometry_msgs_msg_dds__Twist_,
        "EmbeddedCmdType": geometry_module.Twist_,
        "EmbeddedEnableFactory": default_module.std_msgs_msg_dds__String_,
        "EmbeddedEnableType": std_module.String_,
        "LowCmdFactory": default_module.unitree_hg_msg_dds__LowCmd_,
        "LowCmdType": hg_module.LowCmd_,
        "LowStateType": hg_module.LowState_,
        "CRC": crc_module.CRC,
    }


def resolve_default_policy_path() -> str:
    candidate_paths = [
        Path(
            "/home/plate/unitree-g1-dashboard/ros2_ws/src/g1_locomotion/policies/g1_motion.pt"
        ),
        Path(
            "/home/plate/unitree-g1-dashboard/deps/unitree_rl_gym/deploy/pre_train/g1/motion.pt"
        ),
    ]
    current = Path(__file__).resolve()
    for base in [current, *current.parents]:
        candidate_paths.append(base / "policies" / "g1_motion.pt")
        candidate_paths.append(
            base
            / "deps"
            / "unitree_rl_gym"
            / "deploy"
            / "pre_train"
            / "g1"
            / "motion.pt"
        )

    for candidate in candidate_paths:
        if candidate.exists():
            return str(candidate)
    return str(candidate_paths[0])


def build_joint_command(
    joint_command_type: Any,
    positions: list[float],
    kp: list[float],
    kd: list[float],
    mode_machine: int,
    stamp_msg: Any,
) -> Any:
    command = joint_command_type()
    command.header.stamp = stamp_msg
    command.header.frame_id = "base"
    command.mode_pr = MODE_PR
    command.mode_machine = int(mode_machine)
    command.q = [float(value) for value in positions]
    command.dq = [0.0] * JOINT_COUNT
    command.tau = [0.0] * JOINT_COUNT
    command.kp = [float(value) for value in kp]
    command.kd = [float(value) for value in kd]
    command.joint_mask = controlled_joint_mask()
    return command


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
    for joint_index in range(len(low_cmd.motor_cmd)):
        motor_cmd = low_cmd.motor_cmd[joint_index]
        if joint_index >= joint_count or not bool(
            joint_command.joint_mask[joint_index]
        ):
            reset_motor_command(motor_cmd)
            continue

        motor_cmd.mode = 1
        motor_cmd.q = float(joint_command.q[joint_index])
        motor_cmd.dq = float(joint_command.dq[joint_index])
        motor_cmd.tau = float(joint_command.tau[joint_index])
        motor_cmd.kp = float(joint_command.kp[joint_index])
        motor_cmd.kd = float(joint_command.kd[joint_index])


def build_robot_state_proxy(low_state: Any, joint_count: int) -> Any:
    return SimpleNamespace(
        mode_machine=int(low_state.mode_machine),
        joint_position=[float(low_state.motor_state[i].q) for i in range(joint_count)],
        joint_velocity=[float(low_state.motor_state[i].dq) for i in range(joint_count)],
        imu_quaternion_wxyz=[float(value) for value in low_state.imu_state.quaternion],
        imu_gyroscope=[float(value) for value in low_state.imu_state.gyroscope],
    )


def select_command_targets(
    orchestrator_state: str,
    policy_runner: PolicyRunner,
    robot_state: Any,
    cmd_vel: Any,
) -> Any:
    if orchestrator_state == "STANDING":
        return policy_runner.standing_command()
    return policy_runner.compute_command(robot_state, cmd_vel)


class G1LocomotionNode(Node):
    def __init__(self) -> None:
        super().__init__("locomotion_node")

        deps = _load_runtime_types()
        self.joint_command_type = deps["JointCommand"]
        self.locomotion_state_type = deps["LocomotionState"]
        self.robot_state_type = deps["RobotState"]
        self.channel_factory_initialize = deps["ChannelFactoryInitialize"]
        self.channel_publisher_type = deps["ChannelPublisher"]
        self.channel_subscriber_type = deps["ChannelSubscriber"]
        self.embedded_cmd_factory = deps["EmbeddedCmdFactory"]
        self.embedded_cmd_type = deps["EmbeddedCmdType"]
        self.embedded_enable_factory = deps["EmbeddedEnableFactory"]
        self.embedded_enable_type = deps["EmbeddedEnableType"]
        self.low_cmd_factory = deps["LowCmdFactory"]
        self.low_cmd_type = deps["LowCmdType"]
        self.low_state_type = deps["LowStateType"]
        self.crc_type = deps["CRC"]

        self.declare_parameter("enabled", False)
        self.declare_parameter("mode", "sim")
        self.declare_parameter("embedded_sim_policy", False)
        self.declare_parameter("sim_domain_id", 1)
        self.declare_parameter("sim_interface", "lo")
        self.declare_parameter("real_domain_id", 0)
        self.declare_parameter("real_interface", "eth0")
        self.declare_parameter("state_topic", "/robot_state")
        self.declare_parameter("locomotion_state_topic", "/locomotion_state")
        self.declare_parameter("cmd_vel_topic", "/locomotion/cmd_vel")
        self.declare_parameter("command_topic", "/joint_commands")
        self.declare_parameter("enable_service", "/locomotion/enable")
        self.declare_parameter("disable_service", "/locomotion/disable")
        self.declare_parameter("embedded_cmd_vel_topic", TOPIC_EMBEDDED_CMD_VEL)
        self.declare_parameter("embedded_enable_topic", TOPIC_EMBEDDED_ENABLE)
        self.declare_parameter("rl_control_rate_hz", 200.0)
        self.declare_parameter("fallback_control_rate_hz", 200.0)
        self.declare_parameter("state_timeout_sec", 1.0)
        self.declare_parameter("policy_path", resolve_default_policy_path())
        self.declare_parameter("default_pose", [0.0] * JOINT_COUNT)
        self.declare_parameter("leg_joint_kp", [100.0] * 12)
        self.declare_parameter("leg_joint_kd", [2.0] * 12)
        self.declare_parameter("waist_joint_target", [0.0] * 3)
        self.declare_parameter("waist_joint_kp", [300.0] * 3)
        self.declare_parameter("waist_joint_kd", [3.0] * 3)
        self.declare_parameter("arm_joint_target", [0.0] * 14)
        self.declare_parameter(
            "arm_joint_kp",
            [
                100.0,
                100.0,
                50.0,
                50.0,
                20.0,
                20.0,
                20.0,
                100.0,
                100.0,
                50.0,
                50.0,
                20.0,
                20.0,
                20.0,
            ],
        )
        self.declare_parameter(
            "arm_joint_kd",
            [2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 1.0],
        )
        self.declare_parameter("ang_vel_scale", 0.25)
        self.declare_parameter("dof_pos_scale", 1.0)
        self.declare_parameter("dof_vel_scale", 0.05)
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("cmd_scale", [2.0, 2.0, 0.25])
        self.declare_parameter("max_cmd", [0.8, 0.5, 1.57])
        self.declare_parameter("gait_period_sec", 0.8)
        self.declare_parameter("fallback_stride_frequency_hz", 1.5)
        self.declare_parameter("fallback_leg_lift_rad", 0.18)
        self.declare_parameter("fallback_leg_swing_rad", 0.22)
        self.declare_parameter("fallback_yaw_swing_rad", 0.08)
        self.declare_parameter("fallback_strafe_roll_rad", 0.1)
        self.declare_parameter("fallback_kp_scale", 0.9)
        self.declare_parameter("controlled_joint_mins", [-3.14] * 15)
        self.declare_parameter("controlled_joint_maxs", [3.14] * 15)

        self.enabled = self._bool_param("enabled")
        self.state_topic = self._string_param("state_topic")
        self.locomotion_state_topic = self._string_param("locomotion_state_topic")
        self.cmd_vel_topic = self._string_param("cmd_vel_topic")
        self.command_topic = self._string_param("command_topic")
        self.enable_service_name = self._string_param("enable_service")
        self.disable_service_name = self._string_param("disable_service")
        self.rl_control_rate_hz = self._float_param("rl_control_rate_hz")
        self.fallback_control_rate_hz = self._float_param("fallback_control_rate_hz")
        self.state_timeout_sec = self._float_param("state_timeout_sec")
        self.mode = self._string_param("mode")
        self.embedded_sim_policy = self._bool_param("embedded_sim_policy")
        self.embedded_cmd_vel_topic = self._string_param("embedded_cmd_vel_topic")
        self.embedded_enable_topic = self._string_param("embedded_enable_topic")

        policy_config = PolicyConfig(
            policy_path=self._string_param("policy_path"),
            default_pose=self._float_array_param("default_pose"),
            leg_joint_kp=self._float_array_param("leg_joint_kp"),
            leg_joint_kd=self._float_array_param("leg_joint_kd"),
            waist_joint_target=self._float_array_param("waist_joint_target"),
            waist_joint_kp=self._float_array_param("waist_joint_kp"),
            waist_joint_kd=self._float_array_param("waist_joint_kd"),
            arm_joint_target=self._float_array_param("arm_joint_target"),
            arm_joint_kp=self._float_array_param("arm_joint_kp"),
            arm_joint_kd=self._float_array_param("arm_joint_kd"),
            ang_vel_scale=self._float_param("ang_vel_scale"),
            dof_pos_scale=self._float_param("dof_pos_scale"),
            dof_vel_scale=self._float_param("dof_vel_scale"),
            action_scale=self._float_param("action_scale"),
            cmd_scale=self._float_array_param("cmd_scale"),
            max_cmd=self._float_array_param("max_cmd"),
            gait_period_sec=self._float_param("gait_period_sec"),
            rl_control_dt=1.0 / max(self.rl_control_rate_hz, 1e-6),
            fallback_control_dt=1.0 / max(self.fallback_control_rate_hz, 1e-6),
            fallback_stride_frequency_hz=self._float_param(
                "fallback_stride_frequency_hz"
            ),
            fallback_leg_lift_rad=self._float_param("fallback_leg_lift_rad"),
            fallback_leg_swing_rad=self._float_param("fallback_leg_swing_rad"),
            fallback_yaw_swing_rad=self._float_param("fallback_yaw_swing_rad"),
            fallback_strafe_roll_rad=self._float_param("fallback_strafe_roll_rad"),
            fallback_kp_scale=self._float_param("fallback_kp_scale"),
            controlled_joint_mins=self._float_array_param("controlled_joint_mins"),
            controlled_joint_maxs=self._float_array_param("controlled_joint_maxs"),
        )
        self._validate_policy_config(policy_config)
        self.policy_runner = PolicyRunner(policy_config)

        if self.state_timeout_sec <= 0.0:
            raise ValueError("state_timeout_sec must be > 0.")

        domain_id, interface = self._resolve_channel_config()
        self.channel_factory_initialize(domain_id, interface)
        self.crc = self.crc_type()
        self.low_cmd = self.low_cmd_factory()
        self.low_state_lock = Lock()

        self.latest_low_state: Any | None = None
        self.latest_low_state_time = self.get_clock().now()
        self.current_orchestrator_state = ""
        self.latest_cmd_vel = Twist()
        self.last_publish_time_sec = 0.0
        self._was_control_active = False

        self.locomotion_state_subscription = self.create_subscription(
            self.locomotion_state_type,
            self.locomotion_state_topic,
            self._handle_locomotion_state,
            10,
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self._handle_cmd_vel, 10
        )
        self.enable_service = self.create_service(
            Trigger, self.enable_service_name, self._handle_enable
        )
        self.disable_service = self.create_service(
            Trigger, self.disable_service_name, self._handle_disable
        )
        self.timer = self.create_timer(1.0 / 200.0, self._tick)

        if self.embedded_sim_policy:
            self.embedded_cmd_publisher = self.channel_publisher_type(
                self.embedded_cmd_vel_topic, self.embedded_cmd_type
            )
            self.embedded_cmd_publisher.Init()
            self.embedded_enable_publisher = self.channel_publisher_type(
                self.embedded_enable_topic, self.embedded_enable_type
            )
            self.embedded_enable_publisher.Init()
            self.low_cmd_publisher = None
            self.low_state_subscriber = None
        else:
            self.low_cmd_publisher = self.channel_publisher_type(
                TOPIC_LOWCMD, self.low_cmd_type
            )
            self.low_cmd_publisher.Init()
            self.low_state_subscriber = self.channel_subscriber_type(
                TOPIC_LOWSTATE, self.low_state_type
            )
            self.low_state_subscriber.Init(self._handle_low_state, 10)

        self.get_logger().info(
            "Locomotion node ready on "
            f"DDS backend={self.policy_runner.backend} embedded_sim_policy={self.embedded_sim_policy} "
            f"detail={self.policy_runner.backend_reason}. enabled={self.enabled}. "
            f"domain_id={domain_id} interface={interface}."
        )

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return bool(value)

    def _string_param(self, name: str) -> str:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return str(value)

    def _float_param(self, name: str) -> float:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return float(value)

    def _float_array_param(self, name: str) -> list[float]:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return [float(item) for item in value]

    def _validate_policy_config(self, config: PolicyConfig) -> None:
        if len(config.default_pose) != JOINT_COUNT:
            raise ValueError("default_pose must contain 29 joint values.")
        if len(config.leg_joint_kp) != 12 or len(config.leg_joint_kd) != 12:
            raise ValueError("leg_joint_kp and leg_joint_kd must contain 12 values.")
        if len(config.waist_joint_target) != 3:
            raise ValueError("waist_joint_target must contain 3 values.")
        if len(config.waist_joint_kp) != 3 or len(config.waist_joint_kd) != 3:
            raise ValueError("waist_joint_kp and waist_joint_kd must contain 3 values.")
        if len(config.arm_joint_target) != 14:
            raise ValueError("arm_joint_target must contain 14 values.")
        if len(config.arm_joint_kp) != 14 or len(config.arm_joint_kd) != 14:
            raise ValueError("arm_joint_kp and arm_joint_kd must contain 14 values.")
        if len(config.cmd_scale) != 3 or len(config.max_cmd) != 3:
            raise ValueError("cmd_scale and max_cmd must contain 3 values.")
        if (
            len(config.controlled_joint_mins) != 15
            or len(config.controlled_joint_maxs) != 15
        ):
            raise ValueError(
                "controlled_joint_mins and controlled_joint_maxs must contain 15 values."
            )

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

    def _int_param(self, name: str) -> int:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return int(value)

    def _handle_low_state(self, msg: Any) -> None:
        with self.low_state_lock:
            self.latest_low_state = msg
            self.latest_low_state_time = self.get_clock().now()

    def _handle_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd_vel = msg

    def _handle_locomotion_state(self, msg: Any) -> None:
        self.current_orchestrator_state = str(msg.current_state)

    def _handle_enable(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        self.enabled = True
        self.policy_runner.reset()
        self.last_publish_time_sec = 0.0
        self._was_control_active = False
        if self.embedded_sim_policy:
            self._publish_embedded_enable(True)
        response.success = True
        response.message = (
            f"Locomotion enabled with backend={self.policy_runner.backend}."
        )
        return response

    def _handle_disable(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        self.enabled = False
        self.policy_runner.reset()
        self.last_publish_time_sec = 0.0
        self._was_control_active = False
        if self.embedded_sim_policy:
            self._publish_embedded_enable(False)
        response.success = True
        response.message = "Locomotion disabled."
        return response

    def _publish_embedded_enable(self, enabled: bool) -> None:
        message = self.embedded_enable_factory()
        message.data = "enable" if enabled else "disable"
        self.embedded_enable_publisher.Write(message)

    def _publish_embedded_cmd_vel(self, cmd_vel: Twist) -> None:
        message = self.embedded_cmd_factory()
        message.linear.x = float(cmd_vel.linear.x)
        message.linear.y = float(cmd_vel.linear.y)
        message.linear.z = float(cmd_vel.linear.z)
        message.angular.x = float(cmd_vel.angular.x)
        message.angular.y = float(cmd_vel.angular.y)
        message.angular.z = float(cmd_vel.angular.z)
        self.embedded_cmd_publisher.Write(message)

    def _desired_publish_period_sec(self) -> float:
        command = commanded_velocity_triplet(self.latest_cmd_vel)
        if is_zero_command(command):
            return 1.0 / max(self.rl_control_rate_hz, 1e-6)
        if self.policy_runner.uses_rl_policy:
            return 1.0 / max(self.rl_control_rate_hz, 1e-6)
        return 1.0 / max(self.fallback_control_rate_hz, 1e-6)

    def _tick(self) -> None:
        if self.embedded_sim_policy:
            if self.enabled and self.current_orchestrator_state == "WALKING":
                self._publish_embedded_cmd_vel(self.latest_cmd_vel)
            else:
                self._publish_embedded_cmd_vel(Twist())
            return

        with self.low_state_lock:
            low_state = self.latest_low_state
            state_age_sec = (
                self.get_clock().now() - self.latest_low_state_time
            ).nanoseconds / 1e9

        control_active = (
            self.enabled
            and low_state is not None
            and state_age_sec <= self.state_timeout_sec
            and self.current_orchestrator_state in {"STANDING", "WALKING"}
        )
        if not control_active:
            if self._was_control_active:
                self.policy_runner.reset()
                self.last_publish_time_sec = 0.0
            self._was_control_active = False
            return
        self._was_control_active = True

        if low_state is None:
            self.policy_runner.reset()
            self.last_publish_time_sec = 0.0
            self._was_control_active = False
            return

        robot_state = build_robot_state_proxy(low_state, DEFAULT_JOINT_COUNT)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_publish_time_sec < self._desired_publish_period_sec():
            return
        self.last_publish_time_sec = now_sec

        targets = select_command_targets(
            self.current_orchestrator_state,
            self.policy_runner,
            robot_state,
            self.latest_cmd_vel,
        )

        mode_machine = int(robot_state.mode_machine)
        command = build_joint_command(
            self.joint_command_type,
            targets.positions,
            targets.kp,
            targets.kd,
            mode_machine,
            self.get_clock().now().to_msg(),
        )
        apply_joint_command_to_low_cmd(self.low_cmd, command, DEFAULT_JOINT_COUNT)
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        if self.low_cmd_publisher is None:
            raise RuntimeError(
                "low_cmd_publisher unavailable while embedded_sim_policy is disabled."
            )
        self.low_cmd_publisher.Write(self.low_cmd)

    def destroy_node(self) -> None:
        try:
            if self.low_state_subscriber is not None:
                self.low_state_subscriber.Close()
        except Exception:
            pass

        try:
            if self.low_cmd_publisher is not None:
                self.low_cmd_publisher.Close()
        except Exception:
            pass

        try:
            if self.embedded_sim_policy:
                self.embedded_cmd_publisher.Close()
                self.embedded_enable_publisher.Close()
        except Exception:
            pass

        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = G1LocomotionNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
