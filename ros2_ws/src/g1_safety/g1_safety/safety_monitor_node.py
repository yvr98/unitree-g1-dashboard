from __future__ import annotations

from importlib import import_module
import math
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time

from .joint_limits import JOINT_NAME_ORDER, JointLimit, load_joint_limits


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def quaternion_wxyz_to_roll_pitch(
    w: float, x: float, y: float, z: float
) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))
    return roll, pitch


def _load_message_types() -> dict[str, Any]:
    g1_msgs_module = import_module("g1_msgs.msg")
    return {
        "RobotState": g1_msgs_module.RobotState,
        "SafetyStatus": g1_msgs_module.SafetyStatus,
    }


def _first_joint_alert(violation_sources: list[str]) -> str:
    for source in violation_sources:
        if source.endswith(":below_min") or source.endswith(":above_max"):
            return source
    return "joint_limit_violation"


class G1SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor_node")

        deps = _load_message_types()
        self.robot_state_type = deps["RobotState"]
        self.safety_status_type = deps["SafetyStatus"]

        self.declare_parameter("state_topic", "/robot_state")
        self.declare_parameter("safety_topic", "/safety_status")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("state_timeout_sec", 0.5)
        self.declare_parameter("max_pitch_rad", 0.78)
        self.declare_parameter("max_roll_rad", 0.78)
        self.declare_parameter("joint_limit_margin_rad", 0.0)
        self.declare_parameter("joint_limit_names", list(JOINT_NAME_ORDER))
        self.declare_parameter("joint_limit_mins", [0.0] * len(JOINT_NAME_ORDER))
        self.declare_parameter("joint_limit_maxs", [0.0] * len(JOINT_NAME_ORDER))

        self.state_topic = self._string_param("state_topic")
        self.safety_topic = self._string_param("safety_topic")
        self.publish_rate_hz = self._float_param("publish_rate_hz")
        self.state_timeout_sec = self._float_param("state_timeout_sec")
        self.max_pitch_rad = self._float_param("max_pitch_rad")
        self.max_roll_rad = self._float_param("max_roll_rad")
        self.joint_limit_margin_rad = self._float_param("joint_limit_margin_rad")
        self.joint_limits: list[JointLimit] = load_joint_limits(
            self._string_array_param("joint_limit_names"),
            self._float_array_param("joint_limit_mins"),
            self._float_array_param("joint_limit_maxs"),
        )

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.")
        if self.state_timeout_sec <= 0.0:
            raise ValueError("state_timeout_sec must be > 0.")

        self.latest_state: Any | None = None
        self.latest_state_receipt_time: Time | None = None
        self.last_status_safe: bool | None = None

        self.safety_publisher = self.create_publisher(
            self.safety_status_type, self.safety_topic, 10
        )
        self.state_subscription = self.create_subscription(
            self.robot_state_type,
            self.state_topic,
            self._handle_robot_state,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._publish_safety_status
        )

        self.get_logger().info(
            f"Loaded {len(self.joint_limits)} joint limits from parameters and publishing to {self.safety_topic}."
        )

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

    def _string_array_param(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return [str(item) for item in value]

    def _float_array_param(self, name: str) -> list[float]:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return [float(item) for item in value]

    def _handle_robot_state(self, msg: Any) -> None:
        self.latest_state = msg
        self.latest_state_receipt_time = self.get_clock().now()

    def _publish_default_unsafe_status(self, now: Time) -> None:
        status = self.safety_status_type()
        status.header.stamp = now.to_msg()
        status.header.frame_id = "base"
        status.is_safe = False
        status.is_upright = False
        status.estop_active = True
        status.data_stale = True
        status.joints_within_limits = False
        status.pitch = 0.0
        status.roll = 0.0
        status.current_alert = "no_robot_state"
        status.violation_sources = ["no_robot_state"]
        status.violating_joint_indices = []
        self.safety_publisher.publish(status)

    def _publish_safety_status(self) -> None:
        now = self.get_clock().now()
        if self.latest_state is None:
            self._publish_default_unsafe_status(now)
            return

        status = self.safety_status_type()
        status.header.stamp = now.to_msg()
        status.header.frame_id = self.latest_state.header.frame_id or "base"

        state_stamp = Time.from_msg(self.latest_state.header.stamp)
        if (
            self.latest_state.header.stamp.sec == 0
            and self.latest_state.header.stamp.nanosec == 0
        ):
            if self.latest_state_receipt_time is None:
                state_stamp = now
            else:
                state_stamp = self.latest_state_receipt_time

        age_sec = max(0.0, (now - state_stamp).nanoseconds / 1e9)
        data_stale = age_sec > self.state_timeout_sec or not bool(
            self.latest_state.connected
        )

        quaternion = self.latest_state.imu_quaternion_wxyz
        roll, pitch = quaternion_wxyz_to_roll_pitch(
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        )
        is_upright = abs(pitch) <= self.max_pitch_rad and abs(roll) <= self.max_roll_rad

        violating_joint_indices: list[int] = []
        violation_sources: list[str] = []
        for limit in self.joint_limits:
            position = float(self.latest_state.joint_position[limit.index])
            minimum = limit.min_position - self.joint_limit_margin_rad
            maximum = limit.max_position + self.joint_limit_margin_rad
            if position < minimum:
                violating_joint_indices.append(limit.index)
                violation_sources.append(f"{limit.name}:below_min")
            elif position > maximum:
                violating_joint_indices.append(limit.index)
                violation_sources.append(f"{limit.name}:above_max")

        joints_within_limits = len(violating_joint_indices) == 0
        if data_stale:
            violation_sources.append("stale_robot_state")
        if not is_upright:
            violation_sources.append("robot_not_upright")

        status.is_upright = is_upright
        status.data_stale = data_stale
        status.joints_within_limits = joints_within_limits
        status.pitch = float(pitch)
        status.roll = float(roll)
        status.violation_sources = violation_sources
        status.violating_joint_indices = violating_joint_indices
        status.is_safe = is_upright and joints_within_limits and not data_stale
        status.estop_active = data_stale or not is_upright or not joints_within_limits

        if data_stale:
            status.current_alert = "stale_robot_state"
        elif not is_upright:
            status.current_alert = "robot_not_upright"
        elif not joints_within_limits:
            status.current_alert = _first_joint_alert(violation_sources)
        else:
            status.current_alert = "ok"

        if self.last_status_safe is None or self.last_status_safe != status.is_safe:
            if status.is_safe:
                self.get_logger().info("Safety monitor recovered to SAFE state.")
            else:
                self.get_logger().error(
                    f"Safety monitor marked robot unsafe: alert={status.current_alert} pitch={pitch:.3f} roll={roll:.3f}"
                )
        self.last_status_safe = status.is_safe

        self.safety_publisher.publish(status)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = G1SafetyNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
