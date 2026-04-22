from __future__ import annotations

from importlib import import_module
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

_state_machine_module = import_module("g1_orchestrator.state_machine")
MotionPlan = _state_machine_module.MotionPlan
LocomotionStateMachine = _state_machine_module.LocomotionStateMachine
STATE_EMERGENCY_STOP = _state_machine_module.STATE_EMERGENCY_STOP
STATE_ERROR = _state_machine_module.STATE_ERROR
STATE_IDLE = _state_machine_module.STATE_IDLE
STATE_STANDING = _state_machine_module.STATE_STANDING
STATE_WALKING = _state_machine_module.STATE_WALKING


JOINT_COUNT = 29
MODE_PR = 0
DEFAULT_MOTION_KP = [50.0] * JOINT_COUNT
DEFAULT_MOTION_KD = [3.0] * JOINT_COUNT
DEFAULT_ESTOP_KP = [0.0] * JOINT_COUNT
DEFAULT_ESTOP_KD = [10.0] * JOINT_COUNT
CMD_VEL_EPSILON = 1e-3


def _load_runtime_types() -> dict[str, Any]:
    g1_msgs_msg_module = import_module("g1_msgs.msg")
    g1_msgs_srv_module = import_module("g1_msgs.srv")
    return {
        "JointCommand": g1_msgs_msg_module.JointCommand,
        "LocomotionState": g1_msgs_msg_module.LocomotionState,
        "RobotState": g1_msgs_msg_module.RobotState,
        "SafetyStatus": g1_msgs_msg_module.SafetyStatus,
        "SetLocomotionMode": g1_msgs_srv_module.SetLocomotionMode,
    }


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def interpolate_positions(
    start: list[float], target: list[float], progress: float
) -> list[float]:
    alpha = clamp(progress, 0.0, 1.0)
    return [
        (1.0 - alpha) * start_value + alpha * target_value
        for start_value, target_value in zip(start, target)
    ]


def max_position_error(current: list[float], target: list[float]) -> float:
    if len(current) != len(target):
        raise ValueError("current and target must have the same length.")
    return max(
        abs(current_value - target_value)
        for current_value, target_value in zip(current, target)
    )


def max_position_error_for_indices(
    current: list[float], target: list[float], joint_indices: list[int]
) -> float:
    if len(current) != len(target):
        raise ValueError("current and target must have the same length.")
    if not joint_indices:
        raise ValueError("joint_indices must not be empty.")
    return max(
        abs(current[joint_index] - target[joint_index]) for joint_index in joint_indices
    )


def build_joint_command(
    positions: list[float],
    kp: list[float],
    kd: list[float],
    mode_machine: int,
) -> Any:
    deps = _load_runtime_types()
    command = deps["JointCommand"]()
    command.mode_pr = MODE_PR
    command.mode_machine = int(mode_machine)
    command.q = positions
    command.dq = [0.0] * JOINT_COUNT
    command.tau = [0.0] * JOINT_COUNT
    command.kp = kp
    command.kd = kd
    command.joint_mask = [True] * JOINT_COUNT
    return command


def format_joint_debug_snapshot(
    robot_state: Any | None,
    safety_status: Any | None,
    target_positions: list[float] | None,
) -> str:
    if robot_state is None or safety_status is None:
        return "joint debug unavailable"

    violating_indices = [
        int(index) for index in getattr(safety_status, "violating_joint_indices", [])
    ]
    if not violating_indices:
        return "joint debug unavailable"

    details: list[str] = []
    for joint_index in violating_indices:
        current_position = float(robot_state.joint_position[joint_index])
        if target_positions is None:
            details.append(f"j{joint_index}: q={current_position:.3f}")
        else:
            target_position = float(target_positions[joint_index])
            details.append(
                f"j{joint_index}: q={current_position:.3f} target={target_position:.3f} err={current_position - target_position:.3f}"
            )
    return "; ".join(details)


def build_zero_twist() -> Twist:
    return Twist()


def twist_is_nonzero(msg: Twist, epsilon: float = CMD_VEL_EPSILON) -> bool:
    return any(
        abs(value) > epsilon
        for value in [
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        ]
    )


def state_uses_locomotion(state: str) -> bool:
    return state in {STATE_STANDING, STATE_WALKING}


def should_publish_hold_pose(
    state: str, hold_pose: list[float] | None, locomotion_enabled_confirmed: bool
) -> bool:
    return (
        state in {STATE_IDLE, STATE_STANDING}
        and hold_pose is not None
        and not locomotion_enabled_confirmed
    )


def should_ignore_boot_estop(
    current_state: str,
    has_robot_state: bool,
    safety_status: Any,
) -> bool:
    if current_state != STATE_IDLE or has_robot_state:
        return False
    if not bool(getattr(safety_status, "estop_active", False)):
        return False
    violation_sources = list(getattr(safety_status, "violation_sources", []))
    if not violation_sources:
        return False
    allowed_sources = {"no_robot_state", "stale_robot_state"}
    return set(str(source) for source in violation_sources).issubset(allowed_sources)


def should_ignore_passive_idle_estop(
    current_state: str,
    motion_plan: MotionPlan | None,
    locomotion_enabled_confirmed: bool,
    locomotion_request_pending_target: bool | None,
    time_since_start_sec: float,
    safety_status: Any,
) -> bool:
    if (
        current_state != STATE_IDLE
        or motion_plan is not None
        or locomotion_enabled_confirmed
        or locomotion_request_pending_target is True
        or time_since_start_sec > 3.0
        or not bool(getattr(safety_status, "estop_active", False))
    ):
        return False
    violation_sources = list(getattr(safety_status, "violation_sources", []))
    if not violation_sources:
        return False
    allowed_sources = {"stale_robot_state", "robot_not_upright"}
    return set(str(source) for source in violation_sources).issubset(allowed_sources)


class G1OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator_node")

        deps = _load_runtime_types()
        self.joint_command_type = deps["JointCommand"]
        self.locomotion_state_type = deps["LocomotionState"]
        self.robot_state_type = deps["RobotState"]
        self.safety_status_type = deps["SafetyStatus"]
        self.set_locomotion_mode_type = deps["SetLocomotionMode"]

        self.declare_parameter("state_topic", "/robot_state")
        self.declare_parameter("safety_topic", "/safety_status")
        self.declare_parameter("command_topic", "/joint_commands")
        self.declare_parameter("locomotion_state_topic", "/locomotion_state")
        self.declare_parameter("set_mode_service", "/set_locomotion_mode")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("interpolation_duration_sec", 3.0)
        self.declare_parameter("position_tolerance_rad", 0.05)
        self.declare_parameter("transition_settle_timeout_sec", 1.0)
        self.declare_parameter("transition_completion_joint_indices", list(range(15)))
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("locomotion_cmd_vel_topic", "/locomotion/cmd_vel")
        self.declare_parameter("locomotion_enable_service", "/locomotion/enable")
        self.declare_parameter("locomotion_disable_service", "/locomotion/disable")
        self.declare_parameter("walking_idle_timeout_sec", 0.5)
        self.declare_parameter("cmd_vel_timeout_sec", 0.25)
        self.declare_parameter("motion_kp", list(DEFAULT_MOTION_KP))
        self.declare_parameter("motion_kd", list(DEFAULT_MOTION_KD))
        self.declare_parameter("estop_kp", list(DEFAULT_ESTOP_KP))
        self.declare_parameter("estop_kd", list(DEFAULT_ESTOP_KD))
        self.declare_parameter("stand_pose", [0.0] * JOINT_COUNT)
        self.declare_parameter("sit_pose", [0.0] * JOINT_COUNT)

        self.state_topic = self._string_param("state_topic")
        self.safety_topic = self._string_param("safety_topic")
        self.command_topic = self._string_param("command_topic")
        self.locomotion_state_topic = self._string_param("locomotion_state_topic")
        self.set_mode_service_name = self._string_param("set_mode_service")
        self.publish_rate_hz = self._float_param("publish_rate_hz")
        self.interpolation_duration_sec = self._float_param(
            "interpolation_duration_sec"
        )
        self.position_tolerance_rad = self._float_param("position_tolerance_rad")
        self.transition_settle_timeout_sec = self._float_param(
            "transition_settle_timeout_sec"
        )
        self.cmd_vel_topic = self._string_param("cmd_vel_topic")
        self.locomotion_cmd_vel_topic = self._string_param("locomotion_cmd_vel_topic")
        self.locomotion_enable_service_name = self._string_param(
            "locomotion_enable_service"
        )
        self.locomotion_disable_service_name = self._string_param(
            "locomotion_disable_service"
        )
        self.walking_idle_timeout_sec = self._float_param("walking_idle_timeout_sec")
        self.cmd_vel_timeout_sec = self._float_param("cmd_vel_timeout_sec")
        self.transition_completion_joint_indices = self._int_array_param(
            "transition_completion_joint_indices"
        )
        self.motion_kp = self._float_array_param("motion_kp")
        self.motion_kd = self._float_array_param("motion_kd")
        self.estop_kp = self._float_array_param("estop_kp")
        self.estop_kd = self._float_array_param("estop_kd")
        self.stand_pose = self._float_array_param("stand_pose")
        self.sit_pose = self._float_array_param("sit_pose")

        if len(self.stand_pose) != JOINT_COUNT or len(self.sit_pose) != JOINT_COUNT:
            raise ValueError(
                "stand_pose and sit_pose must both contain 29 joint values."
            )
        if any(
            len(values) != JOINT_COUNT
            for values in [self.motion_kp, self.motion_kd, self.estop_kp, self.estop_kd]
        ):
            raise ValueError(
                "motion_kp, motion_kd, estop_kp, and estop_kd must each contain 29 values."
            )
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0.")
        if self.interpolation_duration_sec <= 0.0:
            raise ValueError("interpolation_duration_sec must be > 0.")
        if self.transition_settle_timeout_sec < 0.0:
            raise ValueError("transition_settle_timeout_sec must be >= 0.")
        if not self.transition_completion_joint_indices:
            raise ValueError("transition_completion_joint_indices must not be empty.")
        if any(
            joint_index < 0 or joint_index >= JOINT_COUNT
            for joint_index in self.transition_completion_joint_indices
        ):
            raise ValueError(
                "transition_completion_joint_indices must contain valid joint indices."
            )

        self.fsm = LocomotionStateMachine()
        self.node_start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.motion_plan: MotionPlan | None = None
        self.transition_progress = 0.0
        self.latest_robot_state: Any | None = None
        self.latest_safety_status: Any | None = None
        self.latest_cmd_vel = build_zero_twist()
        self.latest_cmd_vel_receipt_time_sec: float | None = None
        self.last_nonzero_cmd_time_sec: float | None = None
        self.hold_pose: list[float] | None = None
        self.last_rejected_reason = "boot"
        self.locomotion_enable_future: Any | None = None
        self.locomotion_enable_requested: bool = False
        self.locomotion_enabled_confirmed: bool = False
        self.locomotion_request_pending_target: bool | None = None
        self.pending_sit_down_after_disable: bool = False

        self.command_publisher = self.create_publisher(
            self.joint_command_type, self.command_topic, 10
        )
        self.locomotion_state_publisher = self.create_publisher(
            self.locomotion_state_type, self.locomotion_state_topic, 10
        )
        self.robot_state_subscription = self.create_subscription(
            self.robot_state_type, self.state_topic, self._handle_robot_state, 10
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self._handle_cmd_vel, 10
        )
        self.safety_status_subscription = self.create_subscription(
            self.safety_status_type, self.safety_topic, self._handle_safety_status, 10
        )
        self.gated_cmd_vel_publisher = self.create_publisher(
            Twist, self.locomotion_cmd_vel_topic, 10
        )
        self.set_mode_service = self.create_service(
            self.set_locomotion_mode_type,
            self.set_mode_service_name,
            self._handle_set_locomotion_mode,
        )
        self.locomotion_enable_client = self.create_client(
            Trigger, self.locomotion_enable_service_name
        )
        self.locomotion_disable_client = self.create_client(
            Trigger, self.locomotion_disable_service_name
        )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

        self.get_logger().info(
            f"Orchestrator publishing commands to {self.command_topic} and state to {self.locomotion_state_topic}."
        )

    @property
    def current_state(self) -> str:
        return self.fsm.state

    def _safety_allows_motion(self) -> bool:
        return self.latest_safety_status is not None and bool(
            self.latest_safety_status.is_safe
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

    def _float_array_param(self, name: str) -> list[float]:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return [float(item) for item in value]

    def _int_array_param(self, name: str) -> list[int]:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return [int(item) for item in value]

    def _current_robot_positions(self) -> list[float]:
        if self.latest_robot_state is None:
            raise RuntimeError("No /robot_state received yet.")
        return [float(value) for value in self.latest_robot_state.joint_position]

    def _handle_robot_state(self, msg: Any) -> None:
        self.latest_robot_state = msg

    def _handle_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd_vel = msg
        self.latest_cmd_vel_receipt_time_sec = self.get_clock().now().nanoseconds / 1e9
        if twist_is_nonzero(msg):
            self.last_nonzero_cmd_time_sec = self.latest_cmd_vel_receipt_time_sec

    def _handle_safety_status(self, msg: Any) -> None:
        self.latest_safety_status = msg
        if should_ignore_boot_estop(
            self.current_state, self.latest_robot_state is not None, msg
        ):
            return
        if should_ignore_passive_idle_estop(
            self.current_state,
            self.motion_plan,
            self.locomotion_enabled_confirmed,
            self.locomotion_request_pending_target,
            (self.get_clock().now().nanoseconds / 1e9) - self.node_start_time_sec,
            msg,
        ):
            return
        if bool(msg.estop_active) and self.current_state != STATE_EMERGENCY_STOP:
            target_positions: list[float] | None = None
            if self.motion_plan is not None:
                target_positions = self.motion_plan.target_positions
            elif self.hold_pose is not None:
                target_positions = self.hold_pose
            self.get_logger().error(
                "Safety estop snapshot: "
                + format_joint_debug_snapshot(
                    self.latest_robot_state, msg, target_positions
                )
            )
            self._enter_emergency_stop("safety_status estop_active")

    def _reject(self, response: Any, message: str) -> Any:
        response.success = False
        response.accepted = False
        response.current_mode = self.current_state
        response.message = message
        self.last_rejected_reason = message
        return response

    def _accept(self, response: Any, message: str) -> Any:
        response.success = True
        response.accepted = True
        response.current_mode = self.current_state
        response.message = message
        return response

    def _handle_set_locomotion_mode(self, request: Any, response: Any) -> Any:
        requested_mode = request.requested_mode.strip().lower()

        if requested_mode == "reset":
            if self.current_state not in {STATE_EMERGENCY_STOP, STATE_ERROR}:
                return self._reject(
                    response, "Robot is not in EMERGENCY_STOP or ERROR."
                )
            if not self._safety_allows_motion():
                return self._reject(
                    response, "Cannot reset until /safety_status reports is_safe=true."
                )
            self.motion_plan = None
            self.transition_progress = 0.0
            self.hold_pose = self._current_robot_positions()
            self.pending_sit_down_after_disable = False
            self._request_locomotion_enabled(False, "reset")
            self.fsm.transition_to(STATE_IDLE, "reset")
            return self._accept(response, "Reset to IDLE and holding current pose.")

        if self.current_state == STATE_EMERGENCY_STOP:
            return self._reject(
                response, "Cannot accept commands while in EMERGENCY_STOP."
            )

        if self.motion_plan is not None:
            return self._reject(
                response,
                "Cannot accept a new command while a transition is already in progress.",
            )

        if requested_mode == "stand_up":
            if not self._safety_allows_motion():
                return self._reject(
                    response,
                    "Cannot stand_up until /safety_status reports is_safe=true.",
                )
            if self.current_state != STATE_IDLE:
                return self._reject(
                    response, f"Cannot stand_up from {self.current_state}."
                )
            if self.latest_robot_state is None:
                return self._reject(response, "No /robot_state received yet.")
            self._start_motion_plan("stand_up", STATE_STANDING, self.stand_pose)
            return self._accept(response, "Starting stand_up transition.")

        if requested_mode == "sit_down":
            if self.current_state == STATE_IDLE:
                return self._reject(
                    response, "Cannot sit_down from IDLE — already idle."
                )
            if self.current_state != STATE_STANDING:
                return self._reject(
                    response, f"Cannot sit_down from {self.current_state}."
                )
            if self.latest_robot_state is None:
                return self._reject(response, "No /robot_state received yet.")
            if (
                self.locomotion_enabled_confirmed
                or self.locomotion_request_pending_target
            ):
                self.pending_sit_down_after_disable = True
                self._request_locomotion_enabled(False, "sit_down")
                return self._accept(
                    response, "Disabling locomotion and starting sit_down transition."
                )
            self._start_motion_plan("sit_down", STATE_IDLE, self.sit_pose)
            return self._accept(response, "Starting sit_down transition.")

        return self._reject(
            response,
            f"Unsupported mode {request.requested_mode!r}. Expected stand_up, sit_down, or reset.",
        )

    def _start_motion_plan(
        self, request_name: str, target_state: str, target_positions: list[float]
    ) -> None:
        start_positions = self._current_robot_positions()
        source_state = self.current_state
        self.motion_plan = MotionPlan(
            request_name=request_name,
            source_state=source_state,
            target_state=target_state,
            start_positions=start_positions,
            target_positions=list(target_positions),
            start_time_sec=self.get_clock().now().nanoseconds / 1e9,
        )
        self.transition_progress = 0.0
        self.fsm.update_reason(request_name)
        self.get_logger().info(
            f"Starting {request_name} transition from {source_state} to {target_state}."
        )

    def _enter_emergency_stop(self, reason: str) -> None:
        self.motion_plan = None
        self.transition_progress = 0.0
        self.hold_pose = None
        self.pending_sit_down_after_disable = False
        self._request_locomotion_enabled(False, reason)
        self.fsm.transition_to(STATE_EMERGENCY_STOP, reason)
        self.get_logger().error(f"Entering EMERGENCY_STOP because {reason}.")

    def _enter_error_state(self, reason: str) -> None:
        self.motion_plan = None
        self.transition_progress = 0.0
        self.hold_pose = None
        self.pending_sit_down_after_disable = False
        self._request_locomotion_enabled(False, reason)
        self.fsm.transition_to(STATE_ERROR, reason)
        self.get_logger().error(f"Entering ERROR because {reason}.")

    def _publish_gated_cmd_vel(self) -> None:
        if state_uses_locomotion(self.current_state):
            self.gated_cmd_vel_publisher.publish(self._effective_cmd_vel())
            return
        self.gated_cmd_vel_publisher.publish(build_zero_twist())

    def _effective_cmd_vel(self) -> Twist:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.latest_cmd_vel_receipt_time_sec is None:
            return build_zero_twist()
        if now_sec - self.latest_cmd_vel_receipt_time_sec > self.cmd_vel_timeout_sec:
            return build_zero_twist()
        return self.latest_cmd_vel

    def _request_locomotion_enabled(self, enabled: bool, reason: str) -> None:
        if self.locomotion_enabled_confirmed == enabled:
            return
        if self.locomotion_request_pending_target == enabled:
            if (
                self.locomotion_enable_future is not None
                and not self.locomotion_enable_future.done()
            ):
                return

        client = (
            self.locomotion_enable_client if enabled else self.locomotion_disable_client
        )
        if not client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warning(
                f"Locomotion {'enable' if enabled else 'disable'} service unavailable while handling {reason}."
            )
            return

        self.locomotion_enable_requested = enabled
        self.locomotion_request_pending_target = enabled
        self.locomotion_enable_future = client.call_async(Trigger.Request())
        self.locomotion_enable_future.add_done_callback(
            lambda future,
            target=enabled,
            why=reason: self._handle_locomotion_toggle_response(future, target, why)
        )

    def _handle_locomotion_toggle_response(
        self, future: Any, enabled: bool, reason: str
    ) -> None:
        self.locomotion_request_pending_target = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"Locomotion {'enable' if enabled else 'disable'} failed for {reason}: {exc}"
            )
            return

        if response.success:
            self.locomotion_enabled_confirmed = enabled
            if enabled and state_uses_locomotion(self.current_state):
                self.hold_pose = None
            if (
                not enabled
                and self.pending_sit_down_after_disable
                and self.current_state == STATE_STANDING
                and self.motion_plan is None
                and self.latest_robot_state is not None
            ):
                self.pending_sit_down_after_disable = False
                self._start_motion_plan("sit_down", STATE_IDLE, self.sit_pose)
        level = (
            self.get_logger().info if response.success else self.get_logger().warning
        )
        level(
            f"Locomotion {'enabled' if enabled else 'disabled'} for {reason}: {response.message}"
        )

    def _update_walking_state(self) -> None:
        if self.motion_plan is not None:
            return
        if not state_uses_locomotion(self.current_state):
            return
        if not self._safety_allows_motion():
            return

        if not self.locomotion_enabled_confirmed:
            self._request_locomotion_enabled(
                True, f"{self.current_state.lower()}_balance"
            )
            if self.current_state == STATE_STANDING:
                self.hold_pose = list(self.stand_pose)
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        command_is_nonzero = twist_is_nonzero(self._effective_cmd_vel())

        if command_is_nonzero:
            self.last_nonzero_cmd_time_sec = now_sec
            if self.current_state == STATE_STANDING:
                self.fsm.transition_to(STATE_WALKING, "cmd_vel_nonzero")
            return

        if self.current_state != STATE_WALKING:
            return

        last_nonzero = self.last_nonzero_cmd_time_sec
        if last_nonzero is None:
            last_nonzero = now_sec
        if now_sec - last_nonzero < self.walking_idle_timeout_sec:
            return

        self.fsm.transition_to(STATE_STANDING, "cmd_vel_idle_timeout")
        self.hold_pose = None

    def _publish_locomotion_state(self) -> None:
        message = self.locomotion_state_type()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base"
        message.state = self.current_state
        message.previous_state = self.fsm.previous_state
        message.transition_reason = self.fsm.transition_reason
        message.current_state = self.current_state
        if self.motion_plan is None:
            message.requested_state = self.current_state
        else:
            message.requested_state = self.motion_plan.target_state
        message.is_transitioning = self.motion_plan is not None
        message.estop_active = self.current_state == STATE_EMERGENCY_STOP
        message.transition_progress = float(self.transition_progress)
        self.locomotion_state_publisher.publish(message)

    def _tick(self) -> None:
        mode_machine = 0
        if self.latest_robot_state is not None:
            mode_machine = int(self.latest_robot_state.mode_machine)

        self._update_walking_state()

        if self.current_state == STATE_EMERGENCY_STOP:
            if self.latest_robot_state is not None:
                self.command_publisher.publish(
                    build_joint_command(
                        self._current_robot_positions(),
                        self.estop_kp,
                        self.estop_kd,
                        mode_machine,
                    )
                )
            self._publish_gated_cmd_vel()
            self._publish_locomotion_state()
            return

        if self.motion_plan is not None:
            elapsed_sec = (
                self.get_clock().now().nanoseconds / 1e9
            ) - self.motion_plan.start_time_sec
            self.transition_progress = clamp(
                elapsed_sec / self.interpolation_duration_sec, 0.0, 1.0
            )
            commanded_positions = interpolate_positions(
                self.motion_plan.start_positions,
                self.motion_plan.target_positions,
                self.transition_progress,
            )
            self.command_publisher.publish(
                build_joint_command(
                    commanded_positions, self.motion_kp, self.motion_kd, mode_machine
                )
            )

            if self.latest_robot_state is not None and self.transition_progress >= 1.0:
                current_positions = self._current_robot_positions()
                if (
                    max_position_error_for_indices(
                        current_positions,
                        self.motion_plan.target_positions,
                        self.transition_completion_joint_indices,
                    )
                    <= self.position_tolerance_rad
                ):
                    finished_target_state = self.motion_plan.target_state
                    finished_request_name = self.motion_plan.request_name
                    self.motion_plan = None
                    self.transition_progress = 1.0
                    self.fsm.transition_to(finished_target_state, finished_request_name)
                    if finished_target_state == STATE_STANDING:
                        self.hold_pose = list(self.stand_pose)
                        self._request_locomotion_enabled(True, "stand_up_complete")
                    elif finished_target_state == STATE_IDLE:
                        self.hold_pose = None
                        self.pending_sit_down_after_disable = False
                    self.get_logger().info(
                        f"Transition completed. Current state is {self.current_state}."
                    )
                elif (
                    elapsed_sec
                    >= self.interpolation_duration_sec
                    + self.transition_settle_timeout_sec
                ):
                    failed_request_name = self.motion_plan.request_name
                    self._enter_error_state(
                        f"{failed_request_name}_target_not_reached_within_tolerance"
                    )
        elif should_publish_hold_pose(
            self.current_state, self.hold_pose, self.locomotion_enabled_confirmed
        ):
            if self.hold_pose is None:
                raise RuntimeError(
                    "hold_pose must be set before publishing standing hold."
                )
            self.command_publisher.publish(
                build_joint_command(
                    self.hold_pose, self.motion_kp, self.motion_kd, mode_machine
                )
            )
            self.transition_progress = 0.0
        elif self.current_state == STATE_WALKING:
            self.transition_progress = 0.0
        elif self.current_state == STATE_ERROR:
            self.transition_progress = 0.0
        else:
            self.transition_progress = 0.0

        self._publish_gated_cmd_vel()
        self._publish_locomotion_state()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = G1OrchestratorNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
