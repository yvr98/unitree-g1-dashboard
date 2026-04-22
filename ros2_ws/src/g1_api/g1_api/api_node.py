from __future__ import annotations

from collections.abc import Sequence
from contextlib import suppress
from importlib import import_module
from threading import Event, Lock, Thread
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import uvicorn

from .routes.commands import router as commands_router
from .routes.state import router as state_router


def _load_runtime_types() -> dict[str, Any]:
    g1_msgs_msg_module = import_module("g1_msgs.msg")
    g1_msgs_srv_module = import_module("g1_msgs.srv")
    return {
        "RobotState": g1_msgs_msg_module.RobotState,
        "SafetyStatus": g1_msgs_msg_module.SafetyStatus,
        "LocomotionState": g1_msgs_msg_module.LocomotionState,
        "SetLocomotionMode": g1_msgs_srv_module.SetLocomotionMode,
    }


def _message_to_dict(message: Any) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for slot_name in getattr(message, "__slots__", []):
        field_name = slot_name.lstrip("_")
        value = getattr(message, field_name)
        result[field_name] = _normalize_value(value)
    return result


def _normalize_value(value: Any) -> Any:
    if hasattr(value, "__slots__"):
        return _message_to_dict(value)
    if isinstance(value, (str, bytes, bytearray)) or value is None:
        return value
    if hasattr(value, "tolist"):
        return _normalize_value(value.tolist())
    if isinstance(value, Sequence):
        return [_normalize_value(item) for item in value]
    return value


DEFAULT_CORS_ALLOWED_ORIGINS = [
    "http://localhost:5173",
    "http://127.0.0.1:5173",
]


class G1ApiNode(Node):
    def __init__(self) -> None:
        super().__init__("g1_api_node")

        deps = _load_runtime_types()
        self.robot_state_type = deps["RobotState"]
        self.safety_status_type = deps["SafetyStatus"]
        self.locomotion_state_type = deps["LocomotionState"]
        self.set_locomotion_mode_type = deps["SetLocomotionMode"]

        self.declare_parameter("robot_state_topic", "/robot_state")
        self.declare_parameter("safety_status_topic", "/safety_status")
        self.declare_parameter("locomotion_state_topic", "/locomotion_state")
        self.declare_parameter("set_mode_service", "/set_locomotion_mode")
        self.declare_parameter("api_host", "0.0.0.0")
        self.declare_parameter("api_port", 8000)
        self.declare_parameter("command_timeout_sec", 5.0)
        self.declare_parameter("command_list", ["stand_up", "sit_down", "reset"])
        self.declare_parameter("cors_allowed_origins", DEFAULT_CORS_ALLOWED_ORIGINS)

        self.robot_state_topic = self._string_param("robot_state_topic")
        self.safety_status_topic = self._string_param("safety_status_topic")
        self.locomotion_state_topic = self._string_param("locomotion_state_topic")
        self.set_mode_service_name = self._string_param("set_mode_service")
        self.api_host = self._string_param("api_host")
        self.api_port = self._int_param("api_port")
        self.command_timeout_sec = self._float_param("command_timeout_sec")
        self.command_list = self._string_array_param("command_list")
        self.cors_allowed_origins = self._string_array_param("cors_allowed_origins")

        if self.api_port <= 0:
            raise ValueError("api_port must be > 0.")
        if self.command_timeout_sec <= 0.0:
            raise ValueError("command_timeout_sec must be > 0.")

        self._lock = Lock()
        self._latest_robot_state: Any | None = None
        self._latest_safety_status: Any | None = None
        self._latest_locomotion_state: Any | None = None

        self.create_subscription(
            self.robot_state_type,
            self.robot_state_topic,
            self._handle_robot_state,
            10,
        )
        self.create_subscription(
            self.safety_status_type,
            self.safety_status_topic,
            self._handle_safety_status,
            10,
        )
        self.create_subscription(
            self.locomotion_state_type,
            self.locomotion_state_topic,
            self._handle_locomotion_state,
            10,
        )
        self.set_mode_client = self.create_client(
            self.set_locomotion_mode_type,
            self.set_mode_service_name,
        )

        self.get_logger().info(
            f"API node ready. FastAPI on {self.api_host}:{self.api_port}, set_mode_service={self.set_mode_service_name}."
        )

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

    def _string_array_param(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parameter {name!r} is required.")
        return [str(item) for item in value]

    def _handle_robot_state(self, msg: Any) -> None:
        with self._lock:
            self._latest_robot_state = msg

    def _handle_safety_status(self, msg: Any) -> None:
        with self._lock:
            self._latest_safety_status = msg

    def _handle_locomotion_state(self, msg: Any) -> None:
        with self._lock:
            self._latest_locomotion_state = msg

    def get_skills(self) -> list[dict[str, str]]:
        return [
            {"name": command, "type": "command", "transport": "rest"}
            for command in self.command_list
        ]

    def get_state_snapshot(self) -> dict[str, Any]:
        with self._lock:
            robot_state = self._latest_robot_state
            safety_status = self._latest_safety_status
            locomotion_state = self._latest_locomotion_state

        current_state = None
        if locomotion_state is not None:
            current_state = str(locomotion_state.current_state)

        return {
            "current_state": current_state,
            "robot_state": None
            if robot_state is None
            else _message_to_dict(robot_state),
            "safety_status": None
            if safety_status is None
            else _message_to_dict(safety_status),
            "locomotion_state": None
            if locomotion_state is None
            else _message_to_dict(locomotion_state),
        }

    def send_command(self, command: str) -> dict[str, Any]:
        normalized_command = command.strip().lower()
        if normalized_command not in self.command_list:
            raise HTTPException(
                status_code=400,
                detail={
                    "error": "unsupported_command",
                    "message": f"Unsupported command {command!r}.",
                    "supported_commands": self.command_list,
                },
            )

        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            raise HTTPException(
                status_code=503,
                detail={
                    "error": "service_unavailable",
                    "message": f"Service {self.set_mode_service_name} is unavailable.",
                },
            )

        request = self.set_locomotion_mode_type.Request()
        request.requested_mode = normalized_command

        done_event = Event()
        result_holder: dict[str, Any] = {}

        future = self.set_mode_client.call_async(request)

        def _store_result(response_future: Any) -> None:
            try:
                response = response_future.result()
                result_holder["response"] = {
                    "success": bool(response.success),
                    "accepted": bool(response.accepted),
                    "current_mode": str(response.current_mode),
                    "message": str(response.message),
                }
            except (
                Exception
            ) as exc:  # pragma: no cover - defensive for runtime ROS errors
                result_holder["exception"] = exc
            finally:
                done_event.set()

        future.add_done_callback(_store_result)

        if not done_event.wait(timeout=self.command_timeout_sec):
            raise HTTPException(
                status_code=504,
                detail={
                    "error": "command_timeout",
                    "message": f"Timed out waiting for {normalized_command!r} response.",
                },
            )

        if "exception" in result_holder:
            raise HTTPException(
                status_code=500,
                detail={
                    "error": "command_failed",
                    "message": str(result_holder["exception"]),
                },
            )

        return result_holder["response"]


app = FastAPI(title="G1 Dashboard API")
app.include_router(commands_router)
app.include_router(state_router)


def configure_cors(origins: list[str]) -> None:
    app.user_middleware.clear()
    app.middleware_stack = None
    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=False,
        allow_methods=["*"],
        allow_headers=["*"],
    )


def set_api_node(node: G1ApiNode) -> None:
    app.state.api_node = node


def get_api_node() -> G1ApiNode:
    node = getattr(app.state, "api_node", None)
    if node is None:
        raise HTTPException(status_code=503, detail="API node is not initialized.")
    return node


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = G1ApiNode()
    set_api_node(node)
    configure_cors(node.cors_allowed_origins)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        uvicorn.run(app, host=node.api_host, port=node.api_port, log_level="info")
    finally:
        executor.shutdown()
        node.destroy_node()
        with suppress(RuntimeError):
            executor_thread.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()
