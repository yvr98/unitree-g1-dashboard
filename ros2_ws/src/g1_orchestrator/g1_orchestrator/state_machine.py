from __future__ import annotations

from dataclasses import dataclass


STATE_IDLE = "IDLE"
STATE_STANDING = "STANDING"
STATE_WALKING = "WALKING"
STATE_EMERGENCY_STOP = "EMERGENCY_STOP"
STATE_ERROR = "ERROR"


@dataclass
class MotionPlan:
    request_name: str
    source_state: str
    target_state: str
    start_positions: list[float]
    target_positions: list[float]
    start_time_sec: float


class LocomotionStateMachine:
    def __init__(self) -> None:
        self.state: str
        self.previous_state: str
        self.transition_reason: str
        self.state = STATE_IDLE
        self.previous_state = ""
        self.transition_reason = "boot"

    def transition_to(self, next_state: str, reason: str) -> None:
        if next_state != self.state:
            self.previous_state = self.state
            self.state = next_state
        self.transition_reason = reason

    def update_reason(self, reason: str) -> None:
        self.transition_reason = reason
