from __future__ import annotations

from importlib import import_module
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_SRC_ROOT = REPO_ROOT / "ros2_ws" / "src" / "g1_locomotion"
if str(PACKAGE_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC_ROOT))


select_command_targets = import_module(
    "g1_locomotion.locomotion_node"
).select_command_targets


class FakePolicyRunner:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def standing_command(self) -> str:
        self.calls.append("standing")
        return "standing-targets"

    def compute_command(self, robot_state: object, cmd_vel: object) -> str:
        assert robot_state is not None
        assert cmd_vel is not None
        self.calls.append("compute")
        return "walking-targets"


def test_standing_state_uses_pd_hold_targets() -> None:
    runner = FakePolicyRunner()

    targets = select_command_targets("STANDING", runner, object(), object())

    assert targets == "standing-targets"
    assert runner.calls == ["standing"]


def test_walking_state_uses_policy_runner() -> None:
    runner = FakePolicyRunner()

    targets = select_command_targets("WALKING", runner, object(), object())

    assert targets == "walking-targets"
    assert runner.calls == ["compute"]
