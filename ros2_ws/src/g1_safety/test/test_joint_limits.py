from importlib import import_module
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_SRC_ROOT = REPO_ROOT / "ros2_ws" / "src" / "g1_safety"
if str(PACKAGE_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC_ROOT))

SAFETY_CONFIG_PATH = (
    REPO_ROOT / "ros2_ws" / "src" / "g1_safety" / "config" / "safety_params.yaml"
)


def test_joint_limits_load_from_yaml_in_expected_joint_order() -> None:
    joint_limits_module = import_module("g1_safety.joint_limits")
    load_joint_limits_from_yaml = joint_limits_module.load_joint_limits_from_yaml
    joint_name_order = joint_limits_module.JOINT_NAME_ORDER

    limits = load_joint_limits_from_yaml(SAFETY_CONFIG_PATH)
    assert len(limits) == 29
    assert [limit.name for limit in limits] == joint_name_order
    assert limits[0].min_position == -2.5307
    assert limits[0].max_position == 2.8798
    assert limits[-1].min_position == -1.61443
    assert limits[-1].max_position == 1.61443


def test_quaternion_conversion_identity_is_zero_roll_pitch() -> None:
    safety_node_module = import_module("g1_safety.safety_monitor_node")
    quaternion_wxyz_to_roll_pitch = safety_node_module.quaternion_wxyz_to_roll_pitch

    roll, pitch = quaternion_wxyz_to_roll_pitch(1.0, 0.0, 0.0, 0.0)
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9


def test_quaternion_conversion_clamps_pitch_input() -> None:
    safety_node_module = import_module("g1_safety.safety_monitor_node")
    quaternion_wxyz_to_roll_pitch = safety_node_module.quaternion_wxyz_to_roll_pitch

    _roll, pitch = quaternion_wxyz_to_roll_pitch(1.0, 0.0, 1.0, 0.0)
    assert pitch > 1.5
