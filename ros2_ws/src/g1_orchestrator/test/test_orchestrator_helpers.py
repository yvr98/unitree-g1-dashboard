from importlib import import_module
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_SRC_ROOT = REPO_ROOT / "ros2_ws" / "src" / "g1_orchestrator"
if str(PACKAGE_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC_ROOT))


def test_interpolate_positions_midpoint() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    interpolate_positions = orchestrator_node_module.interpolate_positions

    assert interpolate_positions([0.0, 2.0], [2.0, 4.0], 0.5) == [1.0, 3.0]


def test_max_position_error_reports_largest_delta() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    max_position_error = orchestrator_node_module.max_position_error

    assert max_position_error([0.0, 1.0, -1.0], [0.1, 1.5, -1.2]) == 0.5


def test_max_position_error_for_indices_only_checks_selected_joints() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")

    assert (
        orchestrator_node_module.max_position_error_for_indices(
            [0.0, 1.0, -1.0], [0.8, 1.5, -1.2], [1, 2]
        )
        == 0.5
    )


def test_build_joint_command_enables_all_joints() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    build_joint_command = orchestrator_node_module.build_joint_command

    class FakeJointCommand:
        def __init__(self) -> None:
            self.mode_pr = 0
            self.mode_machine = 0
            self.q = []
            self.dq = []
            self.tau = []
            self.kp = []
            self.kd = []
            self.joint_mask = []

    setattr(
        orchestrator_node_module,
        "_load_runtime_types",
        lambda: {
            "JointCommand": FakeJointCommand,
        },
    )

    command = build_joint_command([0.0] * 29, [1.0] * 29, [2.0] * 29, 3)
    assert command.mode_pr == 0
    assert command.mode_machine == 3
    assert len(command.joint_mask) == 29
    assert all(command.joint_mask)


def test_motion_requires_affirmative_safe_status() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")

    node = object.__new__(orchestrator_node_module.G1OrchestratorNode)
    node.latest_safety_status = None
    assert node._safety_allows_motion() is False

    node.latest_safety_status = type("SafetyStatusLike", (), {"is_safe": True})()
    assert node._safety_allows_motion() is True


def test_state_machine_tracks_previous_state_and_reason() -> None:
    state_machine_module = import_module("g1_orchestrator.state_machine")
    machine = state_machine_module.LocomotionStateMachine()

    machine.transition_to(state_machine_module.STATE_STANDING, "stand_up")

    assert machine.state == state_machine_module.STATE_STANDING
    assert machine.previous_state == state_machine_module.STATE_IDLE
    assert machine.transition_reason == "stand_up"


def test_twist_is_nonzero_checks_forward_strafe_and_yaw() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    twist = orchestrator_node_module.build_zero_twist()
    assert orchestrator_node_module.twist_is_nonzero(twist) is False

    twist.angular.z = 0.2
    assert orchestrator_node_module.twist_is_nonzero(twist) is True


def test_state_uses_locomotion_for_standing_and_walking_only() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")

    assert orchestrator_node_module.state_uses_locomotion("STANDING") is True
    assert orchestrator_node_module.state_uses_locomotion("WALKING") is True
    assert orchestrator_node_module.state_uses_locomotion("IDLE") is False
    assert orchestrator_node_module.state_uses_locomotion("EMERGENCY_STOP") is False


def test_should_publish_hold_pose_only_before_locomotion_takes_over() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")

    assert (
        orchestrator_node_module.should_publish_hold_pose("STANDING", [0.0] * 29, False)
        is True
    )
    assert (
        orchestrator_node_module.should_publish_hold_pose("STANDING", [0.0] * 29, True)
        is False
    )
    assert (
        orchestrator_node_module.should_publish_hold_pose("WALKING", [0.0] * 29, False)
        is False
    )
    assert (
        orchestrator_node_module.should_publish_hold_pose("IDLE", [0.0] * 29, False)
        is True
    )


def test_should_ignore_boot_estop_only_for_pre_state_transient() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    safety_status = type(
        "SafetyStatusLike",
        (),
        {"estop_active": True, "violation_sources": ["no_robot_state"]},
    )()

    assert (
        orchestrator_node_module.should_ignore_boot_estop("IDLE", False, safety_status)
        is True
    )
    assert (
        orchestrator_node_module.should_ignore_boot_estop("IDLE", True, safety_status)
        is False
    )
    assert (
        orchestrator_node_module.should_ignore_boot_estop(
            "STANDING", False, safety_status
        )
        is False
    )

    unsafe_joint_status = type(
        "SafetyStatusLike",
        (),
        {"estop_active": True, "violation_sources": ["left_knee_joint:below_min"]},
    )()
    assert (
        orchestrator_node_module.should_ignore_boot_estop(
            "IDLE", False, unsafe_joint_status
        )
        is False
    )


def test_should_ignore_passive_idle_estop_only_while_idle_and_motionless() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    startup_safety_status = type(
        "SafetyStatusLike",
        (),
        {
            "estop_active": True,
            "violation_sources": ["robot_not_upright"],
        },
    )()
    unsafe_joint_status = type(
        "SafetyStatusLike",
        (),
        {
            "estop_active": True,
            "violation_sources": ["left_knee_joint:below_min"],
        },
    )()

    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "IDLE", None, False, None, 0.5, startup_safety_status
        )
        is True
    )
    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "STANDING", None, False, None, 0.5, startup_safety_status
        )
        is False
    )
    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "IDLE", object(), False, None, 0.5, startup_safety_status
        )
        is False
    )
    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "IDLE", None, True, None, 0.5, startup_safety_status
        )
        is False
    )
    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "IDLE", None, False, True, 0.5, startup_safety_status
        )
        is False
    )
    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "IDLE", None, False, None, 4.0, startup_safety_status
        )
        is False
    )
    assert (
        orchestrator_node_module.should_ignore_passive_idle_estop(
            "IDLE", None, False, None, 0.5, unsafe_joint_status
        )
        is False
    )


def test_reset_is_allowed_from_error_state() -> None:
    orchestrator_node_module = import_module("g1_orchestrator.orchestrator_node")
    node = object.__new__(orchestrator_node_module.G1OrchestratorNode)
    node.fsm = type(
        "FsmLike",
        (),
        {
            "state": "ERROR",
            "previous_state": "STANDING",
            "transition_reason": "stand_up_target_not_reached_within_tolerance",
            "transition_to": lambda self, state, reason: None,
        },
    )()
    node.motion_plan = None
    node.transition_progress = 0.0
    node.hold_pose = None
    node.pending_sit_down_after_disable = False
    node.latest_safety_status = type("SafetyStatusLike", (), {"is_safe": True})()
    node._current_robot_positions = lambda: [0.1] * 29
    node._request_locomotion_enabled = lambda enabled, reason: None
    node._accept = lambda response, message: (
        setattr(response, "success", True),
        setattr(response, "accepted", True),
        setattr(response, "current_mode", node.current_state),
        setattr(response, "message", message),
        response,
    )[-1]
    node._reject = lambda response, message: (
        setattr(response, "success", False),
        setattr(response, "accepted", False),
        setattr(response, "current_mode", node.current_state),
        setattr(response, "message", message),
        response,
    )[-1]

    request = type("RequestLike", (), {"requested_mode": "reset"})()
    response = type("ResponseLike", (), {})()

    result = orchestrator_node_module.G1OrchestratorNode._handle_set_locomotion_mode(
        node, request, response
    )

    assert result.success is True
    assert result.accepted is True
    assert node.hold_pose == [0.1] * 29
    assert result.message == "Reset to IDLE and holding current pose."
