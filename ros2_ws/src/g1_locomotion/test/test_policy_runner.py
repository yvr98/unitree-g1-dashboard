from pathlib import Path
from importlib import import_module
import sys
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_SRC_ROOT = REPO_ROOT / "ros2_ws" / "src" / "g1_locomotion"
if str(PACKAGE_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC_ROOT))


policy_runner_module = import_module("g1_locomotion.policy_runner")
JOINT_COUNT = policy_runner_module.JOINT_COUNT
PolicyConfig = policy_runner_module.PolicyConfig
PolicyRunner = policy_runner_module.PolicyRunner
clip_command = policy_runner_module.clip_command
controlled_joint_mask = policy_runner_module.controlled_joint_mask
gravity_orientation_from_quaternion_wxyz = (
    policy_runner_module.gravity_orientation_from_quaternion_wxyz
)


def _config(policy_path: str) -> PolicyConfig:
    return PolicyConfig(
        policy_path=policy_path,
        default_pose=[0.0] * JOINT_COUNT,
        leg_joint_kp=[100.0] * 12,
        leg_joint_kd=[2.0] * 12,
        waist_joint_target=[0.0, 0.0, 0.0],
        waist_joint_kp=[300.0, 300.0, 300.0],
        waist_joint_kd=[3.0, 3.0, 3.0],
        arm_joint_target=[0.0] * 14,
        arm_joint_kp=[
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
        arm_joint_kd=[
            2.0,
            2.0,
            2.0,
            2.0,
            1.0,
            1.0,
            1.0,
            2.0,
            2.0,
            2.0,
            2.0,
            1.0,
            1.0,
            1.0,
        ],
        ang_vel_scale=0.25,
        dof_pos_scale=1.0,
        dof_vel_scale=0.05,
        action_scale=0.25,
        cmd_scale=[2.0, 2.0, 0.25],
        max_cmd=[0.8, 0.5, 1.57],
        gait_period_sec=0.8,
        rl_control_dt=0.02,
        fallback_control_dt=0.005,
        fallback_stride_frequency_hz=1.5,
        fallback_leg_lift_rad=0.18,
        fallback_leg_swing_rad=0.22,
        fallback_yaw_swing_rad=0.08,
        fallback_strafe_roll_rad=0.1,
        fallback_kp_scale=0.9,
        controlled_joint_mins=[-0.5] * 15,
        controlled_joint_maxs=[0.5] * 15,
    )


def test_controlled_joint_mask_enables_all_joints() -> None:
    joint_mask = controlled_joint_mask()
    assert len(joint_mask) == JOINT_COUNT
    assert all(joint_mask)


def test_gravity_orientation_matches_identity_quaternion() -> None:
    orientation = gravity_orientation_from_quaternion_wxyz([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(orientation, [0.0, -0.0, -1.0])


def test_clip_command_limits_each_axis_independently() -> None:
    clipped = clip_command(
        np.array([1.0, -2.0, 3.0], dtype=np.float32), [0.8, 0.5, 1.57]
    )
    assert np.allclose(clipped, [0.8, -0.5, 1.57])


def test_missing_policy_uses_pd_fallback() -> None:
    runner = PolicyRunner(_config("/definitely/missing/policy.pt"))
    assert runner.uses_rl_policy is False
    assert runner.backend == "pd_fallback"
    assert "policy_missing" in runner.backend_reason


def test_zero_cmd_uses_rl_policy_when_available() -> None:
    runner = PolicyRunner(_config("/definitely/missing/policy.pt"))
    runner._policy = object()
    runner._torch_module = object()
    called = {"value": False}

    def fake_run_policy(
        robot_state: object, command: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        del robot_state
        called["value"] = True
        assert np.allclose(command, [0.0, 0.0, 0.0])
        return np.zeros(12, dtype=np.float32)

    runner._run_policy = fake_run_policy
    robot_state = type(
        "RobotStateLike",
        (),
        {
            "joint_position": [0.0] * JOINT_COUNT,
            "joint_velocity": [0.0] * JOINT_COUNT,
            "imu_gyroscope": [0.0, 0.0, 0.0],
            "imu_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
    )()
    cmd_vel = type(
        "TwistLike",
        (),
        {
            "linear": type("LinearLike", (), {"x": 0.0, "y": 0.0})(),
            "angular": type("AngularLike", (), {"z": 0.0})(),
        },
    )()

    targets = runner.compute_command(robot_state, cmd_vel)

    assert called["value"] is True
    assert targets.backend == "rl_policy"


def test_observation_phase_matches_unitree_reference_first_tick() -> None:
    runner = PolicyRunner(_config("/definitely/missing/policy.pt"))
    robot_state = type(
        "RobotStateLike",
        (),
        {
            "joint_position": [0.0] * JOINT_COUNT,
            "joint_velocity": [0.0] * JOINT_COUNT,
            "imu_gyroscope": [0.0, 0.0, 0.0],
            "imu_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
    )()

    observation = runner._build_observation(
        robot_state, np.array([0.0, 0.0, 0.0], dtype=np.float32)
    )

    assert observation.shape == (47,)
    assert np.isclose(observation[45], math.sin(2.0 * math.pi * 0.02 / 0.8))
    assert np.isclose(observation[46], math.cos(2.0 * math.pi * 0.02 / 0.8))


def test_reset_clears_policy_history_and_recurrent_state() -> None:
    runner = PolicyRunner(_config("/definitely/missing/policy.pt"))
    runner.previous_action = np.ones(12, dtype=np.float32)
    runner.rl_step_index = 7
    runner.fallback_time_sec = 1.5

    class FakeState:
        def __init__(self, value: float) -> None:
            self.value = value

        def zero_(self) -> None:
            self.value = 0.0

    fake_policy: Any = type(
        "FakePolicy",
        (),
        {
            "hidden_state": FakeState(1.0),
            "cell_state": FakeState(2.0),
        },
    )()
    runner._policy = fake_policy

    runner.reset()

    assert np.allclose(runner.previous_action, np.zeros(12, dtype=np.float32))
    assert runner.rl_step_index == 0
    assert runner.fallback_time_sec == 0.0
    assert fake_policy.hidden_state.value == 0.0
    assert fake_policy.cell_state.value == 0.0


def test_fallback_uses_fallback_control_dt() -> None:
    config = _config("/definitely/missing/policy.pt")
    runner = PolicyRunner(config)
    robot_state = type(
        "RobotStateLike",
        (),
        {
            "joint_position": [0.0] * JOINT_COUNT,
            "joint_velocity": [0.0] * JOINT_COUNT,
            "imu_gyroscope": [0.0, 0.0, 0.0],
            "imu_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
    )()

    runner._run_fallback(robot_state, np.array([0.3, 0.0, 0.0], dtype=np.float32))

    assert runner.fallback_time_sec == config.fallback_control_dt


def test_build_positions_clamps_controlled_joints_to_configured_limits() -> None:
    config = _config("/definitely/missing/policy.pt")
    runner = PolicyRunner(config)

    leg_targets = [1.2] * 12
    positions = runner._build_positions(leg_targets)

    assert positions[:12] == [0.5] * 12
    assert positions[12:15] == [0.0, 0.0, 0.0]
