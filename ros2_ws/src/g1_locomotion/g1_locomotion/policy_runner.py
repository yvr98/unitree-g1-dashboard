from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray


JOINT_COUNT = 29
LEG_JOINT_INDICES = list(range(12))
WAIST_JOINT_INDICES = [12, 13, 14]
CONTROLLED_JOINT_INDICES = list(range(15))
ARM_JOINT_INDICES = list(range(15, JOINT_COUNT))
ZERO_COMMAND_EPSILON = 1e-3


@dataclass(frozen=True)
class PolicyConfig:
    policy_path: str
    default_pose: list[float]
    leg_joint_kp: list[float]
    leg_joint_kd: list[float]
    waist_joint_target: list[float]
    waist_joint_kp: list[float]
    waist_joint_kd: list[float]
    arm_joint_target: list[float]
    arm_joint_kp: list[float]
    arm_joint_kd: list[float]
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float
    cmd_scale: list[float]
    max_cmd: list[float]
    gait_period_sec: float
    rl_control_dt: float
    fallback_control_dt: float
    fallback_stride_frequency_hz: float
    fallback_leg_lift_rad: float
    fallback_leg_swing_rad: float
    fallback_yaw_swing_rad: float
    fallback_strafe_roll_rad: float
    fallback_kp_scale: float
    controlled_joint_mins: list[float]
    controlled_joint_maxs: list[float]


@dataclass(frozen=True)
class CommandTargets:
    positions: list[float]
    kp: list[float]
    kd: list[float]
    joint_mask: list[bool]
    backend: str


def controlled_joint_mask() -> list[bool]:
    return [True] * JOINT_COUNT


def commanded_velocity_triplet(cmd_vel: Any) -> NDArray[np.float32]:
    return np.array(
        [
            float(cmd_vel.linear.x),
            float(cmd_vel.linear.y),
            float(cmd_vel.angular.z),
        ],
        dtype=np.float32,
    )


def is_zero_command(
    command: NDArray[np.float32], epsilon: float = ZERO_COMMAND_EPSILON
) -> bool:
    return bool(np.linalg.norm(command, ord=np.inf) <= epsilon)


def clip_command(
    command: NDArray[np.float32], max_cmd: list[float]
) -> NDArray[np.float32]:
    limits = np.asarray(max_cmd, dtype=np.float32)
    return np.clip(command, -limits, limits)


def gravity_orientation_from_quaternion_wxyz(
    quaternion_wxyz: list[float],
) -> NDArray[np.float32]:
    qw, qx, qy, qz = [float(value) for value in quaternion_wxyz]
    gravity_orientation = np.zeros(3, dtype=np.float32)
    gravity_orientation[0] = 2.0 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2.0 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return gravity_orientation


class PolicyRunner:
    def __init__(self, config: PolicyConfig) -> None:
        self.config = config
        self.previous_action = np.zeros(len(LEG_JOINT_INDICES), dtype=np.float32)
        self.rl_step_index = 0
        self.fallback_time_sec = 0.0
        self.backend = "pd_fallback"
        self.backend_reason = "policy_not_loaded"
        self._policy: Any | None = None
        self._torch_module: Any | None = None
        self._try_load_policy()

    def reset(self) -> None:
        self.previous_action.fill(0.0)
        self.rl_step_index = 0
        self.fallback_time_sec = 0.0
        if self._policy is None:
            return

        for state_name in ("hidden_state", "cell_state"):
            state = getattr(self._policy, state_name, None)
            if state is None:
                continue
            try:
                state.zero_()
            except Exception:
                continue

    @property
    def uses_rl_policy(self) -> bool:
        return self._policy is not None

    def standing_command(self) -> CommandTargets:
        return CommandTargets(
            positions=self._build_positions(list(self.config.default_pose[:12])),
            kp=self._kp_vector(1.0),
            kd=self._kd_vector(),
            joint_mask=controlled_joint_mask(),
            backend="standing_hold",
        )

    def compute_command(self, robot_state: Any, cmd_vel: Any) -> CommandTargets:
        command = clip_command(commanded_velocity_triplet(cmd_vel), self.config.max_cmd)
        if self.uses_rl_policy:
            leg_targets = self._run_policy(robot_state, command)
            return CommandTargets(
                positions=self._build_positions(leg_targets.tolist()),
                kp=self._kp_vector(1.0),
                kd=self._kd_vector(),
                joint_mask=controlled_joint_mask(),
                backend="rl_policy",
            )

        if is_zero_command(command):
            return self.standing_command()

        leg_targets = self._run_fallback(robot_state, command)
        return CommandTargets(
            positions=self._build_positions(leg_targets.tolist()),
            kp=self._kp_vector(self.config.fallback_kp_scale),
            kd=self._kd_vector(),
            joint_mask=controlled_joint_mask(),
            backend="pd_fallback",
        )

    def _try_load_policy(self) -> None:
        policy_path = Path(self.config.policy_path)
        if not policy_path.exists():
            self.backend_reason = f"policy_missing:{policy_path}"
            return

        try:
            import torch
        except ImportError as exc:
            self.backend_reason = f"torch_import_failed:{exc}"
            return

        try:
            self._torch_module = torch
            policy = torch.jit.load(str(policy_path), map_location="cpu")
            policy.eval()
            self._policy = policy
            self.backend = "rl_policy"
            self.backend_reason = str(policy_path)
        except Exception as exc:
            self._policy = None
            self._torch_module = None
            self.backend_reason = f"policy_load_failed:{exc}"

    def _run_policy(
        self, robot_state: Any, command: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        if self._policy is None or self._torch_module is None:
            raise RuntimeError("Policy requested before loading successfully.")

        observation = self._build_observation(robot_state, command)
        obs_tensor = self._torch_module.from_numpy(observation).unsqueeze(0)
        action = (
            self._policy(obs_tensor).detach().cpu().numpy().squeeze().astype(np.float32)
        )
        self.previous_action = action.copy()
        self.rl_step_index += 1
        default_leg_pose = np.asarray(self.config.default_pose[:12], dtype=np.float32)
        return default_leg_pose + action * float(self.config.action_scale)

    def _build_observation(
        self, robot_state: Any, command: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        clipped_command = clip_command(command, self.config.max_cmd)
        joint_position = np.asarray(robot_state.joint_position[:12], dtype=np.float32)
        joint_velocity = np.asarray(robot_state.joint_velocity[:12], dtype=np.float32)
        default_leg_pose = np.asarray(self.config.default_pose[:12], dtype=np.float32)
        ang_vel = np.asarray(robot_state.imu_gyroscope, dtype=np.float32) * float(
            self.config.ang_vel_scale
        )
        gravity_orientation = gravity_orientation_from_quaternion_wxyz(
            list(robot_state.imu_quaternion_wxyz)
        )
        qj_obs = (joint_position - default_leg_pose) * float(self.config.dof_pos_scale)
        dqj_obs = joint_velocity * float(self.config.dof_vel_scale)

        period = max(float(self.config.gait_period_sec), 1e-6)
        phase = (
            ((self.rl_step_index + 1) * float(self.config.rl_control_dt))
            % period
            / period
        )
        sin_phase = math.sin(2.0 * math.pi * phase)
        cos_phase = math.cos(2.0 * math.pi * phase)

        observation = np.zeros(47, dtype=np.float32)
        observation[:3] = ang_vel
        observation[3:6] = gravity_orientation
        observation[6:9] = clipped_command * np.asarray(
            self.config.cmd_scale, dtype=np.float32
        )
        observation[9:21] = qj_obs
        observation[21:33] = dqj_obs
        observation[33:45] = self.previous_action
        observation[45] = sin_phase
        observation[46] = cos_phase
        return observation

    def _run_fallback(
        self, robot_state: Any, command: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        del robot_state
        self.fallback_time_sec += float(self.config.fallback_control_dt)
        phase = (
            2.0
            * math.pi
            * float(self.config.fallback_stride_frequency_hz)
            * self.fallback_time_sec
        )
        left_phase = math.sin(phase)
        right_phase = math.sin(phase + math.pi)

        forward = float(command[0] / max(self.config.max_cmd[0], 1e-6))
        strafe = float(command[1] / max(self.config.max_cmd[1], 1e-6))
        yaw = float(command[2] / max(self.config.max_cmd[2], 1e-6))

        swing_amplitude = float(self.config.fallback_leg_swing_rad) * max(
            abs(forward), 0.25
        )
        lift_amplitude = float(self.config.fallback_leg_lift_rad) * max(
            abs(forward), abs(yaw), 0.25
        )
        yaw_amplitude = float(self.config.fallback_yaw_swing_rad) * yaw
        strafe_amplitude = float(self.config.fallback_strafe_roll_rad) * strafe

        leg_targets = np.asarray(self.config.default_pose[:12], dtype=np.float32).copy()

        def apply_leg(
            hip_yaw_index: int,
            hip_roll_index: int,
            hip_pitch_index: int,
            knee_index: int,
            ankle_pitch_index: int,
            ankle_roll_index: int,
            leg_phase: float,
            side_sign: float,
        ) -> None:
            swing = swing_amplitude * leg_phase
            lift = lift_amplitude * max(0.0, leg_phase)
            leg_targets[hip_yaw_index] += yaw_amplitude * side_sign
            leg_targets[hip_roll_index] += strafe_amplitude * side_sign
            leg_targets[hip_pitch_index] += -0.7 * swing
            leg_targets[knee_index] += lift
            leg_targets[ankle_pitch_index] += -0.5 * swing - 0.5 * lift
            leg_targets[ankle_roll_index] += -0.5 * strafe_amplitude * side_sign

        apply_leg(0, 1, 2, 3, 4, 5, left_phase, 1.0)
        apply_leg(6, 7, 8, 9, 10, 11, right_phase, -1.0)
        self.previous_action = (
            leg_targets - np.asarray(self.config.default_pose[:12], dtype=np.float32)
        ) / max(float(self.config.action_scale), 1e-6)
        return leg_targets

    def _build_positions(self, leg_targets: list[float]) -> list[float]:
        positions = list(self.config.default_pose)
        for joint_index, value in enumerate(leg_targets):
            positions[joint_index] = float(value)
        for waist_offset, target in enumerate(self.config.waist_joint_target):
            positions[WAIST_JOINT_INDICES[waist_offset]] = float(target)
        for arm_offset, target in enumerate(self.config.arm_joint_target):
            positions[ARM_JOINT_INDICES[arm_offset]] = float(target)
        for joint_index in CONTROLLED_JOINT_INDICES:
            minimum = float(self.config.controlled_joint_mins[joint_index])
            maximum = float(self.config.controlled_joint_maxs[joint_index])
            positions[joint_index] = min(
                max(float(positions[joint_index]), minimum), maximum
            )
        return positions

    def _kp_vector(self, leg_scale: float) -> list[float]:
        kp = [0.0] * JOINT_COUNT
        for joint_index, value in enumerate(self.config.leg_joint_kp):
            kp[joint_index] = float(value) * leg_scale
        for waist_offset, value in enumerate(self.config.waist_joint_kp):
            kp[WAIST_JOINT_INDICES[waist_offset]] = float(value)
        for arm_offset, value in enumerate(self.config.arm_joint_kp):
            kp[ARM_JOINT_INDICES[arm_offset]] = float(value)
        return kp

    def _kd_vector(self) -> list[float]:
        kd = [0.0] * JOINT_COUNT
        for joint_index, value in enumerate(self.config.leg_joint_kd):
            kd[joint_index] = float(value)
        for waist_offset, value in enumerate(self.config.waist_joint_kd):
            kd[WAIST_JOINT_INDICES[waist_offset]] = float(value)
        for arm_offset, value in enumerate(self.config.arm_joint_kd):
            kd[ARM_JOINT_INDICES[arm_offset]] = float(value)
        return kd
