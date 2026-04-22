from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import yaml


JOINT_NAME_ORDER = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


@dataclass(frozen=True)
class JointLimit:
    index: int
    name: str
    min_position: float
    max_position: float


def load_joint_limits(
    joint_names: Sequence[str],
    min_positions: Sequence[float],
    max_positions: Sequence[float],
) -> list[JointLimit]:
    if list(joint_names) != JOINT_NAME_ORDER:
        raise ValueError("joint_limit_names must match the expected 29-joint G1 order.")
    if len(min_positions) != len(JOINT_NAME_ORDER):
        raise ValueError("joint_limit_mins must contain 29 values.")
    if len(max_positions) != len(JOINT_NAME_ORDER):
        raise ValueError("joint_limit_maxs must contain 29 values.")
    if all(
        float(min_positions[index]) == 0.0 for index in range(len(JOINT_NAME_ORDER))
    ) and all(
        float(max_positions[index]) == 0.0 for index in range(len(JOINT_NAME_ORDER))
    ):
        raise ValueError(
            "joint_limit_mins and joint_limit_maxs are all zero; load safety_params.yaml before starting the node."
        )

    limits: list[JointLimit] = []
    for index, joint_name in enumerate(JOINT_NAME_ORDER):
        low = float(min_positions[index])
        high = float(max_positions[index])
        if low > high:
            raise ValueError(f"Joint {joint_name!r} has min_position > max_position.")
        limits.append(
            JointLimit(
                index=index,
                name=joint_name,
                min_position=low,
                max_position=high,
            )
        )

    return limits


def _ros_parameters_from_yaml(config: object) -> Mapping[str, object]:
    if not isinstance(config, Mapping):
        raise ValueError("Safety config YAML must contain a mapping at the root.")

    for value in config.values():
        if isinstance(value, Mapping) and "ros__parameters" in value:
            ros_parameters = value["ros__parameters"]
            if not isinstance(ros_parameters, Mapping):
                raise ValueError("ros__parameters must be a mapping.")
            return ros_parameters

    raise ValueError("Could not find ros__parameters in safety config YAML.")


def _sequence_parameter(
    ros_parameters: Mapping[str, object],
    key: str,
) -> Sequence[object]:
    value = ros_parameters.get(key)
    if value is None:
        raise ValueError(f"Missing required safety config parameter {key!r}.")
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"Safety config parameter {key!r} must be a sequence.")
    return value


def load_joint_limits_from_yaml(config_path: str | Path) -> list[JointLimit]:
    config = cast(object, yaml.safe_load(Path(config_path).read_text(encoding="utf-8")))
    ros_parameters = _ros_parameters_from_yaml(config)
    joint_names = _sequence_parameter(ros_parameters, "joint_limit_names")
    min_positions = _sequence_parameter(ros_parameters, "joint_limit_mins")
    max_positions = _sequence_parameter(ros_parameters, "joint_limit_maxs")
    return load_joint_limits(
        [str(item) for item in joint_names],
        [float(str(item)) for item in min_positions],
        [float(str(item)) for item in max_positions],
    )
