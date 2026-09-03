"""Pinocchio model helpers and canonical Go2 indexing."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pinocchio as pin


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "go2_model.json"

# Official SDK order: front-right, front-left, rear-right, rear-left.
GO2_SDK_JOINT_ORDER = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)

GO2_FOOT_FRAMES = ("FR_foot", "FL_foot", "RR_foot", "RL_foot")


def load_model_config(path: Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    """Load config and convert model paths to absolute project paths."""

    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    config["urdf"] = PROJECT_ROOT / config["urdf"]
    config["mjcf"] = PROJECT_ROOT / config["mjcf"]
    return config


def build_go2_model(urdf_path: Path | None = None) -> pin.Model:
    """Build the official Go2 URDF as an unactuated free-flyer model."""

    if urdf_path is None:
        urdf_path = load_model_config()["urdf"]
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"Go2 URDF not found at {urdf_path}. Follow docs/setup_guide.md."
        )
    return pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())


def nominal_configuration(model: pin.Model) -> np.ndarray:
    """Return the official MuJoCo home posture in Pinocchio coordinates."""

    config = load_model_config()
    q = pin.neutral(model)
    q[2] = float(config["base_height_m"])

    for name, position in config["nominal_joint_positions_rad"].items():
        if not model.existJointName(name):
            raise KeyError(f"Joint {name!r} is missing from the Pinocchio model")
        joint = model.joints[model.getJointId(name)]
        if joint.nq != 1:
            raise ValueError(f"Expected 1-DoF joint {name}, got nq={joint.nq}")
        q[joint.idx_q] = float(position)
    return q


def sdk_joint_mapping(model: pin.Model) -> list[dict[str, int | str | float]]:
    """Map Unitree SDK2 motor order to Pinocchio q/v indices and limits."""

    mapping: list[dict[str, int | str | float]] = []
    for sdk_index, name in enumerate(GO2_SDK_JOINT_ORDER):
        if not model.existJointName(name):
            raise KeyError(f"SDK joint {name!r} is missing from the URDF")
        joint_id = int(model.getJointId(name))
        joint = model.joints[joint_id]
        mapping.append(
            {
                "sdk_index": sdk_index,
                "name": name,
                "pinocchio_joint_id": joint_id,
                "q_index": int(joint.idx_q),
                "v_index": int(joint.idx_v),
                "lower_position_limit_rad": float(
                    model.lowerPositionLimit[joint.idx_q]
                ),
                "upper_position_limit_rad": float(
                    model.upperPositionLimit[joint.idx_q]
                ),
                "velocity_limit_rad_s": float(model.velocityLimit[joint.idx_v]),
                "effort_limit_nm": float(model.effortLimit[joint.idx_v]),
            }
        )
    return mapping


def foot_positions(model: pin.Model, q: np.ndarray) -> dict[str, list[float]]:
    """Compute foot-frame translations in the world frame."""

    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    positions: dict[str, list[float]] = {}
    for name in GO2_FOOT_FRAMES:
        if not model.existFrame(name):
            raise KeyError(f"Foot frame {name!r} is missing from the URDF")
        frame_id = model.getFrameId(name)
        positions[name] = data.oMf[frame_id].translation.tolist()
    return positions

