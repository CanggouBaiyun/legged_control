#!/usr/bin/env python3
"""Inventory the official Go2 URDF using only the Python standard library."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "configs" / "go2_model.json"


def default_urdf_path() -> Path:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return PROJECT_ROOT / config["urdf"]


def inventory(urdf_path: Path) -> dict[str, object]:
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"Go2 URDF not found at {urdf_path}. Follow docs/setup_guide.md."
        )

    robot = ET.parse(urdf_path).getroot()
    links = robot.findall("link")
    joints = robot.findall("joint")
    joint_types = Counter(joint.get("type", "unknown") for joint in joints)

    masses: dict[str, float] = {}
    links_without_inertial: list[str] = []
    for link in links:
        link_name = link.get("name", "<unnamed>")
        mass_element = link.find("./inertial/mass")
        if mass_element is None or mass_element.get("value") is None:
            links_without_inertial.append(link_name)
            continue
        masses[link_name] = float(mass_element.get("value", "0"))

    movable_joints = [
        joint.get("name", "<unnamed>")
        for joint in joints
        if joint.get("type") != "fixed"
    ]
    result: dict[str, object] = {
        "source_urdf": str(urdf_path),
        "robot_name": robot.get("name"),
        "link_count": len(links),
        "joint_count": len(joints),
        "joint_types": dict(sorted(joint_types.items())),
        "movable_joint_count": len(movable_joints),
        "movable_joints": movable_joints,
        "links_with_mass_count": len(masses),
        "links_without_inertial": links_without_inertial,
        "sum_of_declared_link_masses_kg": sum(masses.values()),
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=default_urdf_path())
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "go2_urdf_inventory.json",
    )
    args = parser.parse_args()

    result = inventory(args.urdf)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\nSaved URDF inventory to {args.output}")


if __name__ == "__main__":
    main()

