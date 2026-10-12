"""Export a Go2 URDF using MuJoCo rigid-body inertial parameters."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin

from go2_control.model import load_model_config


project_root = Path(__file__).resolve().parents[1]
config = load_model_config()

mj_model = mujoco.MjModel.from_xml_path(str(config["mjcf"]))
tree = ET.parse(config["urdf"])
root = tree.getroot()

# URDF link name -> MuJoCo body name.
body_mapping = {"base": "base_link"}

for leg in ("FL", "FR", "RL", "RR"):
    for segment in ("hip", "thigh", "calf"):
        name = f"{leg}_{segment}"
        body_mapping[name] = name

links = {
    link.attrib["name"]: link
    for link in root.findall("link")
}

# First verify that all required links and bodies exist.
body_ids = {}

for link_name, body_name in body_mapping.items():
    if link_name not in links:
        raise ValueError(f"Missing URDF link: {link_name}")

    body_id = mujoco.mj_name2id(
        mj_model,
        mujoco.mjtObj.mjOBJ_BODY,
        body_name,
    )

    if body_id < 0:
        raise ValueError(f"Missing MuJoCo body: {body_name}")

    body_ids[link_name] = body_id

# Do not silently omit any massive MuJoCo body.
mapped_body_ids = set(body_ids.values())

for body_id in range(1, mj_model.nbody):
    if mj_model.body_mass[body_id] > 0.0:
        if body_id not in mapped_body_ids:
            raise ValueError(
                f"Unmapped massive MuJoCo body: {body_id}"
            )

# Remove the original inertias from the COPY.
# Otherwise fixed rotor/foot inertias would be counted again.
for link in links.values():
    for inertial in list(link.findall("inertial")):
        link.remove(inertial)


def numbers(values):
    return " ".join(f"{float(value):.17g}" for value in values)


exported_mass = 0.0

for link_name, body_id in body_ids.items():
    mass = float(mj_model.body_mass[body_id])
    com = mj_model.body_ipos[body_id]

    # MuJoCo stores this quaternion as [w, x, y, z].
    w, x, y, z = mj_model.body_iquat[body_id]
    rotation = pin.Quaternion(w, x, y, z).toRotationMatrix()

    # Principal moments -> COM inertia expressed in body axes.
    inertia = (
        rotation
        @ np.diag(mj_model.body_inertia[body_id])
        @ rotation.T
    )

    inertial = ET.SubElement(links[link_name], "inertial")

    ET.SubElement(
        inertial,
        "origin",
        xyz=numbers(com),
        rpy="0 0 0",
    )

    ET.SubElement(inertial, "mass", value=f"{mass:.17g}")

    ET.SubElement(
        inertial,
        "inertia",
        ixx=f"{inertia[0, 0]:.17g}",
        ixy=f"{inertia[0, 1]:.17g}",
        ixz=f"{inertia[0, 2]:.17g}",
        iyy=f"{inertia[1, 1]:.17g}",
        iyz=f"{inertia[1, 2]:.17g}",
        izz=f"{inertia[2, 2]:.17g}",
    )

    exported_mass += mass

output_path = (
    project_root / "models/ocs2/go2_mujoco_inertial.urdf"
)
output_path.parent.mkdir(parents=True, exist_ok=True)

ET.indent(tree, space="  ")
tree.write(output_path, encoding="utf-8", xml_declaration=True)

print("MuJoCo total mass [kg]:")
print(float(np.sum(mj_model.body_mass)))

print("\nExported total mass [kg]:")
print(exported_mass)

print("\nOutput:")
print(output_path)