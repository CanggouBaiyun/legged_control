"""Generate a visualization-only URDF with absolute mesh paths."""

from pathlib import Path
import xml.etree.ElementTree as ET

project_root = Path(__file__).resolve().parents[1]

description_dir = (
    project_root
    / "third_party/unitree_ros/robots/go2_description"
)

source_path = description_dir / "urdf/go2_description.urdf"
output_path = Path("/tmp/go2_ocs2_visual.urdf")

tree = ET.parse(source_path)
prefix = "package://go2_description/"
converted = 0

for mesh in tree.getroot().iter("mesh"):
    filename = mesh.get("filename", "")

    if not filename.startswith(prefix):
        continue

    relative_path = filename[len(prefix):]
    absolute_path = (description_dir / relative_path).resolve()

    if not absolute_path.is_file():
        raise FileNotFoundError(absolute_path)

    mesh.set("filename", absolute_path.as_uri())
    converted += 1

tree.write(
    output_path,
    encoding="utf-8",
    xml_declaration=True,
)

print(f"Converted mesh paths: {converted}")
print(f"Visualization URDF: {output_path}")