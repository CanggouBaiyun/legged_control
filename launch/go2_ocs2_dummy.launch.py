"""Launch the Go2 OCS2 dummy loop and optional RViz visualization."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_root = Path(__file__).resolve().parents[1]

    config_dir = project_root / "config/ocs2/go2"
    task_file = config_dir / "task.info"
    reference_file = config_dir / "reference.info"
    rviz_file = config_dir / "go2.rviz"

    urdf_file = (
        project_root / "models/ocs2/go2_mujoco_inertial.urdf"
    )
    description_dir = (
        project_root
        / "third_party/unitree_ros/robots/go2_description"
    )

    for path in (task_file, reference_file, rviz_file, urdf_file):
        if not path.is_file():
            raise FileNotFoundError(path)

    # Prepare the display description in memory.
    # Do not modify the URDF used by MPC.
    tree = ET.parse(urdf_file)
    prefix = "package://go2_description/"

    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")

        if filename.startswith(prefix):
            mesh_path = (
                description_dir / filename[len(prefix):]
            ).resolve()

            if not mesh_path.is_file():
                raise FileNotFoundError(mesh_path)

            mesh.set("filename", mesh_path.as_uri())

    robot_description = ET.tostring(
        tree.getroot(), encoding="unicode"
    )

    # Both MPC and Dummy receive exactly the same model and config.
    model_parameters = {
        "taskFile": str(task_file),
        "referenceFile": str(reference_file),
        "urdfFile": str(urdf_file),
    }

    mpc = Node(
        package="ocs2_legged_robot_ros",
        executable="legged_robot_ddp_mpc",
        name="go2_mpc",
        parameters=[model_parameters],
        output="screen",
    )

    dummy = Node(
        package="ocs2_legged_robot_ros",
        executable="legged_robot_dummy",
        name="go2_dummy",
        parameters=[model_parameters],
        output="log",
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="go2_robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                robot_description,
                value_type=str,
            ),
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="go2_rviz",
        arguments=["-d", str(rviz_file)],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Open RViz",
        ),
        LogInfo(msg=f"Go2 MPC model: {urdf_file}"),
        LogInfo(msg=f"Go2 MPC task: {task_file}"),
        mpc,
        dummy,
        state_publisher,
        rviz,
    ])