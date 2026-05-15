import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def generate_robot_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("lite6_gello_state_publisher").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []
    for item_name, config in configs.items():
        namespace = config.get("namespace", "")
        com_port = config["com_port"]
        if not com_port.startswith("/dev/"):
            com_port = "/dev/serial/by-id/" + com_port
        nodes.append(
            Node(
                package="lite6_gello_state_publisher",
                executable="gello_publisher",
                name="gello_publisher",
                namespace=namespace,
                output="screen",
                on_exit=Shutdown(),
                parameters=[
                    {"com_port": com_port},
                    {"gello_name": item_name},
                    {"baudrate": config.get("baudrate", 115200)},
                    {"num_arm_joints": config.get("num_arm_joints", 6)},
                    {"joint_signs": config["joint_signs"]},
                    {"gripper": config.get("gripper", True)},
                    {"gripper_range_rad": config.get("gripper_range_rad", [0.0, 0.0])},
                    {"assembly_offsets": config["assembly_offsets"]},
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="example_single.yaml",
                description="Name of the GELLO configuration file to load",
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
