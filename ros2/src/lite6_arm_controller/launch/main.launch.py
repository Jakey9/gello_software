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


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("lite6_arm_controller").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    config = load_yaml(config_file)

    namespace = LaunchConfiguration("namespace").perform(context)

    nodes = [
        Node(
            package="lite6_arm_controller",
            executable="joint_position_controller",
            name="joint_position_controller",
            namespace=namespace,
            output="screen",
            on_exit=Shutdown(),
            parameters=[
                {"robot_ip": config["robot_ip"]},
                {"speed": config.get("speed", 50.0)},
                {"servo_mode": config.get("servo_mode", True)},
            ],
        ),
    ]

    if config.get("gripper_enabled", False):
        nodes.append(
            Node(
                package="lite6_arm_controller",
                executable="gripper_controller",
                name="gripper_controller",
                namespace=namespace,
                output="screen",
                on_exit=Shutdown(),
                parameters=[
                    {"robot_ip": config["robot_ip"]},
                    {"modbus_baudrate": config.get("modbus_baudrate", 115200)},
                    {"gripper_open": config.get("gripper_open", 0)},
                    {"gripper_close": config.get("gripper_close", 250)},
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="example.yaml",
                description="Name of the controller configuration file to load",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS2 namespace for the controller nodes",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
