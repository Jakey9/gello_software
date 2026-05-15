from dataclasses import dataclass
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from typing import Any, Iterator


@dataclass
class ParameterConfig:
    descriptor: ParameterDescriptor
    default: Any


class GelloParameterConfig:
    """Configuration class for Lite6 GELLO ROS2 parameters (Zhonglin servos)."""

    DEFAULT_COM_PORT = "/dev/ttyUSB0"
    DEFAULT_BAUDRATE = 115200
    DEFAULT_NUM_JOINTS = 6
    DEFAULT_JOINT_SIGNS = [1] * DEFAULT_NUM_JOINTS
    DEFAULT_ASSEMBLY_OFFSETS = [0.0] * DEFAULT_NUM_JOINTS
    DEFAULT_GRIPPER_RANGE_RAD = [0.0, 0.0]

    def __init__(self):
        self.hardware_params = [
            ParameterConfig(
                ParameterDescriptor(
                    name="com_port",
                    type=ParameterType.PARAMETER_STRING,
                    description="USB serial port path for Zhonglin servos",
                    read_only=True,
                ),
                self.DEFAULT_COM_PORT,
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="gello_name",
                    type=ParameterType.PARAMETER_STRING,
                    description="GELLO device identifier",
                    read_only=True,
                ),
                "lite6_gello",
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="baudrate",
                    type=ParameterType.PARAMETER_INTEGER,
                    description="Serial baudrate for Zhonglin servos",
                    read_only=True,
                ),
                self.DEFAULT_BAUDRATE,
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="num_arm_joints",
                    type=ParameterType.PARAMETER_INTEGER,
                    description="Number of arm joints (6 for Lite6)",
                    read_only=True,
                ),
                self.DEFAULT_NUM_JOINTS,
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="joint_signs",
                    type=ParameterType.PARAMETER_INTEGER_ARRAY,
                    description="Joint direction signs (1 or -1 per joint)",
                    read_only=True,
                ),
                self.DEFAULT_JOINT_SIGNS,
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="gripper",
                    type=ParameterType.PARAMETER_BOOL,
                    description="Enable gripper servo (ID = num_arm_joints)",
                    read_only=True,
                ),
                True,
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="gripper_range_rad",
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    description="Gripper range in radians [open, closed]",
                    read_only=True,
                ),
                self.DEFAULT_GRIPPER_RANGE_RAD,
            ),
            ParameterConfig(
                ParameterDescriptor(
                    name="assembly_offsets",
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    description="Joint offset calibration values in radians",
                    read_only=True,
                ),
                self.DEFAULT_ASSEMBLY_OFFSETS,
            ),
        ]

    def __iter__(self) -> Iterator[ParameterConfig]:
        return iter(self.hardware_params)
