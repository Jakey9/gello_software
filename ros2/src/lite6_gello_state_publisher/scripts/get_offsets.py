"""Calibration script for Lite6 GELLO with Zhonglin servos.

Place the GELLO leader arm in the Lite6 home pose (all joints at 0, or a known pose),
then run this script to determine assembly_offsets and gripper_range_rad.

Usage:
    python3 get_offsets.py --port /dev/ttyUSB0 --start-joints 0.0 0.0 0.0 0.0 0.0 0.0
"""

import sys
from dataclasses import dataclass
from pathlib import Path
from textwrap import indent
from typing import Tuple

import numpy as np
import tyro

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from gello.zhonglin.driver import ZhonglinDriver
from lite6_gello_state_publisher.gello_hardware import GelloHardware

GRIPPER_OPEN_TO_CLOSED_RAD = -1.22


@dataclass
class Args:
    port: str = "/dev/ttyUSB0"
    """The serial port that GELLO is connected to."""

    baudrate: int = 115200
    """Serial baudrate for Zhonglin servos."""

    start_joints: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    """The joint angles (radians) of the known pose the GELLO is placed in."""

    joint_signs: Tuple[int, ...] = (1, 1, 1, 1, 1, 1)
    """Sign multipliers for each joint to account for motor direction."""

    gripper: bool = True
    """Whether a gripper servo is attached."""

    def __post_init__(self):
        assert len(self.joint_signs) == len(self.start_joints)
        for idx, j in enumerate(self.joint_signs):
            assert j in (-1, 1), f"Joint idx {idx} sign should be -1 or 1, got {j}"

    @property
    def num_arm_joints(self) -> int:
        return len(self.start_joints)

    @property
    def num_total_joints(self) -> int:
        return self.num_arm_joints + (1 if self.gripper else 0)


def determine_offsets(
    arm_joints_raw: np.ndarray, start_joints: np.ndarray, joint_signs: np.ndarray
) -> np.ndarray:
    """Calculate assembly offsets by comparing current pose to expected pose.

    Rounds offsets to the nearest 90 degrees to resolve multi-turn ambiguity.
    """
    arm_joints_normalized = GelloHardware.normalize_joint_positions(
        arm_joints_raw,
        np.zeros(len(arm_joints_raw)),
        joint_signs,
    )
    pose_differences = arm_joints_normalized - start_joints
    offsets = np.round(pose_differences / (np.pi / 2)) * (np.pi / 2)
    offsets_normalized = np.mod(offsets, 2 * np.pi)
    return offsets_normalized


def main(args: Args) -> None:
    joint_ids = list(range(args.num_total_joints))
    driver = ZhonglinDriver(joint_ids, port=args.port, baudrate=args.baudrate)

    import time
    time.sleep(0.5)  # let the reading thread stabilize

    joints_raw = driver.get_joints()
    arm_joints_raw = np.array(joints_raw[: args.num_arm_joints])
    assembly_offsets = determine_offsets(
        arm_joints_raw, np.array(args.start_joints), np.array(args.joint_signs)
    )

    gripper_range_rad = None
    if args.gripper and len(joints_raw) > args.num_arm_joints:
        gripper_open = joints_raw[-1]
        gripper_range_rad = [gripper_open + GRIPPER_OPEN_TO_CLOSED_RAD, gripper_open]

    print("\n--- Calibration Results ---")
    print("Update your config YAML with the following values:\n")
    print(indent(f'com_port: "{args.port}"', "  "))
    print(indent(f"baudrate: {args.baudrate}", "  "))
    print(indent(f"num_arm_joints: {args.num_arm_joints}", "  "))
    print(indent(f"joint_signs: {list(args.joint_signs)}", "  "))
    print(indent(f"gripper: {str(args.gripper).lower()}", "  "))
    print(indent(f"assembly_offsets: {list(np.round(assembly_offsets, 3))} # rad", "  "))
    if args.gripper and gripper_range_rad is not None:
        print(indent(f"gripper_range_rad: {list(np.round(gripper_range_rad, 3))}", "  "))
    print()

    driver.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
