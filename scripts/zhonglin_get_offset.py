"""Calibration script for GELLO with Zhonglin servos.

Usage:
    1. Move the Lite6 to its home/start position (e.g. all zeros).
    2. Manually align the GELLO leader arm so it matches the Lite6 pose.
    3. Run this script to read Zhonglin servo positions and compute offsets.
    4. Copy the printed offsets and signs into your config.
"""

import os
import sys
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import tyro

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from gello.zhonglin.driver import ZhonglinDriver


@dataclass
class Args:
    port: str = "/dev/ttyUSB0"
    """The serial port that the Zhonglin servos are connected to."""

    baudrate: int = 115200
    """Baudrate for Zhonglin serial communication."""

    start_joints: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)
    """The target joint angles the GELLO should represent (in radians).
    Set these to match the Lite6 home position you aligned to."""

    joint_signs: Tuple[int, ...] = (1, 1, 1, 1, 1, 1)
    """Direction mapping for each joint. Use 1 or -1."""

    gripper: bool = True
    """Whether a gripper servo is attached (ID = next after arm joints)."""

    def __post_init__(self):
        assert len(self.joint_signs) == len(self.start_joints)
        for idx, j in enumerate(self.joint_signs):
            assert j in (-1, 1), f"Joint idx {idx}: sign should be -1 or 1, got {j}"

    @property
    def num_robot_joints(self) -> int:
        return len(self.start_joints)

    @property
    def num_joints(self) -> int:
        return self.num_robot_joints + (1 if self.gripper else 0)


def get_config(args: Args) -> None:
    joint_ids = list(range(args.num_joints))
    driver = ZhonglinDriver(joint_ids, port=args.port, baudrate=args.baudrate)

    for _ in range(20):
        driver.get_joints()

    def get_error(offset: float, index: int, joint_state: np.ndarray) -> float:
        joint_sign_i = args.joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = args.start_joints[index]
        return float(np.abs(joint_i - start_i))

    best_offsets = []
    curr_joints = driver.get_joints()
    print(f"\nRaw servo positions (rad): {[f'{x:.4f}' for x in curr_joints]}")
    print(f"Raw servo positions (deg): {[f'{np.rad2deg(x):.1f}' for x in curr_joints]}")

    for i in range(args.num_robot_joints):
        best_offset = 0.0
        best_error = 1e6
        for offset in np.linspace(-8 * np.pi, 8 * np.pi, 8 * 4 + 1):
            error = get_error(offset, i, curr_joints)
            if error < best_error:
                best_error = error
                best_offset = offset
        best_offsets.append(best_offset)

    print()
    print("=" * 60)
    print("CALIBRATION RESULTS")
    print("=" * 60)
    print(f"joint_offsets (raw)        : {[f'{x:.4f}' for x in best_offsets]}")
    print(
        "joint_offsets (as pi/2)    : ["
        + ", ".join(
            [f"{int(np.round(x / (np.pi / 2)))}*np.pi/2" for x in best_offsets]
        )
        + "]"
    )
    print(f"joint_signs                : {list(args.joint_signs)}")

    if args.gripper:
        gripper_rad = curr_joints[-1]
        print(f"\ngripper raw position (deg) : {np.rad2deg(gripper_rad):.1f}")
        print(f"gripper_open  (degrees)    : {np.rad2deg(gripper_rad) - 0.2:.1f}")
        print(f"gripper_close (degrees)    : {np.rad2deg(gripper_rad) - 42:.1f}")

    print()
    print("Copy these into your GelloLite6Config / ZhonglinRobotConfig.")
    print("=" * 60)

    driver.close()


def main(args: Args) -> None:
    get_config(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
