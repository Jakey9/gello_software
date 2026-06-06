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

    # warm-up reads
    for _ in range(20):
        driver.get_joints()

    # average multiple reads for stability
    num_samples = 50
    samples = np.array([driver.get_joints() for _ in range(num_samples)])
    curr_joints = np.mean(samples, axis=0)

    print(f"\nRaw servo positions (rad): {[f'{x:.4f}' for x in curr_joints]}")
    print(f"Raw servo positions (deg): {[f'{np.rad2deg(x):.1f}' for x in curr_joints]}")

    # offset = raw_reading - target / sign
    # so that: sign * (raw - offset) = target
    offsets = []
    for i in range(args.num_robot_joints):
        sign_i = args.joint_signs[i]
        target_i = args.start_joints[i]
        offset_i = curr_joints[i] - target_i / sign_i
        offsets.append(offset_i)

    # verify: compute what the sim would see with these offsets
    verify = []
    for i in range(args.num_robot_joints):
        sim_val = args.joint_signs[i] * (curr_joints[i] - offsets[i])
        verify.append(sim_val)

    print()
    print("=" * 60)
    print("CALIBRATION RESULTS")
    print("=" * 60)
    print(f"joint_offsets              : {[round(x, 4) for x in offsets]}")
    print(f"joint_signs                : {list(args.joint_signs)}")
    print(f"verification (should match start_joints): {[round(x, 4) for x in verify]}")

    if args.gripper:
        gripper_rad = curr_joints[-1]
        print(f"\ngripper raw position (deg) : {np.rad2deg(gripper_rad):.1f}")
        print(f"gripper_open  (degrees)    : {np.rad2deg(gripper_rad) - 0.2:.1f}")
        print(f"gripper_close (degrees)    : {np.rad2deg(gripper_rad) - 42:.1f}")

    offsets_str = ", ".join([f"{x:.4f}" for x in offsets])
    print()
    print("--- Copy-paste for quick_run.py ---")
    print(f"joint_offsets=({offsets_str}),")
    print()
    print("--- Copy-paste for lite6_sim_test.yaml ---")
    yaml_str = ", ".join([f"{x:.4f}" for x in offsets])
    print(f"joint_offsets: [{yaml_str}]")
    print("=" * 60)

    driver.close()


def main(args: Args) -> None:
    get_config(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
