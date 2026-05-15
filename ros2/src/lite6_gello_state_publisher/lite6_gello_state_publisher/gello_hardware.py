import sys
import time
from typing import List, Optional, TypedDict

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4]))
from gello.zhonglin.driver import ZhonglinDriver


class GelloHardwareParams(TypedDict):
    """Type-safe parameter dictionary for Lite6 GelloHardware initialization."""

    com_port: str
    gello_name: str
    baudrate: int
    num_arm_joints: int
    joint_signs: List[int]
    gripper: bool
    gripper_range_rad: List[float]
    assembly_offsets: List[float]


class GelloHardware:
    """Hardware interface for Lite6 GELLO using Zhonglin serial bus servos.

    Unlike the Dynamixel-based Franka GELLO, the Zhonglin servos are passive
    (read-only) and return absolute positions directly. No PID tuning, torque
    control, or delta-based tracking is needed.
    """

    # UFFactory Lite6 joint limits (radians)
    # Source: https://www.ufactory.cc/ufactory-lite-6
    JOINT_POSITION_LIMITS = np.array(
        [
            [-6.2832, 6.2832],   # J1: ±360°
            [-2.618, 2.618],     # J2: ±150°
            [-0.0611, 5.2360],   # J3: -3.5° to 300°
            [-6.2832, 6.2832],   # J4: ±360°
            [-2.1642, 2.1642],   # J5: ±124°
            [-6.2832, 6.2832],   # J6: ±360°
        ]
    )
    MID_JOINT_POSITIONS = JOINT_POSITION_LIMITS.mean(axis=1)

    @staticmethod
    def normalize_joint_positions(
        raw_positions: np.ndarray,
        assembly_offsets: np.ndarray,
        joint_signs: np.ndarray,
    ) -> np.ndarray:
        """Convert raw servo positions to normalized joint positions.

        Applies assembly offsets and joint signs, then wraps to [mid-pi, mid+pi).
        """
        return (
            np.mod(
                (raw_positions - assembly_offsets) * joint_signs
                - GelloHardware.MID_JOINT_POSITIONS,
                2 * np.pi,
            )
            - np.pi
            + GelloHardware.MID_JOINT_POSITIONS
        )

    def __init__(
        self,
        hardware_config: GelloHardwareParams,
        logger,
    ) -> None:
        self._logger = logger
        self._com_port = hardware_config["com_port"]
        self._gello_name = hardware_config["gello_name"]
        self._baudrate = hardware_config["baudrate"]
        self._num_arm_joints = hardware_config["num_arm_joints"]
        self._joint_signs = np.array(hardware_config["joint_signs"])
        self._gripper = hardware_config["gripper"]
        self._num_total_joints = self._num_arm_joints + (1 if self._gripper else 0)
        self._gripper_range_rad = hardware_config["gripper_range_rad"]
        self._assembly_offsets = np.array(hardware_config["assembly_offsets"])

        self._initialize_driver()

        self._initial_arm_joints_raw = self._driver.get_joints()[: self._num_arm_joints]
        initial_arm_joints = self.normalize_joint_positions(
            self._initial_arm_joints_raw,
            self._assembly_offsets,
            self._joint_signs,
        )

        self._prev_arm_joints_raw = self._initial_arm_joints_raw.copy()
        self._prev_arm_joints = initial_arm_joints.copy()

        self._logger.info(
            f"Lite6 GELLO '{self._gello_name}' initialized on {self._com_port} "
            f"({self._num_arm_joints} arm joints, gripper={'yes' if self._gripper else 'no'})"
        )

    def _initialize_driver(self) -> None:
        """Initialize the Zhonglin driver with joint IDs and port."""
        joint_ids = list(range(self._num_total_joints))
        self._driver = ZhonglinDriver(
            joint_ids, port=self._com_port, baudrate=self._baudrate
        )

    def get_joint_and_gripper_positions(self) -> tuple:
        """Return (arm_joint_positions, gripper_percent)."""
        joints_raw = self._driver.get_joints()
        arm_joints_raw = joints_raw[: self._num_arm_joints]
        gripper_position_raw = joints_raw[-1] if self._gripper else 0.0
        return (
            self.process_arm_joint_positions(arm_joints_raw),
            self.process_gripper_position(gripper_position_raw),
        )

    def process_arm_joint_positions(self, arm_joints_raw: np.ndarray) -> np.ndarray:
        """Calculate arm joint positions from raw servo readings.

        Uses delta-based tracking from raw encoder values to maintain continuity,
        then clamps to the robot's joint limits.
        """
        arm_joints_delta = (arm_joints_raw - self._prev_arm_joints_raw) * self._joint_signs
        arm_joints = self._prev_arm_joints + arm_joints_delta

        self._prev_arm_joints = arm_joints.copy()
        self._prev_arm_joints_raw = arm_joints_raw.copy()

        arm_joints_clipped = np.clip(
            arm_joints, self.JOINT_POSITION_LIMITS[:, 0], self.JOINT_POSITION_LIMITS[:, 1]
        )
        return arm_joints_clipped

    def process_gripper_position(self, gripper_position_raw: float) -> float:
        """Convert raw gripper position to 0-1 percentage. Returns 0.0 if no gripper."""
        if not self._gripper:
            return 0.0
        range_min, range_max = self._gripper_range_rad
        if abs(range_max - range_min) < 1e-6:
            return 0.0
        gripper_percent = (gripper_position_raw - range_min) / (range_max - range_min)
        return max(0.0, min(1.0, gripper_percent))

    def disable_torque(self) -> None:
        """Unload torque on all Zhonglin servos."""
        self._driver.set_torque_mode(False)

    def close(self) -> None:
        """Close the Zhonglin driver."""
        self._driver.close()
