import re
import time
from typing import Optional, Sequence, Tuple

import numpy as np
import serial

from gello.dynamixel.driver import DynamixelDriverProtocol

PWM_MIN = 500
PWM_MAX = 2500
ANGLE_RANGE_DEG = 270.0
ANGLE_RANGE_RAD = np.deg2rad(ANGLE_RANGE_DEG)
PWM_SPAN = PWM_MAX - PWM_MIN

_PWM_PATTERN = re.compile(r"P(\d{4})")


def pwm_to_radians(response_str: str) -> Optional[float]:
    """Parse a Zhonglin ASCII response and return the absolute angle in radians."""
    match = _PWM_PATTERN.search(response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    return (pwm_val - PWM_MIN) / PWM_SPAN * ANGLE_RANGE_RAD


class FakeZhonglinDriver(DynamixelDriverProtocol):
    def __init__(self, ids: Sequence[int]):
        self._ids = list(ids)
        self._joint_angles = np.zeros(len(ids), dtype=float)
        self._velocities = np.zeros(len(ids), dtype=float)
        self._torque_enabled = False

    def set_joints(self, joint_angles: Sequence[float]):
        raise NotImplementedError("Zhonglin servos are read-only on the GELLO leader arm")

    def set_current(self, currents: Sequence[float]):
        raise NotImplementedError("Zhonglin servos do not support current control")

    def set_torque(self, torques: Sequence[float]):
        raise NotImplementedError("Zhonglin servos do not support torque control")

    def set_operating_mode(self, mode: int):
        pass

    def verify_operating_mode(self, expected_mode: int):
        pass

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        self._torque_enabled = enable

    def get_joints(self) -> np.ndarray:
        return self._joint_angles.copy()

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._joint_angles.copy(), self._velocities.copy()

    def close(self):
        pass


class ZhonglinDriver(DynamixelDriverProtocol):
    """Driver for Zhonglin serial bus servos using ASCII protocol.

    These servos are used as passive (read-only) joints on the GELLO leader arm.
    Position is read via the PRAD command; write operations are not supported.
    Matches the proven synchronous protocol from servo_zero.py.
    """

    def __init__(
        self,
        ids: Sequence[int],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
    ):
        self._ids = list(ids)
        self._torque_enabled = False

        self._ser = serial.Serial(port, baudrate, timeout=0.01)
        print(f"[ZhonglinDriver] Serial port opened: {port}")

        self._init_servos()

    def _send_command(self, cmd: str) -> str:
        self._ser.write(cmd.encode("ascii"))
        time.sleep(0.01)
        response = self._ser.read_all()
        return response.decode("ascii", errors="ignore")

    def _init_servos(self):
        """Run the Zhonglin init sequence: version check, unload torque."""
        self._send_command(f"#{self._ids[0]:03d}PVER!")
        for servo_id in self._ids:
            response = self._send_command(f"#{servo_id:03d}PULK!")
            print(f"[ZhonglinDriver] Servo {servo_id} torque released: {response.strip()}")
        print(f"[ZhonglinDriver] Initialized {len(self._ids)} servos, torque unloaded")

    def _read_all_joints(self) -> np.ndarray:
        angles = np.zeros(len(self._ids), dtype=float)
        for i, servo_id in enumerate(self._ids):
            response = self._send_command(f"#{servo_id:03d}PRAD!")
            angle = pwm_to_radians(response.strip())
            if angle is not None:
                angles[i] = angle
        return angles

    # -- DynamixelDriverProtocol interface --

    def set_joints(self, joint_angles: Sequence[float]):
        raise NotImplementedError("Zhonglin servos are read-only on the GELLO leader arm")

    def set_current(self, currents: Sequence[float]):
        raise NotImplementedError("Zhonglin servos do not support current control")

    def set_torque(self, torques: Sequence[float]):
        raise NotImplementedError("Zhonglin servos do not support torque control")

    def set_operating_mode(self, mode: int):
        pass

    def verify_operating_mode(self, expected_mode: int):
        pass

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        if not enable:
            for servo_id in self._ids:
                self._send_command(f"#{servo_id:03d}PULK!")
        self._torque_enabled = enable

    def get_joints(self) -> np.ndarray:
        return self._read_all_joints()

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        positions = self.get_joints()
        velocities = np.zeros_like(positions)
        return positions, velocities

    def close(self):
        if self._ser.is_open:
            self._ser.close()
        print("[ZhonglinDriver] Closed")


def main():
    ids = list(range(7))
    driver = ZhonglinDriver(ids)
    try:
        while True:
            joints = driver.get_joints()
            print(f"Joint angles (rad): {np.round(joints, 3)}")
            print(f"Joint angles (deg): {np.round(np.rad2deg(joints), 1)}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        driver.close()


if __name__ == "__main__":
    main()
