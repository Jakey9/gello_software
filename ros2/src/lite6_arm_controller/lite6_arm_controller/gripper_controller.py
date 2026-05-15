"""ROS2 node that subscribes to GELLO gripper commands and drives the OpenParallelGripper.

Subscribes to:
    gripper/target_gripper_width_percent (std_msgs/Float32)

Uses Modbus RTU via xArm's tool port (RS485) to command the gripper servo.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class GripperController(Node):
    """Track GELLO gripper commands on an OpenParallelGripper via Lite6 tool port."""

    MODBUS_SERVO_ADDR = 0x01
    MODBUS_POSITION_REG = 0x002A

    def __init__(self) -> None:
        super().__init__("gripper_controller")

        self.declare_parameter("robot_ip", "192.168.1.111")
        self.declare_parameter("modbus_baudrate", 115200)
        self.declare_parameter("gripper_open", 0)
        self.declare_parameter("gripper_close", 250)

        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._modbus_baudrate = (
            self.get_parameter("modbus_baudrate").get_parameter_value().integer_value
        )
        self._gripper_open = (
            self.get_parameter("gripper_open").get_parameter_value().integer_value
        )
        self._gripper_close = (
            self.get_parameter("gripper_close").get_parameter_value().integer_value
        )

        try:
            from xarm.wrapper import XArmAPI
        except ImportError:
            self.get_logger().fatal(
                "xarm-python-sdk not installed. Install with: pip install xarm-python-sdk"
            )
            raise

        self._arm = XArmAPI(robot_ip)
        self._arm.set_tgpio_modbus_baudrate(self._modbus_baudrate)
        self._arm.set_tgpio_modbus_timeout(20)

        self._arm.set_tgpio_digital(0, 1)
        self._arm.set_tgpio_digital(1, 1)

        self._modbus_write(self._gripper_open)
        self.get_logger().info(
            f"Gripper controller connected to Lite6 at {robot_ip}, "
            f"range [{self._gripper_open}, {self._gripper_close}]"
        )

        self._subscription = self.create_subscription(
            Float32,
            "gripper/target_gripper_width_percent",
            self._gripper_callback,
            10,
        )

    def _modbus_write(self, position: int) -> None:
        """Write a position to the gripper servo via Modbus FC 0x06."""
        data = [
            self.MODBUS_SERVO_ADDR,
            0x06,
            (self.MODBUS_POSITION_REG >> 8) & 0xFF,
            self.MODBUS_POSITION_REG & 0xFF,
            (position >> 8) & 0xFF,
            position & 0xFF,
        ]
        self._arm.getset_tgpio_modbus_data(data, min_res_len=6)

    def _gripper_callback(self, msg: Float32) -> None:
        """Convert 0-1 percentage to gripper servo range and command."""
        percent = max(0.0, min(1.0, msg.data))
        position = int(
            self._gripper_open + percent * (self._gripper_close - self._gripper_open)
        )
        self._modbus_write(position)

    def destroy_node(self) -> None:
        """Open gripper and disconnect."""
        if hasattr(self, "_arm"):
            self._modbus_write(self._gripper_open)
            self._arm.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = GripperController()
    except Exception:
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
