"""ROS2 node that subscribes to GELLO joint states and commands a Lite6 via xArm SDK.

Subscribes to:
    gello/joint_states (sensor_msgs/JointState)

The node calls xArm SDK's set_servo_angle_j() in servo mode for real-time tracking.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointPositionController(Node):
    """Track GELLO joint commands on a real Lite6 using xArm SDK."""

    def __init__(self) -> None:
        super().__init__("joint_position_controller")

        self.declare_parameter("robot_ip", "192.168.1.111")
        self.declare_parameter("speed", 50.0)
        self.declare_parameter("servo_mode", True)

        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._speed = self.get_parameter("speed").get_parameter_value().double_value
        self._servo_mode = self.get_parameter("servo_mode").get_parameter_value().bool_value

        try:
            from xarm.wrapper import XArmAPI
        except ImportError:
            self.get_logger().fatal(
                "xarm-python-sdk not installed. Install with: pip install xarm-python-sdk"
            )
            raise

        self._arm = XArmAPI(robot_ip)
        self._arm.motion_enable(enable=True)
        self._arm.set_mode(0)
        self._arm.set_state(state=0)

        if self._servo_mode:
            self._arm.set_mode(1)  # servo mode
            self._arm.set_state(state=0)

        self.get_logger().info(
            f"Connected to Lite6 at {robot_ip} (servo_mode={self._servo_mode})"
        )

        self._last_joint_state = None
        self._subscription = self.create_subscription(
            JointState,
            "gello/joint_states",
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        """Send received joint positions to the Lite6."""
        if len(msg.position) < 6:
            self.get_logger().warning(
                f"Expected 6 joints, got {len(msg.position)}, skipping"
            )
            return

        angles_deg = [math.degrees(rad) for rad in msg.position[:6]]

        if self._servo_mode:
            self._arm.set_servo_angle_j(angles=angles_deg, is_radian=False)
        else:
            self._arm.set_servo_angle(
                angle=angles_deg, speed=self._speed, wait=False, is_radian=False
            )

    def destroy_node(self) -> None:
        """Clean up xArm connection."""
        if hasattr(self, "_arm"):
            self._arm.set_mode(0)
            self._arm.set_state(state=0)
            self._arm.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = JointPositionController()
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
