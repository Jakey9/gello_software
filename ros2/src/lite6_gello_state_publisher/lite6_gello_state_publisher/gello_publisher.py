import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from rclpy.parameter import parameter_value_to_python
from lite6_gello_state_publisher.gello_hardware import GelloHardware, GelloHardwareParams
from lite6_gello_state_publisher.gello_parameter_config import (
    ParameterConfig,
    GelloParameterConfig,
)


class GelloPublisher(Node):
    """ROS2 node for publishing Lite6 GELLO joint states (Zhonglin servos)."""

    def __init__(self) -> None:
        super().__init__("gello_publisher")
        self.PUBLISHING_RATE = 25  # Hz

        hardware_params: GelloHardwareParams = self._setup_hardware_parameters()

        try:
            self.gello_hardware = GelloHardware(hardware_params, self.get_logger())
        except ConnectionError as e:
            self.get_logger().error(f"Failed to initialize GELLO hardware: {e}")
            raise

        self.arm_joint_publisher = self.create_publisher(JointState, "gello/joint_states", 10)
        self.gripper_joint_publisher = self.create_publisher(
            Float32, "gripper/target_gripper_width_percent", 10
        )

        self.get_logger().info("Publishing Lite6 GELLO joint states.")
        self.timer = self.create_timer(1 / self.PUBLISHING_RATE, self.publish_joint_states)

    def publish_joint_states(self) -> None:
        """Publish current joint states and gripper position."""
        JOINT_NAMES = [
            "lite6_joint1",
            "lite6_joint2",
            "lite6_joint3",
            "lite6_joint4",
            "lite6_joint5",
            "lite6_joint6",
        ]
        [arm_joints, gripper_position] = self.gello_hardware.get_joint_and_gripper_positions()

        arm_joint_states = JointState()
        arm_joint_states.header.stamp = self.get_clock().now().to_msg()
        arm_joint_states.name = JOINT_NAMES
        arm_joint_states.header.frame_id = "lite6_link_base"
        arm_joint_states.position = arm_joints.tolist()

        self.arm_joint_publisher.publish(arm_joint_states)

        gripper_msg = Float32()
        gripper_msg.data = gripper_position
        self.gripper_joint_publisher.publish(gripper_msg)

    def destroy_node(self) -> None:
        """Disable torque and close driver before shutting down."""
        self.gello_hardware.disable_torque()
        self.gello_hardware.close()
        super().destroy_node()

    def _declare_ros2_param(self, param: ParameterConfig):
        """Declare a ROS2 parameter and return its value."""
        parameter_value = self.declare_parameter(
            param.descriptor.name, param.default, param.descriptor
        ).get_parameter_value()
        return parameter_value_to_python(parameter_value)

    def _setup_hardware_parameters(self):
        """Declare and return all hardware configuration parameters."""
        config = GelloParameterConfig()
        hardware_params: GelloHardwareParams = {}
        for param in config:
            hardware_params[param.descriptor.name] = self._declare_ros2_param(param)
        return hardware_params


def main(args=None):
    rclpy.init(args=args)

    try:
        gello_publisher = GelloPublisher()
    except ConnectionError:
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(gello_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        gello_publisher.gello_hardware.disable_torque()
        gello_publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
