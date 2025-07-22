import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float64
from rclpy import timer


class Forward(Node): #moves forward
    def __init__(self):
        super().__init__("bluerow_publisher")
        self.x = 0.0
        self.pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.create_subscription(
            Float64,
            "/desired_forward",
            self.forward,
            10
        )

    def forward(self, msg):
        self.get_logger().info(f"Applying force to {msg.data}")
        move_msg = ManualControl()
        move_msg.x = msg.data
        self.pub.publish(move_msg)
        self.get_logger().info(f"Moving at: {msg.data}")

def main(args=None):
        rclpy.init(args=args)
        node = Forward()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received, shutting down...")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()