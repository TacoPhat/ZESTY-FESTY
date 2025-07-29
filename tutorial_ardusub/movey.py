import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float64
from rclpy import timer


class Forward(Node):
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
            "/desired_side",
            self.side,
            10
        )

    def side(self, msg):
        self.get_logger().info(f"Applying side force to  {msg.data}")
        move_msg = ManualControl()
        move_msg.y = msg.data
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