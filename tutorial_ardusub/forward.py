import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float64
from rclpy import timer


class Forward(Node):
    def __init__(self):
        super().__init__("bluerow_publisher")
        self.pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.sub = self.create_subscription(
            Float64,
            "/desired_forward",
            self.log_forward_command,
            10
        )
    def log_forward_command(self, msg):
         return msg.data    
    def forward(self, msg):
        move_msg = ManualControl()
        move_msg.x = self.log_forward_command
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