import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float64
from rclpy import timer


class Depth_Find(Node):
    def __init__(self):
        super().__init__("bluerow_publisher")
        self.sub = self.create_subscription(
            FluidPressure,        
            "/pressure",    
            self.calculate_depth, 
            10             
        )
        self.depth_pub = self.create_publisher(
            Float64,
            "/depth_state",
            10
        )

    def calculate_depth(self, msg, d=1000, g=9.81, atmospheric_pressure=101325.0):
        depth = -(msg.fluid_pressure - atmospheric_pressure) / (d * g)
        depth_msg = Float64()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)

def main(args=None):
        rclpy.init(args=args)
        node = Depth_Find()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received, shutting down...")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()