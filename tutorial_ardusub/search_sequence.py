import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int16, Float64, Float64MultiArray
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np

class SearchSequence(Node):
    def __init__(self):
        self.depth_pub = self.create_publisher(
            Float64,
            "/desired_depth",
            10
        )
        self.heading_pub = self.create_publisher(
            Float64,
            "/desired_heading",
            10
        )
        self.forward_pub = self.create_publisher(
            Float64,
            "/desired_forward",
            10
        )
        self.side_pub = self.create_publisher(
            Float64,
            "/desired_side",
            10
        )
        self.sub = self.create_subscription(
            Int16,
            "/heading",
            self.search,
            10
        )
    
    def search(self, msg):
        for x in range(3):
            self.current_heading = msg.data
            self.target_heading = self.current_heading + 120.0
        self.target_depth = 3.5
        #seperate
        for y in range(3):
            self.current_heading = msg.data
            self.target_heading = self.current_heading - 120.0
        self.target_depth = 0.5

def main(args=None):
    rclpy.init(args=args)
    node = SearchSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown() 
    

    
    

