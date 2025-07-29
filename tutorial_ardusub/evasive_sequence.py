import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int16, Float64, Float64MultiArray
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np

class EvasiveSequence(Node):
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
        self.head_sub = self.create_subscription(
            Int16,
            "/heading",
            self.face_heading,
            10
        )
        self.depth_sub = self.create_subscription(
            Float64,
            "/depth_state",
            self.depth,
            10
        )
        self.search()
    
    def evade(self, msg):
        self.current_heading = msg.data
        #turn right:
        self.target_heading = self.current_heading + 90.0
        

        #go down:
        self.current_depth = msg.data
        self.target_depth = self.current_depth - 0.75
        
        #side slide over:
        self.target_side = -50.0

        #go up:
        self.target_depth = self.current_depth

        #counter rotate back:
        self.current_heading = msg.data
        self.target_heading = self.current_heading - 90.0 

def main(args=None):
    rclpy.init(args=args)
    node = EvasiveSequence()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown() 
    

    
    

