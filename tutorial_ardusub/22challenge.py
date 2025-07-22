import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int16, Float64
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np
import math
import time

class movenode(Node):
    def __init__(self):
        super().__init__("bluerow_control")
        self.target_depth = -2.0
        self.target_heading = 200.0
        self.target_force = 5.0
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
        self.move_pub = self.create_publisher(
            Float64,
            "/desired_forward",
            10
        )
        self.perform()

    def perform(self):
        self.get_logger().info(f"Descending to {self.target_depth}")
        msg = Float64()
        msg.data = self.target_depth
        self.depth_pub.publish(msg)
        time.sleep(5)
        self.get_logger().info(f"Rotating to {self.target_heading}")
        msg = Float64()
        msg.data = self.target_heading
        self.heading_pub.publish(msg)
        time.sleep(5)
        self.get_logger().info(f"Forward force of {self.target_force}")
        msg = Float64()
        msg.data = self.target_force
        self.move_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = movenode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()