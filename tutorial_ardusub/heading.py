import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int16, Float64
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np
import math
import time

class headingnode(Node):
    def __init__(self):
        super().__init__("bluerow_heading")
        self.target_heading = 0
        self.error_accumulator = 0.0
        self.previous_error = 0.0
        self.timestep = 0.01
        self.rate = self.create_rate(10.0)
        self.manualcontrol_pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.sub = self.create_subscription(
            Int16,
            "/heading",
            self.face_heading,
            10
        )
        self.create_subscription(
            Float64,
            "/desired_heading",
            self.update_target_heading,
            10
        )
    def update_target_heading(self, msg):
            self.target_heading = msg.data
            self.get_logger().info(f"[TARGET SET] New target heading: {self.target_heading:.2f}")

    def face_heading(self, msg):
        self.get_logger().info(f"Rotating to {self.target_heading}")
        current_heading = msg.data
        error = (self.target_heading - current_heading + 180) % 360 - 180
        self.error_accumulator += error * self.timestep
        derivative = (error - self.previous_error) / self.timestep
        Kp = 1.1
        Ki = 0.01
        Kd = 0.1
        integral = min(Ki * self.error_accumulator, 1.0)
        u = Kp * error + integral + Kd * derivative
        u = max(-100, min(u, 100))
        self.previous_error = error
        move_msg = ManualControl()
        move_msg.r = float(u)
        self.manualcontrol_pub.publish(move_msg)
        self.get_logger().info(f"Current: {current_heading}, Target: {self.target_heading}, Cmd: {u:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = headingnode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()