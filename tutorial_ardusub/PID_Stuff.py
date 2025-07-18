import sympy as sp
import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np
import math
import time


KP = 0
KI = 0
KD = 0
def maintain_depth(self, z, t):
        self.get_logger().info("starting function")
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time < rclpy.duration.Duration(seconds = t)) & rclpy.ok():
            msg = ManualControl()
            msg.z = z
            msg.t = t
            self.pub.publish(msg)
            # self.get_logger().info("Time: {:.2f}".format(self.get_clock().now() - start_time))
            time.sleep(.05)
        self.get_logger().info("ending function")
error = target_z - current_z

u = KP*error + KI*sp.integrate(error) + KD*sp.diff(error)