import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np
import math
import time

class dance_node(Node):
    def __init__(self):
        super().__init__("bluerow_publisher")
        self.rate = self.create_rate(10.0)
        self.manualcontrol_pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.dance()

    def send_manual_control(self, x, y, z, r, t):
        self.get_logger().info("starting function")
        start_time = self.get_clock().now()
        self.turn_light1_on(100)
        self.turn_light2_on(100)
        while (self.get_clock().now() - start_time < rclpy.duration.Duration(seconds = t)) & rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            msg = ManualControl()
            msg.x = x
            msg.y = y
            msg.z = z
            msg.r = r
            self.manualcontrol_pub.publish(msg)
            # self.get_logger().info("Time: {:.2f}".format(self.get_clock().now() - start_time))
            time.sleep(.02)
        self.turn_light1_on(20)
        self.turn_light2_on(20)
        self.get_logger().info("ending function")


    def dance(self):
        self.send_manual_control(0.0, 0.0, -70.0, 0.0, 1.44)
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0,40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, 60.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, -60.0, 0.0, 0.0, 1.44)
        self.send_manual_control(5.0, 5.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, 0.0, 40.0, 0.0, 1.44)
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0,40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(50.0, 0.0, -60.0, 0.0, 1.44)
        self.send_manual_control(-50.0, 0.0, -60.0, 0.0, 1.44)
        self.send_manual_control(10.0, 10.0, 50.0, 80.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 50.0, 80.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 40.0, 60.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 30.0, 45.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 20.0, 35.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 0.0, 25.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 0.0, 15.0, 1.44) 
        self.send_manual_control(10.0, 10.0, 0.0, 5.0, 1.44) 
        self.send_manual_control(5.0, 5.0, 0.0, 2.0, 1.44) 
        self.send_manual_control(3.0, 3.0, 0.0, 0.00000001, 1.44) 
        self.send_manual_control(0.0, 0.0, 60.0, -0.00000001, 1.44) 
        self.send_manual_control(40.0, 40.0, 0.0, 0.00000001, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, -0.00000001, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.00000001, 1.44)
        self.send_manual_control(-40.0, 40.0, 0.0, -0.00000001, 1.44)
        self.send_manual_control(10.0, 0.0, 0.0, 0.00000001, 1.44)
        self.send_manual_control(0.0, -50.0, 0.0, -0.00000001, 1.44)
        self.send_manual_control(0.0, 50.0, 0.0, 0.00000001, 1.44)
        self.send_manual_control(40.0, 40.0, 0.0, -0.00000001, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.00000001, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, -0.00000001, 1.44)
        self.send_manual_control(-40.0, 40.0, 0.0, 0.00000001, 1.44)
        self.send_manual_control(20.0, 0.0, 0.0, -0.00000001, 1.44)
        self.send_manual_control(0.0, 0.0, 0.0, -80.0, 2.88)
        self.send_manual_control(0.0, 0.0, 80.0, 80.0, 4.32)
        self.send_manual_control(0.0, 0.0, 0.0, 0.0, 1.0)


    

def main(args=None):
    rclpy.init(args=args)
    node = dance_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
