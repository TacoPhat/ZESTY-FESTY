import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np
import math
import time

class publish(Node):
    def __init__(self):
        super().__init__("bluerow_publisher")
        self.rate = self.create_rate(10.0)
        self.pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.command_pub = self.create_publisher(
            OverrideRCIn, "override_rc", 10
        )
        self.dance()
    def turn_light1_on(self, level):
        self.get_logger().info(f"Turning light 1 on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        self.command_pub.publish(commands)
    def turn_light2_on(self, level):
        self.get_logger().info(f"Turning light 2 on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[9] = 1000 + level * 10
        self.command_pub.publish(commands)
    def send_manual_control(self, x, y, z, r, t):
        self.get_logger().info("starting function")
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time < rclpy.duration.Duration(seconds = t)) & rclpy.ok():
            msg = ManualControl()
            msg.x = x
            msg.y = y
            msg.z = z
            msg.r = r
            msg.t = t
            self.pub.publish(msg)
            # self.get_logger().info("Time: {:.2f}".format(self.get_clock().now() - start_time))
            time.sleep(.05)
        self.get_logger().info("ending function")


    def dance(self):
        self.send_manual_control(0.0, 0.0, -70.0, 0.0, 1.44)
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0,40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, 0.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, 65.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, -60.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, 0.0, 40.0, 0.0, 1.44)
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0,40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(50.0, 0.0, -60.0, 0.0, 1.44)
        self.send_manual_control(-50.0, 0.0, -40.0, 0.0, 1.44)
        self.send_manual_control(5.0, 5.0, 50.0, 30.0, 1.44) 
        self.send_manual_control(5.0, 5.0, 50.0, 20.0, 1.44) 
        self.send_manual_control(-5.0, -5.0, 50.0, 10.0, 1.44) 
        self.send_manual_control(-5.0, -5.0, 0.0, -5.0, 1.44) 
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44) 
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44) 
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44) 
        self.send_manual_control(-40.0, 40.0, 0.0, 0.0, 1.44) 
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44) 
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44) 
        self.send_manual_control(0.0, 0.0, 60.0, 0.0, 1.44) 
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(10.0, 0.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, -50.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, 50.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, -40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(-40.0, 40.0, 0.0, 0.0, 1.44)
        self.send_manual_control(20.0, 0.0, 0.0, 0.0, 1.44)
        self.send_manual_control(0.0, 0.0, 0.0, -80.0, 2.88)
        self.send_manual_control(0.0, 0.0, 80.0, 80.0, 4.32)

    

def main(args=None):
    rclpy.init(args=args)
    node = publish()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
