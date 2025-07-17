import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import BatteryState, Imu
import numpy as np
import math

class subscribe(Node):
    def __init__(self):
        super().__init__("bluerow_subscriber")    # names the node when running

        self.sub = self.create_subscription(
            BatteryState,        # the message type
            "/battery_state",    # the topic name,
            self.log_batterystat,  # the subscription's callback method
            10              # QOS (will be covered later)
        )
        self.sub = self.create_subscription(
            Imu,        # the message type
            "/imu",    # the topic name,
            self.log_imu,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized subscriber node")

    def log_batterystat(self, msg):
        self.get_logger().info(f" Voltage  + {msg.voltage}")
        self.get_logger().info(f" Percentage  + {msg.percentage}")
        if(msg.voltage <= 3.5):
            self.get_logger().info("Warning. Below 3.5 voltage.")
    def log_imu(self, msg):
        self.get_logger().info(f" Orientation  + {msg.orientation}")
        self.get_logger().info(f" Angular_velocity  + {msg.angular_velocity}")
        self.get_logger().info(f"Linear_acceleration  + {msg.linear_acceleration}")


def main(args=None):
    rclpy.init(args=args)
    node = subscribe()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()