import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from rclpy import timer


class Z_axis_PID(Node):
    def __init__(self):
        super().__init__("bluerow_publisher")
        self.pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.sub = self.create_subscription(
            FluidPressure,        
            "/pressure",    
            self.calculate_depth, 
            10             
        )
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.error_accumulator = 0.0
        self.previous_error = 0.0
        self.maintain_depth(-3.0,120)
    def calculate_depth(self, msg: FluidPressure, d = 1000, g = 9.81, atmospheric_prssure = 101325.0):
        self.current_z = (msg.fluid_pressure)/(d*g) + atmospheric_prssure
        
    def depth(self, z, t):
            self.get_logger().info("starting function")
            start_time = self.get_clock().now()
            prev_time = start_time
            while (self.get_clock().now() - start_time < rclpy.duration.Duration(seconds = t)) & rclpy.ok():
                error = z - self.current_z
                proportional = self.Kp * error
                dt = self.get_clock.now() - prev_time
                error_accumulator += error * dt
                integral = min(self.Ki * error_accumulator, 1.0)
                derivative = self.Kd * (error - previous_error) / dt
                previous_error = error
                u = proportional + integral + derivative
                msg = ManualControl()
                msg.z = u
                msg.t = t
                self.pub.publish(msg)
                self.prev_time = self.get_clock().now()
                timer.sleep(0.05)
            self.get_logger().info("ending function")

def main(args=None):
        rclpy.init(args=args)
        node = Z_axis_PID()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt received, shutting down...")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()