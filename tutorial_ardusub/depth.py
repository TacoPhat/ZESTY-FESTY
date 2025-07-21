import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16, Float64
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
            self.depth, 
            10             
        )
        self.create_subscription(
            Float64,        
            "/desired_depth",    
            self.update_target_depth, 
            10             
        )
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.target_depth = -3.0
        self.error_accumulator = 0.0
        self.previous_error = 0.0
        self.timestep = 0.02
    def update_target_depth(self, msg):
            self.target_depth = msg.data
            self.get_logger().info(f"[TARGET SET] New target depth: {self.target_depth:.2f}")

    def calculate_depth(self, fp, d=1000, g=9.81, atmospheric_pressure=101325.0):
        return -(fp - atmospheric_pressure) / (d * g)
        
    def depth(self, msg):
        self.get_logger().info(f"Descending to {self.target_depth}")
        current_depth = self.calculate_depth(msg.fluid_pressure)
        error = (self.target_depth - current_depth)
        self.error_accumulator += error * self.timestep
        derivative = (error - self.previous_error) / self.timestep
        Kp = 50.0
        Ki = 0.3
        Kd = 2.0
        integral = min(Ki * self.error_accumulator, 1.0)
        u = Kp * error + integral + Kd * derivative
        u = max(-100, min(u, 100))
        self.previous_error = error
        move_msg = ManualControl()
        move_msg.z = float(u)
        self.pub.publish(move_msg)
        self.get_logger().info(f"Current: {current_depth}, Target: {self.target_depth}, Cmd: {u:.2f}")

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