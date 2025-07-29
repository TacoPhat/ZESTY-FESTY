import rclpy
from rclpy.node import Node
from rclpy.timer import Rate
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int16, Float64, Float64MultiArray, Bool
from mavros_msgs.msg import OverrideRCIn, ManualControl
import numpy as np
import math
import time

class movenode(Node):
    def __init__(self):
        super().__init__("bluerow_control")
        self.target_depth = -1.0
        self.target_heading = 245.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_fov = 0.0
        self.current_heading = 0.0
        self.current_light = False
        self.depth_tolerance = 0.05
        self.heading_tolerance = 2.0
        self.timestep = 0.1
        self.previousfov_error = 0.0
        self.previousdepth_error = 0.0
        self.previousr_error = 0.0
        self.errorfov_accumulator = 0.0
        self.errordepth_accumulator = 0.0
        self.errorr_accumulator = 0.0
        self.rotate_case = False
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
        self.movex_pub = self.create_publisher(
            Float64,
            "/desired_forward",
            10
        )
        self.movey_pub = self.create_publisher(
            Float64,
            "/desired_side",
            10
        )
        self.attack_pub = self.create_publisher(
            Bool,
            "/attack",
            10
        )
        self.light_pub = self.create_publisher(
            OverrideRCIn, 
            '/mavros/rc/override',
             10
        )
        self.create_subscription(Float64, "/desired_depth", self.update_target_depth, 10)
        self.create_subscription(Float64, "/desired_heading", self.update_target_heading, 10)
        self.create_subscription(Float64, "/desired_forward", self.update_target_forward, 10)
        self.create_subscription(Float64, "/desired_side", self.update_target_side, 10)
        self.create_subscription(Bool, "/attack", self.update_target_light, 10)
        self.create_subscription(Float64MultiArray, "/current_yolo", self.convert_yolo, 10)
        self.create_subscription(Float64, "/depth_state", self.depth_state_callback, 10)
        self.create_subscription(Int16, "/heading", self.heading_state_callback, 10)

        self.perform()
    def update_target_depth(self, msg):
            self.target_depth = msg.data
            # self.get_logger().info(f"Depth: {self.target_depth:.2f}")
    def update_target_heading(self, msg):
            self.target_heading = msg.data
            # self.get_logger().info(f"Heading: {self.target_heading:.2f}")
    def update_target_forward(self, msg):
            self.target_forward = msg.data
            # self.get_logger().info(f"Forward: {self.target_forward:.2f}")
    def update_target_side(self, msg):
            self.target_side = msg.data
            # self.get_logger().info(f"Side: {self.target_side:.2f}")
    def update_target_light(self, msg):
            self.target_light = msg.data
            # self.get_logger().info(f"Light: {self.target_depth:.2f}")
    def depth_state_callback(self, msg):
            self.current_depth = msg.data
    def heading_state_callback(self, msg):
            self.current_heading = msg.data
    def wait_until_reached(self, target, getter_fn, tolerance, name=""):
        while abs(getter_fn() - target) > tolerance:
            self.get_logger().info(f"Waiting for {name}: {getter_fn():.2f} → {target:.2f}")
            rclpy.spin_once(self, timeout_sec=0.1)
    def search_sequence(self):
        self.rotate_case = True
        # self.target_heading = self.current_heading + 20.0
        self.target_depth = -0.5
        # while y < 3.0:
        #     self.target_heading = self.current_heading - 120.0
        #     y+=1.0
        # self.target_depth = 0.5
    def evasive_sequence(self):
        self.rotate_case = False
         #turn right:
        self.target_heading = self.current_heading + 90.0
        #go down:
        self.target_depth = self.current_depth - 0.75
        #side slide over:
        self.target_side = -50.0
        #go up:
        self.target_depth = self.current_depth
        #counter rotate back:
        self.target_heading = self.current_heading - 90.0 

    def attack_sequence(self):
        self.rotate_case = False
        #self.target_forward = 60.0
        light_cmd = OverrideRCIn()
        light_cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        light_cmd.channels[8] = 1900
        light_cmd.channels[9] = 1900
        self.light_pub.publish(light_cmd)

        # self.get_logger().info("Valid dist; Flashing lights")

    def yolodistance(self, w, h): #c = 0: full, 1:front, 2:back, 3:side
        d_pixels = np.sqrt((w)**2 + (h)**2)
        D_real = 0.5183 
        focal_length = 640
        Z = (D_real * focal_length) / d_pixels
        return Z
        
    def yolodist_pid(self, w, h): #c = 0: full, 1:front, 2:back, 3:side
        dist = self.yolodistance(self, w, h)
        target_dist = 2.0
        error = (target_dist - dist)
        self.errordepth_accumulator += error * self.timestep
        derivative = (error - self.previousdepth_error) / self.timestep
        Kp = 60.0
        Ki = 8.0
        Kd = 0.0
        integral = min(Ki * self.errordepth_accumulator, 1.0)
        u = Kp * error + integral + Kd * derivative
        u = max(-100, min(u, 100))
        self.previousdepth_error = error
        return u #in x-direction
    def yolodepth_pid(self, y):
        target_y = 240
        error = (target_y - y)
        self.errordepth_accumulator += error * self.timestep
        derivative = (error - self.previousdepth_error) / self.timestep
        Kp = 60.0
        Ki = 8.0
        Kd = 0.0
        integral = min(Ki * self.errordepth_accumulator, 1.0)
        u = Kp * error + integral + Kd * derivative
        u = max(-100, min(u, 100))
        self.previousdepth_error = error
        return u
    def convertXtoHeading(self, x, y):
        x_new = x - 320
        r = -np.atan(x_new/y)
        return r
    def yolorotate_pid(self, x):
        current_heading = self.convertXtoHeading(x)
        target_heading = 0.0
        error = (target_heading - current_heading + 180) % 360 - 180
        self.errorr_accumulator += error * self.timestep
        derivative = (error - self.previousr_error) / self.timestep
        Kp = 1.1
        Ki = 0.01
        Kd = 0.1
        integral = min(Ki * self.errorr_accumulator, 1.0)
        u = Kp * error + integral + Kd * derivative
        u = max(-100, min(u, 100))
        self.previousr_error = error
        return u
    def decision_yolo(self, x, y, w, h, c):
        if c == -1:
            self.get_logger().info("Sending search sequence.")
            self.search_sequence()
            #search sequence
        else:
            self.rotate_case = False
            distance = self.yolodistance(w, h)
            if distance > 2.0:
                uy = self.yolodepth_pid(y)
                self.target_depth = uy
                ur = self.yolorotate_pid(x, y)
                self.target_heading = ur
                uz = self.yolofov_pid(w, h, c)
                self.target_forward = uz
                #lock on sequence
            elif distance <= 2.0:
                if c == 0:
                    uy = self.yolodepth_pid(y)
                    self.target_depth = uy
                    ur = self.yolorotate_pid(x, y)
                    self.target_heading = ur
                    uz = self.yolofov_pid(w, h, c)
                    self.target_forward = uz
                    #lock on sequence
                if c == 1:
                    self.evasive_sequence()
                    self.get_logger().info("Running evasive sequence.")
                    #evasive sequence
                if c == 2:
                    self.attack_sequence()
                    self.get_logger().info("Running attack sequence.")
                    #attack sequence
                if c == 3:
                    self.target_heading = ur
                    self.target_forward = 0.0
                    #observe sequence

         
    def convert_yolo(self, msg):
            x, y, w, h, c = msg.data
            if x == 0.0 and y == 0.0 and w == 0.0 and h == 0.0:
                self.get_logger().info("No detections")
            else:
                self.get_logger().info(f"Received bbox → x:{x}, y:{y}, w:{w}, h:{h}")
                self.decision_yolo(x, y, w, h, c)
    def perform(self):
        self.get_logger().info("Starting continuous movement control loop")
        self.search_sequence()
        while rclpy.ok():
            self.depth_pub.publish(Float64(data=self.target_depth))
            self.heading_pub.publish(Float64(data=self.target_heading))
            self.movex_pub.publish(Float64(data=self.target_x))
            self.movey_pub.publish(Float64(data=self.target_y))
            # self.get_logger().info(
            #     f"Sending → Depth: {self.target_depth:.2f}, Heading: {self.target_heading:.2f}, "
            #     f"Forward: {self.target_x:.2f}, Side: {self.target_y:.2f}"
            # )
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
    

def main(args=None):
    rclpy.init(args=args)
    node = movenode()
    try:
        node.perform()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
