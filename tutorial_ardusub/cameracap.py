import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import time

class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.save_dir = './dataset'
        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera',
            self.image_callback,
            10
        )
        self.last_saved_time = time.time()
        self.get_logger().info("Camera capture node initialized. Saving images every 2 seconds.")

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= 2.0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = os.path.join(self.save_dir, f'image_{timestamp}.jpg')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Saved image: {filename}")
                self.last_saved_time = current_time
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera capture node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

