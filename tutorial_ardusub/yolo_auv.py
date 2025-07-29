import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge
import os
import time
import threading
from datetime import datetime
from roboflow import Roboflow

class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.rf = Roboflow(api_key="6LjEpf903x4FcUSJF3gG")
        self.project = self.rf.workspace("roboflow-58fyf").project("rock-paper-scissors-sxsw")
        self.model = self.project.version(11).model
        # Image capture setup
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera',
            self.image_callback,
            10
        )
        self.yolo_pub = self.create_publisher(Float64MultiArray, "/yolo_bbox", 10)

        self.get_logger().info("Camera capture node initialized.")


        # Class to integer mapping
        CLASS_MAP = {
            "full": 0,
            "front": 1,
            "back": 2,
            "side": 3
        }

        def image_callback(self, msg):
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            predictions = self.model.predict(image, confidence=40, overlap=30)

            msg_out = Float64MultiArray()

            if predictions:
                best = max(predictions, key=lambda p: p.confidence)
                class_id = CLASS_MAP.get(best.class_name, -1)  # fallback to -1 if unknown
                msg_out.data = [best.x, best.y, best.width, best.height, class_id]
            else:
                msg_out.data = [0.0, 0.0, 0.0, 0.0, -1]  # -1 = no prediction

            self.yolo_pub.publish(msg_out)
            self.get_logger().info(f"Published bbox: {msg_out.data}")



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