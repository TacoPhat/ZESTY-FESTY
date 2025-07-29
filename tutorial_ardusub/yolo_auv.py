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
import cv2
import uuid

CLASS_MAP = {
    "full": 0,
    "front": 1,
    "back": 2,
    "side": 3
}
class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.rf = Roboflow(api_key="6LjEpf903x4FcUSJF3gG")
        self.model = self.rf.workspace().project("auv-yolo-ssqex").version(1).model
        # Image capture setup
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera',
            self.image_callback,
            10
        )
        # Video writer setup
        self.frame_width = 640
        self.frame_height = 480
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(
            './vid/output.avi', fourcc, 20.0,
            (self.frame_width, self.frame_height)
        )
        self.yolo_pub = self.create_publisher(Float64MultiArray, "/yolo_bbox", 10)

        self.get_logger().info("Camera capture node initialized.")

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (self.frame_width, self.frame_height))
        predictions = self.model.predict(image, confidence=40, overlap=30)
        msg_out = Float64MultiArray()
        if predictions:
            first = predictions[0]
            class_id = CLASS_MAP.get(first['class'], -1)
            msg_out.data = [float(first['x']), float(first['y']), float(first['width']), float(class_id) ]
            x, y = int(first['x']), int(first['y'])
            w, h = int(first['width']), int(first['height'])
            class_name = first['class']
            class_id = CLASS_MAP.get(class_name, -1)

            x, y = int(first['x']), int(first['y'])
            w, h = int(first['width']), int(first['height'])
            x1, y1 = int(x - w / 2), int(y - h / 2)
            x2, y2 = int(x + w / 2), int(y + h / 2)
            class_name = first['class']
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, class_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            msg_out.data = [0.0, 0.0, 0.0, 0.0, -1.0]

        self.yolo_pub.publish(msg_out)
        self.get_logger().info(f"Published bbox: {msg_out.data}")
        self.video_writer.write(image)

    def destroy_node(self):
        # Release video writer when node shuts down
        self.video_writer.release()
        super().destroy_node()


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