import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from mavros_msgs.msg import ManualControl, OverrideRCIn
from cv_bridge import CvBridge
from dt_apriltags import Detector

class AprilTagDetectionController(Node):
    def __init__(self):
        super().__init__('apriltag_controller')
        
        self.valid_ids = {1, 2}  # Only respond to tags with these IDs
        self.tag_size = 0.1
        self.camera_params = (273.25, 261.76, 307.89, 153.84)

        self.image_sub = self.create_subscription(
            Image, '/bluerov2/camera', self.image_callback, 10
        )

        # Publishers
        self.heading_pub = self.create_publisher(
            Float64, 
            '/desired_heading', 
            10)
        self.attack_pub = self.create_publisher(
            Bool,
            "/attack",
            10
        )

        self.bridge = CvBridge()
        self.at_detector = Detector(families='tag36h11')

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        tags = self.at_detector.detect(
            cv_img,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size
        )

        closest_tag = None
        min_distance = float('inf')

        for tag in tags:
            if tag.tag_id not in self.valid_ids:
                continue

            t = tag.pose_t
            distance = np.linalg.norm(t)
            R = tag.pose_R
            yaw = np.arctan2(R[1, 0], R[0, 0])

            if distance < min_distance:
                min_distance = distance
                closest_tag = {
                    'id': tag.tag_id,
                    'distance': distance,
                    'yaw': yaw
                }

        if closest_tag:
            self.get_logger().info(f"Tracking Tag ID {closest_tag['id']} at {closest_tag['distance']:.2f} m")
            yaw = closest_tag['yaw']
            if closest_tag['distance'] <= 1.0:
                self.light_pub.publish(True)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
