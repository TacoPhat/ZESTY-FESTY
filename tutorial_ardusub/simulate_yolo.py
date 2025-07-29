import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import os
from roboflow import Roboflow

class CameraSimulatorNode(Node):
    def __init__(self, image_folder, publish_rate=1.0):
        super().__init__('camera_simulator_node')
        self.publisher_ = self.create_publisher(Image, '/bluerov2/camera', 10)
        self.bbox_publisher = self.create_publisher(Float32MultiArray, '/bluerov2/bbox', 10)
        self.bridge = CvBridge()

        self.image_folder = image_folder
        self.images = sorted([os.path.join(image_folder, f) for f in os.listdir(image_folder)
                              if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
        self.index = 0

        # Load your own trained Roboflow model
        rf = Roboflow(api_key="6LjEpf903x4FcUSJF3gG")
        self.model = rf.workspace().project("auv-yolo-ssqex").version(1).model

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info("Camera simulator node initialized.")

    def timer_callback(self):
        if not self.images:
            self.get_logger().warn("No images found in folder.")
            return

        img_path = self.images[self.index]
        frame = cv2.imread(img_path)
        if frame is None:
            self.get_logger().warn(f"Could not read image: {img_path}")
            return

        # Run inference with Roboflow model
        result = self.model.predict(img_path, confidence=40, overlap=30).json()
        predictions = result.get("predictions", [])

        # Draw boxes
        if predictions:
            for pred in predictions:
                x, y, w, h = int(pred['x']), int(pred['y']), int(pred['width']), int(pred['height'])
                label = pred['class']
                # cv2.rectangle(frame, (x - w//2, y - h//2), (x + w//2, y + h//2), (0, 255, 0), 2)
                # cv2.putText(frame, label, (x - w//2, y - h//2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            # Only publishing the first detection
            first = predictions[0]
            bbox_data = [float(first['x']), float(first['y']), float(first['width']), float(first['height'])]
        else:
            bbox_data = [0.0, 0.0, 0.0, 0.0]

        # Publish image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published simulated image: {img_path}")

        # Publish bounding box
        bbox_msg = Float32MultiArray(data=bbox_data)
        self.bbox_publisher.publish(bbox_msg)
        self.get_logger().info(f"Published bbox: {bbox_data}")

        # Save or show the frame with boxes (optional)
        # cv2.imwrite(f"./yolo_test_output/{os.path.basename(img_path)}", frame)
        # cv2.imshow("Detection", frame); cv2.waitKey(1)              # Show

        self.index = (self.index + 1) % len(self.images)

def main(args=None):
    rclpy.init(args=args)
    image_folder = "./dataset2"
    node = CameraSimulatorNode(image_folder=image_folder, publish_rate=10.0)  # 1 Hz
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera simulator node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
