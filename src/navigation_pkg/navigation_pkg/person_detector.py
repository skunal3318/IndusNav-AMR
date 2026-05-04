#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class DetectorNode(Node):
    def __init__(self):
        super().__init__('person_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.alerter = self.create_publisher(
            String,
            '/person_detected',
            10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)

        person_found = False
        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls == 0 and conf > 0.2:
                person_found = True
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                label = f'Person {conf:.2f}'
                cv2.putText(cv_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                alert = String()
                alert.data = f'Human detected: {conf:.2f}'
                self.get_logger().info(f'Person Detected: {conf:.2f}', throttle_duration_sec=5.0)
                self.alerter.publish(alert)


        status_text = 'Person Detected' if person_found else 'MONITORING...'
        status_color = (0, 0, 255) if person_found else (0, 255, 0)
        cv2.putText(cv_image, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)

        cv2.imshow('AMR Person Detection', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()