#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.image_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return 
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(image)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()