#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoViewer(Node):
    def __init__(self):
        super().__init__('video_viewer')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',  # Publisher'daki ile aynı konu adı olmalı
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # ROS 2 mesajını OpenCV görüntüsüne dönüştür
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # OpenCV ile görüntüyü ekranda göster
        cv2.imshow("Video Frame", frame)

        # 'q' tuşuna basıldığında pencereyi kapat
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down viewer.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = VideoViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
