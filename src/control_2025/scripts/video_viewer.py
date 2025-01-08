#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self, video_path):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.bridge = CvBridge()
        self.video_path = video_path
        self.cap = cv2.VideoCapture(self.video_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video file: {self.video_path}")
            raise RuntimeError("Failed to open video file")

        self.get_logger().info(f"Publishing video: {self.video_path}")
        self.timer = self.create_timer(0.04, self.publish_frame)  # 0.1 saniyede bir kare gönder

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video reached.")
            rclpy.shutdown()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info("Published a frame.")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    try:
        video_path = input("Please enter the path to the video file: ").strip()
        node = VideoPublisher(video_path)
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
