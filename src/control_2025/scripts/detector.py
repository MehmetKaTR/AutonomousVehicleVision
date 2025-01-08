#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_2025.msg import DetectedSign, SignList

class DetectedSignPublisher(Node):
    def __init__(self):
        super().__init__('detected_sign_publisher')
        # Publisher oluştur
        self.publisher_ = self.create_publisher(SignList, 'detected_signs_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_detected_signs)  # 1 saniyede bir veri yay
        self.get_logger().info('DetectedSignPublisher is running...')

    def publish_detected_signs(self):
        detected_sign1 = DetectedSign()
        detected_sign1.sign_name = "Stop"
        detected_sign1.x_min = 100.0
        detected_sign1.y_min = 150.0
        detected_sign1.x_max = 200.0
        detected_sign1.y_max = 250.0

        detected_sign2 = DetectedSign()
        detected_sign2.sign_name = "Speed Limit 50"
        detected_sign2.x_min = 300.0
        detected_sign2.y_min = 350.0
        detected_sign2.x_max = 400.0
        detected_sign2.y_max = 450.0

        # Ana mesaj
        detected_signs_msg = SignList()
        detected_signs_msg.signs = [detected_sign1, detected_sign2]

        # Mesajı yayınla
        self.publisher_.publish(detected_signs_msg)
        self.get_logger().info('Published: %s' % str(detected_signs_msg))

def main(args=None):
    rclpy.init(args=args)
    node = DetectedSignPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
