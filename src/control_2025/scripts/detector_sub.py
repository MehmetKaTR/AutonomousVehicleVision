#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from image_processing_2025.msg import SignList
from sensor_msgs.msg import Image,CompressedImage
from message_filters import Subscriber, ApproximateTimeSynchronizer, TimeSynchronizer
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from cv_bridge import CvBridge,CvBridgeError #Image mesaj tipinde gelen verinin görüntüye dönüştürülmesi veya tersi işlem için kullanılan köprünün eklenmesi
import cv2
import time
from simple_pid import PID
import numpy as np
from geometry_msgs.msg import Twist

class DetectedSignSubscriber(Node):
    def __init__(self):
        super().__init__('detected_sign_subscriber')
        Subscriber #oluştur
        self.qos_profile=QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.bridge=CvBridge()
        self.subscriber_=self.create_subscription(SignList,'detected_signs',self.signs_callback,self.qos_profile)

        self.subscriber_camera = Subscriber(self, CompressedImage,'camera_compressed',qos_profile=self.qos_profile)

        self.subscriber_lane_lines = Subscriber(self, CompressedImage,'lane_lines',qos_profile=self.qos_profile)

        self.subscriber_sign_list = Subscriber(self, SignList,'detected_signs',qos_profile=self.qos_profile)

        ts = TimeSynchronizer([
                               self.subscriber_camera,
                               self.subscriber_lane_lines, 
                                self.subscriber_sign_list
                                ]
                                , 10)
        self.pid = PID(0.005, 0, 0.001)

        self.msg=Twist()

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel",10)

        ts.registerCallback(self.listener_callback)

        self.pTime=time.time()

        self.get_logger().info("Camera subscription has started.") #Düğümün başladığına dair bildirim yapılması

    def signs_callback(self,msg):
        print(msg)
        print(1//(time.time()-self.pTime))
        self.pTime=time.time()

    def listener_callback(self, 
                          msg_camera, 
                          ll_seg_mask, 
                          msg_sign_list
                        ):
        
        try:
            #Görüntünün alınması ve boyutlandırılması
            camera_img = self.bridge.compressed_imgmsg_to_cv2(msg_camera, 'bgr8')
            ll_seg_mask=self.bridge.compressed_imgmsg_to_cv2(ll_seg_mask,'bgr8')
        except CvBridgeError as e:
            print(e)

        try:
            #Görüntünün alınması ve boyutlandırılması
            #cv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg_camera, 'bgr8'),
                                  #(1280, 720), interpolation=cv2.INTER_LINEAR)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_camera, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return  #Hata durumunda fonksiyonu sonlandır

        pTime = time.time()
        stop_sign_seen = False
        stop_sign_time = None
        ignore_stop_duration = 5 
        is_accelerated = False
        try:
            for detected in msg_sign_list.signs:
               #area = (detected.y_max- detected.y_min) * (detected.x_max- detected.x_min)
                print(f"Class name: {detected.sign_name}")
                if detected.sign_name == 32 and detected.distance < 10:  
                    is_accelerated = True
                    self.msg.linear.x = 0.0
                    self.msg.angular.z = 0.0
                elif detected.sign_name == 34 and detected.distance < 2:  
                    is_accelerated = True
                    self.msg.angular.z = 7.0
                elif detected.sign_name == 6 and detected.distance < 3: 
                    is_accelerated = False
                    self.msg.linear.x = 0.5
                    self.msg.angular.z = -7.0
                elif detected.sign_name == 22 and detected.distance < 3:
                    is_accelerated = False
                    self.msg.linear.x = 0.5
                    self.msg.angular.z = 7.0
                elif detected.sign_name == 33 and detected.distance < 2:
                    is_accelerated = False
                    self.msg.linear.x = 0.5
                    self.msg.angular.z = -7.0
                elif detected.sign_name == 14 and detected.distance < 3:
                    if not stop_sign_seen:  # İlk kez dur tabelasını görüyoruz
                        is_accelerated = False
                        self.msg.linear.x = 0.0
                        self.msg.angular.z = 0.0
                        stop_sign_seen = True  # Dur tabelasını gördüğümüzü işaretle
                        stop_sign_time = time.time()  # Dur tabelasını gördüğümüz anın zamanını kaydet
                    elif time.time() - stop_sign_time > ignore_stop_duration:  # Belirli bir süre geçtiyse
                        # Dur tabelasından sonra harekete başlama
                        is_accelerated = False
                        self.msg.linear.x = 0.5
                        self.msg.angular.z = 0.0
                        stop_sign_seen = False  # Dur tabelasını görmemeyi başardık, bayrağı sıfırla
                        stop_sign_time = None  # Zamanlayıcıyı sıfırla
                elif detected.sign_name == 39 and detected.distance < 3:
                    is_accelerated = False
                    self.msg.angular.z = 2.0
                elif detected.sign_name == 38 and detected.distance < 3:
                    is_accelerated = False
                    self.msg.angular.z = -2.0
        except Exception as e:
            print(f"YOLO processing error: {e}")
        try:
            contours, _ = cv2.findContours((ll_seg_mask > 0.5).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            lanes, close_lanes = [[0, 0], [0, 0]], [999, 999]
            height, width, _ = cv_image.shape
            mask_width = ll_seg_mask.shape[1]
            mean = mask_width / 2

            for contour in contours:
                if cv2.contourArea(contour) < 50:
                    continue
                points = contour[:, 0, :]
                if len(contour) < 200:
                    continue
                print(len(contour))
                min_x = np.mean(points[:, 0])
                min_y = np.mean(points[:, 1])
                max_y = np.max(points[:, 1])
                if close_lanes[0] > abs(min_x - mean) + abs(max_y - ll_seg_mask.shape[0]):
                    close_lanes[1] = close_lanes[0]
                    lanes[1] = lanes[0]
                    close_lanes[0] = abs(min_x - mean) + abs(max_y - ll_seg_mask.shape[0])
                    lanes[0] = [min_x, min_y]
                elif close_lanes[1] > abs(min_x - mean) + abs(max_y - ll_seg_mask.shape[0]):
                    close_lanes[1] = abs(min_x - mean) + abs(max_y - ll_seg_mask.shape[0])
                    lanes[1] = [min_x, min_y]

            midpoint_x = int((lanes[0][0] + lanes[1][0]) / 2 * (width / mask_width))
            midpoint_y = int((lanes[0][1] + lanes[1][1]) / 2 * (height / ll_seg_mask.shape[0]))
            cv2.circle(cv_image, (midpoint_x, midpoint_y), 7, (255, 255, 255), -1)
            control = self.pid((width // 2) - midpoint_x)
            #Hareket Kontrolü
            if not is_accelerated:
                self.msg.linear.x = 2.0
                self.msg.angular.z = -float(control)

            self.publisher_.publish(self.msg)
        except Exception as e:
            print(f"Lane detection error: {e}")
            control = 0

        

        cv2.imshow("camera_img",cv_image)
        cv2.waitKey(1)

        print(1//(time.time()-self.pTime))
        self.pTime=time.time()
        for idx, sign in enumerate(msg_sign_list.signs):
            self.get_logger().info(
                f'Sign {idx + 1}: Name: {sign.sign_name}, '
                f'Bounding Box: [{sign.x_min}, {sign.y_min}, {sign.x_max}, {sign.y_max}]'
            )

def main(args=None):
    rclpy.init(args=args)
    node = DetectedSignSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
