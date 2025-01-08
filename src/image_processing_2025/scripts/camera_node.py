#!/usr/bin/env python3.10

#ROS için gerekli kütüphanelerin eklenmesi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage #Görüntüyü aktarabilmek için kullanılan mesaj tipi Image'in eklenmesi
from cv_bridge import CvBridge,CvBridgeError #Image mesaj tipinde gelen verinin görüntüye dönüştürülmesi veya tersi işlem için kullanılan köprünün eklenmesi
import cv2 #Görüntünün ekranda gösterilebilmesi için OpenCV'nin eklenmesi
import numpy as np


class CameraNode(Node):
    def __init__(self):
        #ROS düğümünün başlatılması için gerekli kodlar # Bir kez çalışır
        super().__init__("camera_node")

        self.qos_profile=QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        #Arabadaki stereo kameranın bir tanesine düğümün abone edilmesi
        self.subscriber_ = self.create_subscription(CompressedImage, #Düğümün bağlandığı konudaki mesajın tipi
                                                    "/camera_compressed", #Düğümün bağlandığı konu 
                                                    self.callback_camera, #Her mesaj geldiğinde gidilecek fonksiyon
                                                    qos_profile=self.qos_profile) #Gelen mesajlar için oluşturulan sıra sayısı    

        cv2.namedWindow("Control Panel")

        cv2.createTrackbar("TL-X", "Control Panel", 222, 640, self.nothing)
        cv2.createTrackbar("TL-Y", "Control Panel", 387, 480, self.nothing)
        cv2.createTrackbar("BL-X", "Control Panel", 70, 640, self.nothing)
        cv2.createTrackbar("BL-Y", "Control Panel", 472, 480, self.nothing)
        cv2.createTrackbar("TR-X", "Control Panel", 400, 1280, self.nothing)
        cv2.createTrackbar("TR-Y", "Control Panel", 380, 480, self.nothing)
        cv2.createTrackbar("BR-X", "Control Panel", 538, 1280, self.nothing)
        cv2.createTrackbar("BR-Y", "Control Panel", 472, 480, self.nothing)

        cv2.setTrackbarMin("TL-X", "Control Panel", -640)

        cv2.setTrackbarMin("BL-X", "Control Panel", -640)

        self.bridge=CvBridge() #Gelen görüntü için dönüşüm köprüsünün tanımlanması

        self.get_logger().info("Camera subscription has started.") #Düğümün başladığına dair bildirim yapılması

    def callback_camera(self, msg):
        try:
            #Görüntünün alınması ve küçültülmesi
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        except CvBridgeError as e:
            print(e)

        frame = cv2.resize(cv_image, (640, 480))
            
            
        #### KOD YAZILABİLECEK ARALIK BAŞLANGICI ####

        tl_x = cv2.getTrackbarPos("TL-X", "Control Panel")
        tl_y = cv2.getTrackbarPos("TL-Y", "Control Panel")
        bl_x = cv2.getTrackbarPos("BL-X", "Control Panel")
        bl_y = cv2.getTrackbarPos("BL-Y", "Control Panel")
        tr_x = cv2.getTrackbarPos("TR-X", "Control Panel")
        tr_y = cv2.getTrackbarPos("TR-Y", "Control Panel")
        br_x = cv2.getTrackbarPos("BR-X", "Control Panel")
        br_y = cv2.getTrackbarPos("BR-Y", "Control Panel")

        tl = (tl_x, tl_y)
        bl = (bl_x, bl_y)
        tr = (tr_x, tr_y)
        br = (br_x, br_y)

        # Noktaları çiz
        cv2.circle(frame, tl, 5, (0, 0, 255), -1)
        cv2.circle(frame, bl, 5, (0, 0, 255), -1)
        cv2.circle(frame, tr, 5, (0, 0, 255), -1)
        cv2.circle(frame, br, 5, (0, 0, 255), -1)

        # Perspektif dönüşüm
        pts1 = np.float32([tl, bl, tr, br])
        pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        transformed_frame = cv2.warpPerspective(frame, matrix, (640, 480))

        # Görüntüleri birleştir
        combined_frame = cv2.hconcat([frame, transformed_frame])
        
                    
        cv2.imshow("img",combined_frame) #Görüntünün ekrana gösterilmesi
        
        key=cv2.waitKey(1) #Görüntünün ekranda kalması için 1ms'lik key bekleme ve gecikme
        
        #### KOD YAZILABİLECEK ARALIK SONU ####

    def nothing(self):
        pass
            


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
