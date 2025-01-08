#!/usr/bin/env python3.10

#ROS için gerekli kütüphanelerin eklenmesi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage,Image #Görüntüyü aktarabilmek için kullanılan mesaj tipi Image'in eklenmesi
from cv_bridge import CvBridge,CvBridgeError #Image mesaj tipinde gelen verinin görüntüye dönüştürülmesi veya tersi işlem için kullanılan köprünün eklenmesi
import cv2 #Görüntünün ekranda gösterilebilmesi için OpenCV'nin eklenmesi
import torch
import numpy as np
from image_processing_2025.utils import \
    time_synchronized,select_device, increment_path,\
    scale_coords,xyxy2xywh,non_max_suppression,split_for_trace_model,\
    driving_area_mask,lane_line_mask,plot_one_box,show_seg_result,\
    AverageMeter,\
    letterbox

import time
from simple_pid import PID

from geometry_msgs.msg import Twist
from rclpy.clock import Clock


class YOLOPv2DemoNode(Node):
    def __init__(self):
        #ROS düğümünün başlatılması için gerekli kodlar # Bir kez çalışır
        super().__init__("yolopv2_demo_node")

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

        self.bridge=CvBridge() #Gelen görüntü için dönüşüm köprüsünün tanımlanması

        self.weights='yolopv2.pt'
        self.img_size=640

        self.conf_thres, self.iou_thres=0.3, 0.45
        self.classes=None #Sınıfa göre filtreleme 
        self.agnostic_nms='store_true'

        self.stride=32
        self.model=torch.jit.load(self.weights)
        self.device=select_device('0')
        print(self.device)
        self.half=True
        self.model=self.model.to(self.device)

        if self.half:
            self.model.half()
        self.model.eval()

        self.pid = PID(0.005, 0, 0.001)

        self.msg=Twist()

        self.publisher_ = self.create_publisher(CompressedImage, "/lane_lines",self.qos_profile)

        self.get_logger().info("Camera subscription has started.") #Düğümün başladığına dair bildirim yapılması


        self.pTime=time.time()

    def callback_camera(self, msg):
        try:
            #Görüntünün alınması ve küçültülmesi
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
                                  
        except CvBridgeError as e:
            print(e)

            
            
        #### KOD YAZILABİLECEK ARALIK BAŞLANGICI ####

        frame=cv2.resize(cv_image,(1280,720), interpolation=cv2.INTER_LINEAR)

        img = letterbox(frame, self.img_size, stride=self.stride)[0]

        img = img[:, :, ::-1].transpose(2, 0, 1) 

        img = np.ascontiguousarray(img)
        
        img=torch.from_numpy(img).to(self.device)

        img=img.half() if self.half else img.float()

        img/=255.0

        if img.ndimension()==3:
            img= img.unsqueeze(0)


        [pred, anchor_grid], seg, ll= self.model(img) 

        ll_seg_mask = lane_line_mask(ll)

        lane_lines = np.zeros((ll_seg_mask.shape[0], ll_seg_mask.shape[1],3), dtype=np.uint8)
        
        lane_lines[ll_seg_mask == 1] = 255

        lane_lines=cv2.resize(lane_lines,(cv_image.shape[1],cv_image.shape[0]),cv2.INTER_CUBIC)

        img_msg=self.bridge.cv2_to_compressed_imgmsg(ll_seg_mask,'jpeg')

        img_msg.header.stamp=self.get_clock().now().to_msg()
        img_msg.header.frame_id="test"

        self.publisher_.publish(img_msg)

        print(1//(time.time()-self.pTime))
        self.pTime=time.time()

        # cv2.imshow("image",lane_lines)

        # cv2.waitKey(1)
        
        #### KOD YAZILABİLECEK ARALIK SONU ####
            


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2DemoNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
