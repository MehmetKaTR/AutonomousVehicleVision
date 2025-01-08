#!/usr/bin/env python3.10

#ROS için gerekli kütüphanelerin eklenmesi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CompressedImage #Görüntüyü aktarabilmek için kullanılan mesaj tipi Image'in eklenmesi
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

        self.weights='/home/otagg/yolopv2.pt'
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

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel",10)

        self.get_logger().info("Camera subscription has started.") #Düğümün başladığına dair bildirim yapılması

    def callback_camera(self, msg):
        try:
            #Görüntünün alınması ve küçültülmesi
            cv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8'),
                                  (1280,720), interpolation=cv2.INTER_LINEAR)
        except CvBridgeError as e:
            print(e)

        pTime=time.time()
            
            
        #### KOD YAZILABİLECEK ARALIK BAŞLANGICI ####

        img = letterbox(cv_image, self.img_size, stride=self.stride)[0]

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

        ll = ll.squeeze().cpu().numpy()  # Tensor'dan NumPy'ye dönüşüm
        da_seg_mask = driving_area_mask(seg)
        # Segmentasyon çıktısını binary maskeye çevirme
        binary_mask = (ll > 0.5).astype(np.uint8)

        

        contours, _ = cv2.findContours((ll_seg_mask > 0.5).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lanes, close_lanes = [[0, 0], [0, 0]], [999, 999]
        height, width, _ = cv_image.shape
        mask_width = ll_seg_mask.shape[1]
        mean = mask_width / 2

        for contour in contours:
            if cv2.contourArea(contour) < 50:
                continue
            points = contour[:, 0, :]
            if len(contour) < 100:
                continue
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

        self.msg.linear.x=2.0

        self.msg.angular.z=-float(control)

        self.publisher_.publish(self.msg)

        cv2.imshow("img",cv_image) #Görüntünün ekrana gösterilmesi
        
        key=cv2.waitKey(1) #Görüntünün ekranda kalması için 1ms'lik key bekleme ve gecikme
        
        #### KOD YAZILABİLECEK ARALIK SONU ####
            


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2DemoNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
