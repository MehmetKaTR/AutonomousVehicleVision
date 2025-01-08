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
import sys
sys.path.append("/home/otagg/LaneDetection_YOLOP/YOLOPX-main/")
from lib.config import cfg
from lib.config import update_config
from lib.utils.utils import create_logger, select_device, time_synchronized
from lib.models import get_net
from lib.dataset import LoadImages, LoadStreams
from lib.core.general import non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result,letterbox_for_img

from lib.core.function import AverageMeter
from lib.core.postprocess import morphological_process, connect_lane
import torchvision.transforms as transforms
import time


class YOLOPXDemoNode(Node):
    def __init__(self):
        #ROS düğümünün başlatılması için gerekli kodlar # Bir kez çalışır
        super().__init__("yolopX_demo_node")

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

        self.weights='/home/otagg/LaneDetection_YOLOP/weights/yolopX.pth'
        self.normalize = transforms.Normalize(
                mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
            )

        self.transform=transforms.Compose([
                transforms.ToTensor(),
                self.normalize,
            ])

        self.img_size=640
        self.conf_thres, self.iou_thres=0.3, 0.45
        self.classes=None #Sınıfa göre filtreleme 
        self.agnostic_nms='store_true'
        logger = None
        self.device=select_device(logger, '0')
        self.model=get_net(cfg)
        self.checkpoint = torch.load(self.weights, map_location= self.device)
        self.model.load_state_dict(self.checkpoint['state_dict'])
        self.half=True
        self.model=self.model.to(self.device)

        if self.half:
            self.model.half()
        self.model.eval()

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

        h0, w0 = cv_image.shape[:2]
        img, ratio, pad = letterbox_for_img(cv_image, self.img_size, auto=True)
        h, w = img.shape[:2]
        shapes = (h0, w0), ((h / h0, w / w0), pad)
        pad_w, pad_h = shapes[1][1]
        pad_w = int(pad_w)
        pad_h = int(pad_h)
        ratio = shapes[1][0][1]
        # Inference
        img = self.transform(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        _, _,ll_seg_out = self.model(img)

        ll_predict = ll_seg_out[:, :,pad_h:(h-pad_h),pad_w:(w-pad_w)]
        ll_seg_mask = torch.nn.functional.interpolate(ll_predict,  size=(h ,w ), mode='bilinear')
        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze()
        ll_seg_mask = ll_seg_mask.cpu().numpy() 

        lane_lines = np.zeros((ll_seg_mask.shape[0], ll_seg_mask.shape[1],3), dtype=np.uint8)
        
        lane_lines[ll_seg_mask == 1] = 255

        #show_seg_result(cv_image,(da_seg_mask, ll_seg_mask), is_demo=True)

        cv2.putText(lane_lines, f"{1/(time.time()-pTime)}",(0,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,255))


        cv2.imshow("img",lane_lines) #Görüntünün ekrana gösterilmesi
        
        key=cv2.waitKey(1) #Görüntünün ekranda kalması için 1ms'lik key bekleme ve gecikme
        
        #### KOD YAZILABİLECEK ARALIK SONU ####
            


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPXDemoNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
