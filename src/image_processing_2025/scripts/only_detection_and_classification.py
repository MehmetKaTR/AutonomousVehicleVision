#!/usr/bin/env python3.10
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select
import tty, termios
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import rclpy.time_source
from sensor_msgs.msg import CompressedImage, Image  
from cv_bridge import CvBridge, CvBridgeError  
import cv2  
import torch
from ultralytics import YOLO
from torchvision import transforms
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image as PILImage
import threading
import numpy as np
import time

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(3, 1, kernel_size=1)
        self.conv2 = nn.Conv2d(1, 29, kernel_size=5)
        self.maxpool2 = nn.MaxPool2d(3, stride=2 , ceil_mode=True)
        self.conv3 = nn.Conv2d(29, 59, kernel_size=3)
        self.maxpool3 = nn.MaxPool2d(3, stride=2 , ceil_mode=True)
        self.conv4 = nn.Conv2d(59, 74, kernel_size=3)
        self.maxpool4 = nn.MaxPool2d(3, stride=2 , ceil_mode=True)
        self.conv2_drop = nn.Dropout2d()
        self.conv3_drop = nn.Dropout2d()
        self.fc1 = nn.Linear(1184, 300)
        self.fc2 = nn.Linear(300, 43)  # 43 classes for GTSRB (example)
        self.conv0_bn = nn.BatchNorm2d(3)
        self.conv1_bn = nn.BatchNorm2d(1)
        self.conv2_bn = nn.BatchNorm2d(29)
        self.conv3_bn = nn.BatchNorm2d(59)
        self.conv4_bn = nn.BatchNorm2d(74)
        self.dense1_bn = nn.BatchNorm1d(300)

    def forward(self, x):
        x = F.relu(self.conv1_bn(self.conv1(self.conv0_bn(x))))
        x = F.relu(self.conv2_bn(self.conv2(x)))
        x = F.relu(self.conv3_bn(self.conv3(self.maxpool2(x))))
        x = F.relu(self.conv4_bn(self.conv4(self.maxpool3(x))))
        x = self.maxpool4(x)
        x = x.view(-1, 1184)
        x = F.relu(self.fc1(x))
        x = self.dense1_bn(x)
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        x = F.dropout(x, training=self.training)
        return F.log_softmax(x, dim=1)


#ROS için gerekli kütüphanelerin eklenmesi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.time_source import TimeSource

from sensor_msgs.msg import CompressedImage,Image #Görüntüyü aktarabilmek için kullanılan mesaj tipi Image'in eklenmesi
from cv_bridge import CvBridge,CvBridgeError #Image mesaj tipinde gelen verinin görüntüye dönüştürülmesi veya tersi işlem için kullanılan köprünün eklenmesi
import cv2 #Görüntünün ekranda gösterilebilmesi için OpenCV'nin eklenmesi
import torch
import numpy as np
import message_filters

import time
from simple_pid import PID

from geometry_msgs.msg import Twist
from image_processing_2025.msg import DetectedSign, SignList

import os

class OnlyDetectionAndClassification(Node):
    def __init__(self):
        #ROS düğümünün başlatılması için gerekli kodlar # Bir kez çalışır
        super().__init__("only_detection_and_classification")

        self.qos_profile=QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        ###
        ##print("Current working directory:", os.getcwd())

        self.yolo_model = YOLO("ros2_ws/src/image_processing_2025/scripts/best.pt")
        self.yolo_model.to('cuda:1' if torch.cuda.is_available() else 'cpu')

        self.class_model = Net()
        self.class_model.load_state_dict(torch.load("ros2_ws/src/image_processing_2025/scripts/balanced_micron.pth"))
        self.class_model.to('cuda:1' if torch.cuda.is_available() else 'cpu')
        self.class_model.eval()
        self.classes = [
            "20 km/h hiz siniri",
            "30 km/h hiz siniri",
            "50 km/h hiz siniri",
            "60 km/h hiz siniri",
            "70 km/h hiz siniri",
            "80 km/h hiz siniri",
            "mecburi sag",
            "100 km/h hiz siniri",
            "120 km/h hiz siniri",
            "sollamak yasak",
            "kamyonlar icin sollamak yasak",
            "ana yol tali yol kavsagi",
            "park etmek yasak",
            "yol ver",
            "dur",
            "tasit trafigine kapali yol",
            "kamyon giremez",
            "girisi olmayan yol",
            "dikkat",
            "sola donus yasak",
            "saga donus yasak",
            "sola tehlikeli devamli virajlar",
            "sola mecburi yon",
            "yol calismasi",
            "kaygan yol",
            "donel kavsak",
            "trafik isareti",
            "yaya geciti",
            "park",
            "bisiklet giremez",
            "gizli buzlanma",
            "durak",
            "kirmizi isik",
            "ileriden saga mecburi yon",
            "ileriden sola mecburi yon",
            "ileri mecburi yon",
            "ileri ve saga mecburi yon",
            "ileri ve sola mecburi yon",
            "sagdan gidiniz",
            "soldan gidiniz",
            "sari isik",
            "yesil isik",
            "sagdan daralan yol"
        ]
        self.image_ready = False
        self.current_image = None

        self.transform = transforms.Compose([
            transforms.Resize((48, 48)),
            transforms.ToTensor(),
            transforms.Normalize((0.3337, 0.3064, 0.3171), (0.2672, 0.2564, 0.2629))
        ])
        ###
        
        #Arabadaki stereo kameranın bir tanesine düğümün abone edilmesi


        self.subscriber_camera = message_filters.Subscriber(self, CompressedImage,'camera_compressed',qos_profile=self.qos_profile)
        
        
        # self.create_subscription(CompressedImage, #Düğümün bağlandığı konudaki mesajın tipi
        #                                             "/camera_compressed", #Düğümün bağlandığı konu 
        #                                            # self.callback_camera, #Her mesaj geldiğinde gidilecek fonksiyon
        #                                             qos_profile=self.qos_profile) #Gelen mesajlar için oluşturulan sıra sayısı   
        
        self.subscriber_depth = message_filters.Subscriber(self, Image, 'depth/image',qos_profile=self.qos_profile)
        
        
        # self.create_subscription(Image, #Düğümün bağlandığı konudaki mesajın tipi
        #                                             "/depth/image", #Düğümün bağlandığı konu 
        #                                            # self.callback_camera, #Her mesaj geldiğinde gidilecek fonksiyon
        #                                             qos_profile=self.qos_profile) #Gelen mesajlar için oluşturulan sıra sayısı 
        
        ts = message_filters.TimeSynchronizer([self.subscriber_camera, self.subscriber_depth], 10)

        ts.registerCallback(self.callback_camera)



        self.bridge=CvBridge() #Gelen görüntü için dönüşüm köprüsünün tanımlanması

        self.publisher_ = self.create_publisher(SignList, "/detected_signs",self.qos_profile)

        self.get_logger().info("Camera subscription has started.") #Düğümün başladığına dair bildirim yapılması

        self.pTime=time.time()

    def callback_camera(self, msg_cam, msg_depth):
        try:
            #Görüntünün alınması ve küçültülmesi
            img =self.bridge.compressed_imgmsg_to_cv2(msg_cam,'bgr8')
            depth = self.bridge.imgmsg_to_cv2(msg_depth,'passthrough')
        except CvBridgeError as e:
            print(e)

            
            
        detected_signs_msg=SignList()
        #### KOD YAZILABİLECEK ARALIK BAŞLANGICI ####
        results = self.yolo_model(img)
        for r in results:
            boxes = r.boxes
            cls_ids = r.boxes.cls.cpu().numpy()
            for i,box in enumerate(boxes):
                
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                cropped_img = img[int(b[1]):int(b[3]), int(b[0]):int(b[2])] 
                cropped_img_pil = PILImage.fromarray(cropped_img) 
                cropped_img_tensor = self.transform(cropped_img_pil).unsqueeze(0).to('cuda:1' if torch.cuda.is_available() else 'cpu')  
                with torch.no_grad():
                    output = self.class_model(cropped_img_tensor)
                    _, predicted_class = torch.max(output, 1)
                    predicted_class = predicted_class.item()
                area = (int(b[3]- int(b[1]))) * (int(b[2])- int(b[0]))
                distance=depth[(int(b[3])+int(b[1]))//2 , (int(b[0])+int(b[2]))//2 ]
               # print(f"Class name:{predicted_class}; Alan:{area}")

                cv2.rectangle(img,(int(b[0]),int(b[1])),(int(b[2]),int(b[3])),(255,0,0),1)

                detected_sign=DetectedSign()
                detected_sign.sign_name = predicted_class
                detected_sign.x_min = int(b[0])
                detected_sign.y_min = int(b[1])
                detected_sign.x_max = int(b[2])
                detected_sign.y_max = int(b[3])
                detected_sign.distance = float(distance)

                detected_signs_msg.signs.append(detected_sign)

        detected_signs_msg.header.stamp=self.get_clock().now().to_msg()
        detected_signs_msg.header.frame_id="test"

        print(1//(time.time()-self.pTime))
        self.pTime=time.time()

        self.publisher_.publish(detected_signs_msg)
        
        #### KOD YAZILABİLECEK ARALIK SONU ####
            


def main(args=None):
    rclpy.init(args=args)
    node = OnlyDetectionAndClassification()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
