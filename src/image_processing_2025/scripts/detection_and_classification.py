#!/usr/bin/env python3.10 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select
import tty, termios
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
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
        ###
        self.yolo_model = YOLO("/home/otagg/ros2_ws/src/image_processing_2025/scripts/best.pt")
        self.yolo_model.to('cuda:1' if torch.cuda.is_available() else 'cpu')

        self.class_model = Net()
        self.class_model.load_state_dict(torch.load("/home/otagg/ros2_ws/src/image_processing_2025/scripts/balanced_micron.pth"))
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
            # Görüntünün alınması ve boyutlandırılması
            cv_image = cv2.resize(self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8'),
                                  (1280, 720), interpolation=cv2.INTER_LINEAR)
        except CvBridgeError as e:
            print(e)
            return  # Hata durumunda fonksiyonu sonlandır

        pTime = time.time()
        is_accelerated = False

        # YOLO Modeli ile nesne tanıma
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            results = self.yolo_model(img)
            for r in results:
                boxes = r.boxes
                cls_ids = r.boxes.cls.cpu().numpy()
                for i, box in enumerate(boxes):
                    b = box.xyxy[0].to('cpu').detach().numpy().copy()
                    cropped_img = img[int(b[1]):int(b[3]), int(b[0]):int(b[2])]
                    cropped_img_pil = PILImage.fromarray(cropped_img)
                    cropped_img_tensor = self.transform(cropped_img_pil).unsqueeze(0).to('cuda:1' if torch.cuda.is_available() else 'cpu')
                    with torch.no_grad():
                        output = self.class_model(cropped_img_tensor)
                        _, predicted_class = torch.max(output, 1)
                        predicted_class = predicted_class.item()
                    area = (int(b[3] - int(b[1]))) * (int(b[2] - int(b[0])))
                    print(f"Class name: {predicted_class}; Alan: {area}")

                    if predicted_class == 32 and area > 100:  # Kırmızı ışık
                        print("Red light detected")
                        is_accelerated = True
                        self.msg.linear.x = 0.0
                        self.msg.angular.z = 0.0
                    elif predicted_class == 34 and area > 2000:  # Sol dönüş
                        print("Left turn detected")
                        is_accelerated = True
                        self.msg.linear.x = 0.5
                        self.msg.angular.z = 7.0
        except Exception as e:
            print(f"YOLO processing error: {e}")

        # Şerit Algılama
        try:
            img = letterbox(cv_image, self.img_size, stride=self.stride)[0]
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            [pred, anchor_grid], seg, ll = self.model(img)
            ll_seg_mask = lane_line_mask(ll)

            # Şeritlerin orta noktalarını bulma
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
        except Exception as e:
            print(f"Lane detection error: {e}")
            control = 0

        # Hareket Kontrolü
        if not is_accelerated:
            self.msg.linear.x = 2.0
            self.msg.angular.z = -float(control)

        self.publisher_.publish(self.msg)

        # Görüntüleme
        cv2.imshow("Result", cv_image)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2DemoNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()
