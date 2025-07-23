#AutonomousVehicleVision Şerit Takibi, Trafik Tabelası Tanıma ve İnsan Algılama ile Otonom Sürüş

<img width="650" height="366" alt="image" src="https://github.com/user-attachments/assets/fe187b3b-85a3-47ba-89db-193c80f18daa" />

📌 Proje Özeti
Bu proje, otonom sürüş teknolojilerinin temel yapı taşlarından biri olan görsel algılama sistemleri üzerine odaklanmaktadır.
2025 yılına yönelik olarak geliştirilen bu sistem:

Yol şeritlerini tespit edip aracın konumunu korumasını sağlar.

Trafik levhalarını tanıyarak doğru kararlar alınmasına yardımcı olur.

Yaya tespiti yaparak güvenliği artırır.

Mesafe ölçümü ile çarpışma öncesi tepki süresini yönetir.

ROS2 + Gazebo Harmonic ile simüle edilen bir ortamda test edilmiştir.

YOLOPv2 modeli ile gerçek zamanlı görüntü işleme yapılmıştır.

🎯 Hedeflenen Uygulamalar
Özellik	Açıklama
🚧 Şerit Takibi	Aracın şerit içindeki konumunu korur.
📷 Görüntü İşleme	Kamera verileri üzerinden gerçek zamanlı analiz yapılır.
🛑 Tabela Tanıma	Trafik işaretleri tespit edilip sınıflandırılır.
🧍‍♂️ Yaya Algılama	Yolda karşılaşılan insanlar tespit edilip sistem durur.
📏 Mesafe Ölçümü	Lidar/kamera verileriyle öndeki nesnelerle mesafe hesaplanır.
🧠 Otonom Kontrol	ROS2 üzerinden kontrol algoritmaları ile karar verme sağlanır.

📸 Örnek Görseller
Aşağıdaki görseller projede yapılan bazı örnek çıktıları temsil etmektedir:

🔹 Şerit Takibi Örneği
![WhatsApp Görsel 2025-07-16 saat 23 26 52_428179f1](https://github.com/user-attachments/assets/c20fd587-e4d5-4904-9592-a179b2e1f5b8)


🔹 Trafik Levhası Tanıma
![WhatsApp Görsel 2025-07-16 saat 23 26 52_f73b8404](https://github.com/user-attachments/assets/63843c9f-92c9-4fc5-b21c-ed19fb009314)


🔹 İnsan Tespiti
![WhatsApp Görsel 2025-07-16 saat 23 26 52_c707e1ba](https://github.com/user-attachments/assets/ce33362f-68a6-4363-922c-9255a597fa56)


🔹 Mesafe Ölçümü
![WhatsApp Görsel 2025-07-16 saat 23 26 52_02e51741](https://github.com/user-attachments/assets/da5175a2-7ae9-44da-8d50-a72a2833b452)


🔹 Araç Duruş Anı
![WhatsApp Görsel 2025-07-16 saat 23 26 52_3c4e5b39](https://github.com/user-attachments/assets/8bec99b1-f016-4438-8cf5-45adf295563c)




# Gazebo Ortamında Test Etmek İçin;

## GAZEBO HARMONIC KURULUMU

Ubuntu üzerinde bir terminal açın ve aşağıdaki kodları çalıştırın.

Birkaç eklenti kuracağız:
```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Şimdi Gazebo Harmonic'i kuralım:
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

İndirmeler bittiğinde Gazebo Harmonic kurulmuş olacak.

## ROS_GZ

Gazebo Harmonic ile ROS2 Humble arasındaki bağlantının sağlanabilmesi için ros_gz paketinin kurulması gerekmektedir. ROS2 Humble ile son sürüm Gazebo sürümleri arasından resmi olarak Ignition Fortress desteklenmektedir. Bu sebepten dolayı apt kullanılarak ros_gz paketi kurulduğunda Fortress destekli olarak kurulmaktadır. Bu kurulumu Gazebo Harmonic ile uyumlu hale getirmek için kaynaktan kurulum (build from source) yapmak gerekmektedir.


Kaynaktan kurulum yaparken Gazebo sürümünün belirtilmesi gerekmektedir:
```bash
export GZ_VERSION=harmonic
```

Kaynaktan kurulum yapılmasından dolayı paketin çalışma alanında izole olması önerilmektedir. Bundan dolayı home dizini içerisine ros2_ws'den farklı bir dizin oluşturmak gerekmektedir. Bu yapıldıktan sonra dizin içerisindeki src'ye ros_gz deposunun humble branch'ı klonlanmalıdır:
```bash
cd ~
mkdir -p ~/pkgs_ws/src
cd ~/pkgs_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
```

## ROS2 Kurulumu

Set locale:
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Setup Sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 packages
```bash
sudo apt update
```

```bash
sudo apt upgrade
```

```bash
sudo apt install ros-humble-desktop
```

```bash
sudo apt install ros-humble-ros-base
```

```bash
sudo apt install ros-dev-tools
```

Environment setup
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Kurulumu yapabilmek için ~/pkgs_ws dizini içerisine gelip rosdep ile bağımlılıkları kurmak ve colcon build ile kurulumu başlatmak gerekmektedir:

***BU ADIMI YAPARKEN SİSTEMDE HERHANGİ BAŞKA İŞLEMIN/PROGRAMIN AÇIK OLMAMASI GEREKMEKTEDİR***
```bash
cd ~/pkgs_ws
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```

Bütün bu işlemler bittikten sonra pkgs_ws'nin bashrc'den source yapılması gerekmektedir:
```bash
echo "source ~/pkgs_ws/install/setup.bash" >> ~/.bashrc
```

İşlemler sonucunda ros_gz paketi kurulumu tamamlanmış demektir.

## Çalıştırma Evresi

İlk önce "ros2_ws" adlı bir klasör oluşturuyoruz
```bash
mkdir ros2_ws
cd ros2_ws
```

Github reposunu bu klasörün içinde clone yapınız.
```bash
git clone https://github.com/MehmetKaTR/tasarim_2025.git
```

"ros2_ws" klasörü içinde:
```bash
colcon build
```
yapınız

Environment setup
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Artık ortam hazır. Şimdi çalıştırmak için sıraya gerekli kodları yazalım:
```bash
ros2 launch simulation_2025 teknofest_IGN.launch.py
```

```bash
ros2 run image_processing_2025 yolopv2_demo.py
```

```bash
ros2 run image_processing_2025 yolopv2_demo.py
```

```bash
ros2 param set /yolopv2_demo_node use_sim_time true
```

```bash
ros2 param set /only_detection_and_classification use_sim_time true
```

```bash
ros2 run control_2025 detector_sub.py
```





