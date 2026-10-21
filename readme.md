1. Cài ROS 2 cơ bản & Build Tools:

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-rosdep

2. Cài Thư viện Hệ thống (GTSAM & OpenCV):

sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev


3. Cài Thư viện Python (Cho YOLO & WiFi Node):


BƯỚC 2: Bung nén và Build

1. Tạo Workspace mới:

mkdir -p ~/clw_fg_ws
cd ~/clw_fg_ws

2. Giải nén Code vào: Copy file clw_fg_source.tar.gz vào đây rồi giải nén:

tar -xzvf clw_fg_source.tar.gz
# Lúc này bạn sẽ có thư mục ~/clw_fg_ws/src

3. Cài các gói ROS phụ thuộc tự động:

rosdep update
rosdep install --from-paths src --ignore-src -r -y

4. build 


colcon build --packages-select clw_fg_slam


BƯỚC 3: SỬA LỖI ĐƯỜNG DẪN 

Trong file visual_node.cpp, đang để đường dẫn tuyệt đối tới file model ONNX

Build lại sau khi sửa

cd ~/clw_fg_ws

# 1. Build
colcon build --packages-select clw_fg_slam

# 2. Source
source install/setup.bash

# 3. Chạy
ros2 launch clw_fg_slam system.launch.py




cd ~/clw_fg_ws
colcon build --packages-select clw_fg_slam
source install/setup.bash
ros2 launch clw_fg_slam system.launch.py