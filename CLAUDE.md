# CLAUDE.md — CLW-FG SLAM workspace

Hướng dẫn cho Claude Code khi làm việc trong workspace này. Lịch sử thử nghiệm và tiến trình làm việc nằm ở [MEMORY.md](MEMORY.md) — đọc nó trước khi đề xuất thay đổi, và **cập nhật nó** sau mỗi lần thử nghiệm / sửa đổi đáng kể.

## Tổng quan

**CLW-FG SLAM** (Camera–Lidar–WiFi Factor Graph SLAM) — hệ thống SLAM ngữ nghĩa đa phương thức trên ROS 2 Humble, backend tối ưu bằng GTSAM iSAM2 (2D, `Pose2`).

- **Visual frontend** (Python, YOLO segmentation) → phát hiện landmark ngữ nghĩa, ước lượng khoảng cách bằng *chiều cao thật đã biết* của vật thể (monocular) → gửi bearing–range.
- **WiFi frontend** (Python) → fingerprint WiFi **giả lập** từ vị trí odom/ground-truth → phát hiện loop closure.
- **Backend** (C++, GTSAM) → đồ thị nhân tố: odom `BetweenFactor`, landmark `BearingRangeFactor`, WiFi loop `BetweenFactor`; publish TF `map → odom`, path, markers.
- **slam_toolbox** chỉ dùng để vẽ occupancy grid từ `/scan` (TF của nó bị tắt; backend sở hữu `map → odom`).

Dữ liệu chính hiện tại: rosbag robot thật `robot_data_bag/` (mcap, ~996 s, ghi 2026-03-10). Trước đó dùng mô phỏng TurtleBot4 / Ignition Gazebo.

## ⚠️ Giới hạn của dữ liệu (đọc trước khi đánh giá kết quả)

Theo tác giả (2026-10-21):

- **Mất depth.** Lúc thu, robot có camera depth (RealSense D435i). Do dữ liệu quá lớn, phần depth đã bị **bỏ khỏi video và rosbag**; chỉ còn ảnh RGB nén `/camera/image_raw/compressed`. Vì vậy tọa độ landmark không đo được trực tiếp mà phải **tự ước lượng monocular** từ chiều cao thật đã biết (`REAL_HEIGHTS`), nên sai số rất lớn (2–21 m, vật bị che hoặc khung nhận dạng bị cắt thì sai mạnh).
- **Odom đã qua hiệu chuẩn.** Odom bánh xe gốc trôi rất nặng; tác giả đã hiệu chuẩn lại, nên `/odom` trong bag **trông rất mượt, có nhiều đoạn thẳng**. Đó là sản phẩm của hiệu chuẩn, không phải độ chính xác thật; sai số góc ở khúc quay vẫn lớn.
- **Kết luận của tác giả về lần chạy 2026-10-21:** map/quỹ đạo đã bị **nắn theo landmark sai**, nên **không khép vòng được về vị trí ban đầu**.
- Còn dùng được: lidar 2D `/scan` (360°, 720 tia, 0.15–12 m) và scan WiFi **thật** `/wifi_status` (JSON SSID/BSSID/RSSI, khoảng 3 s/lần).

Hệ quả khi làm việc: không coi `/odom` hay landmark monocular là đáng tin; ưu tiên lidar để có hình học chính xác; đánh giá bằng độ khớp với map lidar và độ khép vòng về điểm xuất phát.

### Đặc điểm dữ liệu đã kiểm chứng bằng phân tích (2026-10-21, script trong scratchpad, chi tiết ở MEMORY.md)

1. **Timestamp bị làm tròn tới giây nguyên** ở MỌI topic, cả header lẫn thời gian ghi bag (`nanosec = 0`; khoảng 47 odom, 8 scan, 30 ảnh dùng chung 1 mốc mỗi giây). Khi `ros2 bag play`, dữ liệu phát theo **từng đợt 1 Hz**: visual node bỏ gần hết ảnh, TF có nhiều giá trị cùng một mốc thời gian, không thể đồng bộ ảnh/scan/odom theo thời gian. Muốn xử lý offline phải dựng lại thời gian (chia đều các tin trong mỗi giây theo thứ tự).
2. **Hướng frame bị ngược 180°:** robot luôn di chuyển **ngược** hướng yaw của `/odom` (100% mẫu lệch ±180°). Trục x của `base_link` chỉ về **phía sau** hướng đi. Lidar tia 0° trùng trục x của `base_link` (ICP khớp odom, không lắp ngược chiều quét, nên TF `base_link→laser_frame` không xoay là đúng). **Camera nhìn theo hướng đi thật** (optical flow nở ra 82%), tức camera xoay **π** so với `base_link`. Code hiện tại coi camera nhìn theo +x của `base_link`, nên **mọi landmark bị đặt đối xứng ra sau robot**.
3. **`REAL_HEIGHTS` sai với 2 class** (so với khoảng cách lidar ở cùng hướng, 12.101 detection): door tỉ lệ mono/lidar 0.88 (sai 0.66 m), extinguisher 0.86 (sai 0.98 m), **Fire Cabinet 0.38 (sai 3 m)**, **C7 Window 0.48 (sai 2.3 m)**. Chiều cao hiệu chỉnh gợi ý: cabinet ≈1.6 m, window ≈1.8 m, door ≈2.3 m, extinguisher ≈1.0 m. Lidar chỉ đo được khoảng cách cho 28% detection (giới hạn 12 m, nhiều tia `inf`).
4. **Odom yaw gần như là hàm bậc thang:** 95.5% tin odom có Δyaw = 0; góc chỉ đổi từng nấc (sản phẩm hiệu chuẩn). Lidar (ICP) đo góc quay giữa các scan khớp odom với sai số ~1°/s, nên **lidar dùng được làm nguồn góc quay**.
5. **WiFi thật khả dụng:** 320 scan, 123 BSSID, khoảng 28 AP/scan, nhưng giá trị chỉ đổi 28 lần (cập nhật thực khoảng 35 s/lần). Độ tương đồng cosine với điểm xuất phát giảm từ 1.0 về 0 rồi **tăng lại 1.0 ở cuối bag**, nên **robot quay về điểm xuất phát**. (Đính chính 2026-10-22: map lidar có 2 hành lang dọc riêng biệt, nên quỹ đạo nhiều khả năng là **vòng kín**, không phải quay lại đúng đường cũ.) Độ khép vòng đầu–cuối là một chỉ số đánh giá dùng được.
6. **Tin `/odom` cuối cùng hỏng:** nhảy 332 m tới (76.4, −319.5) và xoay −175°. Bỏ tin này thì odom đã hiệu chuẩn khép vòng sai 3.9 m / 331 m (1.2%). Tích phân góc từ lidar ICP scan-to-scan (153 m) hay video VO (23 m) đều kém hơn → không thay odom bằng dead reckoning thuần.
7. Hành lang dài: tia 0°/180° (trước/sau) hầu như `inf`; ICP **không quan sát được tịnh tiến dọc hành lang** (suy biến), chỉ tin được góc quay và tịnh tiến ngang.

## Cấu trúc

```
clw_fg_ws/
├── src/                         # git repo riêng (src/.git), 1 commit "first commit"
│   └── clw_fg_slam/             # package ament_cmake duy nhất
│       ├── src/clw_fg_node.cpp  # backend GTSAM (toàn bộ logic)
│       ├── src/main.cpp         # MultiThreadedExecutor
│       ├── include/clw_fg_slam/clw_fg_node.h
│       ├── scripts/visual_node.py   # YOLO frontend
│       ├── scripts/wifi_node.py     # WiFi fingerprint frontend
│       ├── scripts/map_snapshot_node.py  # lưu ảnh map + quỹ đạo + landmark mỗi 10s
│       ├── launch/system.launch.py  # launch CHÍNH (các file "copy/old/zin" là bản lưu tay)
│       ├── config/              # slam_toolbox_params.yaml, clw_fg.rviz
│       └── models/              # yolo_c7.pt (đang dùng), yolo11n-seg.pt, *.onnx (cũ)
├── robot_data_bag/              # rosbag2 mcap ~830 MB
├── tf.py                        # /odom → TF odom→base_link (bag không có /tf)
├── landmark_debug_images/       # ảnh landmark do visual_node xuất (tạo theo CWD)
├── backend_logs/                # log backend: backend_<thời gian>.log (mỗi lần chạy) + latest.log (ghi đè) (tạo theo CWD)
├── map_snapshots/               # ảnh tiến trình map: run_<thời gian>/map_NNNN.png + stats.csv, latest.png (tạo theo CWD)
├── 1jpeg                        # ảnh chụp RViz kết quả gần nhất (thực chất là JPEG)
├── readme.md                    # hướng dẫn cài đặt/build
├── readddmeee.txt               # sổ lệnh chạy (Gazebo + quy trình chạy bag)
├── build/ install/ log/         # sản phẩm colcon (log/ = lịch sử build)
```

Không có git ở thư mục gốc; chỉ `src/` là git repo.

## Build & chạy

```bash
cd /media/synv/Data/clw_fg_ws
colcon build --packages-select clw_fg_slam
source install/setup.bash
```

Chạy với rosbag (mỗi lệnh một terminal, theo `readddmeee.txt`):

```bash
ros2 bag play robot_data_bag --clock -l
python3 tf.py                                                        # odom -> base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
ros2 launch clw_fg_slam system.launch.py                             # chạy từ GỐC workspace
```

(`readddmeee.txt` còn một static TF `odom → base_link` — trùng với `tf.py`, không chạy cả hai.)

**Phải launch từ gốc workspace**: `visual_node.py` load model bằng đường dẫn tương đối `src/clw_fg_slam/models/yolo_c7.pt` và ghi ảnh vào `./landmark_debug_images`; `map_snapshot_node.py` ghi vào `./map_snapshots`.

`map_snapshot_node.py` (params `output_dir`, `interval_sec`=10, `max_image_size`=2000, `margin_m`): timer theo đồng hồ thật (STEADY), vẽ `/map` + `/clw_fg/path` (xanh) + `/odom` thô (đỏ) + landmark theo class; `stats.csv` có keyframes, landmarks, độ dài quỹ đạo, độ lệch điểm cuối odom-vs-optimized. Lưu thêm 1 ảnh khi Ctrl-C.

Test headless (không RViz/YOLO) đã dùng: bag `-r 3` + tf.py + static TF + `sync_slam_toolbox_node --params-file install/.../slam_toolbox_params.yaml` + `clw_fg_node` + `map_snapshot_node.py`. Khi dọn tiến trình đừng dùng `pkill -f` với chuỗi có trong chính lệnh shell (sẽ tự kill shell).

Log backend: dùng `logMsg(LogLevel::INFO|WARN|ERROR|DEBUG, fmt, ...)` (kiểu printf) thay cho `RCLCPP_*` — vừa in console vừa ghi `backend_logs/` (param `backend_log_dir`). `writeStatusLog()` cũng ghi vào đó với tag `[STATUS]`. Mỗi dòng: `[giờ thật] [sim t] [LEVEL] nội dung`, flush ngay. Code backend mới phải dùng `logMsg`, không gọi `RCLCPP_*` trực tiếp.

rclpy (Humble): đọc param bằng `.value`, KHÔNG có `.as_double()` (đó là API C++).

Không có test tự động. Kiểm chứng = chạy bag + xem RViz / `clw_slam_status.log` / quỹ đạo.

## Phụ thuộc

ROS 2 Humble, GTSAM 4.x (`libgtsam-dev`, PPA borglab), OpenCV, cv_bridge, slam_toolbox, rviz2; Python: `ultralytics`, `opencv-python`, `numpy`.

## Topic & giao tiếp

| Topic | Kiểu | Hướng |
|---|---|---|
| `/odom` | Odometry | bag → backend, wifi_node, tf.py |
| `/camera/image_raw/compressed` | CompressedImage | bag → visual_node |
| `/scan` | LaserScan | bag → slam_toolbox |
| `/sim_ground_truth_pose` | Odometry | Gazebo → wifi_node (ưu tiên nếu có) |
| `/clw_fg/landmark` | PointStamped | visual → backend |
| `/clw_fg/wifi_loop` | PoseWithCovarianceStamped | wifi → backend |
| `/clw_fg/path`, `/clw_fg/markers` | Path, MarkerArray | backend → RViz |
| `/visual_frontend/debug_image` | Image | visual → RViz |
| `/clw_fg/wifi_status` | String | wifi_node |

**Quy ước "nhồi" dữ liệu vào message (quan trọng):**
- Landmark: `header.frame_id` = ID landmark dạng chuỗi, `point.x` = bearing (rad, trái dương), `point.y` = range (m).
- ID landmark = `class_id * 1000 + (track_id % 1000)`; backend dùng `id / 1000` làm class để data association.
- WiFi loop: `header.frame_id` = ID keyframe cũ (theo bộ đếm của **wifi_node**).

Class của `yolo_c7.pt` (dùng trong `REAL_HEIGHTS`): 0 = C7 Window (0.85 m), 1 = Fire Cabinet (0.60 m), 2 = door (2.00 m), 3 = fire extinguisher (0.85 m).

TF: `map → odom` (backend) · `odom → base_link` (tf.py) · `base_link → laser_frame` (static).

## Tham số backend (ROS params, mặc định trong code)

`odom_linear_sigma 0.05`, `odom_angular_sigma 0.02`, `visual_bearing_sigma 0.05`, `visual_range_sigma 0.1`, `wifi_xy_sigma 5.0` (yaw sigma 3.14 cố định), `keyframe_delta_trans 0.2`, `keyframe_delta_rot 0.15`, `save_trajectory_path`. Kernel: Huber(1.345) cho visual, Cauchy(0.1) cho WiFi. iSAM2: Dogleg, relinearizeThreshold 0.1. Hằng `MAX_OBS_PER_KF = 50`, bán kính data association 1 m.

## Đường dẫn hard-code (kiểm tra trước khi chạy)

Workspace đã chuyển từ `/home/synv/clw_fg_ws` sang `/media/synv/Data/clw_fg_ws`. Còn hard-code (file được tự tạo khi chạy):
- `/home/synv/clw_slam_status.log` — log chung của cả 3 node (`writeStatusLog` / `write_status_log`).
- `/home/synv/clw_trajectory.txt` — quỹ đạo TUM (launch param).

Params slam_toolbox và RViz nay nằm trong package: `src/clw_fg_slam/config/slam_toolbox_params.yaml`, `src/clw_fg_slam/config/clw_fg.rviz` (cài qua `install(DIRECTORY config)`, launch lấy bằng `get_package_share_directory`). Sửa config → phải `colcon build` lại.

## Vấn đề đã biết (đọc trước khi debug)

1. **ID keyframe WiFi ≠ ID keyframe backend.** wifi_node tự đếm node mỗi 0.2 m; backend tạo keyframe khi > 0.2 m *hoặc* > 0.15 rad. `frame_id` loop gửi sang backend vì thế trỏ sai pose `x_k`. Loop factor còn là `Pose2(0,0,0)` (giả định trùng khít).
2. **WiFi là giả lập từ chính odom** (5 AP ảo) → loop closure suy ra từ odom đang trôi, mang tính vòng lặp. Bag có `/wifi_status` (String, 320 msg) nhưng chưa được dùng.
3. Landmark luôn gắn vào keyframe mới nhất `x_{keyframe_count_}`, không khớp theo timestamp ảnh.
4. Intrinsics camera hard-code (fx=fy=615, cx=320, cy=240 — D435i 640×480); `has_intrinsics=True` nên `camera_info` không bao giờ ghi đè. Cần xác minh độ phân giải ảnh trong bag.
5. Comment lệch code: tracker `max_age` = 5 (comment nói 15); confirm landmark cần 5 frame (comment nói 3).
6. Khi `IndeterminantLinearSystemException`: xoá graph_ chờ (mất cả landmark factor), `keyframe_count_--`.
7. File quỹ đạo dùng timestamp giả `index * 0.1` → không so trực tiếp với ground truth bằng EVO theo thời gian.
8. `MAX_OBS_PER_KF` dùng biến `static` toàn cục, không phải member.
9. CMake `install(DIRECTORY launch)` cài cả các file launch bản sao (tên có dấu cách).

## Quy ước làm việc

- Code và comment bằng tiếng Việt xen tiếng Anh; comment kiểu `// [SỬA LỖI n]:`, `# [MỚI]:` đánh dấu thay đổi — giữ phong cách này.
- Sửa `system.launch.py` chứ không phải các bản copy. Không xoá bản copy khi chưa hỏi.
- Đổi chữ ký hàm backend → sửa đồng thời `.h` và `.cpp` (lỗi build lặp lại nhiều nhất trong lịch sử, xem MEMORY.md).
- Sau thử nghiệm: ghi kết quả, tham số, quan sát vào MEMORY.md (mục Nhật ký thử nghiệm).
