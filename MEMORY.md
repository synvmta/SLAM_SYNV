# MEMORY.md — Lịch sử thử nghiệm & tiến trình CLW-FG SLAM

Nhật ký sống của dự án. Ghi **mới nhất ở cuối** mỗi mục. Mỗi lần chạy thử / đổi tham số / sửa lỗi đáng kể → thêm một dòng vào [Nhật ký thử nghiệm](#nhật-ký-thử-nghiệm).

> Giai đoạn trước 2026-10-21 được **dựng lại** từ `log/build_*` (142 lần build), mtime file, rosbag metadata, các bản launch sao lưu và comment trong code. Không có git history chi tiết (`src/.git` chỉ có 1 commit "first commit", 2026-05-07).

---

## Trạng thái hiện tại (cập nhật 2026-10-21)

- Pipeline đầy đủ chạy được trên rosbag thật: backend GTSAM + visual YOLO (`yolo_c7.pt`, 4 class) + WiFi giả lập + slam_toolbox (chỉ vẽ map).
- Lần chạy gần nhất (2026-05-18): visual_node xuất 344 ảnh landmark — door 164, fire extinguisher 87, Fire Cabinet 55, C7 Window 38.
- Ảnh RViz `1jpeg`: map slam_toolbox (hành lang) rõ, nhưng **path backend (xanh dương) lệch nhiều so với hành lang**, landmark (xanh lá) rải rác, nhiều landmark nằm ngoài tường → backend còn drift / loop & landmark chưa kéo được quỹ đạo về đúng.
- Workspace đã chuyển sang `/media/synv/Data/clw_fg_ws`; các file ngoài (`~/clw_slam_status.log`, `~/clw_trajectory.txt`, `~/ros2_converter/my_params.yaml`, `~/tb4_ducsam.rviz`) **không còn** trên máy này → cần tạo lại / đổi đường dẫn trước khi chạy.

## Bối cảnh dữ liệu (tác giả cung cấp 2026-10-21)

- Lúc thu dữ liệu có **camera depth**, nhưng vì dung lượng quá lớn nên depth đã bị **bỏ khỏi video và rosbag**. Không còn tọa độ 3D thật của landmark → phải tự tạo landmark bằng ước lượng monocular (chiều cao vật thể) → **sai số rất lớn**.
- **Odom bánh xe gốc trôi nặng; tác giả đã hiệu chuẩn** → `/odom` trong bag nhìn rất mượt, nhiều đoạn thẳng (do hiệu chuẩn, không phản ánh độ chính xác thật).
- **Kết luận của tác giả:** map đã bị nắn theo landmark sai → không khép vòng được vị trí ban đầu.
- Bổ sung từ phân tích log (Claude): landmark sai là nguyên nhân chính, nhưng không phải duy nhất. 39 loop WiFi cũng sai (lệch số thứ tự keyframe + fingerprint giả lập) và cùng kéo quỹ đạo lệch; landmark gần như không bao giờ được nhận lại khi quay về (1/125) nên không tạo được loop closure nào.

## Việc cần làm / hướng tiếp theo

- [ ] Đồng bộ ID keyframe giữa wifi_node và backend (vd. backend publish keyframe ID + pose, wifi_node dùng ID đó; hoặc khớp theo timestamp).
- [ ] Dùng tín hiệu WiFi thật từ `/wifi_status` trong bag thay cho giả lập từ odom.
- [ ] Gắn landmark vào keyframe theo timestamp ảnh, không phải keyframe mới nhất.
- [ ] Xác minh độ phân giải / intrinsics thật của camera trong bag (đang hard-code D435i 640×480).
- [ ] Đưa các đường dẫn hard-code còn lại (status log, trajectory, model YOLO) thành ROS param.
- [x] Tạo lại params slam_toolbox & file rviz (2026-10-21, `src/clw_fg_slam/config/`).
- [ ] Chạy thử toàn bộ pipeline với bag trên máy mới (đã chạy headless backend + slam_toolbox + snapshot; chưa chạy YOLO/WiFi/RViz).
- [ ] **Backend chỉ tin odom → lệch hướng so với map slam_toolbox sau khi rẽ** (xem nhật ký 2026-10-21). Cân nhắc thêm factor từ scan matching / pose của slam_toolbox hoặc lidar odometry.
- [ ] Ghi timestamp thật vào file quỹ đạo TUM để đánh giá bằng EVO.
- [ ] Dọn các bản launch copy (hỏi người dùng trước).

---

## Dòng thời gian (dựng lại)

### Giai đoạn 1 — Khởi tạo & mô phỏng (12/2025)
- **2025-12-05**: tạo `src/`.
- **2025-12-25 → 12-29** (39 build): dựng package `clw_fg_slam`. Visual frontend ban đầu là **C++ + ONNX** (`yolov8n-seg.onnx`, sau đó `yolov5n-seg.onnx`; readme nhắc `visual_node.cpp` với đường dẫn ONNX tuyệt đối). 12-26 thêm `yolo11n-seg.pt` (COCO).
- **2025-12-27**: launch phiên bản đầu (`system.launch old.py`, `zin.py`, `(4th copy).py`): backend + visual + wifi + rviz, **chưa có slam_toolbox**. Viết `readme.md`.
- Môi trường thử: TurtleBot4 trên Ignition Gazebo — world `custom_maze_v1`, `warehouse`, `hospital` (ghi trong `readddmeee.txt`; vấn đề hay gặp: resource path / mesh không hiện, phải export `IGN_GAZEBO_RESOURCE_PATH`, xoá `~/.ignition`). wifi_node dùng `/sim_ground_truth_pose` khi có.
- `readddmeee copy.txt` (12-28): bản mô tả định vị hệ thống là "Semantic Multi-modal SLAM".

### Giai đoạn 2 — Làm cứng backend (01/2026, 41 build)
- **01-04 → 01-06**: lỗi build lặp `publishTF_cached` — gọi với `Header` trong khi khai báo nhận `rclcpp::Time` (lệch `.h`/`.cpp`). Đã sửa → hiện dùng `publishTF_cached(msg->header.stamp)`.
- **01-15** (15 build): refactor lớn `clw_fg_node.cpp`; có lần dán nhầm code Python (`def`, `#A`) vào C++; lỗi `qualified-id` (thiếu ngoặc).
- **01-21**: thêm noise model visual/wifi, `trajectory_file_path_`, visualization (markers/path). Lỗi `kv.key.chr()` → sửa bằng `gtsam::Symbol sym(kv.key)`; lỗi `km`/`lm` lẫn biến trong marker.
- Kết quả giai đoạn: backend có try/catch `IndeterminantLinearSystemException` (bỏ frame), Huber cho visual, Cauchy cho WiFi, iSAM2 Dogleg, timer viz 2 Hz, lưu quỹ đạo 10 s/lần.

### Giai đoạn 3 — Chuyển sang dữ liệu robot thật (03–04/2026)
- **2026-03-10**: ghi rosbag `robot_data_bag` (996 s, 84 991 msg): `/odom` 47 400, `/camera/image_raw/compressed` 29 559, `/scan` 7 712, `/wifi_status` 320. Không có `/tf` → viết `tf.py` và static TF.
- visual_node chuyển sang chế độ **monocular**: bag không có depth / camera_info → khoảng cách = `fy * H_real / h_pixel`, intrinsics hard-code D435i. Bỏ class "người" khỏi landmark.
- **04-12**: thêm slam_toolbox vào launch để có occupancy map (`system.launch (another copy).py`).
- **04-15**: slam_toolbox bị **tắt publish TF** (`transform_publish_period: 0.0`) để không tranh `map→odom` với backend; RViz ép `use_sim_time`. Đây là cấu hình launch hiện tại.
- **04-19** (12 build), **04-28**: train model riêng `yolo_c7.pt` (4 class: C7 Window, Fire Cabinet, door, fire extinguisher) cho toà nhà C7. Lỗi build 04-28: khai báo lại `Values estimate`, typo `vvoid`.

### Giai đoạn 4 — Chống landmark trùng & drift (05/2026)
- Các thay đổi đánh dấu `[SỬA LỖI n]` trong code (thời điểm chính xác không rõ, khoảng 04–05/2026):
  - Backend: `wifi_xy_sigma` 5.0, yaw sigma 3.14 để WiFi loop **không làm robot "teleport"/giật lùi**.
  - Backend: giới hạn `MAX_OBS_PER_KF = 50` để **tránh nổ graph**.
  - Backend: **data association theo không gian** — ID mới cùng class nằm trong 1 m quanh landmark cũ → alias về ID cũ (`id_alias_map_`), để giảm landmark trùng khi tracker mất dấu.
  - Visual: chỉ lấy 1 detection lớn nhất mỗi class/frame; cần 5 frame liên tiếp (median depth) mới publish landmark; xuất ảnh debug landmark; ThreadPoolExecutor 1 worker để tránh crash.
  - WiFi: không xoá history khi quá 5000 node, chỉ xoá `fp` của 1000 node cũ (giữ map điểm đầu). Tham số: loop radius 10 m, ngưỡng fp 5.0, gap ≥ 100 node, ≥ 10 s giữa 2 loop.
  - Thêm log chung `~/clw_slam_status.log` cho cả 3 node ([BACKEND]/[VISUAL]/[WIFI]).
- **05-07**: `git init` trong `src/`, commit đầu.
- **05-09**: `system.launch with slam toolbox.py`; **05-10**: chốt `system.launch.py`.
- **05-13**: `tf.py` + quy trình 5 terminal trong `readddmeee.txt`.
- **05-14 → 05-18** (11 build): các lần build/chạy cuối; 05-18 chạy bag tạo 344 ảnh landmark + ảnh `1jpeg`.

### Thống kê build
- 142 lần build colcon (12/2025–05/2026): 125 thành công, 14 thất bại (chủ yếu lệch khai báo `.h`/`.cpp`, lỗi cú pháp khi dán code).

---

## Bài học / lưu ý rút ra

- Đổi chữ ký hàm backend → sửa cả `clw_fg_node.h` và `.cpp`; đây là nguồn lỗi build số 1.
- `gtsam::Values` iterate: dùng `gtsam::Symbol sym(kv.key)` rồi `sym.chr()` / `sym.index()`.
- slam_toolbox và backend không được cùng publish `map→odom`.
- WiFi sigma nhỏ + yaw chặt → robot bị kéo giật (teleport); đã nới lỏng.
- Tracker IoU mất dấu sinh ID mới → landmark trùng; xử lý bằng alias theo khoảng cách cùng class.
- Visual node phải chạy với CWD = gốc workspace (đường dẫn model tương đối).

---

## Nhật ký thử nghiệm

Mẫu: `YYYY-MM-DD | dữ liệu | thay đổi/tham số | kết quả | ghi chú`

| Ngày | Dữ liệu | Thay đổi / tham số | Kết quả | Ghi chú |
|---|---|---|---|---|
| 12/2025 | Gazebo custom_maze_v1 / warehouse / hospital | Phiên bản đầu, YOLO ONNX COCO | Chạy được trong mô phỏng | Chi tiết không còn lưu |
| 2026-05-18 | robot_data_bag (loop `-l`) | `system.launch.py` hiện tại, `yolo_c7.pt`, tham số mặc định | 344 ảnh landmark; path backend lệch khỏi hành lang của map slam_toolbox (`1jpeg`) | Nghi do ID WiFi lệch + WiFi giả lập từ odom |
| 2026-10-21 | — | Phân tích toàn dự án, tạo CLAUDE.md & MEMORY.md | — | Phát hiện các file ngoài `/home/synv/...` đã mất |
| 2026-10-21 | — | Ổ cứng cũ hỏng; chỉ còn bản code giai đoạn này. Máy mới: Ubuntu 22.04.5, Python 3.10, RTX 4060 (driver 595), **chưa có ROS 2, pip, git** | Bắt đầu cài lại môi trường | Build cũ dùng **GTSAM 4.3a1 build từ source vào `/usr/local`** (theo `build/clw_fg_slam/CMakeCache.txt`), OpenCV hệ thống, cv_bridge Humble |
| 2026-10-21 | — | Cài xong môi trường: ROS 2 Humble, **GTSAM 4.2 từ source** (`/usr/local`, system Eigen, no march-native, no TBB), `numpy 1.26.4`, `opencv-python 4.11`, ultralytics, torch CUDA OK. Xoá `build/ install/` cũ, build lại OK | Package build thành công | Cần gói `ros-humble-rosbag2-storage-mcap` để đọc bag. Log cài đặt: `Caithuvien.txt`. **Lưu ý**: `pip install ultralytics` kéo `setuptools 84` vào `~/.local`, xung đột `colcon-core` (cần `<80`) → `pip3 install --user "setuptools<80"`. torch thực tế: 2.14.1+cu130. GTSAM config.h ghi "4.2a9" nhưng đúng là tag release 4.2 |
| 2026-10-21 | robot_data_bag (đọc msg đầu) | Kiểm tra cảm biến: `/scan` frame `laser_frame`, 720 tia, 0–2π, 0.15–12 m; `/odom` odom→base_link; ảnh JPEG 640×480 frame `camera_link`; `/wifi_status` = JSON list `{ssid, bssid, signal, channel, type}` (WiFi **thật**, SSID `Hust_C7`) | Intrinsics hard-code 640×480 là khớp | `/wifi_status` dùng được cho fingerprint thật |
| 2026-10-21 | — | Tạo lại `config/slam_toolbox_params.yaml` (từ mẫu online_sync: base_link, laser 0.15–12 m, travel 0.3, TF tắt) và `config/clw_fg.rviz` (Map, TF, Scan, Path, Markers, Odom tắt, ảnh YOLO; fixed frame `map`, nhìn từ trên). Launch dùng `get_package_share_directory` | Launch nạp đủ 5 node; slam_toolbox đọc params OK | Chưa chạy end-to-end với bag |
| 2026-10-21 | robot_data_bag `-r 3`, ~65 s wall (~200 s bag) | Thêm `map_snapshot_node.py` (ảnh mỗi 10 s + `stats.csv`), vào launch & CMake. Test headless: bag + tf + slam_toolbox + backend + snapshot (không YOLO/WiFi) | 9 ảnh, 406 keyframes, 90 m. Không có landmark/WiFi nên path tối ưu ≈ odom (lệch cuối 0.4 m). **Quan sát quan trọng**: đoạn hành lang đầu trùng map, nhưng sau khúc rẽ quỹ đạo backend lệch hướng ~20–25° khỏi hành lang của slam_toolbox (trôi góc odom ở khúc quay) | Giải thích ảnh `1jpeg` cũ: map slam_toolbox đã sửa trôi bằng scan matching, backend thì không. Ảnh test: scratchpad, không lưu trong workspace |
| 2026-10-21 | robot_data_bag `-r 3`, 30 s | Backend: thêm `logMsg()` thay mọi `RCLCPP_*`, ghi file `backend_logs/backend_<time>.log` + `latest.log` (ghi đè); `writeStatusLog` cũng ghi vào với tag `[STATUS]`; banner in thêm WiFi sigma, keyframe thresholds, đường dẫn file; dòng `BACKEND SHUTDOWN` khi tắt. Sửa `rclpy.shutdown()` 2 lần khi Ctrl-C ở wifi_node/visual_node | File log đủ 100% log backend (165 dòng/133 keyframe), không warning khi build; wifi_node tắt sạch | Log rất dày dòng `ISAM2 Update Run #` (1 dòng/keyframe) — lọc bằng `grep -v STATUS` nếu chỉ cần sự kiện |
| 2026-10-21 | robot_data_bag, lần chạy của người dùng 12:16:43 (full launch có YOLO+WiFi+RViz, ~147 s bag) | — | 14 landmark thật (ảnh ghi 28 do lỗi vẽ). Offset odom-vs-optimized tăng đều 0.9 → 21 m; quỹ đạo tối ưu **ngắn hơn** odom (34 vs 49 m) → landmark factor đang co/kéo lệch quỹ đạo (nghi range monocular sai / gán landmark vào keyframe mới nhất). Hướng hành lang trên map slam_toolbox lệch ~35° so với lần test headless | Lỗi vẽ snapshot: ns `landmarks` có cả TEXT marker (id+10000) → đã sửa chỉ lấy SPHERE. Chưa rõ vì sao map xoay khác — nghi chạy thêm static TF odom→base_link song song tf.py |
| 2026-10-21 | robot_data_bag `-l`, full launch, chạy hết 996 s + 39 s vòng lặp (run 12:16:43, 70 ảnh) | Đánh giá tổng thể | **Kết quả CHƯA đạt**: quỹ đạo tối ưu (ảnh #66) nằm lệch ngoài hành lang slam_toolbox 5–10 m, offset odom-vs-opt 50–65 m. Odom thô trôi góc rất nặng ở khúc quay. 125 landmark nhưng **chỉ 1 landmark được nhận lại sau >60 s** (804 lần re-observe, 659 trong <15 s = cùng 1 lần đi ngang) → landmark không tạo loop closure, chỉ là ràng buộc cục bộ; nhiều landmark trùng (cụm dày) & nằm ngoài tường (range monocular sai). 39 WiFi loop đều sai: ID lệch (wifi x237→x137 thành backend x309↔x142), fingerprint giả lập từ odom, err 0.0% khi xa mọi AP (fingerprint toàn 0). Bag loop (`-l`) làm odom về 0 → backend tạo keyframe nhảy ~900 m, phá đồ thị (ảnh #67–70 vô giá trị) | Thứ tự sửa đề xuất: (1) không dùng `-l` / reset khi odom nhảy, (2) tắt hoặc sửa WiFi (đồng bộ ID + dùng `/wifi_status` thật), (3) thêm ràng buộc từ lidar (scan matching), (4) landmark: gắn theo timestamp, association theo vị trí tốt hơn, kiểm tra range |
