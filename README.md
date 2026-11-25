# Xe mecanum điều khiển bằng Jetson Nano và ESP32

Xe 4 bánh mecanum dùng hai TB6612FNG, ESP32 nối USB với Jetson Nano 2 GB, và RPLIDAR A1. Jetson chạy ROS 2 Humble trong Docker. Laptop chạy RViz trên ROS 2 Jazzy và gửi thao tác tay cầm sang Jetson qua SSH.

## Mã nguồn

| Thư mục | Nội dung |
|---|---|
| `mecanum_ros2_usb/firmware/esp32_mecanum_serial/` | Firmware chính của ESP32, sơ đồ chân trong mã và giao thức điều khiển USB. |
| `mecanum_ros2_usb/` | Bridge ROS 2, điều khiển bàn phím trên Jetson, nhận lệnh tay cầm, hiệu chuẩn PWM và kiểm tra kết nối. Chạy `gamepad_drive.py` trên laptop. Xem [hướng dẫn USB và firmware](mecanum_ros2_usb/README_VI.md). |
| `rf2o_jetson/` | LiDAR, RF2O, SLAM Toolbox, AMCL, trình lập đường và điều khiển Nav2. Các tệp `view_map.sh`, `jetson_map.rviz`, `relay_map.py` và `run_selected_follow.sh` dành cho laptop. |
| `rf2o_jetson/rf2o_laser_odometry/` | Mã RF2O dùng trên xe, gồm các sửa đổi để xuất vận tốc ngang và quay trong hệ tọa độ thân xe. Xem nguồn gốc trong `deployment.json` và giấy phép trong thư mục RF2O. |
| `hardware_tests/` | Sketch kiểm tra cũ và bộ kiểm tra giao tiếp trước ROS. Firmware dùng trên xe nằm trong `mecanum_ros2_usb/firmware/`. |

## Cấu hình của xe

- Bán kính bánh 0,047 m. Nửa khoảng cách tâm bánh trước–sau 0,092 m, trái–phải 0,1375 m.
- LiDAR cách tâm xe 0,06 m về phía trước và cao 0,165 m. Hướng 180° của LiDAR trùng với hướng trước xe.
- RF2O tạo `/odom_lidar` từ `/scan`. AMCL định vị xe trên bản đồ đã lưu. Nav2 dùng dữ liệu này để lập đường và bám tuyến. Chế độ chạy thử tuyến yêu cầu giữ L1.

## Triển khai

Tên các thư mục trong các lệnh phản ánh bố trí đã dùng trên xe. Trên Jetson, mã cầu nối đặt tại `~/mecanum_ros2_usb`. Các script LiDAR và Nav2 đặt tại `~/lidar_test`. Container `lidar-ros` gắn thư mục sau thành `/results` và ánh xạ cổng LiDAR thành `/dev/rplidar`. Gói RF2O được build trong workspace `/results/rf2o_ws`. Container động cơ dùng image `jetson-ros2:humble`. Các đường dẫn thiết bị USB trong script tương ứng với cổng đã dùng trên Jetson này.

Trên laptop, `bash rf2o_jetson/view_map.sh` mở RViz và relay SSH. Địa chỉ SSH và đường dẫn khóa trong script cần khớp với máy đang dùng. `run_selected_follow.sh` tìm `gamepad_drive.py` ở `~/Downloads/mecanum_ros2_usb`. Lượt chạy thử dùng `/results/selected_follow_path.json` được chuẩn bị từ tuyến đã chọn và yêu cầu giữ L1. RViz hiện chọn điểm đích bằng **Publish Point**. Script chạy thử hiện dùng hướng đích 0°.

Bản sao trên laptop có các launch file nhưng thiếu bản cấu hình `slam_toolbox_mecanum.yaml` và `slam_localization_mecanum.yaml` từng được dùng trực tiếp trên Jetson. Cần lấy hai tệp này từ `~/lidar_test` khi Jetson hoạt động trở lại. Bản đồ phòng, pose graph, tuyến thử và log là dữ liệu triển khai cục bộ. Khi dùng bản đồ khác, cập nhật `yaml_filename` trong `rf2o_jetson/nav2_plan.yaml`.
