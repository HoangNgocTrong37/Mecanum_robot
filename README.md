# Xe mecanum điều khiển bằng Jetson Nano và ESP32

Xe dùng Jetson Nano 2 GB, ESP32 nối USB, hai driver TB6612FNG và RPLIDAR A1. Jetson chạy ROS 2 Humble trong Docker. Laptop chạy RViz trên ROS 2 Jazzy và gửi thao tác tay cầm qua SSH.

## Đọc mã nguồn

| Phần | Vị trí |
|---|---|
| Firmware ESP32, cầu nối Twist–PWM, bàn phím và tay cầm | [mecanum_ros2_usb](mecanum_ros2_usb/) |
| Sơ đồ chân, mô hình mecanum, giao thức USB và watchdog | [Hướng dẫn phần động cơ](mecanum_ros2_usb/README_VI.md) |
| Docker, LiDAR, bản đồ, định vị, Nav2 và RViz | [rf2o_jetson](rf2o_jetson/) |

RPLIDAR phát `/scan`. RF2O so khớp các lượt quét và phát `/odom_lidar` cùng TF `odom → base_link`. AMCL định vị xe trên bản đồ và phát TF `map → odom`. Nav2 dùng dữ liệu đó để bám tuyến. Bridge quy đổi lệnh thân xe thành PWM. Tốc độ từng bánh vẫn là vòng hở vì xe không có encoder.

Mã RF2O dựa trên [MAPIRlab/rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry/tree/b38c68e46387b98845ecbfeb6660292f967a00d3), có chỉnh sửa để xuất vận tốc dọc, ngang và xoay trong hệ tọa độ thân xe. [Giấy phép RF2O](rf2o_jetson/rf2o_laser_odometry/LICENSE)

## Tái tạo trên xe khác

### 1. ESP32 và cơ khí

Nạp [firmware ESP32](mecanum_ros2_usb/firmware/esp32_mecanum_serial/esp32_mecanum_serial.ino) bằng Arduino IDE với board **ESP32 Dev Module**, core ESP32 3.x. Đấu dây theo bảng GPIO trong hướng dẫn phần động cơ.

Đo lại bán kính bánh và nửa khoảng cách giữa các trục bánh. Xe gốc dùng `wheel_radius=0.047`, `lx=0.092`, `ly=0.1375` (m) và `pwm_per_rad_s=9.55` làm giá trị khởi đầu. Cập nhật chúng trong `keyboard_drive.py` và `gamepad_receiver.py`. Kiểm tra chiều lăn cùng dấu điều khiển khi kê bánh. Sửa vị trí và góc lắp LiDAR trong `rf2o_bringup.launch.py` và phần kiểm tra vật cản của `gamepad_receiver.py`. Cập nhật footprint trong cả hai YAML Nav2. Xe gốc dài 0,28 m, rộng 0,32 m, với LiDAR tiến 0,06 m, cao 0,165 m và yaw 180°.

### 2. Jetson

Trong thư mục repo trên Jetson, chuẩn bị thư mục chứa cấu hình và build image. Tài khoản trên Jetson cần chạy được Docker.

```bash
mkdir -p ~/mecanum_ros2_usb ~/lidar_test/rf2o_ws/src
cp -r mecanum_ros2_usb/. ~/mecanum_ros2_usb/
cp rf2o_jetson/*.launch.py rf2o_jetson/*.yaml rf2o_jetson/run_{rf2o,slam,localization,nav2_plan}.sh rf2o_jetson/follow_selected.py ~/lidar_test/
cp mecanum_ros2_usb/fastdds_udp.xml ~/lidar_test/
cp -r rf2o_jetson/rf2o_laser_odometry ~/lidar_test/rf2o_ws/src/

docker build --build-arg ROBOT_UID="$(id -u)" --build-arg ROBOT_GID="$(id -g)" \
  -t jetson-ros2:humble rf2o_jetson
```

Xác định cổng bằng `ls -l /dev/serial/by-path/`. Gán `LIDAR_DEVICE` theo cổng LiDAR. Cập nhật giá trị mặc định của `ESP32_DEVICE` trong `~/mecanum_ros2_usb/run_gamepad_host.sh` nếu cổng ESP32 khác, để cả lệnh từ SSH dùng đúng cổng.

```bash
export LIDAR_DEVICE=/dev/serial/by-path/THAY_BANG_CONG_LIDAR
lidar_port=$(readlink -e "$LIDAR_DEVICE")
docker create --name lidar-ros --network host \
  --device "$lidar_port:/dev/rplidar" --group-add "$(stat -c %g "$lidar_port")" \
  -v "$HOME/lidar_test:/results" jetson-ros2:humble
docker start lidar-ros
docker exec lidar-ros bash -c '
  source /opt/ros/humble/setup.bash
  cd /results/rf2o_ws
  colcon build --executor sequential
'
```

Mở `bash ~/lidar_test/run_slam.sh` và lái xe theo mục 3 để quét bản đồ. Khi quét xong, lưu bản đồ từ một terminal Jetson khác trong lúc SLAM còn chạy.

```bash
docker exec -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
  bash -c 'source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /results/map'
```

Nếu dùng chế độ định vị bằng SLAM Toolbox, lưu thêm pose graph khi node lập bản đồ còn chạy.

```bash
docker exec -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
  bash -c 'source /opt/ros/humble/setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /results/map}"'
```

Dừng SLAM, đặt `yaml_filename` trong `~/lidar_test/nav2_plan.yaml` thành `/results/map.yaml` và chỉnh `initial_pose` theo vị trí đặt xe trên bản đồ. Mở `bash ~/lidar_test/run_nav2_plan.sh` với biến `LIDAR_DEVICE` đã đặt. Để định vị bằng SLAM Toolbox, đặt `map_file_name: /results/map` và `map_start_pose` trong `slam_localization_mecanum.yaml`, rồi chạy `bash ~/lidar_test/run_localization.sh` thay cho `run_nav2_plan.sh`.

### 3. Laptop và chạy tuyến

Trong thư mục repo trên laptop đã có ROS 2 Jazzy và RViz, thiết lập kết nối SSH rồi mở RViz. Dùng **Publish Point** để chọn đích.

```bash
export JETSON_SSH_HOST=ten_dang_nhap@dia_chi_jetson
export JETSON_SSH_KEY="$HOME/.ssh/ten_khoa_rieng"
bash rf2o_jetson/view_map.sh
```

Để lái tay, ghép tay cầm với laptop và chạy `python3 mecanum_ros2_usb/gamepad_drive.py --drive`. Để lái bằng bàn phím, chạy `bash ~/mecanum_ros2_usb/run_keyboard.sh` trong SSH. Chỉ dùng một chế độ điều khiển tại một thời điểm.

Để bám tuyến đã chọn, mở controller trong một terminal Jetson:

```bash
docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/results/fastdds_udp.xml lidar-ros \
  bash -c 'source /opt/ros/humble/setup.bash && ros2 launch /results/nav2_follow.launch.py'
```

Trong terminal laptop khác, nạp lại hai biến SSH ở trên. Khi RViz đã vẽ tuyến, lưu tuyến hiện tại và chuyển lên Jetson:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=23 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE
route_dir=$(mktemp -d)
ros2 topic echo /plan nav_msgs/msg/Path --qos-durability transient_local --once --timeout 5 > "$route_dir/path.yaml"
python3 - "$route_dir" <<'PY'
import json
import sys
from pathlib import Path

import yaml

directory = Path(sys.argv[1])
path = next(yaml.safe_load_all((directory / 'path.yaml').read_text()))
points = [entry['pose']['position'] for entry in path['poses']]
if len(points) < 2:
    raise SystemExit('Chưa có tuyến hợp lệ trong RViz.')
(directory / 'selected_follow_path.json').write_text(
    json.dumps([[p['x'], p['y']] for p in points])
)
PY
test -s "$route_dir/selected_follow_path.json" &&
  scp -i "$JETSON_SSH_KEY" "$route_dir/selected_follow_path.json" "$JETSON_SSH_HOST:lidar_test/"
```

Chạy `bash rf2o_jetson/run_selected_follow.sh` từ repo trên laptop. Để hai cần analog ở giữa, giữ L1 để bám tuyến và thả L1 để dừng. Tuyến này dùng hướng đích 0°. Bản đồ phòng và tuyến là dữ liệu tạo riêng cho từng xe và môi trường.
