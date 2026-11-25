# Điều khiển mecanum từ ROS 2 qua USB

Luồng: /cmd_vel (geometry_msgs/msg/Twist) → mecanum_bridge.py trên Jetson
→ USB Serial 115200 → ESP32 → hai TB6612 → bốn động cơ.
ESP32 không chạy ROS 2 hoặc micro-ROS. Firmware giải mã PWM từ node cầu nối.
Không encoder: điều khiển vòng hở; không có tốc độ đo hoặc wheel odometry.

## 1. Nạp ESP32
Mở [esp32_mecanum_serial.ino](../../esp32/esp32_mecanum_serial/esp32_mecanum_serial.ino) bằng Arduino IDE.
Chọn ESP32 Dev Module, Arduino-ESP32 core 3.x rồi chọn đúng cổng và Upload.
Code cố ý chặn core khác 3.x vì API LEDC có khác biệt theo phiên bản.
Kê cả bốn bánh trước khi thử. Khởi động không tự chạy.

| Bánh | Driver/kênh | IN1 | IN2 | PWM |
|---|---|---|---|---|
| FL | 1/A | GPIO18 | GPIO19 | GPIO25 |
| FR | 1/B | GPIO21 | GPIO22 | GPIO26 |
| RL | 2/A | GPIO16 | GPIO17 | GPIO23 |
| RR | 2/B | GPIO13 | GPIO14 | GPIO27 |

Cả hai STBY → GPIO4. GPIO12 không dùng. VCC driver → 3V3, VM → 12V,
GND chung. Không cấp 12V vào ESP32/VCC. Không cấp pin 4S trực tiếp vào VM.
Đấu nguồn 5V ngoài và USB đồng thời chỉ khi sơ đồ bo có cơ chế phù hợp chống
cấp ngược nguồn. Không đưa dòng động cơ qua bo ESP32.
MOTOR_SIGN={1,1,1,1} theo FL FR RL RR. Đổi dấu phần tử sang -1 nếu chiều dương
bánh đó chưa phải chiều lăn xe tiến. Không tự đảo dấu trong cả firmware và node.
Khả năng chịu dòng của driver vẫn cần kiểm chứng theo tải, không theo nhãn RPM.

## 2. Chuẩn bị Jetson
Các lệnh dưới đây giả sử Jetson đã có ROS 2 và Python rclpy hoạt động.
Không mặc định Nano đã cài Humble: trước hết kiểm tra môi trường thực tế:

    echo "$ROS_DISTRO"
    python3 -c "import rclpy; from geometry_msgs.msg import Twist"

Nếu import thất bại, cần source setup.bash của bản ROS 2 đang cài hoặc xử lý
môi trường ROS trước. Nếu máy mới chỉ có ROS 1, script này chưa chạy được.

    sudo apt install python3-serial
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

Dùng đúng cổng ESP32; ưu tiên /dev/serial/by-id/... nếu có. Đóng Serial Monitor.
Nếu Permission denied: kiểm tra nhóm cổng. Nếu là dialout:

    sudo usermod -aG dialout "$USER"

Đăng xuất và đăng nhập lại rồi thử. Không cần chạy node bằng sudo.

## 3. Kiểm tra kết nối khi chưa cho phép chạy
Giải nén, cd vào thư mục chứa mecanum_bridge.py và control_core.py.

    python3 mecanum_bridge.py --ros-args -p port:=/dev/ttyUSB0

Mặc định enable_motion=false: chỉ gửi PWM bằng 0, kể cả có /cmd_vel.
Mở terminal khác đã source cùng ROS 2:

    ros2 topic echo /mecanum/status

Ví dụ T,1000,20,0,0,0,0,0: thời gian ESP, số thứ tự lệnh, bốn PWM áp dụng,
trạng thái timeout. Đây không phải tốc độ đo; không xuất /odom hoặc TF giả.

## 4. Đo và khai báo hình học
Quy ước thân xe x trước, y trái, yaw dương ngược chiều kim đồng hồ từ trên nhìn xuống.
- wheel_radius: bán kính lăn của bánh, m.
- lx: nửa khoảng cách giữa tâm bánh trước và tâm bánh sau, m.
- ly: nửa khoảng cách giữa tâm bánh trái và tâm bánh phải, m.
- pwm_per_rad_s: hệ số gần đúng PWM/(rad/s), cần hiệu chuẩn thực nghiệm.

Ma trận sử dụng (thứ tự FL FR RL RR; con lăn 45 độ, bố trí phù hợp ma trận):

    wFL = (vx - vy - (lx+ly)*wz)/r
    wFR = (vx + vy + (lx+ly)*wz)/r
    wRL = (vx + vy - (lx+ly)*wz)/r
    wRR = (vx - vy + (lx+ly)*wz)/r
    pwm_i = pwm_per_rad_s * w_i

Nếu vượt pwm_limit, co đồng thời cả bốn giá trị để giữ tỷ lệ. Không chuẩn hóa
mọi lệnh nhỏ lên PWM tối đa. Đây là mô hình tuyến tính đơn giản chưa bù vùng chết,
điện áp pin, tải và trượt. Không dùng nó để khẳng định xe đạt vận tốc đặt.
Kiểm tra chiều con lăn thực tế trước khi áp dụng ma trận: test tiến, ngang và xoay.

## 5. Cho phép chạy thử
Dừng node cũ bằng Ctrl+C. Kê xe khỏi mặt đất. Ví dụ lệnh SAU CHỈ DÙNG THAM SỐ
GIẢ ĐỊNH: r=0.05m, lx=0.15m, ly=0.13m, gain=40 PWM/(rad/s).
Thay bằng kích thước đo được và hệ số đã thử trên xe; không lấy 319RPM làm tốc độ
thực của bánh khi chịu tải. Nếu chưa biết gain, bắt đầu với giới hạn PWM thấp khi kê bánh.

    python3 mecanum_bridge.py --ros-args \
      -p port:=/dev/ttyUSB0 \
      -p enable_motion:=true \
      -p wheel_radius:=0.05 -p lx:=0.15 -p ly:=0.13 \
      -p pwm_per_rad_s:=40.0 -p pwm_limit:=100

Trong terminal ROS khác, thử tiến ở lệnh 0.1m/s (vận tốc yêu cầu, chưa được đo):

    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}"

Sau vài giây, Ctrl+C ở terminal phát lệnh. Node tự gửi 0 sau cmd_timeout=0.3s.
Thử ngang trái: x=0, y=0.1, z=0. Thử xoay trái: x=0, y=0, z=0.2.
Kiểm tra từng chế độ khi kê bánh trước; chỉ hạ xe xuống khi chiều và bố trí đúng.
Dừng tức thời yêu cầu bằng:

    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"

Chỉ dùng một nguồn phát /cmd_vel. Nếu nguồn khác vẫn phát lệnh khác 0, lệnh dừng
một lần sẽ bị ghi đè. Nút cắt VM là cách dừng vật lý khi thử nghiệm.

## 6. Timeout, phản hồi và lỗi
- Node gửi lệnh 20Hz, QoS cmd_vel reliable/volatile, depth1, nhận Twist thường.
  Nếu publisher dùng TwistStamped thì phải chuyển đổi hoặc sửa subscription.
- cmd_vel không có thời gian trong message; timeout tính từ lúc callback nhận
  bằng đồng hồ monotonic, không phải timestamp nguồn. Không dùng use_sim_time.
- Mất cmd_vel >0.3s: node gửi bốn PWM bằng 0.
- Mất serial command >=300ms: ESP32 dừng độc lập.
- Không ACK mới trong 500ms, reboot ESP32 hoặc serial lỗi: node khóa dừng,
  đóng cổng; sửa lỗi và khởi động lại node. Không tự nối lại và chạy lệnh cũ.
- Đảo chiều: ESP32 chờ 500ms từ lần chạy hướng trước. Đây không phải xác nhận
  bánh đã dừng bằng cảm biến. Trong khoảng chờ PWM phản hồi có thể bằng 0.
- Dừng bằng STBY thấp: thả trôi, không phanh giữ.
- Giới hạn node mặc định: |vx|,|vy|<=0.2m/s; |wz|<=0.5rad/s; pwm_limit100.
  Firmware luôn giới hạn |PWM|<=160 trên thang255.
- Lệnh USB: P,seq,FL,FR,RL,RR\n; truy vấn ?\n.
- Phản hồi: T,ms,seq,FL_applied,FR_applied,RL_applied,RR_applied,timedOut\n.

Nguồn thiết kế:
https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg
https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html
https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html
