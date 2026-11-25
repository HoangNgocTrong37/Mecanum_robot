# ESP32 - Jetson USB serial: PWM và encoder

Giai đoạn 1: kiểm tra truyền thông hai chiều và số đếm encoder. Chưa có ROS,
PID, giới hạn dòng hay odometry. Không phải firmware vận hành tự hành.

## Phần cứng và phần mềm
- ESP32 DevKit loại WROOM-32, Arduino-ESP32 3.x.
- TB6612FNG: AIN1=18, AIN2=19, PWMA=25, BIN1=21, BIN2=22,
  PWMB=26, STBY=27; VCC=3.3V, VM=6V, GND chung.
- Encoder trái A=32 B=33; phải A=34 B=35.
- Chỉ nối encoder sau khi xác minh VCC, GND, A, B và mức điện áp.
  GPIO ESP32 không nhận 5V. GPIO34/35 không có kéo lên nội; tất cả đầu vào
  encoder trong code dùng INPUT, cần tín hiệu được kéo/đẩy mức rõ ràng.
- Không tự cấp 6V cho encoder. Không cấp công suất động cơ từ ESP32.
- Cắm cáp USB DATA từ Jetson tới cổng lập trình của ESP32.
- Jetson cấp nguồn riêng đúng thông số. Đóng Arduino Serial Monitor trước khi chạy Python.

## Thử nghiệm
1. Nạp esp32_usb_encoder/esp32_usb_encoder.ino.
2. Kê hai bánh khỏi mặt đất. Lần đầu để VM động cơ tắt, encoder và ESP32 có nguồn.
3. Trên Jetson (hoặc Linux PC):

```bash
sudo apt update
sudo apt install python3-serial
python3 -m serial.tools.list_ports -v
```

Thay /dev/ttyUSB0 bằng đúng cổng ESP32; nếu LiDAR cũng cắm USB cần phân biệt.
Nếu lỗi quyền truy cập:

```bash
sudo usermod -aG dialout "$USER"
```

Đăng xuất/đăng nhập lại để quyền có hiệu lực; không cần chạy cả chương trình bằng sudo.

```bash
python3 jetson_test.py --port /dev/ttyUSB0 --seconds 20
```

PWM mặc định bằng 0. Quay từng bánh bằng tay và quan sát ticks tương ứng.
Nếu không có tín hiệu, kiểm tra nguồn/tín hiệu encoder trước khi bật động cơ.

Sau khi kiểm tra đúng, bật VM=6V, vẫn kê bánh, thử trái rồi phải:

```bash
python3 jetson_test.py --port /dev/ttyUSB0 --left 80 --right 0 --seconds 3 --enable-motion --csv left.csv
python3 jetson_test.py --port /dev/ttyUSB0 --left 0 --right 80 --seconds 3 --enable-motion --csv right.csv
```

Lệnh số âm quay ngược. Dừng bánh hẳn trước khi thử đảo chiều.
Driver sẽ thả trôi khi dừng, không giữ bánh đứng yên tức thì.
Chạy cả hai bánh chỉ sau khi xác nhận từng bánh và nguồn/driver chịu dòng phù hợp.

## Tham số phải hiệu chỉnh
- LEFT_MOTOR_SIGN và RIGHT_MOTOR_SIGN: đổi 1 thành -1 nếu lệnh dương không làm bánh tiến.
- LEFT_ENCODER_SIGN và RIGHT_ENCODER_SIGN: đổi dấu nếu bánh tiến nhưng ticks giảm.
- Chiều động cơ và dấu encoder là hai cấu hình độc lập.
- Code đếm x2: ngắt CHANGE trên kênh A, đọc B để suy ra chiều.
- Quay bánh đầu ra đúng 10 vòng một chiều: C = abs(ticks_cuoi-ticks_dau)/10.
  Làm lặp lại cho từng bánh và cả hai chiều, dùng cùng firmware.
  Không cố quay tay nếu hộp số khó quay ngược; có thể đánh dấu bánh và dùng
  phép thử quay chậm có kiểm soát, đếm số vòng thực tế.
- Nhãn 130 RPM không cho biết số đếm/vòng hoặc tỉ số truyền chính xác.

## Giao thức
115200 baud, 8N1, kết thúc mỗi gói bằng LF.

Host -> ESP32: P,seq,leftPWM,rightPWM
Ví dụ: P,1,80,0

ESP32 -> host: E,ms,seq,leftTicks,rightTicks,leftApplied,rightApplied,timedOut
ms là millis() trên ESP32; ticks tích lũy có dấu từ lúc reset.
seq là số lệnh nhận gần nhất (ACK), không phải cơ chế bảo mật.
PWM giới hạn -160..160, tương ứng duty |PWM|/255.
Lệnh gửi 20 Hz, phản hồi danh nghĩa 20 Hz.
Không có lệnh hợp lệ trong 300 ms: dừng/thả trôi cả hai bánh.
Lệnh sai định dạng: hủy lệnh chuyển động hiện tại.
Giữ im lặng chưa chắc đồng nghĩa lỗi cáp; có thể host treo hoặc tiến trình chết.
Python đợi liên lạc hai chiều với PWM=0 trước khi gửi lệnh quay.
Mất phản hồi/ACK, reset ESP32 hoặc Ctrl+C: Python yêu cầu dừng.
Nếu không gửi được gói dừng, watchdog ESP32 là lớp dự phòng.
Có khoảng nghỉ 500 ms khi đảo dấu lệnh; không bảo đảm bánh thực tế đã dừng.
Tắt VM vật lý vẫn là cách dừng khi kiểm thử gặp sự cố.

Giao thức ASCII này chưa có CRC và chưa dành cho hệ thống sản phẩm.
Encoder ISR không có lọc nhiễu/phát hiện mọi chuyển trạng thái bất hợp lệ;
nếu tốc độ/xung cao hoặc nhiễu, cần đánh giá mất đếm và có thể dùng PCNT.
Chương trình không phát hiện bánh kẹt và không đo dòng.

## Kích thước và bước tiếp theo
C_L/C_R = số đếm trên một vòng BÁNH theo chế độ x2 hiện tại.
omega = 2*pi*delta_ticks/(C*delta_t), đơn vị rad/s.
v_L = r_L*omega_L; v_R = r_R*omega_R.
v = (v_R+v_L)/2; omega_robot = (v_R-v_L)/b.
r_L/r_R là bán kính lăn hiệu dụng; b là khoảng cách giữa hai vệt tiếp xúc
bánh chủ động theo phương ngang (không phải khoảng cách bánh caster).
Đây là động học phẳng, giả thiết bánh lăn không trượt đáng kể.

Sau khi xác định C, xây PID tốc độ trên ESP32; Jetson gửi tốc độ bánh rad/s.
Node ROS trên Jetson đổi cmd_vel (v, omega) sang hai tốc độ đặt, đọc phản hồi,
tính joint states/odometry và TF. Chỉ một thành phần nên sở hữu odometry/TF.
Khi thay firmware PID, đổi tên/phiên bản giao thức để không nhầm PWM với rad/s.

## Kiểm chứng đã làm
Kiểm tra cú pháp Python và mô phỏng hàm firmware trên PC: parser, PWM,
watchdog 300 ms, thời gian nghỉ đảo chiều, đếm x2 và tràn millis().
Chưa build bằng toolchain ESP32, chưa thử USB/encoder/động cơ thực tế.
