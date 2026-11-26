# Đề xuất cải tiến firmware ESP32 và tầng động cơ

Tài liệu này tổng hợp kết quả rà soát `esp32_mecanum_serial.ino` (commit `42d1358`) và phần
Jetson liên quan (`jetson/motor/`). Mục tiêu: cải thiện tầng động cơ **khi không thể lắp encoder**.

> Các đoạn code dưới đây là **phác thảo thiết kế** để tham khảo, chưa được nạp thử trên xe.
> Mọi thay đổi cần kiểm tra khi **kê bánh khỏi mặt đất** và có nút cắt nguồn VM trong tầm tay.

## Mục lục

1. [Hiện trạng](#1-hiện-trạng)
2. [Danh sách cải tiến](#2-danh-sách-cải-tiến)
3. [Sửa dấu chuyển động ngang và xoay](#3-sửa-dấu-chuyển-động-ngang-và-xoay)
4. [Watchdog phần cứng và điện trở kéo STBY](#4-watchdog-phần-cứng-và-điện-trở-kéo-stby)
5. [Thiết kế đường tín hiệu: ramp, shape, gain, bù pin, kẹp](#5-thiết-kế-đường-tín-hiệu)
6. [Máy trạng thái đầu ra: chạy, phanh, thả trôi](#6-máy-trạng-thái-đầu-ra)
7. [Giao tiếp: phân tích lệnh, seq, lệnh mới, giao thức V2](#7-giao-tiếp)
8. [Xung đột giữa các cải tiến và cách giải quyết](#8-xung-đột-giữa-các-cải-tiến)
9. [Thứ tự triển khai](#9-thứ-tự-triển-khai)
10. [Quy trình hiệu chuẩn không cần encoder](#10-quy-trình-hiệu-chuẩn)
11. [Thay đổi cần làm phía Jetson](#11-thay-đổi-phía-jetson)
12. [Kết quả mong đợi và giới hạn](#12-kết-quả-mong-đợi-và-giới-hạn)
13. [Chỉ số đo trước và sau](#13-chỉ-số-đo-trước-và-sau)

---

## 1. Hiện trạng

### Những điểm đang làm tốt (giữ nguyên)

- Phân tích lệnh chặt: `strtol` có kiểm tra `ERANGE`, giới hạn độ dài buffer, chỉ nhận ký tự in được.
- Watchdog lệnh 300 ms: mất lệnh thì PWM về 0.
- Lệnh `?` không làm mới watchdog lệnh.
- Dùng phép trừ `uint32_t` cho `millis()`, không lỗi khi bộ đếm tràn.
- Chỉ ghi phản hồi khi còn chỗ trong buffer (`availableForWrite`), `loop()` không bị nghẽn.
- Có chuỗi phiên bản `READY,MECANUM_PWM_V1` và chặn biên dịch nếu không phải core 3.x.

### Vấn đề tìm thấy

| # | Vấn đề | Hậu quả |
|---|---|---|
| P1 | Kênh trái/phải nhiều khả năng bị đổi chỗ; đang vá bằng `MANUAL_*_SIGN` ở teleop | `/cmd_vel` chuẩn (Nav2) làm xe đi ngang và xoay ngược, vòng kín thành hồi tiếp dương |
| P2 | Không có watchdog phần cứng | Firmware treo thì PWM cuối được giữ, **xe chạy mãi** |
| P3 | Chặn đảo chiều 500 ms theo **từng bánh** (`guard()`) | Khi chỉ một số bánh đổi dấu, các bánh đó bị ép về 0, xe lệch hướng nửa giây |
| P4 | Không bù vùng chết, không xung mồi | Lệnh nhỏ (Nav2 tiếp cận đích) không làm xe nhúc nhích, progress checker báo lỗi |
| P5 | Quan hệ lệnh và tốc độ phi tuyến, bốn motor lệch nhau | Phải dùng hệ số 3.54 và 4.29 trong `FollowGate`; xe đi thẳng và đi ngang bị trôi |
| P6 | Không bù điện áp pin | Tốc độ giảm theo pin (ước lượng 15–20%), hiệu chuẩn bị trôi |
| P7 | Mọi lần dừng đều thả trôi | Quãng dừng dài, trôi khi đứng trên dốc |
| P8 | Một dòng lệnh lỗi gọi `invalidate()` | Xe khựng (sẽ thành giật mạnh nếu thêm ramp và phanh) |
| P9 | STBY bật tắt mỗi lần PWM thay đổi | Ngắt quãng vài µs không cần thiết, cản trở chế độ phanh |
| P10 | Xung dòng khi khởi động hoặc đảo chiều | TB6612 (1,2 A trung bình, 3,2 A đỉnh mỗi kênh) có thể tự ngắt vì nhiệt, ESP32 có thể sụt áp |
| P11 | Không có thông tin chẩn đoán | Bridge khóa dừng mà không biết nguyên nhân |
| P12 | `seq` không được kiểm tra | Rủi ro thấp qua USB, nhưng không phát hiện lệnh cũ hoặc lặp |

---

## 2. Danh sách cải tiến

| # | Cải tiến | Giải quyết | Loại |
|---|---|---|---|
| C1 | Sửa thứ tự kênh, bỏ `MANUAL_*_SIGN` | P1 | Firmware và Jetson |
| C2 | Watchdog phần cứng 500 ms | P2 | Firmware |
| C3 | Điện trở 10 kΩ kéo STBY xuống GND | P2 (lúc khởi động lại) | Phần cứng, tùy chọn |
| C4 | Ramp đồng bộ, xóa `guard()` | P3, P10 | Firmware |
| C5 | Máy trạng thái đầu ra: chạy, phanh, thả trôi | P7, P9 | Firmware |
| C6 | Shape: vùng chết, xung mồi chung, trễ bật/tắt, `GAIN` | P4, P5 | Firmware |
| C7 | Đo pin, bù sụt áp, ngắt khi pin yếu | P6 | Firmware và mạch phân áp |
| C8 | Kẹp đồng bộ `OUT_MAX` | P10 | Firmware |
| C9 | Bỏ qua dòng lỗi thay vì dừng | P8 | Firmware |
| C10 | Kiểm tra `seq` tăng dần, đặt lại khi nhận `?` | P12 | Firmware |
| C11 | Lệnh `S` (chẩn đoán), `C` (hiệu chuẩn), `W` (lưu flash) | P11 | Firmware |
| C12 | Giao thức V2: thêm điện áp pin và cờ trạng thái | P11, C7 | Firmware và Jetson |
| C13 | PWM 10 bit | Độ mịn ở tốc độ thấp | Firmware, tùy chọn, làm sau cùng |

---

## 3. Sửa dấu chuyển động ngang và xoay

### Vì sao không sửa bằng `MOTOR_SIGN`

Hiện tượng: `vx` đúng, nhưng `vy` và `wz` cùng bị ngược. Thay `vy → −vy`, `wz → −wz` vào ma trận:

```
wFL(vx, −vy, −wz) = (vx + vy + a·wz)/r = wFR
wRL(vx, −vy, −wz) = (vx − vy + a·wz)/r = wRR
```

Kết quả đúng bằng việc **đổi chỗ FL↔FR và RL↔RR**. Đổi dấu từng bánh (`MOTOR_SIGN`) cũng đảo luôn
chiều tiến của bánh đó, nên không tạo ra được hiệu ứng này.

| Chỗ sửa | Đánh giá |
|---|---|
| **Firmware** (đổi vị trí trong mảng chân) | Nên dùng: nhãn FL, FR… trong giao thức khớp với bánh thật |
| **Phần cứng** (đổi dây motor trái/phải trên driver) | Tương đương, không cần nạp lại |
| `control_core.py` | Không nên: phản hồi `T,...` sẽ ghi nhãn sai bánh |
| `MANUAL_*_SIGN` (hiện tại) | Kém nhất: mọi nguồn `/cmd_vel` chuẩn đều sai |

### Bước 1: xác nhận bằng cách quay từng bánh

Kê xe, tắt bridge và container đang giữ cổng serial. Firmware tự dừng sau 300 ms nên cần script gửi liên tục:

```python
# test_wheel.py — dùng: python3 test_wheel.py /dev/ttyUSB0 0 [pwm]  (0=FL 1=FR 2=RL 3=RR)
import serial, sys, time
port, idx = sys.argv[1], int(sys.argv[2])
level = int(sys.argv[3]) if len(sys.argv) > 3 else 60
s = serial.Serial(port=None, baudrate=115200, timeout=0)
s.dtr = s.rts = False            # tránh reset ESP32 khi mở cổng
s.port = port; s.open(); time.sleep(0.5)
pwm = [0, 0, 0, 0]; pwm[idx] = level
end = time.time() + 2
seq = 0
while time.time() < end:
    s.write(f'P,{seq},{pwm[0]},{pwm[1]},{pwm[2]},{pwm[3]}\n'.encode()); seq += 1
    time.sleep(0.05)
s.write(f'P,{seq},0,0,0,0\n'.encode()); s.close()
```

Ghi lại với mỗi chỉ số 0–3: **bánh nào quay** và **quay theo chiều xe tiến hay lùi**.

- `0` làm bánh trước-phải quay, `1` làm bánh trước-trái quay (tương tự 2, 3): kênh trái/phải bị đổi, sửa theo bước 2.
- Đúng bánh nhưng quay theo chiều xe lùi: đổi phần tử tương ứng của `MOTOR_SIGN` thành `-1`.
- Kết quả khác: kiểm tra kiểu lắp con lăn (phải là kiểu X khi nhìn từ trên xuống).

### Bước 2: đổi thứ tự chân trong firmware

```cpp
//                         FL  FR  RL  RR
constexpr uint8_t IN1[4]={21, 18, 13, 16};
constexpr uint8_t IN2[4]={22, 19, 14, 17};
constexpr uint8_t PWM[4]={26, 25, 27, 23};
```

Nếu `MOTOR_SIGN` có phần tử khác 1 thì đổi chỗ theo cùng cặp. Cập nhật bảng chân trong
`jetson/motor/README_VI.md`.

### Bước 3: bỏ phần vá phía Jetson (làm cùng lúc với bước 2)

Trong `jetson/motor/keyboard_drive.py`: đặt `MANUAL_LATERAL_SIGN = 1` và `MANUAL_ROTATION_SIGN = 1`
(`gamepad_receiver.py` và `FollowGate` import hai hằng này). Về lâu dài nên xóa hẳn.

> ⚠️ Chỉ sửa firmware mà quên bước 3 thì hai lần đảo chồng lên nhau, xe lại chạy ngược.

### Bước 4: kiểm tra

Chạy `run_keyboard.sh` khi kê bánh. Theo quy ước ROS: A đi ngang **sang trái**, Q xoay **ngược chiều
kim đồng hồ** nhìn từ trên. Nếu RF2O đang chạy: bấm A thì `/odom_lidar` có `linear.y > 0`, bấm Q thì
`angular.z > 0`.

---

## 4. Watchdog phần cứng và điện trở kéo STBY

### Ba lớp watchdog

| Lớp | Canh cái gì | Khi hỏng thì | Trạng thái |
|---|---|---|---|
| Watchdog lệnh (300 ms) | Jetson, cáp USB | ESP32 tự dừng motor | Đã có |
| Watchdog phần cứng (500 ms) | Chính ESP32 | Chip tự khởi động lại, motor dừng | **Cần thêm** |
| Kiểm tra ACK trong bridge (500 ms) | ESP32 nhìn từ Jetson | Bridge khóa dừng | Đã có |

Watchdog lệnh chỉ là code trong `loop()`. Nếu `loop()` treo thì nó cũng không chạy.

```cpp
#include <esp_task_wdt.h>

void setupWatchdog() {
  esp_task_wdt_config_t cfg = { .timeout_ms = 500, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_reconfigure(&cfg) == ESP_ERR_INVALID_STATE) esp_task_wdt_init(&cfg);
  esp_task_wdt_add(NULL);           // giám sát task loop()
}
// cuối setup(): setupWatchdog();
// đầu loop():   esp_task_wdt_reset();
```

Khi chip khởi động lại, bridge thấy dòng `READY` bất ngờ và tự khóa dừng (đã có trong `tick()`).
Nên ghi `esp_reset_reason()` vào bộ đếm chẩn đoán (xem mục 7).

### Điện trở kéo STBY xuống

Trong lúc vừa cấp điện, đang nạp firmware, hoặc chip đang khởi động lại, GPIO4 bị thả nổi. Điện trở
10 kΩ từ dây STBY chung xuống GND giữ driver ở trạng thái tắt trong các khoảng này.

```
ESP32 GPIO4 ──┬──── STBY (driver 1)
              ├──── STBY (driver 2)
             [10kΩ]
              │
             GND
```

Có thể **không bắt buộc**: theo datasheet, TB6612 có điện trở kéo xuống bên trong (khoảng 200 kΩ, khá
yếu), và GPIO4 của ESP32 thường có kéo xuống lúc reset. Kiểm tra bằng đồng hồ: đo STBY so với GND khi
**giữ nút EN**, khi **đang nạp firmware**, và **ngay lúc cấp điện**. Nếu cả ba đều gần 0 V thì phần có
sẵn đã đủ; 10 kΩ ngoài chỉ là bảo hiểm thêm, chống nhiễu từ motor tốt hơn. GPIO14 (IN2 của RR) phát
xung lúc khởi động, nên STBY ở mức thấp lúc này càng quan trọng.

---

## 5. Thiết kế đường tín hiệu

### 5.1 Thứ tự các khâu và phân chia trách nhiệm

```
Jetson:  Twist → động học → u_i (đơn vị logic, ±160) ──USB──►
ESP32:   u_i → [RAMP đồng bộ] → [SHAPE: bật/tắt, vùng chết, GAIN, xung mồi]
             → [bù pin] → [kẹp đồng bộ] → [máy trạng thái] → TB6612
```

- **Jetson lo động học**: tỉ lệ tốc độ giữa bốn bánh để xe đi đúng hướng.
- **ESP32 lo tuyến tính hóa cơ cấu chấp hành**: làm cho tốc độ bánh tỉ lệ thuận với `u`.

**Định nghĩa lại `u`: "PWM phía trên vùng chết".** `u = 0` là bánh vừa chớm quay, `u = 160` gần tối đa.
Khi đó `pwm_per_rad_s` trên Jetson là hằng số tuyến tính thật sự.

Nên chạy toàn bộ khâu điều khiển ở **nhịp cố định** (ví dụ 500 Hz, dùng `micros()`), tách khỏi phần đọc serial:

```cpp
constexpr uint32_t CONTROL_US = 2000;   // 500 Hz
uint32_t lastTick = 0;
// trong loop():
uint32_t us = micros();
if (uint32_t(us - lastTick) >= CONTROL_US) {
  float dtMs = (us - lastTick) / 1000.0f; lastTick = us;
  control(millis(), dtMs);
}
```

### 5.2 Ramp đồng bộ (C4)

Nguyên tắc:

1. **Đồng bộ**: co cả vector chênh lệch theo cùng một hệ số, để bốn bánh tới đích cùng lúc và hướng
   chuyển động không đổi trong lúc tăng/giảm tốc. Ramp từng bánh độc lập sẽ làm lệch hướng.
2. **Giảm tốc nhanh hơn tăng tốc** (khoảng gấp đôi).
3. **Dừng khẩn (timeout, pin yếu) dùng tốc độ riêng còn nhanh hơn** (`RAMP_STOP`, khoảng 100 ms
   về 0), rồi mới phanh. Không phanh ngắn mạch khi bánh còn quay nhanh (xem xung đột X2).
4. Đi qua 0 tự nhiên khi đảo chiều, nên **xóa `guard()` và `REVERSE_MS`**.

```cpp
constexpr float RAMP_UP   = 0.40f;   // u/ms: 0→160 trong ~400 ms
constexpr float RAMP_DOWN = 0.80f;   // giảm tốc bình thường
constexpr float RAMP_STOP = 1.60f;   // dừng khẩn: 160→0 trong ~100 ms
float state[4] = {};                 // đầu ra ramp; cũng là giá trị báo về trong T,...

void rampSync(const int goal[4], float dtMs, bool emergency) {
  float m = 0, sumG = 0, sumS = 0;
  for (int i = 0; i < 4; ++i) {
    m = fmaxf(m, fabsf(goal[i] - state[i]));
    sumG += fabsf(goal[i]); sumS += fabsf(state[i]);
  }
  if (m < 1e-3f) return;
  float rate = emergency ? RAMP_STOP : (sumG < sumS ? RAMP_DOWN : RAMP_UP);
  float k = fminf(1.0f, rate * dtMs / m);          // cùng một hệ số cho cả 4 bánh
  for (int i = 0; i < 4; ++i) state[i] += (goal[i] - state[i]) * k;
}
```

Chọn `RAMP_UP`: bắt đầu 0,3–0,5 s từ 0 lên tối đa. Nếu `/odom_lidar` có `angular.z` nhảy vọt khi tăng tốc,
hoặc nghe bánh trượt, giảm khoảng 30%. Giữ `max_angular_accel` của Nav2 trong khả năng thật mà ramp cho phép.

### 5.3 Shape: tuyến tính hóa motor (C6)

Mô hình: khi đã quay ổn định, `ω ≈ K_i · (pwm − dead_i)`. Ma sát tĩnh lớn hơn ma sát động, nên ngưỡng
**bắt đầu** quay cao hơn ngưỡng **duy trì** quay.

| Tham số | Ý nghĩa | Cách đo |
|---|---|---|
| `DEAD_FWD/REV[i]` | PWM nhỏ nhất để **duy trì** quay | Bánh đang quay, giảm PWM dần đến khi dừng; lấy giá trị ngay trước đó |
| `KICK_FWD/REV[i]` | PWM để **bắt đầu** quay | Tăng PWM từ 0 đến khi bánh vừa quay, cộng thêm khoảng 10 |
| `KICK_MS` | Thời gian xung mồi | 20–60 ms, ngắn nhất mà vẫn khởi động chắc chắn |
| `KICK_REST_MS` | Xe phải đứng yên bao lâu mới được mồi | Khoảng 100 ms |
| `U_ON / U_OFF` | Ngưỡng bật/tắt có trễ | Ví dụ 4 và 2 |

Quyết định thiết kế quan trọng:

- **Xung mồi chung cho cả xe**, chỉ bật khi xe chuyển từ đứng yên (≥ `KICK_REST_MS`) sang chạy, áp
  cùng lúc cho mọi bánh có lệnh. Xung mồi theo từng bánh sẽ bật lệch thời điểm và làm xe giật sai hướng.
  Bánh đổi dấu giữa chừng (vào cua, đảo chiều) **không** được mồi, chỉ dùng vùng chết.
- **Bật bánh theo lệnh đích (`goal`)**, không theo giá trị đã ramp: khi xe bắt đầu chạy, mọi bánh có lệnh
  cùng bật ở mức vùng chết rồi tăng dần đồng bộ, không có bánh nào "chờ" vượt ngưỡng.
- **`GAIN` chỉ nhân phần phía trên vùng chết.**

```cpp
float GAIN[4]     = {1, 1, 1, 1};
int   DEAD_FWD[4] = {38, 38, 38, 38}, DEAD_REV[4] = {38, 38, 38, 38};
int   KICK_FWD[4] = {70, 70, 70, 70}, KICK_REV[4] = {70, 70, 70, 70};
constexpr int U_ON = 4, U_OFF = 2;
constexpr uint32_t KICK_MS = 40, KICK_REST_MS = 100;
bool wasOn = false; uint32_t offSince = 0, kickUntil = 0;

// Hướng của một bánh: +1, -1, hoặc 0 (tắt).
int wheelDir(float u, int goal) {
  if (abs(goal) >= U_ON && u * goal >= 0) return goal > 0 ? 1 : -1;  // đang tới lệnh đích
  if (fabsf(u) >= U_OFF) return u > 0 ? 1 : -1;                     // đang giảm về 0 / qua 0
  return 0;
}

void shapeAll(const float u[4], const int goal[4], uint32_t now, float out[4]) {
  int dir[4]; bool any = false;
  for (int i = 0; i < 4; ++i) { dir[i] = wheelDir(u[i], goal[i]); any |= dir[i] != 0; }
  if (any && !wasOn && uint32_t(now - offSince) >= KICK_REST_MS) kickUntil = now + KICK_MS;
  if (!any && wasOn) offSince = now;
  wasOn = any;
  bool kick = int32_t(kickUntil - now) > 0;

  for (int i = 0; i < 4; ++i) {
    if (!dir[i]) { out[i] = 0; continue; }
    float a    = (u[i] * dir[i] > 0) ? fabsf(u[i]) : 0.0f;
    float dead = dir[i] > 0 ? DEAD_FWD[i] : DEAD_REV[i];
    float o    = dead + GAIN[i] * a;
    if (kick) o = fmaxf(o, dir[i] > 0 ? KICK_FWD[i] : KICK_REV[i]);
    out[i] = dir[i] * o;
  }
}
```

Nâng cao: nếu quan hệ tốc độ–PWM cong rõ (thường do trượt ở tốc độ cao), thay `dead + GAIN·a`
bằng bảng tra nội suy 5–6 điểm. Làm phiên bản tuyến tính trước.

### 5.4 Gain: cân bằng bốn bánh (C6)

1. Chỉ nhân vào phần phía trên vùng chết (vùng chết đã hiệu chuẩn riêng).
2. **Chuẩn hóa để giá trị lớn nhất bằng 1**: bánh yếu nhất 1,0, bánh khỏe hơn nhỏ hơn 1. Gain không
   bao giờ đẩy PWM vượt giới hạn. Sau khi chỉnh, hiệu chuẩn lại `pwm_per_rad_s` trên Jetson.
3. Giới hạn trong khoảng **0,7–1,0**. Cần nhỏ hơn 0,7 thì nhiều khả năng là lỗi cơ khí (kẹt bạc đạn,
   con lăn bó), không nên bù bằng phần mềm.
4. Bắt đầu với một gain mỗi bánh; chỉ tách `GAIN_FWD/REV` nếu đi lùi lệch khác hẳn đi tiến.

### 5.5 Bù pin (C7)

Mạch: cầu phân áp từ VM vào **GPIO34** (ADC1, chỉ đọc). Với pin 3S (tối đa 12,6 V): 100 kΩ trên, 22 kΩ
dưới, cho khoảng 2,27 V ở 12,6 V. Hiệu chỉnh `VBAT_DIV` bằng đồng hồ vạn năng.

```cpp
constexpr uint8_t VBAT_PIN = 34;
float VBAT_DIV = (100.0f + 22.0f) / 22.0f;      // hiệu chỉnh theo đo thực tế
constexpr float V_NOM = 11.1f;                   // điện áp LÚC HIỆU CHUẨN
constexpr float V_CUT = 9.9f, V_RESUME = 10.5f;  // 3S: ~3,3 V/cell và ~3,5 V/cell
constexpr uint32_t CUT_HOLD_MS = 2000;
float vbat = 0; bool lowBatt = false; uint32_t belowSince = 0;

void updateBattery(uint32_t now) {               // gọi ~50 Hz
  float v = analogReadMilliVolts(VBAT_PIN) * VBAT_DIV / 1000.0f;
  vbat = (vbat == 0) ? v : vbat + (v - vbat) * 0.02f;   // lọc thông thấp, τ ≈ 1 s ở 50 Hz
  if (!lowBatt) {
    if (vbat < V_CUT) { if (!belowSince) belowSince = now;
                        else if (uint32_t(now - belowSince) >= CUT_HOLD_MS) lowBatt = true; }
    else belowSince = 0;
  } else if (vbat > V_RESUME) { lowBatt = false; belowSince = 0; }
}
```

- **Lọc là bắt buộc**: bù theo giá trị tức thời tạo vòng lặp (PWM tăng → dòng tăng → áp sụt → bù tiếp).
- **Ngắt có trễ** (giữ 2 s, mở lại ở ngưỡng cao hơn) để sụt áp lúc tăng tốc không gây ngắt nhầm.
- Chưa có mạch đo thì đặt `vbat = 0` để tắt phần bù.

### 5.6 Kẹp đồng bộ (C8)

Cắt riêng bánh bị vượt sẽ làm sai tỉ lệ giữa các bánh. Co **phần phía trên vùng chết của cả bốn bánh**
theo cùng hệ số:

```cpp
constexpr int OUT_MAX = 230;            // giới hạn cứng (thang 255), chừa khoảng trống
bool clamped = false;

void finalize(float out[4]) {
  float f = (vbat > 6.0f) ? V_NOM / vbat : 1.0f;
  for (int i = 0; i < 4; ++i) out[i] *= f;
  float k = 1.0f;
  for (int i = 0; i < 4; ++i) {
    float a = fabsf(out[i]);
    if (a > OUT_MAX) {
      float dead = (out[i] > 0 ? DEAD_FWD[i] : DEAD_REV[i]) * f;
      k = fminf(k, (OUT_MAX - dead) / (a - dead));
    }
  }
  clamped = k < 1.0f;
  if (clamped) for (int i = 0; i < 4; ++i) if (out[i]) {
    float s = out[i] > 0 ? 1 : -1, dead = (s > 0 ? DEAD_FWD[i] : DEAD_REV[i]) * f;
    out[i] = s * (dead + (fabsf(out[i]) - dead) * k);
  }
}
```

Điều kiện để gần như không bao giờ chạm kẹp trong điều kiện bình thường:

```
(DEAD_max + u_max) × V_NOM / V_CUT ≤ OUT_MAX
```

Ví dụ `V_NOM = 11,1`, `V_CUT = 9,9`, `DEAD = 38`, `OUT_MAX = 230` cho ra `u_max ≈ 167`. Nếu đặt
`V_NOM = 12,6` thì `u_max` chỉ còn khoảng 143. Đặt `pwm_limit` trên Jetson theo công thức này.

---

## 6. Máy trạng thái đầu ra

Phanh ngắn mạch của TB6612 (IN1 = IN2 = HIGH) **chỉ hoạt động khi STBY ở mức HIGH**. Vì vậy STBY không
được tự động kéo xuống mỗi khi PWM bằng 0 như hiện tại.

| Trạng thái | STBY | IN1/IN2 | Khi nào |
|---|---|---|---|
| `RUN` | HIGH | theo dấu; bánh bằng 0 thì LOW/LOW | Có ít nhất một bánh khác 0 |
| `BRAKE` | HIGH | HIGH/HIGH | Vừa dừng khẩn (timeout, pin yếu), giữ `BRAKE_MS` ≈ 300 ms |
| `COAST` | LOW | LOW/LOW | Sau phanh, hoặc sau khi dừng bình thường |

```cpp
enum Mode { RUN, BRAKE, COAST };
Mode mode = COAST; uint32_t modeSince = 0;
constexpr uint32_t BRAKE_MS = 300;
constexpr int PWM_FULL = 255;

void updateMode(uint32_t now, bool emergency, const float out[4]) {
  bool any = false;
  for (int i = 0; i < 4; ++i) any |= out[i] != 0;
  Mode next = mode;
  if (any) next = RUN;
  else if (mode == RUN) next = emergency ? BRAKE : COAST;
  else if (mode == BRAKE && uint32_t(now - modeSince) >= BRAKE_MS) next = COAST;
  if (next != mode) { mode = next; modeSince = now; }
}

void writeDriver(const float out[4]) {
  if (mode == COAST) {
    digitalWrite(STBY, LOW);                       // tắt trước, rồi mới đổi chân
    for (int i = 0; i < 4; ++i) { digitalWrite(IN1[i], LOW); digitalWrite(IN2[i], LOW);
                                  ledcWrite(PWM[i], PWM_FULL); }
    return;
  }
  for (int i = 0; i < 4; ++i) {
    if (mode == BRAKE) { digitalWrite(IN1[i], HIGH); digitalWrite(IN2[i], HIGH);
                         ledcWrite(PWM[i], PWM_FULL); continue; }
    int c = int(lroundf(out[i])) * MOTOR_SIGN[i];
    digitalWrite(IN1[i], c > 0 ? HIGH : LOW);
    digitalWrite(IN2[i], c < 0 ? HIGH : LOW);
    ledcWrite(PWM[i], c ? abs(c) : PWM_FULL);
  }
  digitalWrite(STBY, HIGH);                        // bật sau khi chân đã đúng
}
```

Ghép lại:

```cpp
void control(uint32_t now, float dtMs) {
  bool timedOut  = !validCommand || uint32_t(now - lastCommand) >= TIMEOUT_MS;
  bool emergency = timedOut || lowBatt;
  int goal[4];
  for (int i = 0; i < 4; ++i) goal[i] = emergency ? 0 : target[i];
  rampSync(goal, dtMs, emergency);
  float out[4];
  shapeAll(state, goal, now, out);
  finalize(out);
  updateMode(now, emergency, out);
  writeDriver(out);
}
```

---

## 7. Giao tiếp

### 7.1 Bỏ qua dòng lỗi (C9)

Hiện tại dòng lỗi gọi `invalidate()` và dừng ngay. Khi đã có ramp và phanh, một dòng nhiễu sẽ gây
**phanh gắt rồi tăng tốc chậm**. Thay bằng: bỏ qua dòng đó, tăng `errLines`, giữ lệnh hợp lệ gần nhất.
An toàn vẫn đảm bảo vì timeout 300 ms sẽ dừng xe nếu không có lệnh hợp lệ mới.

### 7.2 Kiểm tra `seq` (C10)

Bridge đếm lại từ 0 mỗi lần khởi động. Nếu firmware nhớ `lastSeq` của phiên trước thì sẽ từ chối mọi
lệnh và **bắt tay thất bại**. Vì vậy:

- **Đặt lại `lastSeq = -1` khi nhận `?`** (bridge luôn gửi `?` trước).
- Có thể đặt lại thêm khi đã timeout.
- Từ chối `seq <= lastSeq`, trừ trường hợp quay vòng (`lastSeq - seq > 500000000`).

### 7.3 Lệnh mới (C11)

| Lệnh | Tác dụng | Ràng buộc |
|---|---|---|
| `S` | Trả về bộ đếm: dòng lỗi, số lần timeout, `esp_reset_reason()`, thời gian chạy, `vbat` | Không làm mới watchdog lệnh |
| `C,<tên>,<bánh>,<giá trị>` | Đổi tham số trong RAM, ví dụ `C,GAIN,2,0.94` | **Chỉ khi xe đang dừng**; kiểm tra khoảng hợp lệ (GAIN 0,7–1,0; DEAD 0–120; KICK ≤ 150; RAMP có giới hạn trên) |
| `W` | Lưu tham số xuống flash (`Preferences`) | **Chỉ khi xe đang dừng**; không ghi tự động |

- Firmware hiện coi mọi dòng lạ là lỗi, nên phải phân tích `S`, `C`, `W` tường minh.
- Mọi lệnh in ra phải kiểm tra `availableForWrite()`; in chặn có thể kéo dài tới mức watchdog phần cứng
  khởi động lại chip.
- `C` phải chặn `RAMP` quá lớn, vì ramp giờ là lớp bảo vệ duy nhất cho tình huống đảo chiều.

### 7.4 Giao thức V2 (C12)

Gộp **mọi thay đổi giao thức vào một lần nâng cấp**:

```
Host:  P,seq,FL,FR,RL,RR          (không đổi; u = PWM phía trên vùng chết, ±160)
       ?  S  C,...  W
Board: READY,MECANUM_PWM_V2
       T,ms,seq,FL,FR,RL,RR,timedOut,vbat_mV,flags
       flags: bit0 pin yếu | bit1 đang kẹp OUT_MAX | bit2 đang phanh | bit3 có dòng lỗi gần đây
```

- Các trường FL…RR báo `lround(state[i])` (giá trị logic sau ramp, luôn trong ±160), **không** báo PWM thật.
- Sửa `feedback()` trong `control_core.py` (đang kiểm tra cứng `len(f) != 8`) và chuỗi `READY` trong
  `mecanum_bridge.py` **trong cùng một commit**. Firmware V2 và bridge V1 sẽ tự từ chối nhau khi bắt tay,
  đây là hành vi đúng.
- Bridge nên coi `flags & 1` (pin yếu) là lỗi có thông báo rõ ràng, thay vì để Nav2 báo lỗi tiến độ.

### 7.5 Bố trí chân

| Chân | Dùng cho | Ghi chú |
|---|---|---|
| 4 | STBY | |
| 13, 14, 16, 17, 18, 19, 21, 22 | IN1/IN2 | **16, 17 là chân PSRAM trên module WROVER**, chỉ dùng được với WROOM |
| 23, 25, 26, 27 | PWM | |
| 34 | Đo pin | ADC1, chỉ đọc vào |
| 32, 33 | I2C cho IMU sau này (`Wire.begin(32, 33)`) | 21, 22 (I2C mặc định) đã dùng cho motor |
| 35, 36, 39 | Còn trống, chỉ đọc vào | |
| 0, 2, 5, 12, 15 | Tránh dùng | Chân strapping |

---

## 8. Xung đột giữa các cải tiến

| # | Mức | Xung đột | Cách giải quyết |
|---|---|---|---|
| X1 | 🔴 | Phanh (C5) cần STBY HIGH, nhưng firmware hiện kéo STBY xuống khi mọi bánh bằng 0 | Máy trạng thái `RUN → BRAKE → COAST`; chỉ đổi STBY khi chuyển trạng thái |
| X2 | 🔴 | Phanh ngắn mạch khi bánh đang nhanh tạo dòng gần bằng dòng kẹt, vượt 3,2 A đỉnh của TB6612 | Dừng khẩn: ramp `RAMP_STOP` về 0 (~100 ms) **rồi mới** phanh |
| X3 | 🔴 | Kiểm tra `seq` (C10) làm hỏng bắt tay vì bridge đếm lại từ 0 | Đặt lại `lastSeq` khi nhận `?` |
| X4 | 🔴 | Ý nghĩa mới của `u` (C6): cùng lệnh `u = 140` thành khoảng 178 PWM, xe chạy nhanh hơn đột ngột | Hạ giới hạn phía Jetson **cùng lúc** với nạp firmware (mục 11) |
| X5 | 🟠 | Dòng lỗi dừng ngay + ramp + phanh gây giật mạnh | Bỏ qua dòng lỗi, dựa vào timeout (C9) |
| X6 | 🟠 | Xung mồi từng bánh bật lệch thời điểm, phá tính đồng bộ của ramp | Xung mồi chung, bật bánh theo `goal` (mục 5.3) |
| X7 | 🟠 | Xung mồi khi đảo chiều tạo xung dòng (bánh còn quán tính), mất bảo vệ của `guard()` cũ | Chỉ mồi khi xe đã đứng yên ≥ `KICK_REST_MS`; `C` chặn `RAMP` quá lớn |
| X8 | 🟠 | Bù pin, `GAIN` và kẹp dùng chung khoảng dư PWM; bù theo giá trị tức thời gây dao động | `V_NOM` = điện áp lúc hiệu chuẩn; `pwm_limit` theo công thức mục 5.6; lọc thông thấp; ngắt có trễ |
| X9 | 🟠 | Ngắt pin yếu mà vẫn ACK bình thường, Jetson không biết lý do | Giao thức V2 có cờ; sửa bridge cùng commit |
| X10 | 🟡 | Lệnh `C`/`S` bị coi là dòng lỗi; in chặn có thể kích watchdog phần cứng | Phân tích tường minh; mọi lệnh in kiểm tra `availableForWrite()` |
| X11 | 🟡 | Ghi flash chặn `loop()` hàng chục ms và làm mòn flash | `C` chỉ sửa RAM; `W` lưu riêng; cả hai chỉ khi xe dừng |
| X12 | 🟡 | PWM 10 bit (C13) làm sai mọi hằng số, kể cả **255 cho trạng thái nghỉ** | Định nghĩa `PWM_FULL = (1 << PWM_BITS) - 1`, viết hằng số theo nó; làm sau cùng |
| X13 | 🟡 | Bảng `DEAD`, `GAIN` đánh số theo bánh; bù pin thay đổi giá trị đo | Sửa thứ tự kênh và lắp mạch đo pin **trước khi** hiệu chuẩn |

### Không xung đột

- Watchdog phần cứng (500 ms) và watchdog lệnh (300 ms) canh hai thứ khác nhau.
- Điện trở kéo STBY hỗ trợ watchdog: khi chip khởi động lại, motor chắc chắn dừng.
- Chân đo pin (34) và I2C cho IMU (32, 33) không trùng chân motor.
- Kẹp đồng bộ và ramp đồng bộ cùng nguyên tắc giữ tỉ lệ giữa các bánh.

---

## 9. Thứ tự triển khai

Mỗi bước là một commit nhỏ, nạp và kiểm tra khi kê bánh trước khi sang bước sau.

1. **Sửa thứ tự kênh** (C1) và bỏ `MANUAL_*_SIGN` phía Jetson.
2. **Watchdog phần cứng** (C2), **máy trạng thái đầu ra** (C5), **bỏ qua dòng lỗi** (C9),
   **`seq` và đặt lại khi nhận `?`** (C10). Đo STBY, lắp 10 kΩ nếu cần (C3).
3. **Ramp đồng bộ** (C4), xóa `guard()` và `REVERSE_MS`.
4. **Mạch đo pin và bộ lọc** (C7), trước khi hiệu chuẩn.
5. **Giao thức V2** (C12) cùng sửa `control_core.py` và `mecanum_bridge.py` trong cùng commit.
6. **Shape** (C6) và **kẹp đồng bộ** (C8), kèm lệnh `S`, `C`, `W` (C11). **Hạ giới hạn phía Jetson cùng lúc.**
7. **Hiệu chuẩn** theo mục 10.
8. (Tùy chọn) **PWM 10 bit** (C13).

---

## 10. Quy trình hiệu chuẩn

Thứ tự: vùng chết và xung mồi → `GAIN` → ramp → `pwm_per_rad_s` → bỏ hệ số 3.54 và 4.29 trong `FollowGate`.

### 10.1 Vùng chết và xung mồi

1. Kê xe. Dùng `test_wheel.py` với tham số `pwm`, mỗi bánh và mỗi chiều:
   - Tăng PWM mỗi lần 2 đơn vị đến khi bánh **bắt đầu** quay: giá trị gốc cho `KICK`.
   - Từ lúc đang quay, giảm dần đến khi bánh **dừng**: giá trị gốc cho `DEAD`.
2. Đặt xe xuống sàn. Tăng toàn bộ `DEAD` cùng một lượng (thường +10–20%) đến khi lệnh `u` nhỏ (khoảng 5)
   vẫn làm cả xe nhích đều.
3. Chỉnh `KICK_MS` ngắn nhất mà xe vẫn khởi động chắc chắn từ đứng yên.

### 10.2 `GAIN` bằng RF2O

Cho xe chạy trên sàn, đọc `/odom_lidar`. Mỗi kiểu lệch chỉ ra một cặp bánh mất cân bằng (suy ra từ
ma trận động học thuận):

| Thử | Độ lệch | Nghĩa là | Chỉnh |
|---|---|---|---|
| Tiến thẳng (W) | `angular.z ≠ 0` | Trái và phải không đều | `wz < 0` (quẹo phải): giảm GAIN FL, RL hoặc tăng FR, RR |
| Ngang trái (A) | `linear.x ≠ 0` | Hai cặp chéo (FR+RL so với FL+RR) không đều | `vx > 0`: giảm GAIN FR, RL |
| Ngang trái (A) | `angular.z ≠ 0` | Trước và sau không đều | `wz > 0`: giảm GAIN FL, FR |

Mỗi lần chỉnh 2–3%, lặp 2–3 vòng, rồi chuẩn hóa để giá trị lớn nhất bằng 1. Mục tiêu: đi 2 m chỉ lệch
vài cm. Dùng `C,GAIN,<bánh>,<giá trị>` để thử, `W` để lưu khi đã ổn.

### 10.3 Ramp

Tăng tốc mạnh trên sàn. Giảm `RAMP_UP` nếu `angular.z` nhảy vọt hoặc nghe bánh trượt. Kiểm tra quãng
dừng khẩn bằng cách tắt bridge khi đang chạy.

### 10.4 `pwm_per_rad_s`

Chạy vài mức `vx` cố định, so tốc độ đo từ `/odom_lidar` với lệnh, fit hệ số tuyến tính. Nếu xoay cần
hệ số khác tịnh tiến (do con lăn trượt), thêm `k_rot` riêng trong `to_pwm`. Sau đó bỏ các hệ số
3.54 và 4.29 trong `FollowGate`.

---

## 11. Thay đổi phía Jetson

| File | Thay đổi | Đi cùng bước |
|---|---|---|
| `keyboard_drive.py` | `MANUAL_LATERAL_SIGN = MANUAL_ROTATION_SIGN = 1` | 1 |
| `keyboard_drive.py`, `gamepad_receiver.py` | Hạ `PWM_LIMIT`, `FORWARD_REQUEST`, `LATERAL_REQUEST`, `ROTATION_REQUEST`, `pwm_limit` của chế độ follow (khoảng −40) | 6 |
| `control_core.py` | `feedback()` nhận 10 trường của V2 | 5 |
| `mecanum_bridge.py` | Chuỗi `READY,MECANUM_PWM_V2`; xử lý cờ pin yếu; đọc bộ đếm `S` khi khóa dừng | 5 |
| `control_core.py` | `pwm_per_rad_s` mới, có thể thêm `k_rot` | 7 |
| `gamepad_receiver.py` | Bỏ hệ số 3.54 và 4.29 trong `FollowGate` | 7 |
| `/boot/extlinux/extlinux.conf` | Thêm `usbcore.autosuspend=-1` để tránh cổng serial rớt ngắn | Bất kỳ |
| Bridge hoặc node mới | Phát hiện kẹt bánh: lệnh khác 0 kéo dài > 1 s mà `/odom_lidar` gần như đứng yên thì dừng | Bất kỳ |

---

## 12. Kết quả mong đợi và giới hạn

### Cải thiện

| Nhóm | Trước | Sau |
|---|---|---|
| Hướng chuyển động | Ngang và xoay ngược; lệch khi đảo chiều | `/cmd_vel` chuẩn chạy đúng; giữ hướng khi tăng/giảm tốc |
| Tốc độ thấp | Lệnh nhỏ không chạy, Nav2 dừng trước đích | Chạy được tốc độ rất thấp, tiếp cận đích mượt |
| Tuyến tính | Cần hệ số ma thuật 3.54 / 4.29 | Một hằng số `pwm_per_rad_s` |
| Cân bằng bánh | Đi thẳng cong, đi ngang trôi chéo | Gần thẳng, rõ nhất ở đi ngang |
| Theo thời gian | Tốc độ giảm theo pin | Ổn định suốt một lần sạc; bảo vệ pin |
| Phần cứng | Xung dòng, TB6612 có thể tự ngắt, ESP32 có thể sụt áp | Dòng đỉnh thấp hơn, driver mát hơn |
| Dừng | Thả trôi, quãng dừng dài | Giảm tốc nhanh rồi phanh |
| An toàn | Firmware treo thì xe chạy mãi | Tự dừng trong ~0,5 s |
| Chẩn đoán | Không biết vì sao bridge khóa dừng | Bộ đếm, lý do khởi động lại, cờ trạng thái |

### Giới hạn khi không có encoder

- **Tải thay đổi** (dốc, mặt sàn, khối lượng) vẫn làm tốc độ thật thay đổi; bù pin chỉ xử lý điện áp.
- **Con lăn mecanum trượt**, nhất là khi đi ngang; chỉ RF2O nhìn thấy được.
- **Đặc tính motor trôi theo nhiệt độ** khi chạy lâu.
- **Một bánh kẹt** không phát hiện được ở tầng động cơ, chỉ phát hiện gián tiếp ở Jetson.
- **Sai số tích lũy** vẫn còn; độ chính xác vị trí vẫn dựa vào vòng ngoài RF2O và AMCL.

Các cải tiến làm cho sai số mà vòng ngoài phải sửa **nhỏ hơn, đều đặn hơn và dễ dự đoán hơn**.

---

## 13. Chỉ số đo trước và sau

| Chỉ số | Cách đo |
|---|---|
| Độ lệch ngang khi đi thẳng 2 m | RF2O hoặc thước dây, lặp 5 lần |
| Độ lệch dọc và độ xoay khi đi ngang 1 m | RF2O |
| Sai số tốc độ lệnh và tốc độ thật ở 3–4 mức | So `/cmd_vel` với `/odom_lidar` |
| Tốc độ nhỏ nhất chạy ổn định | Giảm dần lệnh đến khi xe dừng |
| Tốc độ khi pin đầy và gần cạn | Cùng một lệnh, đo hai lần |
| Quãng dừng khi mất lệnh | Tắt bridge khi đang chạy |
| Tỉ lệ thành công bám tuyến Nav2 | 10 lần chạy cùng một tuyến |

Các con số trong tài liệu (độ lệch giữa motor, mức sụt tốc độ theo pin, giá trị `DEAD` và `KICK` mặc
định) là **ước lượng điển hình**; giá trị thật phải đo trên xe.
