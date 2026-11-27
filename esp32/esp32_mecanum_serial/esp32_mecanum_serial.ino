// =============================================================================
// Firmware ESP32 điều khiển 4 động cơ xe mecanum qua USB Serial
// =============================================================================
// Phần cứng : ESP32-WROOM-32 DevKit, hai driver TB6612FNG, 4 động cơ DC.
// Core      : Arduino-ESP32 3.x (bắt buộc, vì API LEDC khác ở các phiên bản cũ).
// Điều khiển: VÒNG HỞ. Không có encoder, không có PID. Firmware chỉ nhận PWM
//             từ Jetson (mecanum_bridge.py) và xuất ra driver.
//
// Thứ tự bánh dùng ở mọi nơi trong firmware và giao thức: FL FR RL RR
//   FL = trước-trái, FR = trước-phải, RL = sau-trái, RR = sau-phải.
//
// GIAO THỨC (UART qua USB, 115200 8N1, mỗi gói kết thúc bằng '\n'):
//   Jetson → ESP32:
//     P,seq,FL,FR,RL,RR   Lệnh PWM. seq: 0..1000000000; mỗi PWM: -160..160
//                         (dấu = chiều quay, độ lớn = độ rộng xung trên thang 255).
//     ?                   Hỏi định danh firmware. KHÔNG làm mới watchdog lệnh.
//   ESP32 → Jetson:
//     READY,MECANUM_PWM_V1                   Gửi khi khởi động và khi nhận '?'.
//     T,ms,seq,FL,FR,RL,RR,timedOut          Phản hồi mỗi 50 ms: thời gian chạy,
//                                            seq hợp lệ gần nhất, PWM ĐANG ÁP DỤNG
//                                            (không phải tốc độ đo), cờ timeout.
//     ERROR,PWM_INIT                         Khởi tạo PWM thất bại.
//
// AN TOÀN:
//   - Quá TIMEOUT_MS không có lệnh P hợp lệ → toàn bộ PWM về 0.
//   - Mọi gói sai định dạng → hủy lệnh hiện tại, dừng ngay.
//   - Khi tất cả bánh bằng 0, STBY = LOW: hai driver tắt, bánh THẢ TRÔI
//     (không phanh giữ vị trí).
//   - Đảo chiều một bánh phải chờ REVERSE_MS kể từ lần cuối bánh đó chạy
//     theo chiều cũ, để giảm xung dòng.
// =============================================================================

#include <Arduino.h>
#include <esp_arduino_version.h>

// Chặn biên dịch nếu không dùng core 3.x (ledcAttach/ledcWrite theo chân
// chỉ có từ core 3.x).
#if ESP_ARDUINO_VERSION_MAJOR != 3
#error "Select Arduino-ESP32 core 3.x for this firmware"
#endif

#include <stdlib.h>   // strtol
#include <stdio.h>    // snprintf
#include <errno.h>    // errno, ERANGE

// -----------------------------------------------------------------------------
// CẤU HÌNH PHẦN CỨNG
// -----------------------------------------------------------------------------
// Chân điều khiển cho từng bánh, theo thứ tự FL FR RL RR.
//   IN1, IN2: chọn chiều quay (một chân HIGH, chân kia LOW).
//   PWM     : độ rộng xung quyết định tốc độ.
// Bảng đấu dây đầy đủ: jetson/motor/README_VI.md.
constexpr uint8_t IN1[4]={18,21,16,13};
constexpr uint8_t IN2[4]={19,22,17,14};
constexpr uint8_t PWM[4]={25,26,23,27};

// Chân STBY dùng chung cho cả hai TB6612: HIGH = driver hoạt động,
// LOW = driver tắt (motor thả trôi).
constexpr uint8_t STBY=4;

// Dấu chiều quay của từng bánh. Đổi phần tử thành -1 nếu PWM dương làm bánh
// đó quay theo chiều xe LÙI. Chỉ sửa ở đây, không đảo dấu thêm phía Jetson.
constexpr int MOTOR_SIGN[4]={1,1,1,1}; // Change to -1 if positive rotates backward.

// Giới hạn độ lớn PWM nhận từ Jetson (thang 8 bit 0..255).
constexpr int MAX_PWM=160;

// Các mốc thời gian (ms):
//   TIMEOUT_MS : quá thời gian này không có lệnh P hợp lệ thì dừng motor.
//   FEEDBACK_MS: chu kỳ gửi dòng phản hồi T,... về Jetson.
//   REVERSE_MS : thời gian chờ tối thiểu trước khi cho một bánh đảo chiều.
constexpr uint32_t TIMEOUT_MS=300, FEEDBACK_MS=50, REVERSE_MS=500;

// -----------------------------------------------------------------------------
// BIẾN TRẠNG THÁI
// -----------------------------------------------------------------------------
// target[]    : PWM yêu cầu trong lệnh P hợp lệ gần nhất.
// applied[]   : PWM đang thực sự xuất ra driver (sau timeout và chặn đảo chiều).
// lastDir[]   : chiều quay gần nhất của từng bánh (+1, -1, hoặc 0 nếu chưa chạy).
int target[4]={}, applied[4]={}, lastDir[4]={};

// lastActive[]: thời điểm gần nhất từng bánh còn quay (dùng cho chặn đảo chiều).
// lastCommand : thời điểm nhận lệnh P hợp lệ gần nhất (watchdog lệnh).
// lastFeedback: thời điểm gửi phản hồi T gần nhất.
uint32_t lastActive[4]={}, lastCommand=0, lastFeedback=0;

// seq của lệnh P hợp lệ gần nhất; -1 nghĩa là chưa nhận lệnh nào.
long lastSeq=-1;

// validCommand: đang có lệnh hợp lệ (false sau khi khởi động hoặc nhận gói lỗi).
// pwmReady    : khởi tạo PWM thành công, cho phép loop() chạy.
// failed      : khởi tạo PWM thất bại.
bool validCommand=false, pwmReady=false, failed=false;

// Bộ đệm nhận một dòng lệnh.
//   rx     : nội dung dòng đang nhận (tối đa 95 ký tự + '\0').
//   rxLen  : số ký tự đã nhận.
//   discard: dòng hiện tại đã bị đánh dấu lỗi, bỏ qua tới ký tự '\n' tiếp theo.
char rx[96]; size_t rxLen=0; bool discard=false;

// -----------------------------------------------------------------------------
// HÀM TIỆN ÍCH
// -----------------------------------------------------------------------------
// Hủy lệnh hiện tại: đặt mọi PWM yêu cầu về 0 và đánh dấu "không có lệnh hợp lệ",
// nên vòng lặp sẽ coi như timeout và dừng motor ngay.
void invalidate() { validCommand=false; for(int i=0;i<4;++i) target[i]=0; }

// Gửi chuỗi định danh. Bridge trên Jetson dùng chuỗi này để bắt tay, và coi
// việc nhận lại nó giữa chừng là dấu hiệu ESP32 vừa khởi động lại.
void identity() { Serial.println("READY,MECANUM_PWM_V1"); }

// Đọc một số nguyên có dấu tại vị trí p, yêu cầu số phải kết thúc ĐÚNG bằng
// ký tự terminator (',' cho các trường giữa, '\0' cho trường cuối).
// Từ chối: ký tự đầu không phải '-' hoặc chữ số (kể cả '+' và khoảng trắng),
// số tràn kiểu long, chuỗi rỗng, hoặc có ký tự thừa sau số.
// Thành công: ghi kết quả vào value, dời p qua dấu phân cách, trả về true.
bool integerField(const char*& p,long& value,char terminator) {
  if(!(*p=='-' || (*p>='0' && *p<='9'))) return false;
  char* end=nullptr; errno=0; value=strtol(p,&end,10);
  if(errno==ERANGE || end==p || *end!=terminator) return false;
  p=end+(terminator!=0); return true;
}

// -----------------------------------------------------------------------------
// PHÂN TÍCH MỘT DÒNG LỆNH (đã bỏ '\r' và '\n')
// -----------------------------------------------------------------------------
void parse(const char* line) {
  // Lệnh "?" : chỉ trả lời định danh, không ảnh hưởng tới lệnh đang chạy.
  if(line[0]=='?' && line[1]==0) { identity(); return; }

  // Mọi lệnh khác phải bắt đầu bằng "P,". Sai thì dừng ngay để an toàn.
  if(line[0]!='P' || line[1]!=',') { invalidate(); return; }
  const char* p=line+2; long seq,v[4];

  // Trường seq: số nguyên trong khoảng 0..1000000000.
  if(!integerField(p,seq,',') || seq<0 || seq>1000000000L) { invalidate(); return; }

  // Bốn trường PWM: ba trường đầu kết thúc bằng ',', trường cuối kết thúc dòng.
  // Mỗi giá trị phải nằm trong -MAX_PWM..MAX_PWM.
  for(int i=0;i<4;++i) {
    if(!integerField(p,v[i],i==3 ? 0 : ',') || v[i]<-MAX_PWM || v[i]>MAX_PWM) {
      invalidate(); return;
    }
  }

  // Gói hợp lệ hoàn toàn mới được áp dụng (không áp dụng nửa chừng),
  // đồng thời làm mới watchdog lệnh.
  for(int i=0;i<4;++i) target[i]=int(v[i]);
  lastSeq=seq; lastCommand=millis(); validCommand=true;
}

// -----------------------------------------------------------------------------
// CHẶN ĐẢO CHIỀU ĐỘT NGỘT CHO BÁNH i
// -----------------------------------------------------------------------------
// Trả về PWM được phép xuất cho bánh i.
//   - Yêu cầu 0: trả về 0.
//   - Yêu cầu ngược chiều lần chạy trước, và bánh mới chạy theo chiều cũ cách
//     đây chưa tới REVERSE_MS: tạm trả về 0 (bánh thả trôi cho tới khi hết thời gian chờ).
//   - Còn lại: cho phép, đồng thời ghi nhớ chiều và thời điểm chạy.
// Lưu ý: cơ chế này áp dụng riêng từng bánh, nên khi chỉ một vài bánh đổi chiều,
// tỉ lệ giữa các bánh bị sai tạm thời (xem esp32/CAI_TIEN_FIRMWARE.md).
int guard(int request,int i,uint32_t now) {
  if(!request) return 0;
  int dir=request>0 ? 1 : -1;
  if(lastDir[i] && dir!=lastDir[i] && uint32_t(now-lastActive[i])<REVERSE_MS) return 0;
  lastDir[i]=dir; lastActive[i]=now; return request;
}

// -----------------------------------------------------------------------------
// CẬP NHẬT ĐẦU RA CHO 4 ĐỘNG CƠ
// -----------------------------------------------------------------------------
// timeout = true: ép mọi bánh về 0.
// Chỉ ghi ra chân khi giá trị thực sự thay đổi, để tránh ghi lặp mỗi vòng loop().
void motors(uint32_t now,bool timeout) {
  // Tính PWM mới cho từng bánh và kiểm tra có gì thay đổi không.
  int next[4]; bool changed=false, any=false;
  for(int i=0;i<4;++i) {
    next[i]=guard(timeout ? 0 : target[i],i,now);
    changed |= next[i]!=applied[i]; any |= next[i]!=0;
  }
  if(!changed) return;

  // Tắt driver trong lúc đổi chân chiều, tránh trạng thái trung gian sai.
  digitalWrite(STBY,LOW);
  for(int i=0;i<4;++i) {
    // Áp dấu lắp đặt của bánh, rồi chọn chiều theo bảng chân lý TB6612:
    //   c > 0: IN1=HIGH, IN2=LOW  → quay một chiều
    //   c < 0: IN1=LOW,  IN2=HIGH → quay chiều ngược lại
    //   c = 0: IN1=LOW,  IN2=LOW  → dừng
    int c=next[i]*MOTOR_SIGN[i];
    digitalWrite(IN1[i],c>0 ? HIGH:LOW);
    digitalWrite(IN2[i],c<0 ? HIGH:LOW);
    // IN1=IN2=LOW and PWM=HIGH: idle channel high impedance.
    // Khi bánh đứng yên, đặt PWM = 255 (luôn HIGH) cùng IN1=IN2=LOW: theo
    // datasheet TB6612 đây là trạng thái OFF trở kháng cao, bánh thả trôi.
    ledcWrite(PWM[i],c ? abs(c):255);
    applied[i]=next[i];
  }
  // Chỉ bật lại driver nếu có ít nhất một bánh cần quay.
  if(any) digitalWrite(STBY,HIGH);
}

// -----------------------------------------------------------------------------
// KHỞI TẠO
// -----------------------------------------------------------------------------
void setup() {
  // Tắt driver ngay từ đầu, trước khi làm bất cứ việc gì khác.
  pinMode(STBY,OUTPUT); digitalWrite(STBY,LOW);
  Serial.begin(115200);

  // Cấu hình chân chiều về LOW và gắn kênh PWM 20 kHz, độ phân giải 8 bit
  // (0..255) cho từng bánh. 20 kHz nằm ngoài dải nghe nên motor không kêu rít.
  for(int i=0;i<4;++i) {
    pinMode(IN1[i],OUTPUT); pinMode(IN2[i],OUTPUT);
    digitalWrite(IN1[i],LOW); digitalWrite(IN2[i],LOW);
    if(!ledcAttach(PWM[i],20000,8)) { failed=true; break; }
    ledcWrite(PWM[i],255);   // trạng thái nghỉ: OFF trở kháng cao (xem motors())
  }

  // Khởi tạo PWM lỗi: báo lỗi và không bao giờ chạy loop(). STBY vẫn LOW nên
  // motor không thể quay; bridge sẽ không nhận được READY và báo bắt tay thất bại.
  if(failed) { Serial.println("ERROR,PWM_INIT"); return; }
  pwmReady=true; identity();
}

// -----------------------------------------------------------------------------
// VÒNG LẶP CHÍNH
// -----------------------------------------------------------------------------
void loop() {
  if(!pwmReady) return;

  // 1) ĐỌC SERIAL: xử lý tối đa 64 byte mỗi vòng để loop() không bị giữ quá lâu.
  for(int n=0;n<64 && Serial.available();++n) {
    char c=Serial.read();
    if(c=='\r') continue;                 // chấp nhận cả kết thúc dòng kiểu Windows
    if(c=='\n') {
      // Hết một dòng: phân tích nếu dòng không bị đánh dấu lỗi và không rỗng,
      // rồi chuẩn bị nhận dòng mới.
      if(!discard && rxLen) { rx[rxLen]=0; parse(rx); }
      rxLen=0; discard=false;
    } else if(!discard) {
      // Ký tự không in được hoặc dòng quá dài: hủy dòng này, dừng motor ngay,
      // và bỏ qua mọi ký tự còn lại cho tới '\n'.
      if(c<32 || c>126 || rxLen>=sizeof(rx)-1) { discard=true; rxLen=0; invalidate(); }
      else rx[rxLen++]=c;
    }
  }

  // 2) WATCHDOG LỆNH: timeout nếu chưa có lệnh hợp lệ, hoặc lệnh gần nhất đã cũ
  //    hơn TIMEOUT_MS. Phép trừ uint32_t vẫn đúng khi millis() tràn (~49 ngày).
  uint32_t now=millis();
  bool timeout=!validCommand || uint32_t(now-lastCommand)>=TIMEOUT_MS;

  // 3) CẬP NHẬT ĐỘNG CƠ.
  motors(now,timeout);

  // 4) PHẢN HỒI mỗi FEEDBACK_MS: T,ms,seq,FL,FR,RL,RR,timedOut.
  //    Chỉ gửi khi bộ đệm truyền còn đủ chỗ, để loop() không bao giờ bị chặn
  //    vì Jetson đọc chậm. Bridge dùng seq trong dòng này làm ACK.
  if(uint32_t(now-lastFeedback)>=FEEDBACK_MS) {
    lastFeedback=now; char out[128];
    int n=snprintf(out,sizeof(out),"T,%lu,%ld,%d,%d,%d,%d,%d\n",
      (unsigned long)now,lastSeq,applied[0],applied[1],applied[2],applied[3],int(timeout));
    if(n>0 && n<int(sizeof(out)) && Serial.availableForWrite()>=n)
      Serial.write((const uint8_t*)out,n);
  }
}
