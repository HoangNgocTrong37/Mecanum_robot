// ESP32-WROOM-32 DevKit, Arduino-ESP32 3.x, TB6612FNG.
// Stage 1: signed PWM commands + raw cumulative encoder ticks. NO PID/odometry.
// Protocol 115200 8N1, LF terminated:
// host -> P,seq,leftPWM,rightPWM  (seq 0..1000000000, PWM -160..160)
// board -> E,ms,seq,leftTicks,rightTicks,leftApplied,rightApplied,timedOut
// Lift wheels. Encoder signals MUST be 3.3V-compatible; GPIO34/35 need
// external pullups if required by the encoder output type.
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

constexpr uint8_t AIN1=18, AIN2=19, PWMA=25;
constexpr uint8_t BIN1=21, BIN2=22, PWMB=26, STBY=27;
constexpr uint8_t ENC_L_A = 34, ENC_L_B = 35;
constexpr uint8_t ENC_R_A = 32, ENC_R_B = 33;
constexpr int LEFT_MOTOR_SIGN=1, RIGHT_MOTOR_SIGN=1;
constexpr int LEFT_ENCODER_SIGN = 1;
constexpr int RIGHT_ENCODER_SIGN = -1;
constexpr uint32_t COMMAND_TIMEOUT_MS=300, FEEDBACK_MS=50;
constexpr uint32_t REVERSE_PAUSE_MS=500;
constexpr int MAX_PWM=160;

portMUX_TYPE encoderMux=portMUX_INITIALIZER_UNLOCKED;
volatile int64_t leftTicks=0, rightTicks=0;
int targetL=0,targetR=0,appliedL=0,appliedR=0;
int lastDirL=0,lastDirR=0;
uint32_t lastActiveL=0,lastActiveR=0,lastCommandMs=0,lastFeedbackMs=0;
long lastSeq=-1;
bool receivedCommand=false;
char rx[80]; size_t rxLen=0; bool discardLine=false;

// x2 decoding: count BOTH edges of A; B determines direction.
// Counts per wheel revolution must be calibrated for THIS decoding mode.
void ARDUINO_ISR_ATTR onLeftA() {
  int delta=(digitalRead(ENC_L_A)==digitalRead(ENC_L_B)) ? 1 : -1;
  portENTER_CRITICAL_ISR(&encoderMux);
  leftTicks+=LEFT_ENCODER_SIGN*delta;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
void ARDUINO_ISR_ATTR onRightA() {
  int delta=(digitalRead(ENC_R_A)==digitalRead(ENC_R_B)) ? 1 : -1;
  portENTER_CRITICAL_ISR(&encoderMux);
  rightTicks+=RIGHT_ENCODER_SIGN*delta;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void clearCommand() { targetL=targetR=0; receivedCommand=false; }

bool readInteger(const char*& p,long& value,char terminator) {
  if (!(*p=='-' || (*p>='0' && *p<='9'))) return false;
  char* end=nullptr; errno=0;
  value=strtol(p,&end,10);
  if (errno==ERANGE || end==p || *end!=terminator) return false;
  p=end+(terminator!=0); return true;
}
void parseCommand(const char* line) {
  if (line[0]!='P' || line[1]!=',') { clearCommand(); return; }
  const char* p=line+2; long seq,l,r;
  if (!readInteger(p,seq,',') || !readInteger(p,l,',') ||
      !readInteger(p,r,0) || seq<0 || seq>1000000000L ||
      l < -MAX_PWM || l>MAX_PWM || r < -MAX_PWM || r>MAX_PWM) {
    clearCommand(); return;
  }
  targetL=int(l); targetR=int(r); lastSeq=seq;
  lastCommandMs=millis(); receivedCommand=true;
}

int guardedPWM(int request,int& lastDir,uint32_t& lastActive,uint32_t now) {
  if (!request) return 0;
  const int dir=request>0 ? 1 : -1;
  if (lastDir!=0 && dir!=lastDir && uint32_t(now-lastActive)<REVERSE_PAUSE_MS)
    return 0;
  lastDir=dir; lastActive=now; return request;
}
void writeMotor(uint8_t in1,uint8_t in2,uint8_t pwm,int command) {
  digitalWrite(in1,command>0 ? HIGH:LOW);
  digitalWrite(in2,command<0 ? HIGH:LOW);
  // IN1=IN2=LOW, PWM=HIGH -> high impedance (coast) for inactive channel.
  ledcWrite(pwm,command==0 ? 255:abs(command));
}
void updateMotors(uint32_t now,bool timedOut) {
  int l=guardedPWM(timedOut ? 0:targetL,lastDirL,lastActiveL,now);
  int r=guardedPWM(timedOut ? 0:targetR,lastDirR,lastActiveR,now);
  if (l==appliedL && r==appliedR) return;
  digitalWrite(STBY,LOW);
  writeMotor(AIN1,AIN2,PWMA,LEFT_MOTOR_SIGN*l);
  writeMotor(BIN1,BIN2,PWMB,RIGHT_MOTOR_SIGN*r);
  if(l || r) digitalWrite(STBY,HIGH);
  appliedL=l; appliedR=r;
}
void setup() {
  pinMode(STBY,OUTPUT); digitalWrite(STBY,LOW);
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);
  digitalWrite(AIN1,LOW);digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);digitalWrite(BIN2,LOW);
  Serial.begin(115200);
  bool a=ledcAttach(PWMA,20000,8), b=ledcAttach(PWMB,20000,8);
  if(!a || !b) {
    Serial.println("ERROR,PWM_INIT");
    while(true) delay(1000);
  }
  ledcWrite(PWMA,0);ledcWrite(PWMB,0);
  pinMode(ENC_L_A,INPUT);pinMode(ENC_L_B,INPUT);
  pinMode(ENC_R_A,INPUT);pinMode(ENC_R_B,INPUT);
  attachInterrupt(ENC_L_A,onLeftA,CHANGE);
  attachInterrupt(ENC_R_A,onRightA,CHANGE);
  Serial.println("READY,USB_ENCODER_V1_X2");
}
void loop() {
  // Bounded RX work keeps watchdog/motor updates responsive under serial load.
  for(int n=0;n<64 && Serial.available();++n) {
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {
      if(!discardLine && rxLen) {rx[rxLen]=0;parseCommand(rx);}
      rxLen=0;discardLine=false;
    } else if(!discardLine) {
      if(c<32 || c>126 || rxLen>=sizeof(rx)-1) {
        discardLine=true;rxLen=0;clearCommand();
      } else rx[rxLen++]=c;
    }
  }
  const uint32_t now=millis();
  const bool timedOut=!receivedCommand || uint32_t(now-lastCommandMs)>=COMMAND_TIMEOUT_MS;
  updateMotors(now,timedOut);
  if(uint32_t(now-lastFeedbackMs)>=FEEDBACK_MS) {
    lastFeedbackMs=now;
    int64_t l,r;
    portENTER_CRITICAL(&encoderMux);l=leftTicks;r=rightTicks;
    portEXIT_CRITICAL(&encoderMux);
    char out[180];
    int len=snprintf(out,sizeof(out),"E,%lu,%ld,%lld,%lld,%d,%d,%d\n",
      (unsigned long)now,lastSeq,(long long)l,(long long)r,
      appliedL,appliedR,int(timedOut));
    // Skip a frame instead of blocking if the TX buffer is full.
    if(len>0 && len<int(sizeof(out)) && Serial.availableForWrite()>=len)
      Serial.write((const uint8_t*)out,len);
  }
}
