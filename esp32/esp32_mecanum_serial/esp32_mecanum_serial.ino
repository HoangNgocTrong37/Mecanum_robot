// ESP32-WROOM-32 DevKit, Arduino-ESP32 3.x. No encoders / PID.
// USB UART 115200 8N1, newline terminated, order FL FR RL RR.
// Host: P,seq,FL,FR,RL,RR   seq 0..1000000000; PWM -160..160.
// Host: ?                 query firmware identity (does not refresh watchdog).
// Board: T,ms,seq,FL,FR,RL,RR,timedOut   applied logical PWM, NOT measured speed.
// STBY LOW disables both drivers; wheels COAST rather than hold position.
#include <Arduino.h>
#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR != 3
#error "Select Arduino-ESP32 core 3.x for this firmware"
#endif
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

constexpr uint8_t IN1[4]={18,21,16,13};
constexpr uint8_t IN2[4]={19,22,17,14};
constexpr uint8_t PWM[4]={25,26,23,27};
constexpr uint8_t STBY=4;
constexpr int MOTOR_SIGN[4]={1,1,1,1}; // Change to -1 if positive rotates backward.
constexpr int MAX_PWM=160;
constexpr uint32_t TIMEOUT_MS=300, FEEDBACK_MS=50, REVERSE_MS=500;
int target[4]={}, applied[4]={}, lastDir[4]={};
uint32_t lastActive[4]={}, lastCommand=0, lastFeedback=0;
long lastSeq=-1;
bool validCommand=false, pwmReady=false, failed=false;
char rx[96]; size_t rxLen=0; bool discard=false;

void invalidate() { validCommand=false; for(int i=0;i<4;++i) target[i]=0; }
void identity() { Serial.println("READY,MECANUM_PWM_V1"); }
bool integerField(const char*& p,long& value,char terminator) {
  if(!(*p=='-' || (*p>='0' && *p<='9'))) return false;
  char* end=nullptr; errno=0; value=strtol(p,&end,10);
  if(errno==ERANGE || end==p || *end!=terminator) return false;
  p=end+(terminator!=0); return true;
}
void parse(const char* line) {
  if(line[0]=='?' && line[1]==0) { identity(); return; }
  if(line[0]!='P' || line[1]!=',') { invalidate(); return; }
  const char* p=line+2; long seq,v[4];
  if(!integerField(p,seq,',') || seq<0 || seq>1000000000L) { invalidate(); return; }
  for(int i=0;i<4;++i) {
    if(!integerField(p,v[i],i==3 ? 0 : ',') || v[i]<-MAX_PWM || v[i]>MAX_PWM) {
      invalidate(); return;
    }
  }
  for(int i=0;i<4;++i) target[i]=int(v[i]);
  lastSeq=seq; lastCommand=millis(); validCommand=true;
}
int guard(int request,int i,uint32_t now) {
  if(!request) return 0;
  int dir=request>0 ? 1 : -1;
  if(lastDir[i] && dir!=lastDir[i] && uint32_t(now-lastActive[i])<REVERSE_MS) return 0;
  lastDir[i]=dir; lastActive[i]=now; return request;
}
void motors(uint32_t now,bool timeout) {
  int next[4]; bool changed=false, any=false;
  for(int i=0;i<4;++i) {
    next[i]=guard(timeout ? 0 : target[i],i,now);
    changed |= next[i]!=applied[i]; any |= next[i]!=0;
  }
  if(!changed) return;
  digitalWrite(STBY,LOW);
  for(int i=0;i<4;++i) {
    int c=next[i]*MOTOR_SIGN[i];
    digitalWrite(IN1[i],c>0 ? HIGH:LOW);
    digitalWrite(IN2[i],c<0 ? HIGH:LOW);
    // IN1=IN2=LOW and PWM=HIGH: idle channel high impedance.
    ledcWrite(PWM[i],c ? abs(c):255);
    applied[i]=next[i];
  }
  if(any) digitalWrite(STBY,HIGH);
}
void setup() {
  pinMode(STBY,OUTPUT); digitalWrite(STBY,LOW);
  Serial.begin(115200);
  for(int i=0;i<4;++i) {
    pinMode(IN1[i],OUTPUT); pinMode(IN2[i],OUTPUT);
    digitalWrite(IN1[i],LOW); digitalWrite(IN2[i],LOW);
    if(!ledcAttach(PWM[i],20000,8)) { failed=true; break; }
    ledcWrite(PWM[i],255);
  }
  if(failed) { Serial.println("ERROR,PWM_INIT"); return; }
  pwmReady=true; identity();
}
void loop() {
  if(!pwmReady) return;
  for(int n=0;n<64 && Serial.available();++n) {
    char c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n') {
      if(!discard && rxLen) { rx[rxLen]=0; parse(rx); }
      rxLen=0; discard=false;
    } else if(!discard) {
      if(c<32 || c>126 || rxLen>=sizeof(rx)-1) { discard=true; rxLen=0; invalidate(); }
      else rx[rxLen++]=c;
    }
  }
  uint32_t now=millis();
  bool timeout=!validCommand || uint32_t(now-lastCommand)>=TIMEOUT_MS;
  motors(now,timeout);
  if(uint32_t(now-lastFeedback)>=FEEDBACK_MS) {
    lastFeedback=now; char out[128];
    int n=snprintf(out,sizeof(out),"T,%lu,%ld,%d,%d,%d,%d,%d\n",
      (unsigned long)now,lastSeq,applied[0],applied[1],applied[2],applied[3],int(timeout));
    if(n>0 && n<int(sizeof(out)) && Serial.availableForWrite()>=n)
      Serial.write((const uint8_t*)out,n);
  }
}
