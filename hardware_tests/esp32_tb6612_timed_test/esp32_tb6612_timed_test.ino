// ESP32-WROOM-32 DevKit + Arduino-ESP32 3.x + TB6612FNG.
// Automatic, one-pass OPEN-LOOP test. Encoders are not used.
// VM=6V; VCC=3.3V; common GND. Lift BOTH wheels before power-up.
// Reset/power-up runs this sequence again. No serial command is needed.
#include <Arduino.h>

constexpr uint8_t AIN1=18, AIN2=19, PWMA=25;
constexpr uint8_t BIN1=21, BIN2=22, PWMB=26, STBY=27;
constexpr uint32_t PWM_HZ=20000;
constexpr uint8_t PWM_BITS=8;
// Positive electrical direction may not be forward on the assembled vehicle.
constexpr int LEFT_SIGN=1;
constexpr int RIGHT_SIGN=1;

struct Step {
  uint32_t durationMs;
  int leftPWM;
  int rightPWM;
  const char* label;
};

const Step sequence[] = {
  {5000,   0,   0, "WAIT: lift wheels; test starts in 5 s"},
  {2000,  80,   0, "LEFT +80"},
  {2000, 120,   0, "LEFT +120"},
  {2000, 160,   0, "LEFT +160"},
  {3000,   0,   0, "COAST STOP"},
  {2000, -80,   0, "LEFT -80"},
  {2000,-120,   0, "LEFT -120"},
  {2000,-160,   0, "LEFT -160"},
  {3000,   0,   0, "COAST STOP"},
  {2000,   0,  80, "RIGHT +80"},
  {2000,   0, 120, "RIGHT +120"},
  {2000,   0, 160, "RIGHT +160"},
  {3000,   0,   0, "COAST STOP"},
  {2000,   0, -80, "RIGHT -80"},
  {2000,   0,-120, "RIGHT -120"},
  {2000,   0,-160, "RIGHT -160"},
  {3000,   0,   0, "COAST STOP"},
  {2000,  80,  80, "BOTH +80"},
  {2000, 120, 120, "BOTH +120"},
  {2000, 160, 160, "BOTH +160"},
  {3000,   0,   0, "COAST STOP"},
  {2000, -80, -80, "BOTH -80"},
  {2000,-120,-120, "BOTH -120"},
  {2000,-160,-160, "BOTH -160"},
  {3000,   0,   0, "FINAL COAST STOP"}
};
constexpr size_t STEP_COUNT=sizeof(sequence)/sizeof(sequence[0]);
size_t stepIndex=0;
uint32_t stepStartedAt=0;
bool finished=false;

void writeMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int command) {
  command=constrain(command, -255, 255);
  // At command=0 use IN1=IN2=LOW and PWM=HIGH for coast (high impedance).
  digitalWrite(in1, command>0 ? HIGH : LOW);
  digitalWrite(in2, command<0 ? HIGH : LOW);
  ledcWrite(pwmPin, command==0 ? 255 : abs(command));
}

void applyStep() {
  const Step &s=sequence[stepIndex];
  digitalWrite(STBY, LOW);
  writeMotor(AIN1, AIN2, PWMA, LEFT_SIGN*s.leftPWM);
  writeMotor(BIN1, BIN2, PWMB, RIGHT_SIGN*s.rightPWM);
  if (s.leftPWM!=0 || s.rightPWM!=0) digitalWrite(STBY, HIGH);
  stepStartedAt=millis();
  Serial.printf("Step %u/%u: %s; duration=%lu ms\n",
    (unsigned)(stepIndex+1), (unsigned)STEP_COUNT, s.label,
    (unsigned long)s.durationMs);
}

void setup() {
  pinMode(STBY, OUTPUT); digitalWrite(STBY, LOW);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  Serial.begin(115200);
  const bool okA=ledcAttach(PWMA, PWM_HZ, PWM_BITS);
  const bool okB=ledcAttach(PWMB, PWM_HZ, PWM_BITS);
  if (!okA || !okB) {
    Serial.println("PWM setup failed. Driver disabled.");
    while (true) delay(1000);
  }
  applyStep();
}

void loop() {
  if (finished) return;
  // Unsigned subtraction remains correct across millis() wraparound.
  if (uint32_t(millis()-stepStartedAt)>=sequence[stepIndex].durationMs) {
    ++stepIndex;
    if (stepIndex>=STEP_COUNT) {
      digitalWrite(STBY, LOW);
      finished=true;
      Serial.println("DONE: driver disabled. Reset to repeat.");
    } else {
      applyStep();
    }
  }
}
