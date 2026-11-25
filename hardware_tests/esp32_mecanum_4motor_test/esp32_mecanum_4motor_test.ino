// ESP32 DevKit / ESP32-WROOM-32. Arduino-ESP32 core 3.x.
// Two TB6612FNG modules; FOUR motors tested individually, without encoders/PID.
// VM=12V, VCC=3.3V, common GND. Lift ALL wheels before power-up/reset.
// Shared STBY is GPIO4 (NOT GPIO27 from the earlier two-motor sketch).
// Runs once automatically. Reset/power cycling starts the test again.
#include <Arduino.h>

// Order: Front Left, Front Right, Rear Left, Rear Right.
constexpr uint8_t IN1[4] = {18, 21, 16, 13};
constexpr uint8_t IN2[4] = {19, 22, 17, 14};
constexpr uint8_t PWM[4] = {25, 26, 23, 27};
constexpr uint8_t STBY = 4;
const char* MOTOR_NAME[4] = {"FL", "FR", "RL", "RR"};

// Adjust each sign AFTER checking that motor individually.
// +1 means electrical direction, not necessarily robot forward.
constexpr int MOTOR_SIGN[4] = {1, 1, 1, 1};
constexpr int DUTY[3] = {80, 120, 160}; // Out of 255; NOT RPM.
constexpr uint32_t START_DELAY_MS=5000;
constexpr uint32_t LEVEL_MS=2000;
constexpr uint32_t REST_MS=3000;
constexpr uint32_t PWM_HZ=20000;

bool pwmReady=false, started=false, finished=false;
uint8_t motorIndex=0, phase=0;
uint32_t phaseStart=0;

void stopAll() {
  digitalWrite(STBY, LOW); // Both drivers disabled; coast, not holding brake.
  if (pwmReady) {
    for (int i=0; i<4; ++i) {
      digitalWrite(IN1[i], LOW);
      digitalWrite(IN2[i], LOW);
      ledcWrite(PWM[i], 255);
    }
  }
}

void applyPhase() {
  stopAll();
  phaseStart=millis();
  if (phase==3 || phase==7) {
    Serial.printf("%s: coast stop for 3 seconds\n", MOTOR_NAME[motorIndex]);
    return;
  }
  const bool positive = phase<3;
  const int level = positive ? phase : phase-4;
  const int command = (positive ? 1 : -1)*MOTOR_SIGN[motorIndex];
  digitalWrite(IN1[motorIndex], command>0 ? HIGH : LOW);
  digitalWrite(IN2[motorIndex], command<0 ? HIGH : LOW);
  ledcWrite(PWM[motorIndex], DUTY[level]);
  digitalWrite(STBY, HIGH);
  Serial.printf("%s: direction=%s, PWM=%d/255, 2 seconds\n",
    MOTOR_NAME[motorIndex], positive ? "+" : "-", DUTY[level]);
}

void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  Serial.begin(115200);
  for (int i=0; i<4; ++i) {
    pinMode(IN1[i], OUTPUT); pinMode(IN2[i], OUTPUT);
    digitalWrite(IN1[i], LOW); digitalWrite(IN2[i], LOW);
    if (!ledcAttach(PWM[i], PWM_HZ, 8)) {
      Serial.println("ERROR: PWM initialization failed; drivers disabled.");
      finished=true;
      return;
    }
    ledcWrite(PWM[i], 0);
  }
  pwmReady=true;
  stopAll();
  Serial.println("Lift all wheels. Automatic test begins in 5 seconds.");
  Serial.println("Optional: send x to abort. Power off VM for physical stop.");
  phaseStart=millis();
}

void loop() {
  if (finished) return;
  // Optional abort only. No serial command is needed to start the sequence.
  for (int n=0; n<32 && Serial.available(); ++n) {
    const char c=Serial.read();
    if (c=='x' || c=='X') {
      stopAll(); finished=true;
      Serial.println("ABORTED: reset required to repeat.");
      return;
    }
  }
  const uint32_t now=millis();
  if (!started) {
    if (uint32_t(now-phaseStart)>=START_DELAY_MS) {
      started=true;
      applyPhase();
    }
    return;
  }
  const uint32_t duration=(phase==3 || phase==7) ? REST_MS : LEVEL_MS;
  if (uint32_t(now-phaseStart)>=duration) {
    ++phase;
    if (phase>=8) {
      phase=0;
      ++motorIndex;
    }
    if (motorIndex>=4) {
      stopAll(); finished=true;
      Serial.println("DONE: four motors tested; reset to repeat.");
    } else {
      applyPhase();
    }
  }
}
