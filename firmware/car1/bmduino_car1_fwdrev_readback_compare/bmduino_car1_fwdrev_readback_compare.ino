// 直接證明「同一顆馬達，同樣duty，正轉/反轉測出來的RPM真的差很多」，
// 排除是不同顆馬達搞混、或channel/接線判斷錯誤的疑慮：
//   - 每顆馬達（M1、M2）各自在同一個duty值下，先正轉量3秒、停1秒、
//     再反轉量3秒，直接對照同一顆馬達、同一個duty的正反轉RPM讀值
//   - 分別在 duty=150（中段）跟 duty=230（接近之前發現的門檻）各測一次
//   - 全程只操作同一組PCA9685 channel/PWM腳/編碼器腳，沒有切換到別顆馬達，
//     確保比較的是同一顆馬達本身，不是接線混淆

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint8_t PIN_M1_A = 2;  // 2026-09-08改接：原本D4接觸不良，改到D2
const uint8_t PIN_M1_B = 3;  // 原本D5有雜訊，改到D3

const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;
const uint8_t M2_PWM_PIN = 8;
const uint8_t PIN_M2_A = 6;
const uint8_t PIN_M2_B = 7;

const uint32_t PWM_PERIOD_US = 200;
const float TICKS_PER_REV = 1550.0;
const uint32_t SETTLE_MS = 700;
const uint32_t MEASURE_MS = 1500;

volatile int32_t posM1 = 0, posM2 = 0;
volatile uint8_t stateM1 = 0, stateM2 = 0;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};
void updateQuadM1() {
  uint8_t a = digitalRead(PIN_M1_A), b = digitalRead(PIN_M1_B);
  uint8_t ns = (a << 1) | b;
  posM1 += QDEC_TABLE[((stateM1 << 2) | ns) & 0x0F];
  stateM1 = ns;
}
void updateQuadM2() {
  uint8_t a = digitalRead(PIN_M2_A), b = digitalRead(PIN_M2_B);
  uint8_t ns = (a << 1) | b;
  posM2 += QDEC_TABLE[((stateM2 << 2) | ns) & 0x0F];
  stateM2 = ns;
}
void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

uint8_t m1Duty = 0, m2Duty = 0;

void setDir(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) { pwm.setPin(in1Ch, 4095); pwm.setPin(in2Ch, 0); }
  else if (dir < 0) { pwm.setPin(in1Ch, 0); pwm.setPin(in2Ch, 4095); }
  else { pwm.setPin(in1Ch, 4095); pwm.setPin(in2Ch, 4095); }  // coast
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  digitalWrite(M1_PWM_PIN, phase < (uint32_t)m1Duty * PWM_PERIOD_US / 255 ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < (uint32_t)m2Duty * PWM_PERIOD_US / 255 ? HIGH : LOW);
}

void busyWaitPWM(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) updateMotorPWM();
}

void measureOne(const char* motorLabel, const char* dirLabel, uint8_t duty,
                 uint8_t* dutyVar, volatile int32_t* pos, uint8_t in1Ch, uint8_t in2Ch, int dir) {
  setDir(in1Ch, in2Ch, dir);
  *dutyVar = duty;
  busyWaitPWM(SETTLE_MS);
  int32_t before = *pos;
  busyWaitPWM(MEASURE_MS);
  int32_t after = *pos;
  float rpm = ((after - before) / TICKS_PER_REV) * (60000.0f / MEASURE_MS);
  Serial.print(F("["));
  Serial.print(motorLabel);
  Serial.print(F(" "));
  Serial.print(dirLabel);
  Serial.print(F(" duty="));
  Serial.print(duty);
  Serial.print(F("] deltaTicks="));
  Serial.print(after - before);
  Serial.print(F("  RPM="));
  Serial.println(rpm, 2);
  *dutyVar = 0;
  setDir(in1Ch, in2Ch, 0);
  busyWaitPWM(1000);
}

void compareMotor(const char* label, uint8_t* dutyVar, volatile int32_t* pos,
                   uint8_t in1Ch, uint8_t in2Ch, uint8_t testDuty) {
  Serial.print(F("--- "));
  Serial.print(label);
  Serial.print(F(" @ duty="));
  Serial.print(testDuty);
  Serial.println(F(" : forward vs reverse ---"));
  measureOne(label, "fwd", testDuty, dutyVar, pos, in1Ch, in2Ch, 1);
  measureOne(label, "rev", testDuty, dutyVar, pos, in1Ch, in2Ch, -1);
}

void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  setDir(CH_M1_IN1, CH_M1_IN2, 0);
  setDir(CH_M2_IN1, CH_M2_IN2, 0);
  delay(1000);

  Serial.println(F("=== M1/M2 forward vs reverse readback comparison, same motor, same duty ==="));

  compareMotor("M1", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2, 150);
  compareMotor("M1", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2, 230);
  compareMotor("M2", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2, 150);
  compareMotor("M2", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2, 230);

  Serial.println(F("=== done ==="));
}

void loop() {
  updateMotorPWM();
}
