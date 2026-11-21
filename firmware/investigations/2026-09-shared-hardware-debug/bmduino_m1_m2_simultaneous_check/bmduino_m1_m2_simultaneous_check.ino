// 讓M1、M2同時轉動（不是輪流），方便同時觀察/聆聽兩顆馬達的反應，
// 對照M1是不是真的完全沒反應、M2是否正常。兩顆都用duty=200、正轉，
// 跑8秒，同時印出兩顆馬達的原始編碼器腳位toggle次數（不經解碼）。

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
const uint8_t DUTY = 200;
const uint32_t RUN_MS = 8000;

volatile uint32_t cntM1A = 0, cntM1B = 0, cntM2A = 0, cntM2B = 0;

void isrM1A() { cntM1A++; }
void isrM1B() { cntM1B++; }
void isrM2A() { cntM2A++; }
void isrM2B() { cntM2B++; }

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

  setDir(CH_M1_IN1, CH_M1_IN2, 1);   // M1正轉
  setDir(CH_M2_IN1, CH_M2_IN2, -1);  // M2反轉（故意跟M1不同方向，方便看有沒有接線交叉）

  Serial.println(F("=== M1 forward + M2 reverse, simultaneous, duty=200 ==="));
  m1Duty = DUTY;
  m2Duty = DUTY;

  uint32_t start = millis();
  uint32_t lastPrint = start;
  while (millis() - start < RUN_MS) {
    updateMotorPWM();
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.print(F("M1: cntA="));
      Serial.print(cntM1A);
      Serial.print(F(" cntB="));
      Serial.print(cntM1B);
      Serial.print(F("  |  M2: cntA="));
      Serial.print(cntM2A);
      Serial.print(F(" cntB="));
      Serial.println(cntM2B);
    }
  }

  m1Duty = 0;
  m2Duty = 0;
  setDir(CH_M1_IN1, CH_M1_IN2, 0);
  setDir(CH_M2_IN1, CH_M2_IN2, 0);
  Serial.println(F("=== done, both stopped ==="));
}

void loop() {
  updateMotorPWM();
}
