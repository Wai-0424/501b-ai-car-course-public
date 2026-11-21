// 檢查M1編碼器A/B兩相是不是真正的正交關係（90度相位差），還是像同一個
// 訊號一樣同時變化（代表接線有問題，例如兩相短接、或其中一相沒接好被
// 另一相crosstalk帶動）。
//
// 做法：M1正轉duty=200，記錄前200次A或B的電位變化事件，每筆記錄
// (時間us, 觸發的是A還是B, A當下的值, B當下的值)，跑完一次印出全部，
// 人工/我來判讀是不是正常的四態循環(00->01->11->10->00...)。
//
// 正常正交：每次事件只有A或B其中一個變化，另一個維持不變，狀態序列
// 應該是 00,01,11,10 這四個值循環出現，不會有00直接跳到11（那樣代表
// A、B同時變化，不是正常正交訊號)。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint8_t PIN_M1_A = 2;  // 改接到D2（原本D4不穩定）
const uint8_t PIN_M1_B = 3;  // 改接到D3（原本D5有雜訊）

const uint32_t PWM_PERIOD_US = 200;
const uint8_t DUTY = 200;

const uint16_t LOG_SIZE = 200;
volatile uint32_t logTimeUs[LOG_SIZE];
volatile uint8_t logSrc[LOG_SIZE];  // 0=A觸發, 1=B觸發
volatile uint8_t logA[LOG_SIZE];
volatile uint8_t logB[LOG_SIZE];
volatile uint16_t logCount = 0;

void isrA() {
  if (logCount < LOG_SIZE) {
    logTimeUs[logCount] = micros();
    logSrc[logCount] = 0;
    logA[logCount] = digitalRead(PIN_M1_A);
    logB[logCount] = digitalRead(PIN_M1_B);
    logCount++;
  }
}
void isrB() {
  if (logCount < LOG_SIZE) {
    logTimeUs[logCount] = micros();
    logSrc[logCount] = 1;
    logA[logCount] = digitalRead(PIN_M1_A);
    logB[logCount] = digitalRead(PIN_M1_B);
    logCount++;
  }
}

uint8_t m1Duty = 0;

void setDir(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) { pwm.setPin(in1Ch, 4095); pwm.setPin(in2Ch, 0); }
  else { pwm.setPin(in1Ch, 4095); pwm.setPin(in2Ch, 4095); }
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  digitalWrite(M1_PWM_PIN, phase < (uint32_t)m1Duty * PWM_PERIOD_US / 255 ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrB, CHANGE);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  setDir(CH_M1_IN1, CH_M1_IN2, 1);

  Serial.println(F("=== M1 A/B phase check: logging first 200 transitions ==="));
  m1Duty = DUTY;

  uint32_t start = millis();
  while (logCount < LOG_SIZE && millis() - start < 6000) {
    updateMotorPWM();
  }

  m1Duty = 0;
  setDir(CH_M1_IN1, CH_M1_IN2, 0);

  Serial.print(F("Captured "));
  Serial.print(logCount);
  Serial.println(F(" transitions:"));
  Serial.println(F("idx  dt_us  src  A  B  state"));
  uint32_t prevT = logTimeUs[0];
  for (uint16_t i = 0; i < logCount; i++) {
    uint32_t dt = (i == 0) ? 0 : (logTimeUs[i] - prevT);
    prevT = logTimeUs[i];
    Serial.print(i);
    Serial.print(F("  "));
    Serial.print(dt);
    Serial.print(F("  "));
    Serial.print(logSrc[i] == 0 ? 'A' : 'B');
    Serial.print(F("  "));
    Serial.print(logA[i]);
    Serial.print(F("  "));
    Serial.print(logB[i]);
    Serial.print(F("  "));
    Serial.println((logA[i] << 1) | logB[i]);
  }
  Serial.println(F("=== done ==="));
}

void loop() {
  updateMotorPWM();
}
