// 排查用：完全繞開PID，開機直接固定duty=OPEN_LOOP_DUTY(對應正式版PID測試中
// 觀察到「duty衝到高檔、位置卻凍結不動」的那個區間)，同時每20ms印一次
// 編碼器原始累積計數。目的是排除「PID增益太衝」跟「馬達/供電/編碼器在
// 固定高duty下本身就會失去扭力或斷訊」這兩種可能——如果開迴路下還是會
// 卡，就跟PID調參無關，是硬體訊號本身的問題。
// 開機後自動跑RUN_MS毫秒，時間到自動停止並保持停止，不需要下任何指令。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 2;
const uint8_t CH_M1_IN2 = 1;
const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;

const uint8_t M1_PWM_PIN = 8;
const uint8_t M2_PWM_PIN = 9;

const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const uint32_t PWM_PERIOD_US = 200;  // 5kHz，跟正式版一致
const uint8_t OPEN_LOOP_DUTY = 75;   // 對應剛剛閉環測試(target=20%)實際卡住那幾次
                                      // 當下的duty(65~82區間)，同樣條件下開迴路
                                      // 重測一次比較。
const uint32_t RUN_MS = 4000;        // 跟剛剛閉環測試的時間長度接近
const uint32_t LOG_PERIOD_MS = 20;

volatile int32_t posM1 = 0;
volatile int32_t posM2 = 0;
volatile uint8_t stateM1 = 0;
volatile uint8_t stateM2 = 0;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void updateQuadM1() {
  uint8_t a = digitalRead(PIN_M1_A);
  uint8_t b = digitalRead(PIN_M1_B);
  uint8_t newState = (a << 1) | b;
  posM1 += QDEC_TABLE[((stateM1 << 2) | newState) & 0x0F];
  stateM1 = newState;
}
void updateQuadM2() {
  uint8_t a = digitalRead(PIN_M2_A);
  uint8_t b = digitalRead(PIN_M2_B);
  uint8_t newState = (a << 1) | b;
  posM2 += QDEC_TABLE[((stateM2 << 2) | newState) & 0x0F];
  stateM2 = newState;
}
void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

uint8_t m1Duty = 0;
uint8_t m2Duty = 0;
uint32_t startMs = 0;
uint32_t lastLogMs = 0;
bool stopped = false;

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(300);

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

  pwm.setPin(CH_M1_IN1, 4095);
  pwm.setPin(CH_M1_IN2, 0);
  pwm.setPin(CH_M2_IN1, 4095);
  pwm.setPin(CH_M2_IN2, 0);

  Serial.print(F("=== 開迴路排除測試：固定duty="));
  Serial.print(OPEN_LOOP_DUTY);
  Serial.println(F("，跑3秒後自動停止 ==="));

  startMs = millis();
  lastLogMs = startMs;
  m1Duty = OPEN_LOOP_DUTY;
  m2Duty = OPEN_LOOP_DUTY;
}

void loop() {
  updateMotorPWM();
  uint32_t now = millis();

  if (!stopped && (now - startMs >= RUN_MS)) {
    stopped = true;
    m1Duty = 0;
    m2Duty = 0;
    pwm.setPin(CH_M1_IN1, 4095);
    pwm.setPin(CH_M1_IN2, 4095);
    pwm.setPin(CH_M2_IN1, 4095);
    pwm.setPin(CH_M2_IN2, 4095);
    Serial.println(F("=== 測試結束，已停止 ==="));
  }

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    Serial.print(F("[ol] t="));
    Serial.print(now - startMs);
    Serial.print(F(" posM1="));
    Serial.print(posM1);
    Serial.print(F(" posM2="));
    Serial.print(posM2);
    Serial.print(F(" dutyM1="));
    Serial.print(m1Duty);
    Serial.print(F(" dutyM2="));
    Serial.println(m2Duty);
  }
}
