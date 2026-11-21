// 排查用：完全繞開PID/編碼器回授，但duty不是固定常數，而是像閉環測試
// 觀察到的那樣緩慢爬升(65->82，每20ms週期+1)，藉此隔離「duty持續變動
// 這件事本身」是否會造成卡頓，跟「PID演算法/回授邏輯」分開判斷。
// 如果這樣測還是會卡，代表卡頓真正的根因是duty變動時的電流暫態或
// PWM/編碼器ISR時序交互作用，跟PID本身無關；如果不會卡，才能確定是
// PID演算法造成的。
// 開機後自動執行，不需要下任何指令。

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
const uint8_t RAMP_START_DUTY = 65;  // 對應閉環測試卡住時的duty區間下緣
const uint8_t RAMP_END_DUTY = 82;    // 對應閉環測試卡住時的duty區間上緣
const uint32_t RAMP_STEP_PERIOD_MS = 20;  // 跟PID週期一致，每個週期duty+1
const uint32_t HOLD_AFTER_MS = 2000;      // 爬到頂之後再固定維持一段時間
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
uint32_t lastRampMs = 0;
uint32_t lastLogMs = 0;
uint32_t reachedTopMs = 0;
bool atTop = false;
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

  Serial.println(F("=== 開迴路duty緩慢爬升測試：65->82，不經過PID ==="));
  startMs = millis();
  lastRampMs = startMs;
  lastLogMs = startMs;
  m1Duty = RAMP_START_DUTY;
  m2Duty = RAMP_START_DUTY;
}

void loop() {
  updateMotorPWM();
  uint32_t now = millis();

  if (!stopped && !atTop && (now - lastRampMs >= RAMP_STEP_PERIOD_MS)) {
    lastRampMs = now;
    if (m1Duty < RAMP_END_DUTY) {
      m1Duty++;
      m2Duty++;
    } else {
      atTop = true;
      reachedTopMs = now;
    }
  }

  if (!stopped && atTop && (now - reachedTopMs >= HOLD_AFTER_MS)) {
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
