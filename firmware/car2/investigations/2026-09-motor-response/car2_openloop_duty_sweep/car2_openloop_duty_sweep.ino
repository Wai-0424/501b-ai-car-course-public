// 排查用：完全繞開PID，依序跑過一串固定duty值(低~中檔，對應閉環模式下
// 觀察到會卡頓的區間)，每個duty停留一段時間，同時每20ms印一次編碼器
// 原始累積計數。目的是把「降壓模組/供電問題」跟「PID控制邏輯問題」
// 徹底分開判斷：如果開迴路(沒有PID、duty完全固定不變)在某個duty值下
// 還是會卡頓/停滯，那就是供電或機構問題，跟PID演算法無關；如果開迴路
// 每個duty值都能穩定平順轉動，才能說低速卡頓是PID造成的。
// 開機後自動依序掃描，不需要下任何指令。

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
const uint32_t HOLD_MS = 2000;       // 每個duty停留多久
const uint32_t LOG_PERIOD_MS = 20;   // 50Hz，跟正式版PID頻率一致，方便對照

// 掃描的duty值：涵蓋閉環模式下觀察到會卡頓的低~中檔區間
const uint8_t DUTY_LIST[] = {30, 40, 50, 60, 70, 80, 90};
const uint8_t DUTY_COUNT = sizeof(DUTY_LIST) / sizeof(DUTY_LIST[0]);

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
uint8_t dutyIndex = 0;
uint32_t stepStartMs = 0;
uint32_t lastLogMs = 0;
int32_t lastPosM1 = 0;
int32_t lastPosM2 = 0;
bool finished = false;

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

  Serial.println(F("=== 開迴路duty掃描：不經過PID，依序跑固定duty ==="));
  dutyIndex = 0;
  m1Duty = DUTY_LIST[0];
  m2Duty = DUTY_LIST[0];
  stepStartMs = millis();
  lastLogMs = stepStartMs;
  Serial.print(F(">>> duty="));
  Serial.println(m1Duty);
}

void loop() {
  updateMotorPWM();
  uint32_t now = millis();

  if (!finished && (now - stepStartMs >= HOLD_MS)) {
    dutyIndex++;
    if (dutyIndex >= DUTY_COUNT) {
      finished = true;
      m1Duty = 0;
      m2Duty = 0;
      pwm.setPin(CH_M1_IN1, 4095);
      pwm.setPin(CH_M1_IN2, 4095);
      pwm.setPin(CH_M2_IN1, 4095);
      pwm.setPin(CH_M2_IN2, 4095);
      Serial.println(F("=== 掃描結束，已停止 ==="));
    } else {
      m1Duty = DUTY_LIST[dutyIndex];
      m2Duty = DUTY_LIST[dutyIndex];
      stepStartMs = now;
      Serial.print(F(">>> duty="));
      Serial.println(m1Duty);
    }
  }

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    int32_t deltaM1 = posM1 - lastPosM1;
    int32_t deltaM2 = posM2 - lastPosM2;
    lastPosM1 = posM1;
    lastPosM2 = posM2;
    Serial.print(F("[ol] duty="));
    Serial.print(m1Duty);
    Serial.print(F(" posM1="));
    Serial.print(posM1);
    Serial.print(F(" dM1="));
    Serial.print(deltaM1);
    Serial.print(F(" posM2="));
    Serial.print(posM2);
    Serial.print(F(" dM2="));
    Serial.println(deltaM2);
  }
}
