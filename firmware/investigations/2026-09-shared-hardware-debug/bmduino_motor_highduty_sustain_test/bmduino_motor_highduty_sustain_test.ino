// 承接 bmduino_motor_pwm50_encoder_check：50% duty 已確認可以長時間持續轉動
// 不會停。這裡改成固定高 duty（不爬升，一開機就是高負載），同樣持續運行、
// 每0.5秒印一次編碼器計數，用來確認「衝一下就停」是不是跟 duty(電流)高低
// 有關——如果這裡也能穩定持續轉動不停，代表問題只在爬升過程本身；如果
// 這裡很快就複現「轉一下就停」，就能確定是高電流觸發某種保護/跳脫。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint8_t M1_PWM_PIN = 9;
const uint8_t M2_PWM_PIN = 8;
const uint8_t PIN_M1_A = 6;
const uint8_t PIN_M1_B = 7;
const uint8_t PIN_M2_A = 4;
const uint8_t PIN_M2_B = 5;

const uint16_t STEER_CENTER_US = 1500;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz
const uint8_t TEST_DUTY = 250;       // 接近全速(約98%)，固定不爬升

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
  uint8_t ns = (a << 1) | b;
  posM1 += QDEC_TABLE[((stateM1 << 2) | ns) & 0x0F];
  stateM1 = ns;
}
void updateQuadM2() {
  uint8_t a = digitalRead(PIN_M2_A);
  uint8_t b = digitalRead(PIN_M2_B);
  uint8_t ns = (a << 1) | b;
  posM2 += QDEC_TABLE[((stateM2 << 2) | ns) & 0x0F];
  stateM2 = ns;
}
void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

void setup() {
  Serial.begin(115200);

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
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  pwm.setPin(1, 0);      // CH1(IN1) = LOW
  pwm.setPin(2, 4095);   // CH2(IN2) = HIGH
  pwm.setPin(3, 0);      // CH3(IN3) = LOW
  pwm.setPin(4, 4095);   // CH4(IN4) = HIGH

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  Serial.println(F("=== SUSTAIN TEST: fixed duty=250(~98%), watching for stall/cutout ==="));
}

uint32_t lastPrintMs = 0;
int32_t lastM1 = 0;
int32_t lastM2 = 0;

void loop() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)TEST_DUTY * PWM_PERIOD_US / 255;
  bool high = phase < onUs;
  digitalWrite(M1_PWM_PIN, high ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, high ? HIGH : LOW);

  uint32_t now = millis();
  if (now - lastPrintMs >= 500) {
    lastPrintMs = now;
    int32_t curM1 = posM1;
    int32_t curM2 = posM2;
    Serial.print(F("t="));
    Serial.print(now);
    Serial.print(F("ms  posM1="));
    Serial.print(curM1);
    Serial.print(F(" (d="));
    Serial.print(curM1 - lastM1);
    Serial.print(F(")  posM2="));
    Serial.print(curM2);
    Serial.print(F(" (d="));
    Serial.print(curM2 - lastM2);
    Serial.println(F(")"));
    lastM1 = curM1;
    lastM2 = curM2;
  }
}
