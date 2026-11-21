// 排查用：M2 在閉環測試裡 duty 卡在177、實際轉速卻是0，這裡繞開PID，
// 直接開迴路固定 duty=177 驅動 M2，持續監看編碼器原始計數，確認這個
// duty 到底會不會讓 M2 真的轉動。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M2_IN1 = 3;
const uint8_t CH_M2_IN2 = 4;
const uint8_t M2_PWM_PIN = 9;
const uint8_t PIN_M2_A = 10;  // 原本D6，懷疑腳位本身有問題，改測D10
const uint8_t PIN_M2_B = 11;  // 原本D7，改測D11

const uint32_t PWM_PERIOD_US = 200;
const uint8_t TEST_DUTY = 177;

volatile int32_t posM2 = 0;
volatile uint8_t stateM2 = 0;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void updateQuadM2() {
  uint8_t a = digitalRead(PIN_M2_A);
  uint8_t b = digitalRead(PIN_M2_B);
  uint8_t ns = (a << 1) | b;
  posM2 += QDEC_TABLE[((stateM2 << 2) | ns) & 0x0F];
  stateM2 = ns;
}
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

void setup() {
  Serial.begin(115200);

  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  pwm.setPin(CH_M2_IN1, 4095);  // 正轉方向
  pwm.setPin(CH_M2_IN2, 0);

  pinMode(M2_PWM_PIN, OUTPUT);

  Serial.println(F("=== M2 raw open-loop duty=177 test ==="));
}

uint32_t lastPrintMs = 0;

void loop() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)TEST_DUTY * PWM_PERIOD_US / 255;
  digitalWrite(M2_PWM_PIN, phase < onUs ? HIGH : LOW);

  uint32_t now = millis();
  if (now - lastPrintMs >= 300) {
    lastPrintMs = now;
    Serial.print(F("t="));
    Serial.print(now);
    Serial.print(F("ms posM2="));
    Serial.println(posM2);
  }
}
