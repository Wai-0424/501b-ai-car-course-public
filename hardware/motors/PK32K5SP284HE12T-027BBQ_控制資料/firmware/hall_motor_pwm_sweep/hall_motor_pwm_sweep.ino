// PWM duty -> RPM 掃描測試（給 Hennkwell PK32K5SP284HE12T-027BBQ 這顆
// 測試馬達用，架構沿用 hall_encoder_feedback_test，只是這裡改成自動
// 掃描 duty 0~255，量每個 duty 穩態下的實際轉速，輸出 CSV 方便畫圖、
// 回歸算 feedforward 常數（跟車輛專案 pwm_rpm_sweep 手法一致）。
//
// PWM 頻率拉高到 10kHz（100us週期，車輛專案原本是5kHz/200us）。
//
// 輸出格式：CSV,<duty>,<rpm>（rpm 是馬達軸轉速，TICKS_PER_REV=48）
// 測完記得把 hall_encoder_feedback_test.ino 燒回去（如果還要用那個工具）。

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint32_t PWM_PERIOD_US = 100;  // 10kHz

const uint8_t PIN_M1_A = 2;
const uint8_t PIN_M1_B = 3;

const float TICKS_PER_REV = 48.0;  // 這顆馬達的值（12PPR x 4）
const uint32_t SETTLE_MS = 700;
const uint32_t MEASURE_MS = 1000;
const uint8_t DUTY_STEP = 10;

volatile int32_t posM1 = 0;
volatile uint8_t stateM1 = 0;

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
void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }

uint8_t m1Duty = 0;

void setDirection(int dir) {
  if (dir > 0) {
    pwm.setPin(CH_M1_IN1, 4095);
    pwm.setPin(CH_M1_IN2, 0);
  } else if (dir < 0) {
    pwm.setPin(CH_M1_IN1, 0);
    pwm.setPin(CH_M1_IN2, 4095);
  } else {
    pwm.setPin(CH_M1_IN1, 0);
    pwm.setPin(CH_M1_IN2, 0);
  }
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
}

void busyWaitPWM(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    updateMotorPWM();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  setDirection(0);

  delay(1000);
  Serial.println(F("=== PWM->RPM sweep start (10kHz) ==="));

  setDirection(1);
  for (uint16_t duty = 0; duty <= 255; duty += DUTY_STEP) {
    m1Duty = (uint8_t)duty;
    busyWaitPWM(SETTLE_MS);
    int32_t before = posM1;
    busyWaitPWM(MEASURE_MS);
    int32_t after = posM1;
    int32_t deltaTicks = after - before;
    float rpm = (deltaTicks / TICKS_PER_REV) * (60000.0f / MEASURE_MS);
    Serial.print(F("CSV,"));
    Serial.print(duty);
    Serial.print(F(","));
    Serial.println(rpm, 2);
  }
  m1Duty = 0;
  setDirection(0);

  Serial.println(F("=== sweep done ==="));
}

void loop() {
}
