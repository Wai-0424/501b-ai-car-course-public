// 承接 bmduino_motor_ch1ch3_low_pwm50_test（已確認這組方向+50%duty會讓馬達轉）
// 加上編碼器讀值即時列印，確認車2的編碼器（D6/D7=M1、D4/D5=M2）在馬達
// 實際轉動時，posM1/posM2 有沒有真的在變化。跟 car2_motor_pwm_sweep_bidir
// 掃描測試量到全部RPM=0.00的問題對照用。

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
const uint8_t TEST_DUTY = 128;       // 50%

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

  Serial.println(F("=== M1/M2 both driven @ 50% duty, watching encoder counts ==="));
}

uint32_t lastPrintMs = 0;

void loop() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)TEST_DUTY * PWM_PERIOD_US / 255;
  bool high = phase < onUs;
  digitalWrite(M1_PWM_PIN, high ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, high ? HIGH : LOW);

  uint32_t now = millis();
  if (now - lastPrintMs >= 500) {
    lastPrintMs = now;
    Serial.print(F("posM1="));
    Serial.print(posM1);
    Serial.print(F("  posM2="));
    Serial.println(posM2);
  }
}
