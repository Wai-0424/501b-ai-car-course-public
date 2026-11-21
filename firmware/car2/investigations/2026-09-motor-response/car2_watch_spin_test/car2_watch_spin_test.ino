// 排查用：固定duty=200持續轉3秒，人眼直接看/聽輪子有沒有真的在轉，
// 用來判斷car2_openloop_coldstart_test.ino量到的「duty80以上全部0
// ticks」是馬達真的沒轉，還是編碼器計數沒在動而已。純觀察用，不需要
// 精確重複多次。

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

const uint32_t PWM_PERIOD_US = 200;
const uint8_t RUN_DUTY = 200;
const uint32_t RUN_MS = 3000;

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

uint8_t curDuty = 0;
uint32_t startMs = 0;
uint32_t lastLogMs = 0;
bool done = false;

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)curDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < onUs ? HIGH : LOW);
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

  Serial.println(F("=== 3秒固定duty=200觀察測試，請直接看/聽輪子 ==="));
  delay(500);
  curDuty = RUN_DUTY;
  startMs = millis();
  lastLogMs = startMs;
}

void loop() {
  updateMotorPWM();
  uint32_t now = millis();

  if (!done && (now - startMs >= RUN_MS)) {
    done = true;
    curDuty = 0;
    pwm.setPin(CH_M1_IN1, 4095);
    pwm.setPin(CH_M1_IN2, 4095);
    pwm.setPin(CH_M2_IN1, 4095);
    pwm.setPin(CH_M2_IN2, 4095);
    Serial.println(F("=== 結束，已停止 ==="));
  }

  if (!done && (now - lastLogMs >= 200)) {
    lastLogMs = now;
    Serial.print(F("[watch] t="));
    Serial.print(now - startMs);
    Serial.print(F(" duty="));
    Serial.print(curDuty);
    Serial.print(F(" posM1="));
    Serial.print(posM1);
    Serial.print(F(" posM2="));
    Serial.println(posM2);
  }
}
