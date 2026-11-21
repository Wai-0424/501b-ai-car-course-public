// 確認 D8/D9 這兩條 PWM 線實際各自驅動哪顆馬達（不透過「M1/M2」這種
// 可能搞混的命名，直接用「D4/D5 編碼器」跟「D6/D7 編碼器」兩組原始
// 訊號來比對）。
//
// 作法：CH1~CH4(兩顆馬達方向腳)都先設成「有效驅動」狀態（不是煞車、
// 不是懸空），確保只要對應的 ENA/ENB 有給 PWM，馬達就一定會轉。
// 然後分兩個階段：
//   階段A：只給 D9 50% duty，D8=0，看 D4/D5 還是 D6/D7 那組編碼器在動
//   階段B：只給 D8 50% duty，D9=0，看換成哪組編碼器在動
// 兩階段各跑5秒，中間停1秒。跑完自動停止、進入安全煞車狀態。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint8_t CH1 = 1;  // 對應驅動板 IN1
const uint8_t CH2 = 2;  // 對應驅動板 IN2
const uint8_t CH3 = 3;  // 對應驅動板 IN3
const uint8_t CH4 = 4;  // 對應驅動板 IN4

const uint8_t D8 = 8;
const uint8_t D9 = 9;

const uint8_t PIN_A_D4 = 4;
const uint8_t PIN_B_D5 = 5;
const uint8_t PIN_A_D6 = 6;
const uint8_t PIN_B_D7 = 7;

const uint16_t STEER_CENTER_US = 1500;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz
const uint8_t TEST_DUTY = 128;       // 50%

volatile int32_t posD45 = 0;  // D4/D5 這組編碼器
volatile int32_t posD67 = 0;  // D6/D7 這組編碼器
volatile uint8_t stateD45 = 0;
volatile uint8_t stateD67 = 0;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void updateQuadD45() {
  uint8_t a = digitalRead(PIN_A_D4);
  uint8_t b = digitalRead(PIN_B_D5);
  uint8_t ns = (a << 1) | b;
  posD45 += QDEC_TABLE[((stateD45 << 2) | ns) & 0x0F];
  stateD45 = ns;
}
void updateQuadD67() {
  uint8_t a = digitalRead(PIN_A_D6);
  uint8_t b = digitalRead(PIN_B_D7);
  uint8_t ns = (a << 1) | b;
  posD67 += QDEC_TABLE[((stateD67 << 2) | ns) & 0x0F];
  stateD67 = ns;
}
void isrD4() { updateQuadD45(); }
void isrD5() { updateQuadD45(); }
void isrD6() { updateQuadD67(); }
void isrD7() { updateQuadD67(); }

uint8_t dutyD8 = 0;
uint8_t dutyD9 = 0;

void updatePWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onD8 = (uint32_t)dutyD8 * PWM_PERIOD_US / 255;
  uint32_t onD9 = (uint32_t)dutyD9 * PWM_PERIOD_US / 255;
  digitalWrite(D8, phase < onD8 ? HIGH : LOW);
  digitalWrite(D9, phase < onD9 ? HIGH : LOW);
}

void holdAndPrint(uint32_t ms, const char* label) {
  uint32_t start = millis();
  uint32_t lastPrint = start;
  while (millis() - start < ms) {
    updatePWM();
    uint32_t now = millis();
    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.print(label);
      Serial.print(F("  posD45="));
      Serial.print(posD45);
      Serial.print(F("  posD67="));
      Serial.println(posD67);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_A_D4, INPUT);
  pinMode(PIN_B_D5, INPUT);
  pinMode(PIN_A_D6, INPUT);
  pinMode(PIN_B_D7, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_A_D4), isrD4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B_D5), isrD5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_A_D6), isrD6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B_D7), isrD7, CHANGE);

  pinMode(D8, OUTPUT);
  pinMode(D9, OUTPUT);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  // 兩顆馬達方向都設成「有效驅動」狀態（IN1=HIGH,IN2=LOW / IN3=HIGH,IN4=LOW），
  // 確保只要對應ENA/ENB有PWM，馬達就會轉，不會卡在煞車或懸空狀態。
  pwm.setPin(CH1, 4095);
  pwm.setPin(CH2, 0);
  pwm.setPin(CH3, 4095);
  pwm.setPin(CH4, 0);
}

bool done = false;

void loop() {
  if (done) return;
  done = true;

  delay(1000);
  Serial.println(F("=== 階段A: 只給 D9=50%, D8=0 ==="));
  dutyD9 = TEST_DUTY;
  dutyD8 = 0;
  holdAndPrint(5000, "[A]");

  dutyD8 = 0;
  dutyD9 = 0;
  holdAndPrint(1000, "[stop]");

  Serial.println(F("=== 階段B: 只給 D8=50%, D9=0 ==="));
  dutyD8 = TEST_DUTY;
  dutyD9 = 0;
  holdAndPrint(5000, "[B]");

  dutyD8 = 0;
  dutyD9 = 0;
  pwm.setPin(CH1, 0);
  pwm.setPin(CH2, 0);
  pwm.setPin(CH3, 0);
  pwm.setPin(CH4, 0);
  Serial.println(F("=== 測試結束，已停止 ==="));
}
