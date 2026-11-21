// 排查用：車2實際5kHz PWM下的完整duty(0~250)->RPM開迴路掃描，跟乾淨測試板
// 那次(10kHz)做法一樣：連續斜坡(馬達全程保持轉動、不歸零重啟)，不是
// car2_openloop_coldstart_test.ino那種「每個duty都從死停重新啟動」的
// 冷啟動測試——連續斜坡電流風險低很多，因為除了deadband以下，馬達
// 一直在轉，不會有前次測試那種「堵轉電流撐900ms」的狀況。
//
// 安全機制：一旦偵測到「前一個duty值還有在轉，換到更高的duty卻完全
// 沒動」，代表不是死區(死區只會發生在低duty、且應該是連續的)、而是
// 疑似驅動異常(跟今天稍早car2_openloop_coldstart_test.ino之後懷疑
// 的驅動板/保險絲狀況一樣)，立刻中止整個掃描並切斷輸出，不要傻等
// HOLD_MS跑完、更不會繼續往更高duty測。

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

const uint32_t PWM_PERIOD_US = 100;  // 2026-09-14診斷測試：改成10kHz，跟5kHz版本
                                      // (car2_openloop_duty_sweep_5khz_full.ino)做
                                      // 同一天、同一套接線的RPM-vs-duty曲線對照。
const uint32_t HOLD_MS = 1000;       // 每個duty停留多久
const uint32_t SETTLE_MS = 300;      // 前面這段時間量到的ticks不算，只讓轉速穩定
const uint32_t LOG_PERIOD_MS = 20;   // 50Hz，跟正式版PID頻率一致

// 0~250，每10一階，涵蓋deadband前後到接近滿duty
const uint8_t DUTY_START = 0;
const uint8_t DUTY_END = 250;
const uint8_t DUTY_STEP = 10;

const float TICKS_PER_REV = 1288.86;  // 車2實測修正值(見car2_speed_control_test.ino)

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

uint8_t curDuty = DUTY_START;
uint32_t stepStartMs = 0;
uint32_t lastLogMs = 0;
int32_t stepStartPosM1 = 0;
int32_t stepStartPosM2 = 0;
int32_t settlePosM1 = 0;  // settle結束那一刻的位置，RPM只算settle之後這段，
int32_t settlePosM2 = 0;  // 不要把settle期間的加速爬升也算進穩態轉速裡
bool settleCaptured = false;
bool everMoved = false;  // 一旦量到過真的有轉動就恆為true——只有「已經證實會動、
                          // 之後卻不動了」才算異常，還在死區以下本來就不會動，
                          // 不能拿來當異常判斷基準(這是上一版的bug，duty=10這種
                          // 死區以下的值會被誤判成異常中止)
bool aborted = false;
bool finished = false;

void stopAll(const __FlashStringHelper* reason) {
  curDuty = 0;
  pwm.setPin(CH_M1_IN1, 4095);
  pwm.setPin(CH_M1_IN2, 4095);
  pwm.setPin(CH_M2_IN1, 4095);
  pwm.setPin(CH_M2_IN2, 4095);
  Serial.println(reason);
}

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

  Serial.println(F("=== 車2 5kHz 完整duty->RPM 開迴路掃描 ==="));
  Serial.println(F("csv,duty,rpmM1,rpmM2"));
  curDuty = DUTY_START;
  stepStartMs = millis();
  stepStartPosM1 = posM1;
  stepStartPosM2 = posM2;
  lastLogMs = stepStartMs;
}

void loop() {
  if (aborted || finished) {
    updateMotorPWM();
    return;
  }
  updateMotorPWM();
  uint32_t now = millis();
  uint32_t elapsed = now - stepStartMs;

  if (elapsed >= SETTLE_MS && !settleCaptured) {
    settlePosM1 = posM1;
    settlePosM2 = posM2;
    settleCaptured = true;
  }

  // 安全中止判斷：settle時間過後，若這個duty完全沒有新增ticks，
  // 但上一階(較低duty)是有在動的，代表異常，立刻中止。
  if (elapsed >= SETTLE_MS && curDuty > 0 && everMoved) {
    int32_t movedThisStep1 = posM1 - stepStartPosM1;
    int32_t movedThisStep2 = posM2 - stepStartPosM2;
    if (movedThisStep1 == 0 && movedThisStep2 == 0) {
      aborted = true;
      stopAll(F("!!! 異常中止：先前已經證實會轉，這一階完全沒動，懷疑驅動異常，已切斷輸出 !!!"));
      return;
    }
  }

  if (elapsed >= HOLD_MS) {
    int32_t movedM1 = posM1 - settlePosM1;  // 只算settle結束之後這段，避免把
    int32_t movedM2 = posM2 - settlePosM2;  // 加速爬升期間也算進穩態RPM，之前
                                             // 版本錯拿stepStartPos(整段HOLD_MS
                                             // 都算)去除以只有(HOLD_MS-SETTLE_MS)
                                             // 的時間，會把RPM灌水多算約40%+
    float measureS = (HOLD_MS - SETTLE_MS) / 1000.0f;
    float rpmM1 = (movedM1 / TICKS_PER_REV) / measureS * 60.0f;
    float rpmM2 = (-movedM2 / TICKS_PER_REV) / measureS * 60.0f;

    Serial.print(F("csv,"));
    Serial.print(curDuty);
    Serial.print(',');
    Serial.print(rpmM1, 2);
    Serial.print(',');
    Serial.println(rpmM2, 2);

    // 門檻設20 ticks，濾掉編碼器雜訊/震動誤觸發（duty=10這種死區以下的
    // 值實測過量到過個位數ticks的雜訊，不是真的在轉）
    const int32_t MOVE_NOISE_FLOOR = 20;
    if (abs(movedM1) >= MOVE_NOISE_FLOOR || abs(movedM2) >= MOVE_NOISE_FLOOR) everMoved = true;

    if (curDuty >= DUTY_END) {
      finished = true;
      stopAll(F("=== 掃描完成，已停止 ==="));
      return;
    }
    curDuty = (uint8_t)min((int)curDuty + DUTY_STEP, (int)DUTY_END);
    stepStartMs = now;
    stepStartPosM1 = posM1;
    stepStartPosM2 = posM2;
    settleCaptured = false;
  }

  if (!aborted && !finished && (now - lastLogMs >= LOG_PERIOD_MS)) {
    lastLogMs = now;
    Serial.print(F("[ol] duty="));
    Serial.print(curDuty);
    Serial.print(F(" posM1="));
    Serial.print(posM1);
    Serial.print(F(" posM2="));
    Serial.println(posM2);
  }
}
