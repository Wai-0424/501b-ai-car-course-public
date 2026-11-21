// 排查用：量測「冷啟動掙脫duty」，不是量測「維持運轉duty」。
//
// car2_openloop_duty_sweep.ino量到的是duty 30~90時「已經在轉的狀態下
// 換到下一個duty」還能不能維持轉動——它從頭到尾沒有讓馬達真正停下來
// 過(30->40->50...->90連續換，中間沒歸零)，量到的是動摩擦區間，不是
// 靜摩擦/掙脫門檻。
//
// 這次car2_speed_control_test.ino的低速測試(見docs底下車2低速測試紀錄)
// 發現：閉環模式下，duty在~125~165這段「爬升途中」有時會讓兩輪同時
// 卡住300ms~1s以上才突然掙脫暴衝，但duty<=~120或一次到位衝到~175以上
// 時反而不會卡——懷疑是「從真正靜止狀態，固定duty能不能一次掙脫」這個
// 冷啟動門檻，在某個duty區間本身就不穩定(有時掙脫有時卡住)，不是連續
// 動摩擦掃描量得到的。
//
// 測法：每個duty值重複REPEATS次，每次都先讓馬達完全停止(coast)並停留
// STOP_MS讓殘餘轉動完全歸零，再瞬間切到固定duty(不經過PID)、觀察
// TEST_WINDOW_MS內的編碼器位置變化，找「第一次真的開始持續轉動」的
// 時間點，以及過程中有沒有「動一點又停住」的情形。M1/M2同時測(跟正式
// 版一樣兩輪永遠一起動)，因為車2低速卡頓看到的是兩輪同時卡住，懷疑
// 跟共用的降壓模組瞬間電流有關，要在同樣的負載條件下量才有意義。

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

// 涵蓋正式版低速測試看到「卡頓帶」前後的完整範圍，不只是舊掃描的30~90
const uint8_t DUTY_LIST[] = {40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 250};
const uint8_t DUTY_COUNT = sizeof(DUTY_LIST) / sizeof(DUTY_LIST[0]);
const uint8_t REPEATS = 3;

const uint32_t STOP_MS = 500;         // 每次trial開始前先完全停止、歸零殘餘轉動
const uint32_t TEST_WINDOW_MS = 900;  // 固定duty後觀察多久(車2閉環測試看過卡住超過1秒才掙脫)
const uint32_t LOG_PERIOD_MS = 10;    // 100Hz，要抓到「動一點又停」的細節

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

enum Phase { PH_STOP, PH_RUN, PH_DONE };
Phase phase = PH_STOP;
uint8_t dutyIndex = 0;
uint8_t repeatIndex = 0;
uint32_t phaseStartMs = 0;
uint32_t lastLogMs = 0;

void updateMotorPWM() {
  uint32_t phaseUs = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phaseUs < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phaseUs < m2OnUs ? HIGH : LOW);
}

void startStopPhase() {
  phase = PH_STOP;
  m1Duty = 0;
  m2Duty = 0;
  phaseStartMs = millis();
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

  Serial.println(F("=== 冷啟動掙脫duty量測 ==="));
  dutyIndex = 0;
  repeatIndex = 0;
  startStopPhase();
  lastLogMs = phaseStartMs;
}

void loop() {
  updateMotorPWM();
  uint32_t now = millis();

  if (phase == PH_STOP && (now - phaseStartMs >= STOP_MS)) {
    // 歸零累積位置，讓接下來的deltaTicks/位置變化直接代表這次trial自己的轉動量
    noInterrupts();
    posM1 = 0;
    posM2 = 0;
    interrupts();
    m1Duty = DUTY_LIST[dutyIndex];
    m2Duty = DUTY_LIST[dutyIndex];
    phase = PH_RUN;
    phaseStartMs = now;
    Serial.print(F(">>> trial duty="));
    Serial.print(m1Duty);
    Serial.print(F(" repeat="));
    Serial.println(repeatIndex);
  } else if (phase == PH_RUN && (now - phaseStartMs >= TEST_WINDOW_MS)) {
    repeatIndex++;
    if (repeatIndex >= REPEATS) {
      repeatIndex = 0;
      dutyIndex++;
    }
    if (dutyIndex >= DUTY_COUNT) {
      phase = PH_DONE;
      m1Duty = 0;
      m2Duty = 0;
      pwm.setPin(CH_M1_IN1, 4095);
      pwm.setPin(CH_M1_IN2, 4095);
      pwm.setPin(CH_M2_IN1, 4095);
      pwm.setPin(CH_M2_IN2, 4095);
      Serial.println(F("=== 量測結束，已停止 ==="));
    } else {
      startStopPhase();
    }
  }

  if (phase != PH_DONE && (now - lastLogMs >= LOG_PERIOD_MS)) {
    lastLogMs = now;
    Serial.print(F("[cs] ph="));
    Serial.print(phase == PH_STOP ? F("stop") : F("run"));
    Serial.print(F(" duty="));
    Serial.print(phase == PH_RUN ? DUTY_LIST[dutyIndex] : 0);
    Serial.print(F(" rep="));
    Serial.print(repeatIndex);
    Serial.print(F(" t="));
    Serial.print(now - phaseStartMs);
    Serial.print(F(" posM1="));
    Serial.print(posM1);
    Serial.print(F(" posM2="));
    Serial.println(posM2);
  }
}
