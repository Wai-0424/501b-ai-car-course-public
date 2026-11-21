// 銜接 bmduino_pca9685_wire1_servo_center 的下一步驗證：確認換到 BMCOM1(Wire1)
// 之後，馬達方向/轉速這條路（PCA9685 CH1~CH4 方向 + D8/D9 軟體PWM經74LS14
// 到驅動板）接線正確，尤其是懷疑 ENA/ENB（D9/D8）或方向腳可能兩顆馬達
// 之間接反/接錯的狀況，用固定 50% duty 正反轉各跑一次來驗證。
//
// 一次只讓一顆馬達動，方便用眼睛對照：
//   1. M1 正轉 50% -> 停 -> M1 反轉 50% -> 停
//   2. M2 正轉 50% -> 停 -> M2 反轉 50% -> 停
// 每個階段 Serial 都會印出目前應該是哪顆馬達、哪個方向，人工比對實際
// 轉動的是哪顆輪子、哪個方向，藉此抓出：
//   - CH1~CH4 / D8 / D9 兩顆馬達之間是否接反（例如指令是 M1 結果轉的是
//     實際 M2 那顆馬達）
//   - IN1/IN2 方向腳是否對調（例如指令是「正轉」結果實際是反轉）
//
// 開環方向/PWM 測試，不含 PID/閉環控制，但額外加了編碼器讀值，純被動讀取
// 印出來、不參與控制。兩種讀法都印，互相對照：
//   1. 4倍正交解碼後的 posM1/posM2（假設 D6/D7=M1、D4/D5=M2 這個配對正確）
//   2. **不做任何A/B配對假設**，D4/D5/D6/D7 各自獨立的原始toggle次數
//      （cntD4~cntD7）——這組數字才是拿來抓「插反/插錯」最直接的證據：
//      不管實際接線配對對不對，只要某根線真的接在某顆正在轉的馬達編碼器
//      上，它的toggle次數就一定會跳；哪一根線在哪個馬達指令(M1/M2)期間
//      跳、哪一根完全不跳，就能反推實際接線狀況，不會被4倍解碼的假設
//      誤導。
// 測完兩顆馬達自動停止並保持煞車狀態（duty=0）。驗證沒問題後才把
// Wire1 這個改動搬回正式版。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t CH_M2_IN1 = 4;  // 對應正式韌體對調後的常數（實接驅動板 IN3）
const uint8_t CH_M2_IN2 = 3;  // 實接驅動板 IN4

const uint8_t M1_PWM_PIN = 9;  // D9 -> 74LS14 -> M1 ENA
const uint8_t M2_PWM_PIN = 8;  // D8 -> 74LS14 -> M2 ENB

const uint8_t PIN_M1_A = 6;  // M1 編碼器 A相，跟 bmduino_car_control.ino 一致
const uint8_t PIN_M1_B = 7;  // M1 編碼器 B相
const uint8_t PIN_M2_A = 4;  // M2 編碼器 A相
const uint8_t PIN_M2_B = 5;  // M2 編碼器 B相

const uint16_t STEER_CENTER_US = 1500;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz，跟正式韌體一致
const uint8_t TEST_DUTY = 255;       // 100%

const uint32_t RUN_MS = 3000;   // 每個方向跑多久
const uint32_t STOP_MS = 1200;  // 兩個階段之間停多久，讓馬達完全停下來再換方向

volatile uint8_t m1Duty = 0;
volatile uint8_t m2Duty = 0;

// ---- 編碼器計數（4倍正交解碼，ISR更新，純讀取不參與控制）----
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

// ---- 原始腳位 toggle 次數，不做A/B配對假設，純粹「這根線有沒有在動」----
volatile uint32_t cntD4 = 0, cntD5 = 0, cntD6 = 0, cntD7 = 0;

void isrM1A() { cntD6++; updateQuadM1(); }  // D6 = PIN_M1_A
void isrM1B() { cntD7++; updateQuadM1(); }  // D7 = PIN_M1_B
void isrM2A() { cntD4++; updateQuadM2(); }  // D4 = PIN_M2_A
void isrM2B() { cntD5++; updateQuadM2(); }  // D5 = PIN_M2_B

void updatePWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

void setDir(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 0);
  } else if (dir < 0) {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 4095);
  } else {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 0);  // 煞車
  }
}

void printRawPins() {
  Serial.print(F("  [raw] D4="));
  Serial.print(digitalRead(PIN_M2_A));
  Serial.print(F(" D5="));
  Serial.print(digitalRead(PIN_M2_B));
  Serial.print(F(" D6="));
  Serial.print(digitalRead(PIN_M1_A));
  Serial.print(F(" D7="));
  Serial.print(digitalRead(PIN_M1_B));
  Serial.print(F("  cntD4="));
  Serial.print(cntD4);
  Serial.print(F(" cntD5="));
  Serial.print(cntD5);
  Serial.print(F(" cntD6="));
  Serial.print(cntD6);
  Serial.print(F(" cntD7="));
  Serial.println(cntD7);
}

void holdFor(uint32_t ms) {
  uint32_t start = millis();
  uint32_t lastPrint = start;
  while (millis() - start < ms) {
    updatePWM();
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.print(F("  posM1="));
      Serial.print(posM1);
      Serial.print(F(" posM2="));
      Serial.println(posM2);
      printRawPins();
    }
  }
}

void runPhase(const char* label, uint8_t in1Ch, uint8_t in2Ch, int dir,
              volatile uint8_t* dutyVar, uint8_t duty) {
  int32_t startM1 = posM1;
  int32_t startM2 = posM2;
  uint32_t startD4 = cntD4, startD5 = cntD5, startD6 = cntD6, startD7 = cntD7;
  Serial.println(label);
  setDir(in1Ch, in2Ch, dir);
  *dutyVar = duty;
  holdFor(duty == 0 ? STOP_MS : RUN_MS);
  Serial.print(F("  -> deltaM1="));
  Serial.print(posM1 - startM1);
  Serial.print(F(" deltaM2="));
  Serial.println(posM2 - startM2);
  Serial.print(F("  -> raw toggle counts this phase: D4="));
  Serial.print(cntD4 - startD4);
  Serial.print(F(" D5="));
  Serial.print(cntD5 - startD5);
  Serial.print(F(" D6="));
  Serial.print(cntD6 - startD6);
  Serial.print(F(" D7="));
  Serial.println(cntD7 - startD7);
}

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  setDir(CH_M1_IN1, CH_M1_IN2, 0);
  setDir(CH_M2_IN1, CH_M2_IN2, 0);

  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Serial.println(F("=== Motor direction test, 100% duty, Wire1(BMCOM1) ==="));
  Serial.println(F("Encoder ticks (D6/D7=M1, D4/D5=M2) printed every 500ms, read-only."));
  holdFor(1000);

  runPhase("[M1] forward 100% (CH1=HIGH,CH2=LOW, D9=255)", CH_M1_IN1, CH_M1_IN2, 1, &m1Duty, TEST_DUTY);
  runPhase("[M1] stop", CH_M1_IN1, CH_M1_IN2, 0, &m1Duty, 0);
  runPhase("[M1] reverse 100% (CH1=LOW,CH2=HIGH, D9=255)", CH_M1_IN1, CH_M1_IN2, -1, &m1Duty, TEST_DUTY);
  runPhase("[M1] stop", CH_M1_IN1, CH_M1_IN2, 0, &m1Duty, 0);

  runPhase("[M2] forward 100% (CH4=HIGH,CH3=LOW, D8=255)", CH_M2_IN1, CH_M2_IN2, 1, &m2Duty, TEST_DUTY);
  runPhase("[M2] stop", CH_M2_IN1, CH_M2_IN2, 0, &m2Duty, 0);
  runPhase("[M2] reverse 100% (CH4=LOW,CH3=HIGH, D8=255)", CH_M2_IN1, CH_M2_IN2, -1, &m2Duty, TEST_DUTY);
  runPhase("[M2] stop", CH_M2_IN1, CH_M2_IN2, 0, &m2Duty, 0);

  Serial.println(F("=== Sequence done. Both motors stopped (duty=0, brake). ==="));
  Serial.println(F("請確認：1) M1/M2各自轉的是不是預期的那顆輪子 2) forward/reverse跟實際轉向是否相符"));
}

void loop() {
  updatePWM();  // 維持兩顆馬達 duty=0 的煞車狀態
}
