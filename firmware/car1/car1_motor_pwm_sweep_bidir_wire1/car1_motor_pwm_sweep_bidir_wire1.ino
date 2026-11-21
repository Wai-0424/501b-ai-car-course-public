// 車1新馬達（馳名電機 JGB37-520 DC12V 330RPM）duty -> RPM 正反轉掃描，
// 套用今天實測驗證過的新接線狀態，改寫自 car2_motor_pwm_sweep_bidir.ino
// 的雙向掃描架構（原本車1版本 car_motor_pwm_sweep.ino 只掃單方向，
// 這次因為新馬達兩個方向啟動特性明顯不對稱，需要各自分開校正）。
//
// 接線（今天用 bmduino_motor_dir_test.ino 逐步驗證確認）：
//   PCA9685 走 Wire1(BMCOM1)
//   M1（左輪）：PCA9685 CH1(IN1)/CH2(IN2)、PWM腳 D9、編碼器 D4/D5
//   M2（右輪）：PCA9685 CH4(IN1)/CH3(IN2)、PWM腳 D8、編碼器 D6/D7
//   （driver通道對應跟舊文件一致，只有編碼器D4/D5<->D6/D7對調）
//
// 測試前務必確認：車輪離地/固定，馬達轉動不會讓車子移動。
// 一次只讓一顆馬達動（M1 fwd -> M1 rev -> M2 fwd -> M2 rev），duty 從
// 0 慢慢升到 255（=100%），每個 step 間距 DUTY_STEP。
// 輸出格式：CSV,<motor>,<dir>,<duty 0-255>,<rpm>
// 全部跑完大約 4個方向 x 26個duty點 x 1.7秒 ≈ 3分鐘。
// TICKS_PER_REV 沿用車1原本估計值1550，之後拿這次CSV重新回歸
// FF_DEADBAND_*/FF_SLOPE_* 常數（正反轉分開算，因為今天已確認兩個
// 方向啟動特性不對稱）。

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint8_t PIN_M1_A = 2;  // 2026-09-08改接：原本D4接觸不良，改到D2
const uint8_t PIN_M1_B = 3;  // 原本D5有雜訊，改到D3

const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;
const uint8_t M2_PWM_PIN = 8;
const uint8_t PIN_M2_A = 6;
const uint8_t PIN_M2_B = 7;

const uint32_t PWM_PERIOD_US = 200;  // 5kHz
const float TICKS_PER_REV = 1550.0;
const uint32_t SETTLE_MS = 700;
const uint32_t MEASURE_MS = 1000;
const uint32_t STOP_GAP_MS = 1000;  // 換方向前先完全停止、留緩衝時間
const uint8_t DUTY_STEP = 10;
const uint8_t DUTY_MAX = 255;

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

uint8_t m1Duty = 0;
uint8_t m2Duty = 0;

void setDirection(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 0);
  } else if (dir < 0) {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 4095);
  } else {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 4095);  // 懸空(coast)，高速跑完後不要硬煞車，避免再生制動電流衝擊
  }
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

void busyWaitPWM(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    updateMotorPWM();
  }
}

void sweepMotorDir(const char* motorLabel, const char* dirLabel, uint8_t* dutyVar,
                    volatile int32_t* pos, uint8_t in1Ch, uint8_t in2Ch, int dir) {
  setDirection(in1Ch, in2Ch, dir);
  for (uint16_t duty = 0; duty <= DUTY_MAX; duty += DUTY_STEP) {
    *dutyVar = (uint8_t)duty;
    busyWaitPWM(SETTLE_MS);
    int32_t before = *pos;
    busyWaitPWM(MEASURE_MS);
    int32_t after = *pos;
    int32_t deltaTicks = after - before;
    float rpm = (deltaTicks / TICKS_PER_REV) * (60000.0f / MEASURE_MS);
    Serial.print(F("CSV,"));
    Serial.print(motorLabel);
    Serial.print(F(","));
    Serial.print(dirLabel);
    Serial.print(F(","));
    Serial.print(duty);
    Serial.print(F(","));
    Serial.println(rpm, 2);
  }
  *dutyVar = 0;
  setDirection(in1Ch, in2Ch, 0);
  busyWaitPWM(STOP_GAP_MS);
}

void setup() {
  Serial.begin(115200);

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
  setDirection(CH_M1_IN1, CH_M1_IN2, 0);
  setDirection(CH_M2_IN1, CH_M2_IN2, 0);
}

// 掃描本體放在loop()裡只跑一次，跟已驗證的其他測試環境對齊。
bool testDone = false;

void loop() {
  if (testDone) return;
  testDone = true;

  delay(1000);
  Serial.println(F("=== M1 forward sweep (CH1=HIGH,CH2=LOW) ==="));
  sweepMotorDir("M1", "fwd", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2, 1);
  Serial.println(F("=== M1 reverse sweep (CH1=LOW,CH2=HIGH) ==="));
  sweepMotorDir("M1", "rev", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2, -1);
  Serial.println(F("=== M2 forward sweep (CH4=HIGH,CH3=LOW) ==="));
  sweepMotorDir("M2", "fwd", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2, 1);
  Serial.println(F("=== M2 reverse sweep (CH4=LOW,CH3=HIGH) ==="));
  sweepMotorDir("M2", "rev", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2, -1);
  Serial.println(F("=== all sweeps done, motors stopped ==="));
}
