// 第二台車專用：馬達 duty -> RPM 正反轉掃描測試（改寫自 car_motor_pwm_sweep.ino，
// 那份是車1的版本）。差異：
//   - PCA9685 I2C 走 Wire1(BMCOM1)，不是主 Wire(A4/A5)（這台車的新接法）
//   - CH3/CH4 直接對應驅動板 IN3/IN4，沒有對調（car1 因為實際接線關係
//     CH3<->IN4、CH4<->IN3 對調，這台車目前實測直接對應即可轉動，
//     細節見 docs/architecture/wiring_plan.md 車1「M2 channel 接線備註」）
//   - 車1原始版本只掃單一方向，這裡兩顆馬達各自正/反轉都掃一次
//
// 測試前務必確認：車輪離地/固定，馬達轉動不會讓車子移動。
// 一次只讓一顆馬達動（M1 fwd -> M1 rev -> M2 fwd -> M2 rev），duty 從
// 0 慢慢升到 255（=100%），每個 step 間距 DUTY_STEP。
// 輸出格式：CSV,<motor>,<dir>,<duty 0-255>,<rpm>
// 全部跑完大約 4個方向 x 26個duty點 x 1.7秒 ≈ 3分鐘。
// TICKS_PER_REV 沿用車1的估計值 1550，這台車之後如果要做PID/feedforward
// 校正，建議拿這次量到的CSV重新回歸專屬這台車的常數，不要直接沿用車1的。

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

// 已用 bmduino_d8d9_identify_test 實測確認：D9 實際驅動的是右輪(M2)、
// D8 實際驅動的是左輪(M1)；編碼器 D4/D5 是左輪(M1)、D6/D7 是右輪(M2)，
// 跟先前假設（M1=D6/D7編碼器、PWM D9；M2=D4/D5編碼器、PWM D8）相反，
// 這裡改成實測確認的對應。
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 8;  // 實測確認：D8 = M1(左輪)
const uint8_t PIN_M1_A = 4;    // 實測確認：D4/D5 = M1(左輪)編碼器
const uint8_t PIN_M1_B = 5;

const uint8_t CH_M2_IN1 = 3;
const uint8_t CH_M2_IN2 = 4;
const uint8_t M2_PWM_PIN = 9;  // 實測確認：D9 = M2(右輪)
const uint8_t PIN_M2_A = 10;   // 原本D6/D7有問題讀不到訊號，已改接到D10/D11
const uint8_t PIN_M2_B = 11;

const uint32_t PWM_PERIOD_US = 200;  // 5kHz
const float TICKS_PER_REV = 1550.0;  // 沿用車1估計值，這台車之後可重新校正
const uint32_t SETTLE_MS = 700;
const uint32_t MEASURE_MS = 1000;
const uint32_t STOP_GAP_MS = 1000;  // 換方向前先完全停止、留緩衝時間
const uint8_t DUTY_STEP = 10;
const uint8_t DUTY_MAX = 255;  // 腳位已修正確認正常，補測完整範圍拿真實全速值

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
    pwm.setPin(in2Ch, 4095);  // 懸空(coast)，高速(接近全速)跑完後不要硬煞車，避免再生制動電流衝擊驅動板/齒輪箱
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

// 掃描本體移到 loop() 裡執行、只跑一次：懷疑這片 HT32 核心的外部中斷
// 要等 setup() 真正 return、進入 loop() 之後才會確實生效，跟已驗證有效
// 的 bmduino_motor_pwm50_encoder_check.ino（計數邏輯本來就在 loop()）
// 執行環境對齊，藉此排除「掃描全部量到0.00」是不是這個原因造成的。
bool testDone = false;

void loop() {
  if (testDone) return;
  testDone = true;

  delay(1000);
  Serial.println(F("=== M1 forward sweep (IN1=HIGH,IN2=LOW) ==="));
  sweepMotorDir("M1", "fwd", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2, 1);
  Serial.println(F("=== M1 reverse sweep (IN1=LOW,IN2=HIGH) ==="));
  sweepMotorDir("M1", "rev", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2, -1);
  Serial.println(F("=== M2 forward sweep (IN3=HIGH,IN4=LOW) ==="));
  sweepMotorDir("M2", "fwd", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2, 1);
  Serial.println(F("=== M2 reverse sweep (IN3=LOW,IN4=HIGH) ==="));
  sweepMotorDir("M2", "rev", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2, -1);
  Serial.println(F("=== all sweeps done, motors stopped ==="));
}
