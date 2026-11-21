// 車1閉環控制（PID+feedforward）第一次上機測試，套用今天實測驗證過的
// 新接線狀態，跟正式版 bmduino_car_control.ino 的差異只有：
//   1. PCA9685 走 Wire1(BMCOM1)，不是主 Wire(A4/A5)
//   2. 編碼器腳位對調：M1(左輪，CH1/CH2+D9)的編碼器現在在 D4/D5，
//      M2(右輪，CH4/CH3+D8)的編碼器現在在 D6/D7——今天用
//      bmduino_motor_dir_test.ino 逐步驗證確認的，過程見對話紀錄
//   3. 不接 Serial1/CMD 協定，改成內建固定測試序列（不需要Jetson）：
//      兩輪同時 40% 正轉5秒 -> 停2秒 -> 40% 反轉5秒 -> 停，然後結束
//   4. PID/feedforward 常數沿用舊馬達校正值（FF_DEADBAND/FF_SLOPE/
//      MAX_TICKS_PER_INTERVAL），新馬達還沒重新校正，這次測試主要是
//      看PID能不能追上目標、要不要重新調——不代表這組常數已經適用新馬達。
//      今天已經觀察到新馬達在開環測試時需要遠高於50%的duty才能穩定
//      啟動，這裡的PID+feedforward實際跑起來需要多少duty也一併記錄下來。
//
// updateMotorPID()/QDEC_TABLE/driveDirectionCached() 邏輯完全照抄
// bmduino_car_control.ino，未改動。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SerialBM Serial1  // 保留but不使用，避免之後合併時漏改

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

// ---- PCA9685 channel 配置（跟正式版一致，driver端今天驗證沒問題）----
const uint8_t CH_STEER = 0;
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;

const uint8_t M1_PWM_PIN = 9;
const uint8_t M2_PWM_PIN = 8;

// ---- 編碼器輸入：今天實測確認對調，M1在D4/D5、M2在D6/D7 ----
const uint8_t PIN_M1_A = 2;  // 原本D4不穩定，改接到D2
const uint8_t PIN_M1_B = 3;  // 原本D5有雜訊，改接到D3
const uint8_t PIN_M2_A = 6;
const uint8_t PIN_M2_B = 7;

const uint16_t STEER_CENTER_US = 1500;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz

const float TICKS_PER_REV = 1550.0;
const uint32_t PID_INTERVAL_MS = 20;

// ---- feedforward常數：今天用 car1_motor_pwm_sweep_bidir_wire1.ino 對新馬達
// 正反轉分開實測回歸出來的（M1 rev / M2 fwd 是21點乾淨線性回歸；
// 換到目前這片驅動板+現有馬達之後，重新掃描（car1_sweep_final.log）
// 結果四個方向全部健康對稱、死區都在37~41之間，不再有先前那種「難轉
// 方向死區高達237」的不對稱情況——常數全部換成這次乾淨掃描的回歸值）----
const float FF_DEADBAND_M1_FWD = 37.39;
const float FF_SLOPE_M1_FWD = 1.5632;
const float MAX_TICKS_PER_INTERVAL_M1_FWD = 142.66;

const float FF_DEADBAND_M1_REV = 37.13;
const float FF_SLOPE_M1_REV = 1.7372;
const float MAX_TICKS_PER_INTERVAL_M1_REV = 127.98;

const float FF_DEADBAND_M2_FWD = 40.83;
const float FF_SLOPE_M2_FWD = 1.3280;
const float MAX_TICKS_PER_INTERVAL_M2_FWD = 164.00;

const float FF_DEADBAND_M2_REV = 41.30;
const float FF_SLOPE_M2_REV = 1.3044;
const float MAX_TICKS_PER_INTERVAL_M2_REV = 166.36;

// 死區恢復正常後，PID_KP/PID_INTEGRAL_LIMIT 也跟著調回較保守的值，
// 不再需要之前為了衝過237高門檻而放大的設定。
const float PID_KP = 0.5;
const float PID_KI = 0.08;
const float PID_INTEGRAL_LIMIT = 300.0;
const float MAX_DUTY_STEP_PER_CYCLE = 15.0;

// ---- 測試序列參數 ----
const int TEST_TARGET_PERCENT = 40;  // 目標速度百分比
const uint32_t RUN_MS = 5000;
const uint32_t STOP_MS = 2000;
const uint32_t PRINT_MS = 300;

struct PidState {
  float integral;
  int32_t lastPos;
  float filteredSpeed;
  int8_t lastDir;
};

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

PidState pidM1State = { 0, 0, 0, 2 };
PidState pidM2State = { 0, 0, 0, 2 };

uint8_t m1DriverDuty = 0;
uint8_t m2DriverDuty = 0;
float lastActualM1 = 0;
float lastActualM2 = 0;

int leftSpeed = 0;
int rightSpeed = 0;

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1DriverDuty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2DriverDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

void driveDirection(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 0);
  } else if (dir < 0) {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 4095);
  } else {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 0);
  }
}

void driveDirectionCached(uint8_t in1Ch, uint8_t in2Ch, int dir, int8_t* lastDir) {
  if (*lastDir == dir) return;
  driveDirection(in1Ch, in2Ch, dir);
  *lastDir = (int8_t)dir;
}

// 方向不對稱：正反轉各自的feedforward常數差異太大（尤其是"難轉"方向的
// deadband高達~237、"好轉"方向只要~40），不能再共用一組常數，改成傳入
// 正反轉各自的deadband/slope/maxTicks，內部依targetPercent正負選用。
void updateMotorPID(int targetPercent, volatile int32_t* pos, PidState* pid,
                     uint8_t* dutyOut, uint8_t in1Ch, uint8_t in2Ch, float* actualOut,
                     float ffDeadbandFwd, float ffSlopeFwd, float maxTicksFwd,
                     float ffDeadbandRev, float ffSlopeRev, float maxTicksRev,
                     int encSign) {
  int32_t currentPos = *pos;
  int32_t deltaTicks = currentPos - pid->lastPos;
  pid->lastPos = currentPos;

  if (targetPercent == 0) {
    driveDirectionCached(in1Ch, in2Ch, 0, &pid->lastDir);
    *dutyOut = 0;
    pid->integral = 0;
    pid->filteredSpeed = 0;
    *actualOut = 0;
    return;
  }

  int dir = targetPercent > 0 ? 1 : -1;
  float ffDeadband = (dir > 0) ? ffDeadbandFwd : ffDeadbandRev;
  float ffSlope = (dir > 0) ? ffSlopeFwd : ffSlopeRev;
  float maxTicksPerInterval = (dir > 0) ? maxTicksFwd : maxTicksRev;
  float rawActualSpeed = deltaTicks * dir * encSign;

  const float ALPHA = 0.3f;
  pid->filteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pid->filteredSpeed;
  float actualSpeed = pid->filteredSpeed;
  *actualOut = actualSpeed;

  float targetSpeed = (abs(targetPercent) / 100.0f) * maxTicksPerInterval;
  float error = targetSpeed - actualSpeed;

  float feedforward = ffDeadband + ffSlope * targetSpeed;

  float tentativeOutput = feedforward + PID_KP * error + PID_KI * pid->integral;
  bool willSaturate = (tentativeOutput <= 0.0f) || (tentativeOutput >= 255.0f);
  if (!willSaturate) {
    pid->integral = constrain(pid->integral + error, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  }

  float output = feedforward + PID_KP * error + PID_KI * pid->integral;

  float prevDuty = (float)(*dutyOut);
  float slewedOutput = constrain(output, prevDuty - MAX_DUTY_STEP_PER_CYCLE, prevDuty + MAX_DUTY_STEP_PER_CYCLE);

  float newDuty = constrain(slewedOutput, 0.0f, 255.0f);
  *dutyOut = (uint8_t)newDuty;

  driveDirectionCached(in1Ch, in2Ch, dir, &pid->lastDir);
}

uint32_t lastPidMs = 0;
uint32_t lastPrintMs = 0;

void runPidFor(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    updateMotorPWM();
    uint32_t now = millis();
    if (now - lastPidMs >= PID_INTERVAL_MS) {
      lastPidMs = now;
      updateMotorPID(leftSpeed, &posM1, &pidM1State, &m1DriverDuty, CH_M1_IN1, CH_M1_IN2, &lastActualM1,
                     FF_DEADBAND_M1_FWD, FF_SLOPE_M1_FWD, MAX_TICKS_PER_INTERVAL_M1_FWD,
                     FF_DEADBAND_M1_REV, FF_SLOPE_M1_REV, MAX_TICKS_PER_INTERVAL_M1_REV, -1);
      updateMotorPID(rightSpeed, &posM2, &pidM2State, &m2DriverDuty, CH_M2_IN1, CH_M2_IN2, &lastActualM2,
                     FF_DEADBAND_M2_FWD, FF_SLOPE_M2_FWD, MAX_TICKS_PER_INTERVAL_M2_FWD,
                     FF_DEADBAND_M2_REV, FF_SLOPE_M2_REV, MAX_TICKS_PER_INTERVAL_M2_REV, 1);
    }
    if (now - lastPrintMs >= PRINT_MS) {
      lastPrintMs = now;
      float targetTicksM1 = (abs(leftSpeed) / 100.0f) * (leftSpeed >= 0 ? MAX_TICKS_PER_INTERVAL_M1_FWD : MAX_TICKS_PER_INTERVAL_M1_REV);
      float targetTicksM2 = (abs(rightSpeed) / 100.0f) * (rightSpeed >= 0 ? MAX_TICKS_PER_INTERVAL_M2_FWD : MAX_TICKS_PER_INTERVAL_M2_REV);
      Serial.print(F("M1 target%="));
      Serial.print(leftSpeed);
      Serial.print(F(" targetTick="));
      Serial.print(targetTicksM1, 1);
      Serial.print(F(" actualTick="));
      Serial.print(lastActualM1, 1);
      Serial.print(F(" duty="));
      Serial.print(m1DriverDuty);
      Serial.print(F(" | M2 target%="));
      Serial.print(rightSpeed);
      Serial.print(F(" targetTick="));
      Serial.print(targetTicksM2, 1);
      Serial.print(F(" actualTick="));
      Serial.print(lastActualM2, 1);
      Serial.print(F(" duty="));
      Serial.println(m2DriverDuty);
    }
  }
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
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  driveDirection(CH_M1_IN1, CH_M1_IN2, 0);
  driveDirection(CH_M2_IN1, CH_M2_IN2, 0);

  Serial.println(F("=== Car1 closed-loop (PID+feedforward) test, Wire1(BMCOM1) ==="));
  Serial.println(F("Encoder pins swapped per today's verification: M1=D4/D5, M2=D6/D7."));
  Serial.print(F("Target speed: "));
  Serial.print(TEST_TARGET_PERCENT);
  Serial.println(F("% both wheels."));

  lastPidMs = millis();
  lastPrintMs = millis();

  Serial.println(F(">>> forward"));
  leftSpeed = TEST_TARGET_PERCENT;
  rightSpeed = TEST_TARGET_PERCENT;
  runPidFor(RUN_MS);

  Serial.println(F(">>> stop"));
  leftSpeed = 0;
  rightSpeed = 0;
  runPidFor(STOP_MS);

  Serial.println(F(">>> reverse"));
  leftSpeed = -TEST_TARGET_PERCENT;
  rightSpeed = -TEST_TARGET_PERCENT;
  runPidFor(RUN_MS);

  Serial.println(F(">>> stop"));
  leftSpeed = 0;
  rightSpeed = 0;
  runPidFor(STOP_MS);

  Serial.println(F("=== Sequence done. Both motors stopped (duty=0, brake). ==="));
}

void loop() {
  updateMotorPWM();  // 維持 duty=0 煞車狀態
}
