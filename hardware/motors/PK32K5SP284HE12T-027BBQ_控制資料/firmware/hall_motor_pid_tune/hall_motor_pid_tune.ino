// PID 速度閉環測試工具（給 Hennkwell PK32K5SP284HE12T-027BBQ 這顆測試
// 馬達用）
//
// feedforward 常數來自 hall_motor_pwm_sweep 的掃描結果（10kHz PWM）：
//   duty = 55.82 + 1.55*(tick/20ms)，死區duty約56，
//   MAX_TICKS_PER_INTERVAL(duty=250時) = 129.52
//
// Serial 指令（一行一個，直接文字輸入即可）：
//   t <-100~100>   設定目標速度百分比
//   kp <float>     設定 Kp
//   ki <float>     設定 Ki
//   slew <float>   設定輸出斜率限制（duty/PID週期）
//   reset          清空積分項、濾波速度（換一組參數測新的step前建議先reset）
//
// 每個 PID 週期（20ms）印一行：
//   PID,target,actual,duty
// 方便電腦端 parse。

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint32_t PWM_PERIOD_US = 100;  // 10kHz

const uint8_t PIN_M1_A = 2;
const uint8_t PIN_M1_B = 3;

const float TICKS_PER_REV = 48.0;
const uint32_t PID_INTERVAL_MS = 20;

const float FF_DEADBAND = 55.82;
const float FF_SLOPE = 1.55;
const float MAX_TICKS_PER_INTERVAL = 129.52;
const float PID_INTEGRAL_LIMIT = 300.0;

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

// ---- 可調參數（透過 Serial 指令即時修改，不用重新燒錄）----
// 自動調參結果（電腦端跑 grid search，target=60%階躍測試，見
// pid_autotune.py/pid_autotune2.py）：Kp=0.25, Ki=0.04 為最終選擇
// —— overshoot~10.7%、穩態誤差~0.01%（幾乎為0）、震盪~0.5%、
// 上升時間~319ms。Ki=0 分數技術上更高（overshoot更低、更穩定），
// 但會永遠留著3~4%穩態誤差；這裡選擇犧牲一點overshoot換取長期
// 追蹤準確度。
float pidKp = 0.25;
float pidKi = 0.04;
float maxDutyStepPerCycle = 15.0;
int targetPercent = 0;

// ---- PID 狀態 ----
float pidIntegral = 0;
int32_t pidLastPos = 0;
float pidFilteredSpeed = 0;
int8_t pidLastDir = 2;
uint8_t m1DriverDuty = 0;
float lastActualSpeed = 0;
float lastTargetSpeed = 0;

void setDirectionCached(int dir) {
  if (pidLastDir == dir) return;
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
  pidLastDir = (int8_t)dir;
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)m1DriverDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
}

void resetPidState() {
  pidIntegral = 0;
  pidFilteredSpeed = 0;
  pidLastPos = posM1;
}

void updateMotorPID() {
  int32_t currentPos = posM1;
  int32_t deltaTicks = currentPos - pidLastPos;
  pidLastPos = currentPos;

  if (targetPercent == 0) {
    setDirectionCached(0);
    m1DriverDuty = 0;
    pidIntegral = 0;
    pidFilteredSpeed = 0;
    lastActualSpeed = 0;
    lastTargetSpeed = 0;
    return;
  }

  int dir = targetPercent > 0 ? 1 : -1;
  float rawActualSpeed = deltaTicks * dir;

  const float ALPHA = 0.3f;
  pidFilteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pidFilteredSpeed;
  float actualSpeed = pidFilteredSpeed;
  lastActualSpeed = actualSpeed;

  float targetSpeed = (abs(targetPercent) / 100.0f) * MAX_TICKS_PER_INTERVAL;
  lastTargetSpeed = targetSpeed;
  float error = targetSpeed - actualSpeed;

  float feedforward = FF_DEADBAND + FF_SLOPE * targetSpeed;

  float tentativeOutput = feedforward + pidKp * error + pidKi * pidIntegral;
  bool willSaturate = (tentativeOutput <= 0.0f) || (tentativeOutput >= 255.0f);
  if (!willSaturate) {
    pidIntegral = constrain(pidIntegral + error, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  }

  float output = feedforward + pidKp * error + pidKi * pidIntegral;

  float prevDuty = (float)m1DriverDuty;
  float slewedOutput = constrain(output, prevDuty - maxDutyStepPerCycle, prevDuty + maxDutyStepPerCycle);
  float newDuty = constrain(slewedOutput, 0.0f, 255.0f);
  m1DriverDuty = (uint8_t)newDuty;

  setDirectionCached(dir);
}

// ---- Serial 指令解析（緩衝逐字元組行再處理，避免 parseInt 的阻塞問題）----
char inputBuf[32];
uint8_t inputLen = 0;

// 不用 sscanf 的 %f：這個核心的 C library 很可能沒連進浮點數版的
// scanf（embedded toolchain常見的省flash做法），實測 sscanf(..,"%f",..)
// 完全解不出值（n 卡在1），改用手動找空白分隔 + atof() 代替，atof()
// 一般都有正常支援。
void handleCommand(char* line) {
  char* sp = strchr(line, ' ');
  char* valStr = nullptr;
  if (sp != nullptr) {
    *sp = '\0';
    valStr = sp + 1;
  }
  char* cmd = line;
  float val = (valStr != nullptr) ? atof(valStr) : 0;

  if (strcmp(cmd, "t") == 0 && valStr != nullptr) {
    targetPercent = constrain((int)val, -100, 100);
    Serial.print(F("OK target="));
    Serial.println(targetPercent);
  } else if (strcmp(cmd, "kp") == 0 && valStr != nullptr) {
    pidKp = val;
    Serial.print(F("OK kp="));
    Serial.println(pidKp, 4);
  } else if (strcmp(cmd, "ki") == 0 && valStr != nullptr) {
    pidKi = val;
    Serial.print(F("OK ki="));
    Serial.println(pidKi, 4);
  } else if (strcmp(cmd, "slew") == 0 && valStr != nullptr) {
    maxDutyStepPerCycle = val;
    Serial.print(F("OK slew="));
    Serial.println(maxDutyStepPerCycle, 2);
  } else if (strcmp(cmd, "reset") == 0) {
    resetPidState();
    Serial.println(F("OK reset"));
  }
}

void pollSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLen > 0) {
        inputBuf[inputLen] = '\0';
        handleCommand(inputBuf);
        inputLen = 0;
      }
    } else if (inputLen < sizeof(inputBuf) - 1) {
      inputBuf[inputLen++] = c;
    }
  }
}

uint32_t lastPidMs = 0;

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
  setDirectionCached(0);

  delay(500);
  Serial.println(F("=== PID tune tool ready ==="));
  lastPidMs = millis();
}

void loop() {
  updateMotorPWM();
  pollSerialInput();

  uint32_t now = millis();
  if (now - lastPidMs < PID_INTERVAL_MS) return;
  lastPidMs = now;

  updateMotorPID();

  Serial.print(F("PID,"));
  Serial.print(lastTargetSpeed, 1);
  Serial.print(F(","));
  Serial.print(lastActualSpeed, 1);
  Serial.print(F(","));
  Serial.println(m1DriverDuty);
}
