// 車2：硬體 PWM 開迴路致動器 + 編碼器串流（2026-10-10）——給 duty -> rpm 校正用
//
// 很薄的一支韌體：電腦端送「兩輪各自的 duty(帶號=方向)」，這裡用 analogWrite() 5 kHz 輸出，
// 並以 50 Hz 串流編碼器計數。所有掃描邏輯(階梯、保持、取平均)都在電腦端
// (host_tools/calibrate_hw.py)，這裡不做 PID、不做前饋、不做 kick。
// 腳位、方向腳、編碼器極性跟正式版 car2_car_control.ino 完全一致。
//
// 序列埠 115200（USB，要把 DTR/RTS 關掉才不會重啟板子）：
//   電腦 -> 板子（每行以 \n 結尾）
//     D <l> <r>   左右輪 duty，-255~255，正=前進、負=後退、0=停（方向腳全關）
//     X           全停
//   板子 -> 電腦
//     e,<ms>,<posL>,<posR>,<dutyL>,<dutyR>   每 20 ms 一行，pos 已換成「前進為正」的累積計數，
//                                            duty 是目前實際套用的帶號 duty
//     ready ... / fault,<L|R> / pca,<0|1>    事件
//
// 保護（L298N 沒有過流保護）：
//   - 看門狗：400 ms 沒收到任何指令 -> 全停。電腦端要持續送 D（建議 100 ms 一次）。
//   - 反轉保護：方向要翻轉時先停 REVERSE_HOLD_MS 才套用新方向。
//   - 堵轉切斷：duty 絕對值 >= STALL_MIN_DUTY 的輪子，編碼器連續 STALL_MS 沒動 -> 該輪
//     鎖定為 0，並回報 fault；收到 X 才解除。
//   - PCA9685 沒回應 -> 不輸出任何 PWM。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_IN1[2] = {3, 1};   // [0]=M1 左輪, [1]=M2 右輪；正轉時 IN1 高、IN2 低
const uint8_t CH_IN2[2] = {4, 2};
const uint8_t PWM_PIN[2] = {9, 8};  // 左 D9(MCTM0 CH0N)、右 D8(PWM0 CH0)

const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const uint32_t PWM_FREQ_HZ = 5000;
const uint32_t WATCHDOG_MS = 400;
const uint32_t REVERSE_HOLD_MS = 300;
const uint32_t STREAM_MS = 20;
const uint16_t STALL_MIN_DUTY = 110;   // 低於這個 duty 允許不轉（起轉門檻掃描時）
const uint32_t STALL_MS = 1200;

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

int32_t fwdPos(uint8_t w) { return w == 0 ? posM1 : -posM2; }

bool pcaReady = false;

bool pcaPin(uint8_t ch, bool high) {
  uint8_t r = high ? pwm.setPWM(ch, 4096, 0) : pwm.setPWM(ch, 0, 4096);
  return r == 0;
}

bool pcaProbe() {
  Wire1.beginTransmission(0x40);
  Wire1.write((uint8_t)0x00);
  return Wire1.endTransmission() == 0;
}

int16_t reqDuty[2] = {0, 0};     // 電腦要求的
int16_t appliedDuty[2] = {0, 0}; // 實際套用的
int8_t curDir[2] = {0, 0};
uint32_t holdUntilMs[2] = {0, 0};
bool latched[2] = {false, false};
int32_t lastPosForStall[2] = {0, 0};
uint32_t lastMoveMs[2] = {0, 0};
uint32_t lastCmdMs = 0;
uint32_t lastStreamMs = 0;

// 先把 PWM 設 0、再改方向腳，最後才設 PWM，避免方向切換瞬間帶電流。
void setWheelOff(uint8_t w) {
  analogWrite(PWM_PIN[w], 0);
  if (pcaReady) {
    bool a = pcaPin(CH_IN1[w], false);
    bool b = pcaPin(CH_IN2[w], false);
    if (!(a && b)) pcaReady = false;
  }
  curDir[w] = 0;
  appliedDuty[w] = 0;
}

void applyWheel(uint8_t w, uint32_t now) {
  int16_t want = reqDuty[w];
  if (latched[w] || !pcaReady) want = 0;
  if (want == 0) {
    if (appliedDuty[w] != 0 || curDir[w] != 0) setWheelOff(w);
    return;
  }
  int8_t dir = (want > 0) ? 1 : -1;
  uint8_t mag = (uint8_t)min(abs(want), 255);
  if (curDir[w] != 0 && curDir[w] != dir) {
    setWheelOff(w);
    holdUntilMs[w] = now + REVERSE_HOLD_MS;
    return;
  }
  if ((int32_t)(now - holdUntilMs[w]) < 0) return;
  if (curDir[w] != dir) {
    analogWrite(PWM_PIN[w], 0);
    bool a = pcaPin(dir > 0 ? CH_IN1[w] : CH_IN2[w], true);
    bool b = pcaPin(dir > 0 ? CH_IN2[w] : CH_IN1[w], false);
    if (!(a && b)) { pcaReady = false; setWheelOff(w); return; }
    curDir[w] = dir;
    lastPosForStall[w] = fwdPos(w);
    lastMoveMs[w] = now;
  }
  analogWrite(PWM_PIN[w], mag);
  appliedDuty[w] = dir * (int16_t)mag;
}

char lineBuf[40];
uint8_t lineLen = 0;

void handleLine(const char* s, uint32_t now) {
  if (s[0] == 'X') {
    reqDuty[0] = reqDuty[1] = 0;
    latched[0] = latched[1] = false;
    lastCmdMs = now;
  } else if (s[0] == 'D') {
    int l = 0, r = 0;
    if (sscanf(s + 1, "%d %d", &l, &r) == 2) {
      reqDuty[0] = (int16_t)constrain(l, -255, 255);
      reqDuty[1] = (int16_t)constrain(r, -255, 255);
      lastCmdMs = now;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PWM_PIN[0], OUTPUT);
  pinMode(PWM_PIN[1], OUTPUT);
  digitalWrite(PWM_PIN[0], LOW);
  digitalWrite(PWM_PIN[1], LOW);
  analogWriteFrequency(PWM_FREQ_HZ);
  analogWrite(PWM_PIN[0], 0);
  analogWrite(PWM_PIN[1], 0);

  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  pcaReady = pcaProbe();
  if (pcaReady) {
    pwm.begin();
    pwm.setPWMFreq(50);
    setWheelOff(0);
    setWheelOff(1);
  }
  lastCmdMs = millis();
  Serial.println();
  Serial.print(F("ready car2_hwpwm_openloop pwm_hz="));
  Serial.print(PWM_FREQ_HZ);
  Serial.print(F(" pca="));
  Serial.println(pcaReady ? 1 : 0);
}

void loop() {
  uint32_t now = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = '\0'; handleLine(lineBuf, now); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }

  if (now - lastCmdMs > WATCHDOG_MS) {
    reqDuty[0] = reqDuty[1] = 0;
  }

  // 堵轉切斷
  for (uint8_t w = 0; w < 2; w++) {
    int32_t p = fwdPos(w);
    if (abs(appliedDuty[w]) >= STALL_MIN_DUTY) {
      if (p != lastPosForStall[w]) { lastPosForStall[w] = p; lastMoveMs[w] = now; }
      else if (now - lastMoveMs[w] >= STALL_MS && !latched[w]) {
        latched[w] = true;
        Serial.print(F("fault,"));
        Serial.println(w == 0 ? 'L' : 'R');
      }
    } else {
      lastPosForStall[w] = p;
      lastMoveMs[w] = now;
    }
    applyWheel(w, now);
  }

  if (now - lastStreamMs >= STREAM_MS) {
    lastStreamMs = now;
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "e,%lu,%ld,%ld,%d,%d\n", (unsigned long)now,
                     (long)fwdPos(0), (long)fwdPos(1), appliedDuty[0], appliedDuty[1]);
    Serial.write((const uint8_t*)buf, n);
  }
}
