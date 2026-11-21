// 車2 BTS7960 基礎版韌體（2026-10-12，bring-up 用，**尚未上板實測**）
//
// 從 car2_car_control（L298N 版）簡化而來：先把「BTS7960 驅動板 + 編碼器回授 + Orin 通訊」跑起來，
// 所以是【開迴路】：指令百分比直接換成 duty（有上限、有斜率限制），沒有 PID、沒有前饋表、沒有 turbo。
// 前饋表要在 BTS7960 + 實際頻率下重新量過才有意義（規劃稿 hardware/car2_bts7960_wiring_plan_2026-10-11.md §3.1），
// 量完再把 PID 搬回來。編碼器只用來回報速度與做堵轉切斷。
//
// 接線（規劃稿 §2；每個模組的 RPWM/LPWM 用相鄰兩腳）：
//   模組 A = M1 左輪：RPWM D2、LPWM D3、R_EN = PCA9685 CH3、L_EN = CH4、編碼器 A/B = D4/D5
//   模組 B = M2 右輪：RPWM D8、LPWM D9、R_EN = PCA9685 CH1、L_EN = CH2、編碼器 A/B = D10/D11
//   PCA9685 走 Wire1（I2C 0x40），CH0 = 方向盤；Orin 走 Serial3（460800）；USB Serial 115200 除錯
//   IS（電流）腳先不接。
//
// BTS7960 驅動邏輯（資料表 4.4.5）：
//   致能(R_EN/L_EN) 兩個都拉高 + RPWM 打 PWM、LPWM=0 → 正轉；RPWM=0、LPWM 打 PWM → 反轉；
//   致能拉低 → 兩個半橋都關（滑行）。致能是整座橋開不開，不是方向訊號。
//   「RPWM 算正轉」是任意的：車輪實際轉向反了，就把 M1_SWAP_PWM / M2_SWAP_PWM 改 true（或對調馬達線）。
//
// ★ 第一次上電前（輪子離地、電源限流 2~5 A、先低電壓）：
//   1. 先用 firmware/car2/investigations/2026-10-encoder-phase-test/ 確認編碼器 A/B 相，再把結果填到 ENC_SIGN_M1/M2。
//   2. 量 D2/D3/D8/D9 四條腳在 duty 0 時是不是穩定低電位（閒置那條若卡高電位，對面半橋會導通）。
//   3. USB 序列埠輸入  m 50 50  讓兩輪各以 50% 指令轉 2 秒（見下方 USB 文字指令），看轉向、看編碼器計數正負、
//      看 dbg 行的 pos 是否「前進時 M1 正、M2 正」（本檔的 pos 已乘上 ENC_SIGN，前進為正）。
//      注意：100% 指令只對應 duty 100/255，所以 m 15 的 duty 只有 15（約 6%、不到 1 V），馬達不會動；
//      duty 低於 STALL_MIN_DUTY(30) 也不會觸發堵轉切斷。要先從 m 40 40 左右開始往上加。
//
// 指令格式沿用 protocol.md：<CMD,seq 1,fwa 0,bwl 30,bwr 30,cs XX>（bwl=M1 左、bwr=M2 右，-100~100）。
// STA 回報格式與 car2_car_control 相同（rpmL/rpmR/vL/vR/f），Orin 端不用改：
//   f：bit0 = M1 堵轉切斷、bit1 = M2 堵轉切斷、bit2 = PCA9685/致能腳 I2C 故障。
//
// USB 文字指令（一行一個，結尾換行；與 <CMD,...> 框共用 USB 序列埠）：
//   m L R   手動驅動，L/R 為 -100~100 百分比，持續 MANUAL_HOLD_MS 後自動停
//   s       立即停止
//   z       編碼器計數歸零
//   h       說明

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SerialBM Serial3

// ================= 可調參數 =================
const uint32_t PWM_FREQ_HZ = 5000;      // 先 5 kHz 把換板跑通；之後依規劃稿 §3.1 試 10 → 20 kHz（前饋表要重量）
const uint8_t OPEN_LOOP_MAX_DUTY = 100; // 100% 指令對應的 duty（255 = 全開）。100 ≈ 39%，第一次上電的安全上限
const uint8_t DUTY_STEP_PER_TICK = 8;   // 每個 20 ms 週期 duty 最多變多少（0→100 約 0.3 s），抑制起步電流突波
const bool M1_SWAP_PWM = false;         // true = 把「前進用的 PWM 腳」換成 LPWM
const bool M2_SWAP_PWM = true;          // 2026-10-12 實測：M2 用 RPWM 當前進時右輪往後轉，所以對調
// 編碼器極性：前進時計數為正 = +1，為負 = -1。
// 2026-10-12 車子拆裝後手轉實測（car2_encoder_phase_test）：往前轉 M1 計數為正、M2 計數也為正，
// A/B 邊緣數平衡、非法跳變 < 0.3%，兩顆編碼器的 A/B 線都有訊號。舊版 car2_car_control 的 M2 是 -1，
// 拆裝後 M2 的 A/B 相順序變了，所以這裡改成 +1（不需要重接線）。
const int ENC_SIGN_M1 = +1;
const int ENC_SIGN_M2 = +1;
const uint32_t MANUAL_HOLD_MS = 2000;

// ================= 固定參數 =================
const uint8_t PIN_M1_A = 4, PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10, PIN_M2_B = 11;
const uint8_t M1_RPWM = 2, M1_LPWM = 3;
const uint8_t M2_RPWM = 8, M2_LPWM = 9;
const uint8_t CH_STEER = 0;
const uint8_t CH_M1_REN = 3, CH_M1_LEN = 4;
const uint8_t CH_M2_REN = 1, CH_M2_LEN = 2;
const uint8_t PCA9685_ADDR = 0x40;
const uint32_t PCA_RETRY_PERIOD_MS = 1000;

const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

const uint32_t PID_INTERVAL_MS = 20;    // 控制週期（沿用名稱：這版沒有 PID，仍是 50 Hz 更新輸出）
const uint32_t CMD_TIMEOUT_MS = 500;
const uint32_t STATUS_PERIOD_MS = 50;   // STA 20 Hz
const uint8_t USB_DBG_DIV = 4;          // USB dbg 每 200 ms 一行
const uint32_t REVERSE_BRAKE_MS = 200;  // 換方向前滑行(致能拉低)至少這麼久
const uint32_t STALL_CUTOFF_MS = 600;   // 有驅動(duty >= STALL_MIN_DUTY)卻連續這麼久編碼器不動 → 切斷鎖定
const uint8_t STALL_MIN_DUTY = 30;      // duty 太低馬達本來就可能不動，不算堵轉
const uint32_t FAULT_CLEAR_MS = 300;    // 鎖定後指令需連續為 0 這麼久才解除
const float SIGNED_SPEED_ALPHA = 0.3f;
const float TICKS_PER_REV = 1288.86f;   // 12 PPR × 4 × 3249/121（輸出軸一圈）

// ================= 型別（要放在所有函式之前）=================
struct ChunkedSender {
  char buf[255];  // len 是 uint8_t，上限 255
  uint8_t len;
  uint8_t pos;
  uint32_t nextSendMs;
  bool active;
};

struct LinkState {
  Stream* port;
  char buf[80];
  uint8_t len;
  bool active;
};

struct Motor {
  uint8_t pinFwd;          // 前進時打 PWM 的腳
  uint8_t pinRev;          // 反轉時打 PWM 的腳
  uint8_t chEnR, chEnL;    // PCA9685 致能通道
  int encSign;
  volatile int32_t* pos;
  int32_t lastPos;
  float signedSpeed;       // ticks/週期，前進為正（EMA）
  int8_t dir;              // 目前輸出方向：1 前進、-1 反轉、0 停止（致能拉低）
  int8_t lastNonZeroDir;
  uint8_t duty;
  bool enabled;            // 致能腳快取（true = 兩個都拉高）
  uint32_t zeroSinceMs;
  uint16_t noMotionMs;
  bool fault;
  bool faultClearing;
  uint32_t faultClearSinceMs;
};

// ================= 編碼器 =================
volatile int32_t posM1 = 0;
volatile int32_t posM2 = 0;
volatile uint8_t stateM1 = 0;
volatile uint8_t stateM2 = 0;
volatile uint32_t illegalM1 = 0;  // 跳過一格或兩相同時變化的次數（訊號品質指標）
volatile uint32_t illegalM2 = 0;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void updateQuadM1() {
  uint8_t ns = (digitalRead(PIN_M1_A) << 1) | digitalRead(PIN_M1_B);
  if (ns == stateM1) return;
  int8_t d = QDEC_TABLE[((stateM1 << 2) | ns) & 0x0F];
  if (d == 0) illegalM1++;
  posM1 += d;
  stateM1 = ns;
}
void updateQuadM2() {
  uint8_t ns = (digitalRead(PIN_M2_A) << 1) | digitalRead(PIN_M2_B);
  if (ns == stateM2) return;
  int8_t d = QDEC_TABLE[((stateM2 << 2) | ns) & 0x0F];
  if (d == 0) illegalM2++;
  posM2 += d;
  stateM2 = ns;
}
void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

// ================= 全域狀態 =================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR, Wire1);
bool pcaReady = false;
uint32_t pcaNextCheckMs = 0;
uint32_t dirI2cCallCount = 0;
uint32_t dirI2cErrorCount = 0;
double steerPulselengthUs = 0;
int lastSentSteerAngle = -1000;

Motor m1 = {M1_SWAP_PWM ? M1_LPWM : M1_RPWM, M1_SWAP_PWM ? M1_RPWM : M1_LPWM, CH_M1_REN, CH_M1_LEN,
            ENC_SIGN_M1, &posM1, 0, 0, 0, 0, 0, false, 0, 0, false, false, 0};
Motor m2 = {M2_SWAP_PWM ? M2_LPWM : M2_RPWM, M2_SWAP_PWM ? M2_RPWM : M2_LPWM, CH_M2_REN, CH_M2_LEN,
            ENC_SIGN_M2, &posM2, 0, 0, 0, 0, 0, false, 0, 0, false, false, 0};

int steerAngle = 0;
int leftSpeed = 0;    // 來自 <CMD> 的 bwl（M1 左輪）
int rightSpeed = 0;   // bwr（M2 右輪）
int lastRxSeq = -1;
uint32_t cmdChecksumFailCount = 0;
uint32_t lastCmdMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastPidMs = 0;
uint8_t usbDbgDiv = 0;
uint32_t staSkipBM = 0;

int manualL = 0, manualR = 0;
uint32_t manualUntilMs = 0;  // 0 = 沒有手動驅動

ChunkedSender bmSender = { {0}, 0, 0, 0, false };
ChunkedSender usbSender = { {0}, 0, 0, 0, false };
LinkState linkBM = { &SerialBM, {0}, 0, false };
char usbBuf[80];
uint8_t usbLen = 0;
bool usbFrameActive = false;

// ================= 非阻塞分段傳送 =================
// 這片 HT32 核心的 Serial1/Serial3 單次 write() 送較長訊息有機率整包損毀（見 docs/architecture/uart_tx_corruption_bug.md），
// 所以每次 loop() 最多送 5 byte、間隔 1 ms，從不 delay。
void startSend(ChunkedSender& cs, const char* s) {
  size_t n = strlen(s);
  if (n >= sizeof(cs.buf)) n = sizeof(cs.buf) - 1;
  memcpy(cs.buf, s, n);
  cs.buf[n] = '\0';
  cs.len = (uint8_t)n;
  cs.pos = 0;
  cs.nextSendMs = millis();
  cs.active = true;
}

void pumpSend(ChunkedSender& cs, Stream& port) {
  if (!cs.active) return;
  if ((int32_t)(millis() - cs.nextSendMs) < 0) return;
  uint8_t remain = cs.len - cs.pos;
  uint8_t n = remain < 5 ? remain : 5;
  port.write((const uint8_t*)(cs.buf + cs.pos), n);
  cs.pos += n;
  cs.nextSendMs = millis() + 1;
  if (cs.pos >= cs.len) cs.active = false;
}

// ================= checksum（XOR8，範圍 = frame 內容除了 ,cs 欄位）=================
uint8_t xorChecksum(const char* s, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= (uint8_t)s[i];
  return cs;
}

bool verifyAndStripChecksum(char* frame) {
  char* csPos = strstr(frame, ",cs ");
  if (csPos == nullptr) return false;
  size_t payloadLen = (size_t)(csPos - frame);
  uint8_t expected = (uint8_t)strtoul(csPos + 4, nullptr, 16);
  if (xorChecksum(frame, payloadLen) != expected) return false;
  frame[payloadLen] = '\0';
  return true;
}

// ================= PCA9685 =================
// 回傳 I2C 是否成功；失敗時 pcaReady=false，交給 pcaRecoverTick() 重試。
bool pcaWritePinChecked(uint8_t ch, bool high) {
  uint8_t result = high ? pwm.setPWM(ch, 4096, 0) : pwm.setPWM(ch, 0, 4096);
  dirI2cCallCount++;
  if (result != 0) {
    dirI2cErrorCount++;
    pcaReady = false;
    return false;
  }
  return true;
}

bool pcaProbe() {
  Wire1.beginTransmission(PCA9685_ADDR);
  Wire1.write((uint8_t)0x00);
  return Wire1.endTransmission() == 0;
}

// 致能腳：兩個都拉高 = 橋可以動；兩個都拉低 = 滑行。寫入失敗不更新快取，下個週期重試。
bool setEnable(Motor& m, bool on) {
  if (m.enabled == on) return true;
  bool a = pcaWritePinChecked(m.chEnR, on);
  bool b = pcaWritePinChecked(m.chEnL, on);
  if (!(a && b)) return false;
  m.enabled = on;
  return true;
}

bool pcaInit() {
  if (!pcaProbe()) return false;
  pwm.begin();
  pwm.setPWMFreq(50);
  uint16_t prescale = pwm.readPrescale();
  steerPulselengthUs = 1000000.0;
  steerPulselengthUs *= (double)(prescale + 1);
  steerPulselengthUs /= (double)pwm.getOscillatorFrequency();
  // 致能全部拉低（滑行）並讓快取與實際輸出一致。
  m1.enabled = true;  // 強制下面真的寫一次
  m2.enabled = true;
  bool a = setEnable(m1, false);
  bool b = setEnable(m2, false);
  if (!(a && b)) return false;
  lastSentSteerAngle = -1000;
  return true;
}

void updateSteering() {
  if (!pcaReady) return;
  if (steerAngle == lastSentSteerAngle) return;
  lastSentSteerAngle = steerAngle;
  uint16_t steerUs = map(steerAngle, -90, 90, STEER_MIN_US, STEER_MAX_US);
  double pulse = (double)steerUs / steerPulselengthUs;
  uint8_t r = pwm.setPWM(CH_STEER, 0, (uint16_t)pulse);
  if (r != 0) pcaReady = false;
}

// 只在「不健康」時才每 PCA_RETRY_PERIOD_MS 重試初始化；健康時完全不碰 I2C。
void pcaRecoverTick(uint32_t now) {
  if (pcaReady) return;
  if ((int32_t)(now - pcaNextCheckMs) < 0) return;
  pcaNextCheckMs = now + PCA_RETRY_PERIOD_MS;
  if (pcaInit()) {
    pcaReady = true;
    updateSteering();
  }
}

// ================= 馬達輸出 =================
void writePwm(Motor& m, int8_t dir, uint8_t duty) {
  if (dir > 0) {
    analogWrite(m.pinRev, 0);
    analogWrite(m.pinFwd, duty);
  } else if (dir < 0) {
    analogWrite(m.pinFwd, 0);
    analogWrite(m.pinRev, duty);
  } else {
    analogWrite(m.pinFwd, 0);
    analogWrite(m.pinRev, 0);
  }
}

// 把 m.dir / m.duty 寫到硬體。順序：要停先關 PWM 再關致能；要動先開致能再給 PWM。
// 致能寫不成功（或 PCA9685 掉線）時絕不輸出 PWM。
void commit(Motor& m) {
  if (m.dir == 0) {
    m.duty = 0;
    writePwm(m, 0, 0);
    setEnable(m, false);
    return;
  }
  if (!pcaReady || !setEnable(m, true)) {
    m.duty = 0;
    m.dir = 0;
    writePwm(m, 0, 0);
    return;
  }
  writePwm(m, m.dir, m.duty);
}

void stopMotor(Motor& m) {
  m.dir = 0;
  m.duty = 0;
  commit(m);
}

// 每個控制週期呼叫一次。cmdPct：-100~100。
void updateMotor(Motor& m, int cmdPct) {
  uint32_t now = millis();
  int32_t cur = *m.pos;
  int32_t delta = cur - m.lastPos;
  m.lastPos = cur;
  m.signedSpeed = SIGNED_SPEED_ALPHA * (delta * m.encSign) + (1.0f - SIGNED_SPEED_ALPHA) * m.signedSpeed;

  // 堵轉鎖定：鎖定中一律停；指令回到 0 連續 FAULT_CLEAR_MS 才解除。
  // 解除後致能會重新拉高，順便重置 BTS7960 的過溫鎖存（INH 拉低 >= 3 µs）。
  if (m.fault) {
    if (cmdPct == 0) {
      if (!m.faultClearing) {
        m.faultClearing = true;
        m.faultClearSinceMs = now;
      }
      if ((int32_t)(now - m.faultClearSinceMs) >= (int32_t)FAULT_CLEAR_MS) {
        m.fault = false;
        m.faultClearing = false;
        m.noMotionMs = 0;
      }
    } else {
      m.faultClearing = false;
    }
    if (m.fault) {
      stopMotor(m);
      return;
    }
  }

  if (cmdPct == 0) {
    if (m.dir != 0) m.zeroSinceMs = now;
    m.noMotionMs = 0;
    stopMotor(m);
    return;
  }

  int8_t dir = (cmdPct > 0) ? 1 : -1;

  // 反轉保護：不管上層是直接翻轉還是先 0 再反向，只要和上一次驅動方向相反，
  // 就要先量到已經停止(致能拉低、滑行)滿 REVERSE_BRAKE_MS 才放行。
  if (m.lastNonZeroDir != 0 && m.lastNonZeroDir != dir) {
    if (m.dir != 0) {
      m.zeroSinceMs = now;
      m.noMotionMs = 0;
      stopMotor(m);
      return;
    }
    if ((int32_t)(now - m.zeroSinceMs) < (int32_t)REVERSE_BRAKE_MS) {
      m.noMotionMs = 0;
      stopMotor(m);
      return;
    }
  }
  m.lastNonZeroDir = dir;

  // 開迴路：百分比直接換 duty，再用斜率限制往目標靠。
  int absPct = abs(cmdPct);
  if (absPct > 100) absPct = 100;
  uint8_t target = (uint8_t)(((uint32_t)absPct * OPEN_LOOP_MAX_DUTY + 50) / 100);
  if (m.dir != dir) m.duty = 0;  // 從停止或換方向開始，從 0 往上爬
  if (m.duty < target) {
    m.duty = (target - m.duty > DUTY_STEP_PER_TICK) ? (uint8_t)(m.duty + DUTY_STEP_PER_TICK) : target;
  } else if (m.duty > target) {
    m.duty = (m.duty - target > DUTY_STEP_PER_TICK) ? (uint8_t)(m.duty - DUTY_STEP_PER_TICK) : target;
  }
  m.dir = dir;

  // 堵轉/編碼器失效切斷：有在驅動(duty 夠大)卻連續 STALL_CUTOFF_MS 一個 tick 都沒動。
  // BTS7960 的限流(43~60 A)遠高於馬達堵轉電流，驅動板不會替馬達、電線、電源保護，這條是主要的保命線。
  if (m.duty >= STALL_MIN_DUTY && delta == 0) {
    if (m.noMotionMs < 60000) m.noMotionMs += PID_INTERVAL_MS;
  } else {
    m.noMotionMs = 0;
  }
  if (m.noMotionMs >= STALL_CUTOFF_MS) {
    m.fault = true;
    m.faultClearing = false;
    m.zeroSinceMs = now;
    stopMotor(m);
    return;
  }

  commit(m);
}

// ================= 指令處理 =================
void applyCommand(const char* key, int value) {
  if (strcmp(key, "seq") == 0) {
    lastRxSeq = value;
  } else if (strcmp(key, "fwa") == 0) {
    steerAngle = constrain(value, -60, 60);
  } else if (strcmp(key, "bwl") == 0) {
    leftSpeed = constrain(value, -100, 100);
  } else if (strcmp(key, "bwr") == 0) {
    rightSpeed = constrain(value, -100, 100);
  }
}

void handleFrame(char* frame) {
  if (!verifyAndStripChecksum(frame)) {
    cmdChecksumFailCount++;
    return;  // checksum 不對整包丟棄，不更新 lastCmdMs，讓逾時保護介入
  }
  char* tok = strtok(frame, ",");
  if (tok == nullptr || strcmp(tok, "CMD") != 0) return;
  while ((tok = strtok(nullptr, ",")) != nullptr) {
    char key[8];
    int value;
    if (sscanf(tok, "%7s %d", key, &value) == 2) applyCommand(key, value);
  }
  lastCmdMs = millis();
  updateSteering();
}

void pollBM() {
  LinkState& ls = linkBM;
  while (ls.port->available()) {
    char c = ls.port->read();
    if (c == '<') {
      ls.len = 0;
      ls.active = true;
      continue;
    }
    if (!ls.active) continue;
    if (c == '>') {
      ls.buf[ls.len] = '\0';
      handleFrame(ls.buf);
      ls.active = false;
      ls.len = 0;
      continue;
    }
    if (ls.len < sizeof(ls.buf) - 1) {
      ls.buf[ls.len++] = c;
    } else {
      ls.active = false;
      ls.len = 0;
    }
  }
}

void printUsbHelp() {
  Serial.println(F("m L R : 手動驅動 L/R=-100~100(%) 持續2秒    s : 停止    z : 編碼器歸零    h : 說明"));
  Serial.println(F("dbg 欄位: cmdL,cmdR,dutyL,dutyR,dirL,dirR,posL,posR(前進為正),illL,illR,vL,vR(rpm),f,pca,enL,enR"));
}

void handleTextCommand(char* s) {
  int a, b;
  if (s[0] == 'm' && sscanf(s + 1, "%d %d", &a, &b) == 2) {
    manualL = constrain(a, -100, 100);
    manualR = constrain(b, -100, 100);
    manualUntilMs = millis() + MANUAL_HOLD_MS;
    if (manualUntilMs == 0) manualUntilMs = 1;
    Serial.print(F("手動驅動 L="));
    Serial.print(manualL);
    Serial.print(F(" R="));
    Serial.print(manualR);
    Serial.println(F("（2 秒後自動停）"));
  } else if (s[0] == 's') {
    manualUntilMs = 0;
    manualL = manualR = 0;
    leftSpeed = rightSpeed = 0;
    Serial.println(F("停止"));
  } else if (s[0] == 'z') {
    noInterrupts();
    posM1 = 0;
    posM2 = 0;
    illegalM1 = 0;
    illegalM2 = 0;
    interrupts();
    m1.lastPos = 0;
    m2.lastPos = 0;
    Serial.println(F("編碼器已歸零"));
  } else if (s[0] == 'h' || s[0] == '?') {
    printUsbHelp();
  }
}

// USB：'<' 開頭是 <CMD,...> 框，其他是一行文字指令。
void pollUsb() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') {
      usbLen = 0;
      usbFrameActive = true;
      continue;
    }
    if (usbFrameActive) {
      if (c == '>') {
        usbBuf[usbLen] = '\0';
        handleFrame(usbBuf);
        usbFrameActive = false;
        usbLen = 0;
      } else if (usbLen < sizeof(usbBuf) - 1) {
        usbBuf[usbLen++] = c;
      } else {
        usbFrameActive = false;
        usbLen = 0;
      }
      continue;
    }
    if (c == '\r') continue;
    if (c == '\n') {
      usbBuf[usbLen] = '\0';
      if (usbLen > 0) handleTextCommand(usbBuf);
      usbLen = 0;
      continue;
    }
    if (usbLen < sizeof(usbBuf) - 1) usbBuf[usbLen++] = c;
    else usbLen = 0;
  }
}

// ================= 狀態回報 =================
float ticksToRPM(float ticksPerInterval) {
  return (ticksPerInterval / TICKS_PER_REV) * (60000.0f / PID_INTERVAL_MS);
}

void sendStatus(int cmdL, int cmdR) {
  // rpmL/rpmR：沿目標方向為正（指令 0 時為 0）；vL/vR：常駐速度，前進為正，手轉/滑行/被推都看得到。
  float vM1 = ticksToRPM(m1.signedSpeed);
  float vM2 = ticksToRPM(m2.signedSpeed);
  float rpmM1 = (cmdL > 0) ? vM1 : (cmdL < 0 ? -vM1 : 0.0f);
  float rpmM2 = (cmdR > 0) ? vM2 : (cmdR < 0 ? -vM2 : 0.0f);
  int vL = (int)(vM1 + (vM1 >= 0 ? 0.5f : -0.5f));
  int vR = (int)(vM2 + (vM2 >= 0 ? 0.5f : -0.5f));
  int faultFlags = (m1.fault ? 1 : 0) | (m2.fault ? 2 : 0) | (pcaReady ? 0 : 4);

  char payload[96];
  snprintf(payload, sizeof(payload), "STA,ackseq %d,S1 -1,rpmL %.1f,rpmR %.1f,vL %d,vR %d,f %d",
           lastRxSeq, rpmM1, rpmM2, vL, vR, faultFlags);
  uint8_t cs = xorChecksum(payload, strlen(payload));
  char line[112];
  snprintf(line, sizeof(line), "<%s,cs %02X>\n", payload, cs);

  if (bmSender.active) {
    staSkipBM++;  // 上一則還沒送完就略過這次，不要覆蓋送到一半的緩衝
  } else {
    startSend(bmSender, line);
  }

  if (++usbDbgDiv < USB_DBG_DIV) return;
  usbDbgDiv = 0;
  if (usbSender.active) return;
  char dbg[200];
  snprintf(dbg, sizeof(dbg), "dbg,%d,%d,%u,%u,%d,%d,%ld,%ld,%lu,%lu,%d,%d,%d,%d,%d,%d\n",
           cmdL, cmdR, m1.duty, m2.duty, m1.dir, m2.dir,
           (long)(posM1 * ENC_SIGN_M1), (long)(posM2 * ENC_SIGN_M2),
           (unsigned long)illegalM1, (unsigned long)illegalM2,
           vL, vR, faultFlags, pcaReady ? 1 : 0, m1.enabled ? 1 : 0, m2.enabled ? 1 : 0);
  startSend(usbSender, dbg);
}

// ================= setup / loop =================
void setup() {
  Serial.begin(115200);
  SerialBM.begin(460800);

  // 四條 PWM 腳先拉低，再設頻率、寫 duty 0。頻率必須在第一次 analogWrite 之前設。
  const uint8_t pwmPins[] = {M1_RPWM, M1_LPWM, M2_RPWM, M2_LPWM};
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW);
  }
  analogWriteFrequency(PWM_FREQ_HZ);
  writePwm(m1, 0, 0);
  writePwm(m2, 0, 0);

  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  stateM1 = (digitalRead(PIN_M1_A) << 1) | digitalRead(PIN_M1_B);
  stateM2 = (digitalRead(PIN_M2_A) << 1) | digitalRead(PIN_M2_B);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  pcaReady = pcaInit();  // 沒回應不卡死；loop() 會每秒重試。PCA9685 沒好時馬達一律不輸出。

  lastCmdMs = millis();
  lastPidMs = millis();
  if (pcaReady) updateSteering();

  Serial.println(F("=== car2 BTS7960 基礎版（開迴路，未實測）==="));
  Serial.print(F("PWM "));
  Serial.print(PWM_FREQ_HZ);
  Serial.print(F(" Hz，100% 指令 = duty "));
  Serial.print(OPEN_LOOP_MAX_DUTY);
  Serial.print(F("/255，PCA9685 "));
  Serial.println(pcaReady ? F("OK") : F("沒回應（馬達不會輸出）"));
  printUsbHelp();
}

void loop() {
  pumpSend(bmSender, SerialBM);
  pumpSend(usbSender, Serial);
  pollBM();
  pollUsb();

  uint32_t now = millis();
  pcaRecoverTick(now);

  int cmdL, cmdR;
  if (manualUntilMs != 0) {
    if ((int32_t)(now - manualUntilMs) >= 0) {
      manualUntilMs = 0;
      manualL = manualR = 0;
      cmdL = cmdR = 0;
    } else {
      cmdL = manualL;
      cmdR = manualR;
    }
  } else {
    if (now - lastCmdMs > CMD_TIMEOUT_MS) {
      leftSpeed = 0;
      rightSpeed = 0;
    }
    cmdL = leftSpeed;
    cmdR = rightSpeed;
  }

  if (now - lastPidMs >= PID_INTERVAL_MS) {
    lastPidMs = now;
    updateMotor(m1, cmdL);
    updateMotor(m2, cmdR);
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus(cmdL, cmdR);
  }
}
