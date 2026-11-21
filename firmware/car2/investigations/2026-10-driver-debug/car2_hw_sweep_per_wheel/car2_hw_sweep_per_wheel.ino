// 排查用（2026-10-02）：逐輪開迴路 duty 掃描，等 USB 指令才啟動，一次只動一顆馬達。
//
// 背景：連續換了好幾片驅動板，閉迴路韌體對「兩輪都沒反應」的情況只能在 600ms 堵轉切斷
// 之前把 duty 推到約 40%；如果新驅動板需要更高的 duty 才開始導通(先前右輪曾延遲約 0.9 秒才
// 暴衝)，閉迴路根本看不到那個門檻。這份直接開迴路從 0 掃到指定上限，量出每個輪子「從哪個
// duty 開始動」以及 duty->rpm 曲線。
//
// 跟 2026-09 的 car2_openloop_duty_sweep_5khz_full.ino 的差別：
//   * 開機什麼都不輸出(D8/D9 拉低、方向腳懸空)，不會一上電就自己跑；
//   * 一次只掃一輪(另一輪的 PWM 腳固定 LOW、方向腳懸空)；
//   * 上限百分比可指定，任何時候收到 'x' 立刻停；
//   * 方向腳改用有回傳值的 setPWM()，I2C 失敗就拒絕/中止；
//   * 保留「轉過之後更高 duty 卻完全不動就中止」的保護，另外加總時間上限。
//
// USB 指令(115200，結尾換行)：
//   L[pct] / R[pct]   左輪(M1) / 右輪(M2) 正轉掃描，pct=上限百分比 1~100，省略=100
//   l[pct] / r[pct]   反轉掃描
//   H<L|R|l|r>[pct]  定值保持(例如 HL50=左輪正轉固定50%，Hr30=右輪反轉固定30%)，
//                     供電表/示波器在「有輸出」的狀態下量測；最多 MAX_HOLD_MS，x 立刻停
//                     (2026-10-05) HB[pct]=兩顆同時正轉、Hb[pct]=兩顆同時反轉，溫升/長時間負載測試用
//   A                 IN1~IN4 全部拉高(待機=懸空)、PWM(D8/D9)拉低，並回報 4 次 I2C 寫入結果
//   S                 掃描 Wire1(PCA9685 那條 I2C)上有回應的位址
//   W                 逐條走訪：PCA 通道 CH1/CH2/CH3/CH4 依序單獨拉高各 2.5 秒(PWM 全程拉低)，
//                     用來確認每一條 IN 的訊號都真的到驅動板
//   D<L|R>            PWM 腳慢速閃爍(D8=左/D9=右，高 1.5 秒/低 1.5 秒，共 3 次)，IN 全程拉高
//   K                 煞車態：IN1~IN4 全部拉低(XY-160D：兩腳都低=煞車)、PWM 拉低，保持到 x
//   P<L|R>[pct]       煞車態 + 只控 PWM：IN1~IN4 全低，只有 D8(左)/D9(右) 輸出 pct%(0~100，省略=100)，
//                     保持最多 MAX_HOLD_MS，x 立刻停。煞車時馬達端子短路，不會被驅動
//   PS<L|R>           煞車態 + PWM 階梯：0,10,...,100%，每階 2 秒(共約 22 秒)，IN 全程全低
//   PB                煞車態 + 兩邊 PWM(D8 和 D9)同時循環 0→100%→0%(每階 1.5 秒)，一直跑到 x
//                     (最長 MAX_CONTINUOUS_MS)，方便同時量 ENA/ENB 的變化
//   E / EB            D8 和 D9 同時恆高(100%，直流)，保持到 x：E=IN1~IN4 全高(懸空)、
//                     EB=IN1~IN4 全低(煞車)。兩種都不會驅動馬達，方便同時量 D8/D9/ENA/ENB
//   F[pct]            兩顆馬達「同時」正轉、duty 由慢到快 0→pct%(省略=100)，每階約 1.2 秒，同時印兩邊
//                     編碼器；某一輪轉過之後又不動就中止。輪子請架空
//   V                 兩顆「同時」正轉，先衝到 duty 200 轉順，再由快到慢往下掃(每階 1.2 秒，
//                     120 以上每階 -10、以下每階 -5)，量「轉起來之後最低能維持到多少 duty」
//   T<duty>           階躍響應(從靜止)：兩顆同時正轉，0 -> duty(0~255)，以 20 ms 取樣記錄 3.2 秒的編碼器位置
//   U<d1>,<d2>        階躍響應(已轉動的兩個 duty 之間)：先衝到 200 轉順、穩定在 d1 約 1.5 秒，再階躍到 d2 並記錄
//                     取樣先存在 RAM，結束停輪後才一次印出：st,<i>,<ms>,<posL 前進為正>,<posR 前進為正>
//   x                 立刻停止、輸出歸零
// 輸出行：
//   sw,<L|R>,<dir f|r>,<duty 0~255>,<pct>,<moved ticks>,<rpm 前進為正>
//   done,<L|R>,first_move_duty=<n|none>,max_rpm=<x>   /   abort,<原因>   /   err,<原因>
//
// 注意：開迴路沒有電流保護。如果「驅動板有導通但輪子被卡住」，高 duty 會是堵轉電流——
// 請輪子架空、電源供應器設好電流限制，人在旁邊。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 非阻塞分段傳送（沿用正式版/掃描版已驗證過的做法：每次 loop 最多送 5 byte）
struct ChunkedSender {
  char buf[96];
  uint8_t len;
  uint8_t pos;
  uint32_t nextSendMs;
  bool active;
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);
const uint8_t PCA9685_ADDR = 0x40;

// 2026-10-02 夜：換成 L298N 後，馬達電源線在驅動板上跟編碼器的對應是交叉的(左編碼器那顆接在
// D9/CH4/CH3 那組、右編碼器那顆接在 D8/CH1/CH2 那組)，線已經沒有餘裕再換，改成在韌體對調：
//   M1(左，編碼器 D4/D5)  -> D9 + CH4(高)/CH3(低)=正轉   (實測這組正轉時 posM1 為正，符合慣例)
//   M2(右，編碼器 D10/D11) -> D8 + CH1(高)/CH2(低)=正轉   (CH2 高/CH1 低時 posM2 為正=反向，所以順序反過來)
// 診斷輸出裡寫到的「D8/D9」「CH 順序」仍是實體腳位，不是邏輯輪子。
// (換線後再對調一次 M1 的順序：同樣的 D9+CH4高/CH3低 讓 posM1 往負走，所以 M1 改成 CH3 高/CH4 低 = 正轉)
const uint8_t CH_M1_IN1 = 3;
const uint8_t CH_M1_IN2 = 4;
const uint8_t CH_M2_IN1 = 1;
const uint8_t CH_M2_IN2 = 2;

const uint8_t M1_PWM_PIN = 9;
const uint8_t M2_PWM_PIN = 8;

const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const uint32_t PWM_PERIOD_US = 200;       // 5kHz，跟 car2 正式版一致
const uint32_t HOLD_MS = 600;             // 每個 duty 停留多久
const uint32_t SETTLE_MS = 250;           // 前面這段不算進轉速，只讓它穩定
const uint32_t PREPARE_MS = 400;          // 方向腳設好之後、開始掃描之前的等待
const uint32_t MAX_SWEEP_TOTAL_MS = 60000;  // 單次掃描總時間硬上限
const uint32_t MAX_HOLD_MS = 600000;        // 定值保持的時間硬上限(10 分鐘；x 隨時停)
const uint32_t MAX_CONTINUOUS_MS = 120000;  // PB 連續循環的時間硬上限
const uint8_t DUTY_STEP = 5;  // 2026-10-05：10 -> 5，起轉/維持門檻要 5 個 duty 的解析度(上限用 L/R[pct] 的 pct 控制掃描時間)
const int32_t MOVE_NOISE_FLOOR = 20;      // 一個量測窗口內少於這個 ticks 視為雜訊
const float TICKS_PER_REV = 1288.86f;     // 車2實測修正值

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

enum SweepState { ST_IDLE, ST_PREPARE, ST_SWEEP, ST_HOLD };
SweepState sweepState = ST_IDLE;
int activeWheel = 0;       // 1=左輪(M1)，2=右輪(M2)
int dirSign = 1;           // +1 正轉，-1 反轉
uint8_t dutyCap = 255;     // 這次掃描的上限(0~255)
uint8_t curDuty = 0;
uint32_t sweepStartMs = 0;
uint32_t stepStartMs = 0;
int32_t stepStartPos = 0;
int32_t settlePos = 0;
bool settleCaptured = false;
bool everMoved = false;
int firstMoveDuty = -1;
float maxRpm = 0.0f;
bool pcaReady = false;
uint32_t lastIdleBeatMs = 0;
bool forceM1High = false;   // 診斷用：強制 D8 / D9 輸出高電位(只有 D 指令會用)
bool forceM2High = false;
bool diagBusy = false;      // 診斷指令(A/S/W/D)執行中
bool abortRequested = false;

ChunkedSender usbSender = { {0}, 0, 0, 0, false };

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

// 佇列還在送前一行時，新的一行直接跳過、不阻塞。
void queueLine(const char* s) {
  if (usbSender.active) return;
  startSend(usbSender, s);
}

// ---- PCA9685 方向腳(有回傳值版本) ----
bool pcaPin(uint8_t ch, bool high) {
  uint8_t r = high ? pwm.setPWM(ch, 4096, 0) : pwm.setPWM(ch, 0, 4096);
  return r == 0;
}

// dir: +1 正轉(IN1=H,IN2=L)，-1 反轉(IN1=L,IN2=H)，0 懸空(兩腳都 H，XY-160D 的自由停止)
bool setDirection(uint8_t in1, uint8_t in2, int dir) {
  bool a, b;
  if (dir == 9) {  // 煞車：兩腳都拉低(XY-160D 真值表)
    a = pcaPin(in1, false);
    b = pcaPin(in2, false);
    return a && b;
  }
  if (dir > 0) {
    a = pcaPin(in1, true);
    b = pcaPin(in2, false);
  } else if (dir < 0) {
    a = pcaPin(in1, false);
    b = pcaPin(in2, true);
  } else {
    a = pcaPin(in1, true);
    b = pcaPin(in2, true);
  }
  return a && b;
}

// 不用零長度傳輸探測(這個核心的 Wire 對零長度疑似會卡死)，改寫一個位元組的暫存器指標。
bool pcaProbe() {
  Wire1.beginTransmission(PCA9685_ADDR);
  Wire1.write((uint8_t)0x00);
  return Wire1.endTransmission() == 0;
}

bool pcaInit() {
  if (!pcaProbe()) return false;
  pwm.begin();
  pwm.setPWMFreq(50);
  bool a = setDirection(CH_M1_IN1, CH_M1_IN2, 0);
  bool b = setDirection(CH_M2_IN1, CH_M2_IN2, 0);
  return a && b;
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)curDuty * PWM_PERIOD_US / 255;
  bool on = (sweepState == ST_SWEEP || sweepState == ST_HOLD) && (phase < onUs);
  // activeWheel: 1=只有 D8，2=只有 D9，3=兩邊同時(PB 模式)
  digitalWrite(M1_PWM_PIN, ((on && (activeWheel == 1 || activeWheel == 3)) || forceM1High) ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, ((on && (activeWheel == 2 || activeWheel == 3)) || forceM2High) ? HIGH : LOW);
}

// 立刻停：duty 歸零、PWM 腳拉低、兩輪方向腳都回到懸空。
void stopAll() {
  curDuty = 0;
  sweepState = ST_IDLE;
  forceM1High = false;
  forceM2High = false;
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  setDirection(CH_M1_IN1, CH_M1_IN2, 0);
  setDirection(CH_M2_IN1, CH_M2_IN2, 0);
}

// 掃描已經結束/中止，馬達不再運轉，這裡用阻塞的 println 沒有風險。
void finishWith(const char* kind, const char* reason) {
  stopAll();
  char line[96];
  if (strcmp(kind, "done") == 0) {
    char first[12];
    if (firstMoveDuty < 0) strcpy(first, "none");
    else snprintf(first, sizeof(first), "%d", firstMoveDuty);
    snprintf(line, sizeof(line), "done,%c,first_move_duty=%s,max_rpm=%.1f",
             activeWheel == 1 ? 'L' : 'R', first, maxRpm);
  } else {
    snprintf(line, sizeof(line), "%s,%s", kind, reason);
  }
  while (usbSender.active) pumpSend(usbSender, Serial);
  Serial.println(line);
}

int32_t activePos() {
  return (activeWheel == 1) ? posM1 : -posM2;  // 前進為正(M2 編碼器方向相反)
}

void beginSweep(int wheel, int dir, int pct) {
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return;
  }
  activeWheel = wheel;
  dirSign = dir;
  dutyCap = (uint8_t)((pct * 255 + 50) / 100);
  curDuty = 0;
  everMoved = false;
  firstMoveDuty = -1;
  maxRpm = 0.0f;
  // 另一輪維持懸空、這一輪設方向；任何一條方向腳寫入失敗就不開始。
  bool ok = setDirection(CH_M1_IN1, CH_M1_IN2, wheel == 1 ? dir : 0);
  ok = setDirection(CH_M2_IN1, CH_M2_IN2, wheel == 2 ? dir : 0) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("err,direction pin I2C write failed"));
    return;
  }
  char line[64];
  snprintf(line, sizeof(line), "start,%c,%c,cap_duty=%u,cap_pct=%d",
           wheel == 1 ? 'L' : 'R', dir > 0 ? 'f' : 'r', dutyCap, pct);
  Serial.println(line);
  sweepStartMs = millis();
  stepStartMs = sweepStartMs;
  sweepState = ST_PREPARE;
}

// 定值保持：設好方向後直接把 duty 固定在指定百分比，直到 x 或 MAX_HOLD_MS。
void beginHold(int wheel, int dir, int pct) {
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return;
  }
  bool ok = setDirection(CH_M1_IN1, CH_M1_IN2, (wheel == 1 || wheel == 3) ? dir : 0);
  ok = setDirection(CH_M2_IN1, CH_M2_IN2, (wheel == 2 || wheel == 3) ? dir : 0) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("err,direction pin I2C write failed"));
    return;
  }
  activeWheel = wheel;
  dirSign = dir;
  curDuty = (uint8_t)((pct * 255 + 50) / 100);
  char line[64];
  snprintf(line, sizeof(line), "hold,%c,%c,duty=%u,pct=%d", wheel == 1 ? 'L' : (wheel == 2 ? 'R' : 'B'),
           dir > 0 ? 'f' : 'r', curDuty, pct);
  Serial.println(line);
  sweepStartMs = millis();
  lastIdleBeatMs = sweepStartMs;
  sweepState = ST_HOLD;
}

// ---- 診斷指令(A/S/W/D)：全部在 PWM 為低、馬達不會被驅動的條件下執行 ----
// 可中斷等待：等待期間照常送資料、收指令，收到 x 就回報中止。
bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    pumpSend(usbSender, Serial);
    updateMotorPWM();
    pollSerial();
    if (abortRequested) return false;
  }
  return true;
}

// IN1~IN4 全部拉高(兩顆馬達的待機/懸空態)、PWM 拉低，回報 4 條 I2C 寫入結果。
bool assertIdleHigh() {
  forceM1High = false;
  forceM2High = false;
  curDuty = 0;
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return false;
  }
  bool r1 = pcaPin(CH_M1_IN1, true);
  bool r2 = pcaPin(CH_M1_IN2, true);
  bool r3 = pcaPin(CH_M2_IN1, true);
  bool r4 = pcaPin(CH_M2_IN2, true);
  char line[96];
  snprintf(line, sizeof(line), "a,PWM D8/D9=LOW,PCA CH2/CH1/CH4/CH3=HIGH,i2c_ok=%d%d%d%d",
           r1, r2, r3, r4);
  Serial.println(line);
  if (!(r1 && r2 && r3 && r4)) pcaReady = false;
  return r1 && r2 && r3 && r4;
}

void diagScan() {
  Serial.println(F("scan,start (Wire1, addr 0x01~0x7E)"));
  int found = 0;
  char line[48];
  for (uint8_t a = 1; a < 127; a++) {
    Wire1.beginTransmission(a);
    uint8_t err = Wire1.endTransmission();
    if (err == 0) {
      snprintf(line, sizeof(line), "found,0x%02X", a);
      Serial.println(line);
      found++;
    }
    pollSerial();
    if (abortRequested) break;
  }
  snprintf(line, sizeof(line), "scan,done,found=%d", found);
  Serial.println(line);
}

void diagWalk() {
  const uint8_t chs[4] = { 1, 2, 3, 4 };
  for (uint8_t i = 0; i < 4; i++) {
    bool ok = true;
    for (uint8_t j = 0; j < 4; j++) ok = pcaPin(chs[j], chs[j] == chs[i]) && ok;
    char line[64];
    snprintf(line, sizeof(line), "walk,PCA CH%u=HIGH (others LOW),i2c_ok=%d", chs[i], ok ? 1 : 0);
    Serial.println(line);
    if (!ok) { pcaReady = false; break; }
    if (!waitMs(2500)) break;
  }
  assertIdleHigh();
}

void diagPwmBlink(int wheel) {
  if (!assertIdleHigh()) return;
  for (int i = 0; i < 3; i++) {
    if (wheel == 1) forceM1High = true; else forceM2High = true;
    Serial.println(wheel == 1 ? F("pwm,D8=HIGH (IN1~IN4 HIGH)") : F("pwm,D9=HIGH (IN1~IN4 HIGH)"));
    if (!waitMs(1500)) break;
    forceM1High = false;
    forceM2High = false;
    Serial.println(F("pwm,LOW"));
    if (!waitMs(1500)) break;
  }
  forceM1High = false;
  forceM2High = false;
}

// IN1~IN4 全部拉低(兩顆馬達都煞車)、PWM 拉低，保持到下一個指令。
bool assertBrakeLow() {
  forceM1High = false;
  forceM2High = false;
  curDuty = 0;
  sweepState = ST_IDLE;
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return false;
  }
  bool r1 = pcaPin(CH_M1_IN1, false);
  bool r2 = pcaPin(CH_M1_IN2, false);
  bool r3 = pcaPin(CH_M2_IN1, false);
  bool r4 = pcaPin(CH_M2_IN2, false);
  char line[96];
  snprintf(line, sizeof(line), "k,PWM D8/D9=LOW,PCA CH2/CH1/CH4/CH3=LOW (brake),i2c_ok=%d%d%d%d",
           r1, r2, r3, r4);
  Serial.println(line);
  if (!(r1 && r2 && r3 && r4)) pcaReady = false;
  return r1 && r2 && r3 && r4;
}

// 煞車態 + 單輪 PWM 定值保持(最多 MAX_HOLD_MS，x 立刻停)。
void beginBrakePwm(int wheel, int pct) {
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return;
  }
  bool ok = setDirection(CH_M1_IN1, CH_M1_IN2, 9);
  ok = setDirection(CH_M2_IN1, CH_M2_IN2, 9) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("err,direction pin I2C write failed"));
    return;
  }
  activeWheel = wheel;
  dirSign = 1;
  curDuty = (uint8_t)((pct * 255 + 50) / 100);
  char line[80];
  snprintf(line, sizeof(line), "pwmhold,%c,duty=%u,pct=%d,IN1~IN4=LOW(brake)", wheel == 1 ? 'L' : 'R',
           curDuty, pct);
  Serial.println(line);
  sweepStartMs = millis();
  lastIdleBeatMs = sweepStartMs;
  sweepState = ST_HOLD;
}

// 煞車態 + PWM 階梯 0,10,...,100%，每階 2 秒；結束或 x 之後回到待機(IN 全高=懸空)。
void diagPwmRampBrake(int wheel) {
  if (!assertBrakeLow()) return;
  activeWheel = wheel;
  dirSign = 1;
  sweepState = ST_HOLD;  // 只是為了讓 updateMotorPWM() 輸出 PWM；等待期間由 waitMs() 驅動
  for (int pct = 0; pct <= 100; pct += 10) {
    curDuty = (uint8_t)((pct * 255 + 50) / 100);
    char line[64];
    snprintf(line, sizeof(line), "ramp,%c,pct=%d,duty=%u", wheel == 1 ? 'L' : 'R', pct, curDuty);
    Serial.println(line);
    if (!waitMs(2000)) break;
  }
  stopAll();
}

// 煞車態 + D8/D9 同時循環 0→100→0%，每階 1.5 秒，直到 x 或 MAX_CONTINUOUS_MS。
void diagPwmRampBothBrake() {
  if (!assertBrakeLow()) return;
  activeWheel = 3;
  dirSign = 1;
  sweepState = ST_HOLD;  // 讓 updateMotorPWM() 輸出 PWM；等待期間由 waitMs() 驅動
  Serial.println(F("pb,D8+D9 PWM ramp 0-100-0 loop, IN1~IN4=LOW(brake); x to stop"));
  uint32_t t0 = millis();
  bool up = true;
  int pct = 0;
  while (millis() - t0 < MAX_CONTINUOUS_MS) {
    curDuty = (uint8_t)((pct * 255 + 50) / 100);
    char line[64];
    snprintf(line, sizeof(line), "pb,pct=%d,duty=%u,%s", pct, curDuty, up ? "up" : "down");
    Serial.println(line);
    if (!waitMs(1500)) break;
    if (up) {
      if (pct >= 100) { up = false; pct -= 10; } else pct += 10;
    } else {
      if (pct <= 0) { up = true; pct += 10; } else pct -= 10;
    }
  }
  stopAll();
}

// D8、D9 同時恆高；IN1~IN4 全高(懸空)或全低(煞車)，馬達不會被驅動。保持到 x。
void enableBoth(bool brake) {
  bool ok = brake ? assertBrakeLow() : assertIdleHigh();
  if (!ok) return;
  forceM1High = true;
  forceM2High = true;
  Serial.println(brake ? F("e,D8=HIGH,D9=HIGH,IN1~IN4=LOW(brake)")
                       : F("e,D8=HIGH,D9=HIGH,IN1~IN4=HIGH(float)"));
}

// 兩顆馬達同時正轉、duty 每階 +10(0~255)，每階 BOTH_STEP_MS，印出兩邊的 ticks/rpm(前進為正)。
void diagBothForwardSweep(int pctCap) {
  const uint32_t BOTH_STEP_MS = 1200;
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return;
  }
  bool ok = setDirection(CH_M1_IN1, CH_M1_IN2, 1);
  ok = setDirection(CH_M2_IN1, CH_M2_IN2, 1) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("err,direction pin I2C write failed"));
    return;
  }
  uint8_t cap = (uint8_t)((pctCap * 255 + 50) / 100);
  char line[80];
  snprintf(line, sizeof(line), "fstart,both forward,cap_duty=%u,cap_pct=%d", cap, pctCap);
  Serial.println(line);
  activeWheel = 3;
  dirSign = 1;
  sweepState = ST_HOLD;  // 讓 updateMotorPWM() 兩邊同時輸出；等待期間由 waitMs() 驅動
  bool moved1Ever = false, moved2Ever = false;
  float maxRpm1 = 0.0f, maxRpm2 = 0.0f;
  int firstMove1 = -1, firstMove2 = -1;
  bool aborted = false;
  for (int duty = 0; duty <= (int)cap; duty += 10) {
    curDuty = (uint8_t)duty;
    int32_t a1 = posM1;
    int32_t a2 = -posM2;  // 前進為正(M2 編碼器方向相反)
    if (!waitMs(BOTH_STEP_MS)) { aborted = true; break; }
    int32_t m1 = posM1 - a1;
    int32_t m2 = -posM2 - a2;
    float secs = BOTH_STEP_MS / 1000.0f;
    float rpm1 = (m1 / TICKS_PER_REV) / secs * 60.0f;
    float rpm2 = (m2 / TICKS_PER_REV) / secs * 60.0f;
    if (abs(rpm1) > abs(maxRpm1)) maxRpm1 = rpm1;
    if (abs(rpm2) > abs(maxRpm2)) maxRpm2 = rpm2;
    snprintf(line, sizeof(line), "fb,%u,%d,%ld,%ld,%.1f,%.1f", curDuty, (int)(curDuty * 100 / 255),
             (long)m1, (long)m2, rpm1, rpm2);
    Serial.println(line);
    if (duty > 0 && abs(m1) >= MOVE_NOISE_FLOOR) { moved1Ever = true; if (firstMove1 < 0) firstMove1 = duty; }
    if (duty > 0 && abs(m2) >= MOVE_NOISE_FLOOR) { moved2Ever = true; if (firstMove2 < 0) firstMove2 = duty; }
    // 安全中止：已經證實會轉的那一輪，換到更高 duty 卻完全不動。
    if (duty > 0 && ((moved1Ever && m1 == 0) || (moved2Ever && m2 == 0))) {
      aborted = true;
      Serial.println(F("abort,a wheel moved at lower duty but stopped at higher duty (driver/stall?)"));
      break;
    }
  }
  stopAll();
  snprintf(line, sizeof(line), "fdone,%s,L_first_move_duty=%d,R_first_move_duty=%d,L_max_rpm=%.1f,R_max_rpm=%.1f",
           aborted ? "aborted" : "complete", firstMove1, firstMove2, maxRpm1, maxRpm2);
  Serial.println(line);
}

// 兩顆同時正轉：先 duty 200 轉順，再由快到慢往下掃，量維持轉動的最低 duty(遲滯)。
// 每階前 600 ms 當穩定時間，後 600 ms 當量測窗口。印 vb,duty,pct,ticksL,ticksR,rpmL,rpmR。
void diagBothDownSweep() {
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return;
  }
  bool ok = setDirection(CH_M1_IN1, CH_M1_IN2, 1);
  ok = setDirection(CH_M2_IN1, CH_M2_IN2, 1) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("err,direction pin I2C write failed"));
    return;
  }
  Serial.println(F("vstart,both forward, spin up at 200 then step down"));
  activeWheel = 3;
  dirSign = 1;
  sweepState = ST_HOLD;
  curDuty = 200;
  if (!waitMs(2000)) { stopAll(); return; }
  char line[80];
  int zeroSteps = 0;
  int d = 200;
  while (d >= 0) {
    curDuty = (uint8_t)d;
    if (!waitMs(600)) break;                 // 穩定時間
    int32_t a1 = posM1;
    int32_t a2 = -posM2;                     // 前進為正
    if (!waitMs(600)) break;                 // 量測窗口
    int32_t m1 = posM1 - a1;
    int32_t m2 = -posM2 - a2;
    float rpm1 = (m1 / TICKS_PER_REV) / 0.6f * 60.0f;
    float rpm2 = (m2 / TICKS_PER_REV) / 0.6f * 60.0f;
    snprintf(line, sizeof(line), "vb,%d,%d,%ld,%ld,%.1f,%.1f", d, d * 100 / 255, (long)m1, (long)m2,
             rpm1, rpm2);
    Serial.println(line);
    if (abs(m1) < MOVE_NOISE_FLOOR && abs(m2) < MOVE_NOISE_FLOOR) {
      if (++zeroSteps >= 2) break;           // 兩階都完全不動就不必再往下
    } else {
      zeroSteps = 0;
    }
    d -= (d > 120) ? 10 : 5;
  }
  stopAll();
  Serial.println(F("vdone"));
}

// 階躍響應：取樣存 RAM(20 ms 一筆，共 160 筆=3.2 秒)，測完停輪後才印，量測期間不印字、不拖慢 PWM。
const int STEP_SAMPLES = 160;
const uint32_t STEP_SAMPLE_MS = 20;
int32_t stepPos1[STEP_SAMPLES];
int32_t stepPos2[STEP_SAMPLES];

void diagStep(int d1, int d2) {
  if (!pcaReady) pcaReady = pcaInit();
  if (!pcaReady) {
    Serial.println(F("err,PCA9685 not responding (power? I2C wiring?)"));
    return;
  }
  bool ok = setDirection(CH_M1_IN1, CH_M1_IN2, 1);
  ok = setDirection(CH_M2_IN1, CH_M2_IN2, 1) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("err,direction pin I2C write failed"));
    return;
  }
  activeWheel = 3;
  dirSign = 1;
  sweepState = ST_HOLD;
  bool completed = false;
  if (d1 > 0) {
    // 先衝到 200 讓兩顆都確實轉起來，再落到 d1 穩定 1.5 秒
    curDuty = 200;
    if (!waitMs(800)) goto finish;
    curDuty = (uint8_t)d1;
    if (!waitMs(1500)) goto finish;
  } else {
    curDuty = 0;
    if (!waitMs(1200)) goto finish;  // 確認完全停住
  }
  {
    uint32_t nextS = millis();
    curDuty = (uint8_t)d2;
    for (int n = 0; n < STEP_SAMPLES; n++) {
      while ((int32_t)(millis() - nextS) < 0) {
        pumpSend(usbSender, Serial);
        updateMotorPWM();
        pollSerial();
        if (abortRequested) goto finish;
      }
      stepPos1[n] = posM1;
      stepPos2[n] = -posM2;  // 前進為正
      nextS += STEP_SAMPLE_MS;
    }
    completed = true;
  }
finish:
  stopAll();
  if (!completed) {
    Serial.println(F("abort,step test interrupted"));
    return;
  }
  char line[64];
  snprintf(line, sizeof(line), "stepstart,d1=%d,d2=%d,sample_ms=%lu", d1, d2, (unsigned long)STEP_SAMPLE_MS);
  Serial.println(line);
  for (int n = 0; n < STEP_SAMPLES; n++) {
    snprintf(line, sizeof(line), "st,%d,%lu,%ld,%ld", n, (unsigned long)(n * STEP_SAMPLE_MS),
             (long)stepPos1[n], (long)stepPos2[n]);
    Serial.println(line);
  }
  Serial.println(F("stepdone"));
}

void runDiag(char c, char arg) {
  diagBusy = true;
  abortRequested = false;
  if (c == 'A') assertIdleHigh();
  else if (c == 'S') diagScan();
  else if (c == 'W') diagWalk();
  else if (c == 'K') assertBrakeLow();
  else if (c == 'Y') diagPwmRampBothBrake();
  else if (c == 'Q') {
    if (arg == 'L' || arg == 'l') diagPwmRampBrake(1);
    else if (arg == 'R' || arg == 'r') diagPwmRampBrake(2);
    else Serial.println(F("err,PS needs L or R (PSL / PSR)"));
  }
  else if (c == 'D') {
    if (arg == 'L' || arg == 'l') diagPwmBlink(1);
    else if (arg == 'R' || arg == 'r') diagPwmBlink(2);
    else Serial.println(F("err,D needs L or R (DL / DR)"));
  }
  abortRequested = false;
  diagBusy = false;
  Serial.println(F("diag,done"));
}

void handleCommand(const char* cmd) {
  char c = cmd[0];
  if (c == 'x' || c == 'X') {
    abortRequested = true;
    stopAll();
    Serial.println(F("ok,stopped"));
    return;
  }
  if (sweepState != ST_IDLE || diagBusy) {
    Serial.println(F("err,busy (send x to stop first)"));
    return;
  }
  // 新指令一律先放掉 E 留下的強制輸出
  forceM1High = false;
  forceM2High = false;
  if (c == 'E') {
    enableBoth(cmd[1] == 'B' || cmd[1] == 'b');
    return;
  }
  if (c == 'T' || c == 'U') {
    int d1 = 0, d2 = 0;
    if (c == 'T') d2 = atoi(cmd + 1);
    else sscanf(cmd + 1, "%d,%d", &d1, &d2);
    if (d1 < 0) d1 = 0;
    if (d1 > 255) d1 = 255;
    if (d2 < 0) d2 = 0;
    if (d2 > 255) d2 = 255;
    diagBusy = true;
    abortRequested = false;
    diagStep(d1, d2);
    abortRequested = false;
    diagBusy = false;
    Serial.println(F("diag,done"));
    return;
  }
  if (c == 'V') {
    diagBusy = true;
    abortRequested = false;
    diagBothDownSweep();
    abortRequested = false;
    diagBusy = false;
    Serial.println(F("diag,done"));
    return;
  }
  if (c == 'F') {
    int pct = 100;
    if (cmd[1] >= '0' && cmd[1] <= '9') pct = atoi(cmd + 1);
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    diagBusy = true;
    abortRequested = false;
    diagBothForwardSweep(pct);
    abortRequested = false;
    diagBusy = false;
    Serial.println(F("diag,done"));
    return;
  }
  if (c == 'A' || c == 'S' || c == 'W' || c == 'D' || c == 'K') {
    runDiag(c, cmd[1]);
    return;
  }
  if (c == 'P') {
    if (cmd[1] == 'B' || cmd[1] == 'b') {
      runDiag('Y', 0);
      return;
    }
    if (cmd[1] == 'S' || cmd[1] == 's') {
      runDiag('Q', cmd[2]);
      return;
    }
    int w = 0;
    if (cmd[1] == 'L' || cmd[1] == 'l') w = 1;
    else if (cmd[1] == 'R' || cmd[1] == 'r') w = 2;
    if (w == 0) {
      Serial.println(F("err,P needs L or R (PL50 / PR50 / PSL / PSR)"));
      return;
    }
    int pct = 100;
    if (cmd[2] >= '0' && cmd[2] <= '9') pct = atoi(cmd + 2);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    beginBrakePwm(w, pct);
    return;
  }
  bool hold = false;
  if (c == 'H' || c == 'h') {
    hold = true;
    cmd++;  // 往後吃一個字元：H 後面接輪子/方向字母
    c = cmd[0];
  }
  int wheel = 0, dir = 0;
  if (c == 'L') { wheel = 1; dir = 1; }
  else if (c == 'l') { wheel = 1; dir = -1; }
  else if (c == 'R') { wheel = 2; dir = 1; }
  else if (c == 'r') { wheel = 2; dir = -1; }
  // 2026-10-05：HB<pct>=兩顆同時正轉定值保持、Hb<pct>=兩顆同時反轉(溫升/長時間負載測試用)。
  // 只給H(定值保持)使用；wheel=3 在 updateMotorPWM() 已經是「兩邊同時輸出」。
  else if (hold && c == 'B') { wheel = 3; dir = 1; }
  else if (hold && c == 'b') { wheel = 3; dir = -1; }
  if (wheel == 0) {
    Serial.println(F("err,unknown command (L/R forward, l/r reverse, x stop)"));
    return;
  }
  int pct = 100;
  if (cmd[1] >= '0' && cmd[1] <= '9') pct = atoi(cmd + 1);
  if (pct < 1) pct = 1;
  if (pct > 100) pct = 100;
  if (hold) beginHold(wheel, dir, pct);
  else beginSweep(wheel, dir, pct);
}

char cmdBuf[16];
uint8_t cmdLen = 0;

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
      continue;
    }
    // 'x' 不等換行，收到就立刻停(緊急停止)
    if (c == 'x' || c == 'X') {
      cmdLen = 0;
      abortRequested = true;
      stopAll();
      Serial.println(F("ok,stopped"));
      continue;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
}

void setup() {
  // 一進 setup 就先把 PWM 腳拉低，之後才做其他初始化。
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);

  Serial.begin(115200);
  delay(300);

  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  pcaReady = pcaInit();

  Serial.println(F("=== car2 逐輪 duty 開迴路掃描(等指令) ==="));
  Serial.println(pcaReady ? F("pca,ready") : F("pca,NOT ready"));
  Serial.println(F("cmd: L/R[pct] forward, l/r[pct] reverse, x stop"));
}

void loop() {
  pumpSend(usbSender, Serial);
  updateMotorPWM();
  pollSerial();
  uint32_t now = millis();

  if (sweepState == ST_IDLE) {
    // 待機心跳：讓你從 USB 看得到韌體還活著。
    if (now - lastIdleBeatMs >= 2000) {
      lastIdleBeatMs = now;
      char line[64];
      snprintf(line, sizeof(line), "idle,pca=%d,posL=%ld,posR=%ld,d8=%d,d9=%d\n", pcaReady ? 1 : 0,
               (long)posM1, (long)posM2, forceM1High ? 1 : 0, forceM2High ? 1 : 0);
      queueLine(line);
    }
    return;
  }

  // 掃描總時間上限只管 PREPARE/SWEEP；定值保持(ST_HOLD)有自己的 MAX_HOLD_MS，不能被這個 60 秒
  // 提前中止(2026-10-02 修正：之前保持超過 60 秒就被這裡切掉，輸出全部歸零)。
  if (sweepState != ST_HOLD && now - sweepStartMs > MAX_SWEEP_TOTAL_MS) {
    finishWith("abort", "total time limit");
    return;
  }

  if (sweepState == ST_HOLD) {
    if (now - sweepStartMs > MAX_HOLD_MS) {
      finishWith("abort", "hold time limit");
      return;
    }
    if (now - lastIdleBeatMs >= 1000) {
      lastIdleBeatMs = now;
      char line[48];
      snprintf(line, sizeof(line), "holding,t=%lus,posL=%ld,posR=%ld\n",
               (unsigned long)((now - sweepStartMs) / 1000), (long)posM1, (long)posM2);
      queueLine(line);
    }
    return;
  }

  if (sweepState == ST_PREPARE) {
    if (now - stepStartMs >= PREPARE_MS) {
      sweepState = ST_SWEEP;
      curDuty = 0;
      stepStartMs = now;
      stepStartPos = activePos();
      settleCaptured = false;
    }
    return;
  }

  // ---- ST_SWEEP ----
  uint32_t elapsed = now - stepStartMs;

  if (elapsed >= SETTLE_MS && !settleCaptured) {
    settlePos = activePos();
    settleCaptured = true;
  }

  // 安全中止：已經證實這輪會動，換到更高 duty 卻完全沒新增 ticks。
  if (elapsed >= SETTLE_MS && curDuty > 0 && everMoved) {
    if (activePos() - stepStartPos == 0) {
      finishWith("abort", "wheel moved at lower duty but stopped at higher duty (driver/stall?)");
      return;
    }
  }

  if (elapsed >= HOLD_MS) {
    int32_t moved = activePos() - settlePos;  // 只算 settle 之後這段
    float measureS = (HOLD_MS - SETTLE_MS) / 1000.0f;
    float rpm = (moved / TICKS_PER_REV) / measureS * 60.0f;
    if (abs(rpm) > abs(maxRpm)) maxRpm = rpm;

    char line[72];
    snprintf(line, sizeof(line), "sw,%c,%c,%u,%d,%ld,%.1f\n", activeWheel == 1 ? 'L' : 'R',
             dirSign > 0 ? 'f' : 'r', curDuty, (int)(curDuty * 100 / 255), (long)moved, rpm);
    queueLine(line);

    if (curDuty > 0 && abs(moved) >= MOVE_NOISE_FLOOR) {
      everMoved = true;
      if (firstMoveDuty < 0) firstMoveDuty = curDuty;
    }

    if (curDuty >= dutyCap) {
      finishWith("done", "");
      return;
    }
    curDuty = (uint8_t)min((int)curDuty + DUTY_STEP, (int)dutyCap);
    stepStartMs = now;
    stepStartPos = activePos();
    settleCaptured = false;
  }
}
