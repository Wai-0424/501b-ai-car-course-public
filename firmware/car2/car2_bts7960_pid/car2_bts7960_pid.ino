// 車2 BTS7960 速度閉環韌體（2026-10-12，**bring-up 中**）
//
// 從 car2_car_control（L298N 最後版）移植：速度閉環 = 查表前饋 + PID + 變化率限制 + kick-start + 低速維持下限
// + 堵轉切斷 + 反轉保護 + PCA9685 互鎖，STA/CMD 通訊格式不變（Orin 端不用改）。
// 差異：
//   * 驅動板換成兩顆 BTS7960：每顆馬達 RPWM/LPWM 兩條硬體 PWM + 兩條致能(PCA9685)。接線見 car2_bts7960_basic.ino
//     與 hardware/car2_bts7960_wiring_plan_2026-10-11.md。
//   * PID 週期 60 Hz（用 micros 排程）、PWM 頻率 10 kHz（可用 USB 指令 f 即時改）。
//   * PID 改成 rpm 單位、用實際 dt 換算：週期改了增益不用跟著換算；積分直接累積成 duty，限幅 I_LIMIT_DUTY。
//   * 前饋表改成 duty -> rpm（BTS7960 + 10 kHz 重新量）。表的資料由 host_tools/bts_calibrate.py 量、
//     build_ff.py 寫回本檔的 FF_LUT 區塊；FF_CALIBRATED=false 表示還是占位值，不要拿去上地面。
//   * 拿掉 turbo（沒實測過，上限又取決於新的前饋表）；舊實作在 car2_car_control.ino。
//
// USB 文字指令（一行一個；也接受 <CMD,...> 框）：
//   m L R        百分比指令（走完整 PID）-100~100，持續 MANUAL_HOLD_MS
//   d L R        開迴路原始 duty -255~255，持續 RAW_HOLD_MS（要每 <0.5 s 重送，校正用；不走 PID、沒有堵轉切斷）
//   s            停止      z  編碼器歸零     t 0/1  開關 30 Hz 遙測串流 e,ms,posL,posR,dutyL,dutyR
//   k KP KI      即時改 PID 增益（duty/rpm、duty/(rpm·s)）
//   f HZ         即時改 PWM 頻率（1000~25000）
//   c DUTY MS    即時改 kick 脈衝      n DUTY   即時改 MIN_RUN_DUTY      r RATE  即時改變化率限制(duty/s)
//   g            印目前參數            h  說明
//
// dbg 欄位：dbg,cmdL,cmdR,dutyL,dutyR,rpmL,rpmR(沿目標方向為正),posL,posR(前進為正),illL,illR,f,pca,enL,enR,
//           loopGapMaxUs,staSkipBM,led(實際生效的車燈模式，2026-10-14 追加)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SerialBM Serial3

// ================= 可調參數 =================
const uint32_t PID_INTERVAL_US = 16667;   // 60 Hz
const uint32_t PWM_FREQ_DEFAULT_HZ = 20000;  // 前饋表必須和這個頻率一致（10 kHz 的表在 .bak 備份與 results/bts_sustain_20261012_*.csv）
const bool M1_SWAP_PWM = false;           // true = 把「前進用的 PWM 腳」換成 LPWM
const bool M2_SWAP_PWM = true;            // 2026-10-12 實測：M2 用 RPWM 當前進時右輪往後轉，所以對調
const int ENC_SIGN_M1 = +1;               // 前進時計數為正 = +1（2026-10-12 手轉實測，兩輪都是 +1）
const int ENC_SIGN_M2 = +1;
const uint32_t MANUAL_HOLD_MS = 2000;
const uint32_t RAW_HOLD_MS = 600;

// ---- 前饋表（duty -> rpm，每輪一條）。FF_CALIBRATED=false 時是占位值。----
// BEGIN_FF_LUT（build_ff.py 會改寫這個區塊）
const bool FF_CALIBRATED = true;   // 20261012_20k：2 次 pass 平均，BTS7960、20 kHz、輪子離地
const int FF_LUT_N = 31;
const float FF_LUT_DUTY[FF_LUT_N] = {36.0f, 40.0f, 44.0f, 48.0f, 52.0f, 56.0f, 60.0f, 64.0f, 68.0f, 72.0f, 76.0f, 80.0f, 85.0f, 90.0f, 95.0f, 100.0f, 110.0f, 120.0f, 130.0f, 140.0f, 150.0f, 160.0f, 170.0f, 180.0f, 190.0f, 200.0f, 210.0f, 220.0f, 230.0f, 240.0f, 255.0f};
const float FF_LUT_RPM_M1[FF_LUT_N] = {12.38f, 22.23f, 32.16f, 41.7f, 45.7f, 52.8f, 59.39f, 65.95f, 72.9f, 79.57f, 86.01f, 92.17f, 99.66f, 106.6f, 113.43f, 120.34f, 134.0f, 147.38f, 161.12f, 174.42f, 187.41f, 200.95f, 214.95f, 229.43f, 247.89f, 265.5f, 287.31f, 313.53f, 332.08f, 332.59f, 332.64f};
const float FF_LUT_RPM_M2[FF_LUT_N] = {11.72f, 21.8f, 31.97f, 41.94f, 45.7f, 52.91f, 59.55f, 66.18f, 73.12f, 79.91f, 86.4f, 92.68f, 100.09f, 107.07f, 114.09f, 121.03f, 134.62f, 148.43f, 162.16f, 175.89f, 189.43f, 203.16f, 216.78f, 231.48f, 249.21f, 267.25f, 289.09f, 315.82f, 337.98f, 339.5f, 339.55f};
// END_FF_LUT

// 100% 外部指令 = 這個轉速。2026-10-12 由 270（L298N 版）開放到 300 rpm：20 kHz 前饋表上 300 rpm 需要 duty 約 215，
// 表在 duty 230 以上就飽和在約 333 rpm（M1）／338 rpm（M2），所以 300 rpm 只留約 15 duty 的 PID 餘裕；
// 上地面有負載後曲線右移，滿速可能追不到 300 rpm。Orin 端的百分比因此全部放大 11%（同樣的 % 變快）。
const float MAX_RPM_100 = 300.0f;
const float CMD_MAP_FLOOR_PCT = 5.0f;     // 任何非 0 指令線性映射到 [5,100]%，油門行程前段不會沒反應

// PID：duty/rpm、duty/(rpm·s)。占位值，等量出 BTS 的 duty->rpm 斜率後再依步階測試整定。
float pidKp = 0.8f;
float pidKi = 0.8f;
const float I_LIMIT_DUTY = 40.0f;         // 積分項（以 duty 計）的限幅
// 變化率限制：每秒 duty 最多變多少；USB 指令 r 可即時改。步階測試(10 kHz，輪子離地，rpm 目標 30->60->30->100->50->15->0 %)：
// 600 上升 0.13~0.2 s、過衝 6~12%；300 上升 0.17~0.4 s、過衝 4~8%；150 上升 0.27~0.9 s、過衝 2~7%；穩態誤差都 < 1.2%。
// 取 300（舊 L298N 版是 600）：加速時的瞬間電流小一半，BTS7960 沒有過流保護。
float maxDutyRate = 300.0f;

// 20 kHz 下的低速死區（量到的，輪子離地）：起轉門檻 L 38/R 36（10 kHz 是 26/22）、維持下限 duty 36 約 12 rpm（10 kHz 是 20 → 6~9 rpm），
// duty 32 停住。死區約 7 µs 的有效脈寬（和資料表 SR=0 Ω 時 tdr 3.1 µs 加上升時間同量級），頻率越高占週期的比例越大。
float minRunDuty = 37.0f;                 // 維持下限；上地面有負載後會上升，需重測
float kickDuty = 60.0f;                   // 起步脈衝：要明顯高過起轉門檻(38)；上地面起不來就加大並調 kickMs
uint32_t kickMs = 100;

const uint32_t CMD_TIMEOUT_MS = 500;
const uint32_t STATUS_PERIOD_MS = 50;     // STA 20 Hz
const uint8_t USB_DBG_DIV = 4;            // USB dbg 每 200 ms 一行
const uint32_t REVERSE_COAST_MS = 200;    // 換方向前滑行(致能拉低)至少這麼久，並且轉速要降到下面這個值以下
const float REVERSE_MAX_RPM = 20.0f;
const uint32_t STALL_CUTOFF_MS = 600;     // 有驅動卻連續這麼久編碼器不動 → 切斷鎖定
const uint8_t STALL_MIN_DUTY = 30;        // duty 太低馬達本來就可能不動，不算堵轉
const uint32_t FAULT_CLEAR_MS = 300;      // 鎖定後指令需連續為 0 這麼久才解除
const float SPEED_ALPHA = 0.3f;           // 轉速 EMA（每個 PID 週期）
const float TICKS_PER_REV = 1288.86f;     // 12 PPR × 4 × 3249/121（輸出軸一圈）

// ================= 固定腳位 =================
const uint8_t PIN_M1_A = 4, PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10, PIN_M2_B = 11;
// ---- 車燈：WS2812 可定址 LED，資料線接 D13 (=PC11) ----
// 2026-10-14 新增。原本想用 FastLED，但 FastLED 3.10.7 不支援 HT32(編譯失敗：
// "Platform does not have a clockless controller defined")，所以自己寫一個最小的
// WS2812 bit-bang 驅動(ledShow)，時序用 SysTick(60 MHz 向下計數、每 1 ms 重載)當計時基準。
// LED_COUNT 是串在資料線上的顆數(多設無害，多出來的資料會被忽略；少設則後面的不亮)。
// 全亮白每顆約 60 mA，要降亮度就調 LED_BRIGHTNESS。
//
// Orin 控制：CMD 加一個 `led <模式>` 欄位(沒帶這個欄位就維持上一次的模式，舊版 Orin 程式不受影響)，
// 閃爍的時序由韌體自己產生(Orin 的 CMD 時間抖動不能拿來決定閃爍節奏)，STA 回報 `led <實際生效的模式>`。
//   0 = 熄滅
//   1 = 白色常亮(大燈：Orin 判斷鏡頭光度不足時送)
//   2 = 橘色爆閃(F1 式：停車/準備階段/循線結束)
//   3 = 紅色呼吸燈(失去 Orin：CMD 逾時時韌體自己強制切到這個；Orin 也可以主動送來測試)
const uint8_t PIN_LED = 13;
const uint8_t LED_COUNT = 16;
const uint8_t LED_BRIGHTNESS = 255;  // 0~255
enum LedMode : uint8_t { LED_OFF = 0, LED_WHITE = 1, LED_STROBE_ORANGE = 2, LED_BREATH_RED = 3, LED_MODE_COUNT };
const uint8_t LED_BOOT_MODE = LED_WHITE;  // 開機後 LED_BOOT_GRACE_MS 內、還沒收到 CMD 時的模式(白燈可以直接看出韌體活著)
const uint32_t LED_BOOT_GRACE_MS = 2000;  // 超過這段時間還沒收到任何合法 CMD，就視為沒有 Orin -> 紅色呼吸
// 橘色爆閃：2 Hz(週期 500 ms)，每個週期前 LED_STROBE_ON_MS 亮、其餘熄滅。要更短促的閃光就把 ON_MS 調小。
const uint32_t LED_STROBE_PERIOD_MS = 500;
const uint32_t LED_STROBE_ON_MS = 250;
// 紅色呼吸：亮度走三角波再平方(近似人眼感受的線性呼吸)，最暗不到全滅(LED_BREATH_MIN)，
// 每 LED_BREATH_STEP_MS 才更新一次(~33 Hz)，每次更新約 0.5 ms，只會出現在馬達已被失效保護歸零之後。
const uint32_t LED_BREATH_PERIOD_MS = 2000;
const uint8_t LED_BREATH_MIN = 6;
const uint32_t LED_BREATH_STEP_MS = 30;
// 沒有 Orin 時是否自動切紅色呼吸，提示「車子停了、沒有控制」：
//   * 收過合法 CMD 之後，CMD 逾時 CMD_TIMEOUT_MS(此時馬達已被失效保護歸零)；
//   * 開機後超過 LED_BOOT_GRACE_MS 還一個合法 CMD 都沒收到(Orin 還沒啟動/控制介面沒開)。
const bool LED_LINKLOST_BREATH = true;

const uint8_t M1_RPWM = 2, M1_LPWM = 3;
const uint8_t M2_RPWM = 8, M2_LPWM = 9;
const uint8_t CH_STEER = 0;
const uint8_t CH_M1_REN = 3, CH_M1_LEN = 4;
const uint8_t CH_M2_REN = 1, CH_M2_LEN = 2;
const uint8_t PCA9685_ADDR = 0x40;
const uint32_t PCA_RETRY_PERIOD_MS = 1000;
const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

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
  const float* ffRpm;      // 前饋表（rpm）
  int32_t lastPos;
  float signedRpm;         // 前進為正（EMA）
  float alongRpm;          // 沿目標方向為正（EMA 值乘上方向，指令 0 時為 0）
  int8_t dir;              // 目前輸出方向：1 前進、-1 反轉、0 停止（致能拉低）
  int8_t lastNonZeroDir;
  uint8_t duty;
  bool enabled;            // 致能腳快取（true = 兩個都拉高）
  bool kicking;
  uint32_t kickEndMs;
  float iTerm;             // 積分項（duty）
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
volatile uint32_t illegalM1 = 0;
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
double steerPulselengthUs = 0;
int lastSentSteerAngle = -1000;
uint32_t pwmFreqHz = PWM_FREQ_DEFAULT_HZ;

Motor m1 = {M1_SWAP_PWM ? M1_LPWM : M1_RPWM, M1_SWAP_PWM ? M1_RPWM : M1_LPWM, CH_M1_REN, CH_M1_LEN,
            ENC_SIGN_M1, &posM1, FF_LUT_RPM_M1, 0, 0, 0, 0, 0, 0, false, false, 0, 0, 0, 0, false, false, 0};
Motor m2 = {M2_SWAP_PWM ? M2_LPWM : M2_RPWM, M2_SWAP_PWM ? M2_RPWM : M2_LPWM, CH_M2_REN, CH_M2_LEN,
            ENC_SIGN_M2, &posM2, FF_LUT_RPM_M2, 0, 0, 0, 0, 0, 0, false, false, 0, 0, 0, 0, false, false, 0};

int steerAngle = 0;
int leftSpeed = 0;    // 來自 <CMD> 的 bwl（M1 左輪）
int rightSpeed = 0;   // bwr（M2 右輪）
int lastRxSeq = -1;
uint32_t cmdChecksumFailCount = 0;
uint32_t lastCmdMs = 0;
uint8_t ledMode = LED_BOOT_MODE;  // Orin 以 CMD 的 led 欄位指定的車燈模式
bool cmdEverReceived = false;     // 開機後是否收過合法 CMD(handleFrame 設)
int32_t ledShown = -1;            // 目前實際送到 LED 的顏色(0xRRGGBB)，-1=還沒送過；沒變就不重送
uint32_t lastStatusMs = 0;
uint32_t nextPidUs = 0;
uint32_t lastPidUs = 0;
uint8_t usbDbgDiv = 0;
uint32_t staSkipBM = 0;
uint32_t maxLoopGapUs = 0;
bool streamOn = false;
uint8_t streamDiv = 0;

int manualL = 0, manualR = 0;
uint32_t manualUntilMs = 0;   // 0 = 沒有
int rawL = 0, rawR = 0;
uint32_t rawUntilMs = 0;      // 0 = 沒有

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
bool pcaWritePinChecked(uint8_t ch, bool high) {
  uint8_t result = high ? pwm.setPWM(ch, 4096, 0) : pwm.setPWM(ch, 0, 4096);
  if (result != 0) {
    pcaReady = false;  // 交給 pcaRecoverTick() 重試
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
  m.kicking = false;
  m.iTerm = 0;
  commit(m);
}

// ================= 前饋 =================
// 目標 rpm -> 需要的 duty（表內線性內插；低於表頭沿第一段斜率往下外插；高於表尾夾在最後一點）。
float ffDutyForRpm(const float* lutRpm, float rpm) {
  if (rpm <= lutRpm[0]) {
    float slope = (FF_LUT_DUTY[1] - FF_LUT_DUTY[0]) / max(lutRpm[1] - lutRpm[0], 0.1f);
    return max(0.0f, FF_LUT_DUTY[0] - (lutRpm[0] - rpm) * slope);
  }
  for (int i = 1; i < FF_LUT_N; i++) {
    if (rpm <= lutRpm[i]) {
      float span = lutRpm[i] - lutRpm[i - 1];
      float f = span > 0.01f ? (rpm - lutRpm[i - 1]) / span : 0.0f;
      return FF_LUT_DUTY[i - 1] + f * (FF_LUT_DUTY[i] - FF_LUT_DUTY[i - 1]);
    }
  }
  return FF_LUT_DUTY[FF_LUT_N - 1];
}

// ================= 控制 =================
// 每個控制週期、不管哪種模式都先呼叫：更新轉速。
void sampleEncoder(Motor& m, uint32_t dtUs) {
  int32_t cur = *m.pos;
  int32_t delta = cur - m.lastPos;
  m.lastPos = cur;
  float rpm = ((float)(delta * m.encSign) / TICKS_PER_REV) * (60.0f * 1000000.0f / (float)dtUs);
  m.signedRpm = SPEED_ALPHA * rpm + (1.0f - SPEED_ALPHA) * m.signedRpm;
}

// 開迴路原始 duty（校正用）：不走 PID、不做堵轉切斷；host 要每 < RAW_HOLD_MS 重送。
void updateMotorRaw(Motor& m, int signedDuty) {
  m.fault = false;
  m.faultClearing = false;
  m.noMotionMs = 0;
  m.kicking = false;
  m.iTerm = 0;
  if (signedDuty == 0) {
    stopMotor(m);
    return;
  }
  m.dir = (signedDuty > 0) ? 1 : -1;
  m.lastNonZeroDir = m.dir;
  m.duty = (uint8_t)constrain(abs(signedDuty), 0, 255);
  m.alongRpm = m.signedRpm * m.dir;
  commit(m);
}

// 每個控制週期呼叫一次。cmdPct：-100~100。
void updateMotor(Motor& m, int cmdPct, uint32_t dtUs) {
  uint32_t now = millis();
  float dt = (float)dtUs * 1e-6f;
  uint16_t dtMs = (uint16_t)((dtUs + 500) / 1000);

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
      m.alongRpm = 0;
      return;
    }
  }

  if (cmdPct == 0) {
    if (m.dir != 0) m.zeroSinceMs = now;
    m.noMotionMs = 0;
    m.alongRpm = 0;
    stopMotor(m);
    return;
  }

  int8_t dir = (cmdPct > 0) ? 1 : -1;

  // 反轉保護：和上一次驅動方向相反時，要先停止(致能拉低、滑行)滿 REVERSE_COAST_MS，
  // 而且轉速降到 REVERSE_MAX_RPM 以下才放行。
  if (m.lastNonZeroDir != 0 && m.lastNonZeroDir != dir) {
    if (m.dir != 0) {
      m.zeroSinceMs = now;
      m.noMotionMs = 0;
      m.alongRpm = 0;
      stopMotor(m);
      return;
    }
    if ((int32_t)(now - m.zeroSinceMs) < (int32_t)REVERSE_COAST_MS || fabsf(m.signedRpm) > REVERSE_MAX_RPM) {
      m.noMotionMs = 0;
      m.alongRpm = 0;
      stopMotor(m);
      return;
    }
  }
  m.lastNonZeroDir = dir;

  // 外部命令 [0,100] -> 內部目標 [CMD_MAP_FLOOR_PCT,100]%（外部 0 才是停止）-> 目標 rpm
  float absPct = (float)abs(cmdPct);
  if (absPct > 100.0f) absPct = 100.0f;
  float mappedPct = CMD_MAP_FLOOR_PCT + (absPct / 100.0f) * (100.0f - CMD_MAP_FLOOR_PCT);
  float targetRpm = (mappedPct / 100.0f) * MAX_RPM_100;
  float ff = ffDutyForRpm(m.ffRpm, targetRpm);

  float actual = m.signedRpm * dir;
  m.alongRpm = actual;
  float prevDuty = (m.dir == dir) ? (float)m.duty : 0.0f;

  // 堵轉/編碼器失效切斷：有在驅動(duty 夠大)卻連續 STALL_CUTOFF_MS 沒有轉動（轉速 EMA 近 0）。
  // BTS7960 的限流(43~60 A)遠高於馬達堵轉電流，驅動板不會替馬達、電線、電源保護，這條是主要的保命線。
  if (m.duty >= STALL_MIN_DUTY && m.dir == dir && fabsf(m.signedRpm) < 1.0f) {
    if (m.noMotionMs < 60000) m.noMotionMs += dtMs;
  } else {
    m.noMotionMs = 0;
  }
  if (m.noMotionMs >= STALL_CUTOFF_MS) {
    m.fault = true;
    m.faultClearing = false;
    m.zeroSinceMs = now;
    m.alongRpm = 0;
    stopMotor(m);
    return;
  }

  // kick-start：從靜止(上一週期輸出 0)先用 kickDuty 打一個固定時長的開迴路脈衝掙脫靜摩擦，
  // 脈衝期間不跑 PID/積分，避免高 duty 被誤當成收斂過程灌進積分。
  m.dir = dir;
  if (prevDuty == 0.0f && !m.kicking) {
    m.kicking = true;
    m.kickEndMs = now + kickMs;
  }
  if (m.kicking) {
    if ((int32_t)(now - m.kickEndMs) < 0) {
      m.duty = (uint8_t)constrain(kickDuty, 0.0f, 255.0f);
      m.iTerm = 0;
      commit(m);
      return;
    }
    m.kicking = false;
    prevDuty = (float)m.duty;
  }

  // PID（rpm 單位）。積分直接累積成 duty；輸出飽和、碰到維持下限或變化率限制時暫停積分（防 windup）。
  float err = targetRpm - actual;
  float maxStep = maxDutyRate * dt;
  float tentative = ff + pidKp * err + m.iTerm;
  bool slewLimited = (tentative > prevDuty + maxStep) || (tentative < prevDuty - maxStep);
  bool willSaturate = (tentative <= minRunDuty) || (tentative >= 255.0f) || slewLimited;
  if (!willSaturate) {
    m.iTerm = constrain(m.iTerm + pidKi * err * dt, -I_LIMIT_DUTY, I_LIMIT_DUTY);
  }
  float output = ff + pidKp * err + m.iTerm;
  output = max(output, minRunDuty);
  float slewed = constrain(output, prevDuty - maxStep, prevDuty + maxStep);
  m.duty = (uint8_t)constrain(slewed, 0.0f, 255.0f);
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
  } else if (strcmp(key, "led") == 0) {
    if (value >= 0 && value < LED_MODE_COUNT) ledMode = value;  // 不認得的值忽略，維持原模式
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
  cmdEverReceived = true;
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
  Serial.println(F("m L R:百分比(PID) 2s | d L R:原始duty(校正,600ms) | s:停 | z:歸零 | t 0/1:串流 | k KP KI | f HZ | c DUTY MS | n DUTY | r RATE | g:參數 | h"));
  Serial.println(F("dbg: cmdL,cmdR,dutyL,dutyR,rpmL,rpmR,posL,posR,illL,illR,f,pca,enL,enR,loopGapMaxUs,staSkipBM"));
}

void printParams() {
  Serial.print(F("PID "));
  Serial.print(1000000.0f / PID_INTERVAL_US, 1);
  Serial.print(F(" Hz | PWM "));
  Serial.print(pwmFreqHz);
  Serial.print(F(" Hz | KP "));
  Serial.print(pidKp, 3);
  Serial.print(F(" KI "));
  Serial.print(pidKi, 3);
  Serial.print(F(" | kick "));
  Serial.print(kickDuty, 0);
  Serial.print(F("/"));
  Serial.print(kickMs);
  Serial.print(F(" ms | rate "));
  Serial.print(maxDutyRate, 0);
  Serial.print(F(" | minRun "));
  Serial.print(minRunDuty, 0);
  Serial.print(F(" | MAX_RPM_100 "));
  Serial.print(MAX_RPM_100, 0);
  Serial.print(F(" | 前饋表 "));
  Serial.println(FF_CALIBRATED ? F("已校正") : F("★占位值（未校正）"));
}

// 解析 n 個以空白分隔的浮點數（這個工具鏈的 sscanf 不支援 %f，會靜默回傳 0，所以用 strtod）。
int parseFloats(const char* s, float* out, int n) {
  int k = 0;
  while (k < n) {
    while (*s == ' ') s++;
    if (*s == '\0') break;
    char* end;
    double v = strtod(s, &end);
    if (end == s) break;
    out[k++] = (float)v;
    s = end;
  }
  return k;
}

void handleTextCommand(char* s) {
  int a, b;
  float x, y;
  float fv[2];
  if (s[0] == 'm' && sscanf(s + 1, "%d %d", &a, &b) == 2) {
    manualL = constrain(a, -100, 100);
    manualR = constrain(b, -100, 100);
    manualUntilMs = millis() + MANUAL_HOLD_MS;
    if (manualUntilMs == 0) manualUntilMs = 1;
    Serial.print(F("手動(PID) L="));
    Serial.print(manualL);
    Serial.print(F(" R="));
    Serial.println(manualR);
  } else if (s[0] == 'd' && sscanf(s + 1, "%d %d", &a, &b) == 2) {
    rawL = constrain(a, -255, 255);
    rawR = constrain(b, -255, 255);
    rawUntilMs = millis() + RAW_HOLD_MS;
    if (rawUntilMs == 0) rawUntilMs = 1;
  } else if (s[0] == 's') {
    manualUntilMs = 0;
    rawUntilMs = 0;
    manualL = manualR = rawL = rawR = 0;
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
  } else if (s[0] == 't' && sscanf(s + 1, "%d", &a) == 1) {
    streamOn = (a != 0);
  } else if (s[0] == 'k' && parseFloats(s + 1, fv, 2) == 2) {
    x = fv[0];
    y = fv[1];
    pidKp = constrain(x, 0.0f, 10.0f);
    pidKi = constrain(y, 0.0f, 20.0f);
    printParams();
  } else if (s[0] == 'f' && sscanf(s + 1, "%d", &a) == 1) {
    pwmFreqHz = (uint32_t)constrain(a, 1000, 25000);
    analogWriteFrequency(pwmFreqHz);
    printParams();
  } else if (s[0] == 'c' && parseFloats(s + 1, fv, 2) == 2) {
    x = fv[0];
    a = (int)fv[1];
    kickDuty = constrain(x, 0.0f, 255.0f);
    kickMs = (uint32_t)constrain(a, 0, 1000);
    printParams();
  } else if (s[0] == 'r' && parseFloats(s + 1, fv, 1) == 1) {
    x = fv[0];
    maxDutyRate = constrain(x, 50.0f, 5000.0f);
    printParams();
  } else if (s[0] == 'n' && parseFloats(s + 1, fv, 1) == 1) {
    x = fv[0];
    minRunDuty = constrain(x, 0.0f, 255.0f);
    printParams();
  } else if (s[0] == 'g') {
    printParams();
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
void sendStatus(int cmdL, int cmdR) {
  // rpmL/rpmR：沿目標方向為正（指令 0 時為 0）；vL/vR：常駐速度，前進為正，手轉/滑行/被推都看得到。
  float vM1 = m1.signedRpm;
  float vM2 = m2.signedRpm;
  int vL = (int)(vM1 + (vM1 >= 0 ? 0.5f : -0.5f));
  int vR = (int)(vM2 + (vM2 >= 0 ? 0.5f : -0.5f));
  int faultFlags = (m1.fault ? 1 : 0) | (m2.fault ? 2 : 0) | (pcaReady ? 0 : 4);

  char payload[96];
  snprintf(payload, sizeof(payload), "STA,ackseq %d,S1 -1,rpmL %.1f,rpmR %.1f,vL %d,vR %d,f %d,led %d",
           lastRxSeq, m1.alongRpm, m2.alongRpm, vL, vR, faultFlags, ledEffectiveMode(millis()));
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
  snprintf(dbg, sizeof(dbg), "dbg,%d,%d,%u,%u,%.1f,%.1f,%ld,%ld,%lu,%lu,%d,%d,%d,%d,%lu,%lu,%d\n",
           cmdL, cmdR, m1.duty, m2.duty, m1.alongRpm, m2.alongRpm,
           (long)(posM1 * ENC_SIGN_M1), (long)(posM2 * ENC_SIGN_M2),
           (unsigned long)illegalM1, (unsigned long)illegalM2,
           faultFlags, pcaReady ? 1 : 0, m1.enabled ? 1 : 0, m2.enabled ? 1 : 0,
           (unsigned long)maxLoopGapUs, (unsigned long)staSkipBM, ledEffectiveMode(millis()));
  startSend(usbSender, dbg);
  maxLoopGapUs = 0;
}

void sendStream() {
  if (usbSender.active) return;
  char e[80];
  snprintf(e, sizeof(e), "e,%lu,%ld,%ld,%u,%u\n", (unsigned long)millis(),
           (long)(posM1 * ENC_SIGN_M1), (long)(posM2 * ENC_SIGN_M2), m1.duty, m2.duty);
  startSend(usbSender, e);
}

// ================= setup / loop =================
// ---- WS2812 驅動(D13) ----
// 時序用「直接寫 GPIO 暫存器 + 固定次數的忙等迴圈」，不用 SysTick、也不呼叫 GPIO_SetOutBits/ClearOutBits。
// 2026-10-14 實測(investigations 無，見 firmware/car2/README.md)：
//   * GPIO_SetOutBits/ClearOutBits 是真的函式呼叫(含時脈開關)，光一次 set->clear 的高電位就 1.3 us，
//     比 WS2812 的 0 碼(~0.35 us)長太多 -> 所有 0 碼被讀成 1 -> 熄滅/橘/紅全都變白(全 1 的白光反而不受影響，
//     所以第一版常亮白燈看起來是對的)；
//   * 讀 SysTick->VAL 本身就要約 0.4 us，拿它當計時基準解析度不夠。
// 忙等迴圈每次 3 個時脈週期(60 MHz -> 50 ns，校準值：每次迴圈 +50 ns，每顆 bit 固定開銷約 220 ns)：
//   高電位 ~= 50 ns * 次數 + ~70 ns；位元週期 ~= 220 ns + 50 ns * (高次數 + 低次數)，兩種碼都湊到約 1.25 us。
// 改了編譯器/核心版本或 CPU 時脈要重新量(量測用的小程式在 PR/對話紀錄，沒進 repo)。
const uint32_t LED_T0H_LOOPS = 6;   // 高電位約 0.37 us(規格 0.25~0.55)
const uint32_t LED_T0L_LOOPS = 15;
const uint32_t LED_T1H_LOOPS = 14;  // 高電位約 0.77 us(規格 0.65~0.95)
const uint32_t LED_T1L_LOOPS = 7;

static inline __attribute__((always_inline)) void ledSpin(uint32_t n) {
  // n >= 1。.syntax unified：這個編譯設定預設是舊語法，subs 在 Thumb16 會報錯。
  __asm volatile(".syntax unified\n\t1: subs %0, %0, #1\n\t bne 1b" : "+l"(n));
}

// 關中斷只在「送一顆 LED(24 bit)」的約 30 us 內，顆與顆之間放開讓 ISR(編碼器、UART、SysTick)進來——
// 一口氣送完 16 顆要關 ~0.5 ms，460800 baud 下約 23 byte，會超過 UART 接收 FIFO。
// 顆與顆之間的空檔只有幾個 us(沒被 ISR 打斷時)，遠小於 WS2812 的鎖存門檻(>50 us)；
// 萬一某顆之間被一個很長的 ISR 拖過門檻，只會讓後面幾顆這一次顯示錯誤，下一次更新就恢復。
static void ledSendByte(HT_GPIO_TypeDef* port, uint32_t mask, uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    bool one = (b & 0x80) != 0;
    b <<= 1;
    uint32_t hi = one ? LED_T1H_LOOPS : LED_T0H_LOOPS;
    uint32_t lo = one ? LED_T1L_LOOPS : LED_T0L_LOOPS;
    port->SRR = mask;
    ledSpin(hi);
    port->RR = mask;
    ledSpin(lo);
  }
}

// 所有 LED 設成同一個顏色(WS2812 順序是 GRB)。
void ledShow(uint8_t r, uint8_t g, uint8_t b) {
  r = (uint16_t)r * LED_BRIGHTNESS / 255;
  g = (uint16_t)g * LED_BRIGHTNESS / 255;
  b = (uint16_t)b * LED_BRIGHTNESS / 255;
  HT_GPIO_TypeDef* port = digitalPinToPort(PIN_LED);
  uint32_t mask = digitalPinToBitMask(PIN_LED);
  uint32_t primask = __get_PRIMASK();
  for (uint8_t n = 0; n < LED_COUNT; n++) {
    __disable_irq();
    ledSendByte(port, mask, g);
    ledSendByte(port, mask, r);
    ledSendByte(port, mask, b);
    if (!primask) __enable_irq();
  }
  port->RR = mask;  // 之後資料線維持低電位，WS2812 自己在 >50 us 後鎖存
}

// ---- 車燈模式狀態機(狀態變數 ledMode/cmdEverReceived/ledShown 宣告在全域變數區) ----
// 目前生效的模式：沒有 Orin 且設定允許時強制紅色呼吸。
uint8_t ledEffectiveMode(uint32_t now) {
  if (LED_LINKLOST_BREATH) {
    bool noOrin = cmdEverReceived ? (now - lastCmdMs > CMD_TIMEOUT_MS) : (now > LED_BOOT_GRACE_MS);
    if (noOrin) return LED_BREATH_RED;
  }
  return ledMode;
}

// 每圈 loop() 呼叫。顏色沒變不會碰 LED：爆閃每個週期只有 2 次更新(亮、滅)，呼吸每 30 ms 一次。
void ledTick(uint32_t now) {
  uint8_t r = 0, g = 0, b = 0;
  switch (ledEffectiveMode(now)) {
    case LED_WHITE:
      r = g = b = 255;
      break;
    case LED_STROBE_ORANGE: {
      uint32_t phase = now % LED_STROBE_PERIOD_MS;
      bool on = phase < LED_STROBE_ON_MS;
      if (on) { r = 255; g = 90; }  // WS2812 的橘：G 約 R 的三分之一
      break;
    }
    case LED_BREATH_RED: {
      uint32_t t = (now / LED_BREATH_STEP_MS) * LED_BREATH_STEP_MS;  // 量化到 30 ms，同一格內顏色不變
      uint32_t phase = t % LED_BREATH_PERIOD_MS;
      uint32_t half = LED_BREATH_PERIOD_MS / 2;
      uint32_t tri = (phase < half ? phase : LED_BREATH_PERIOD_MS - phase) * 255 / half;  // 0~255 三角波
      r = LED_BREATH_MIN + tri * tri / 255 * (255 - LED_BREATH_MIN) / 255;
      break;
    }
    default:
      break;
  }
  int32_t packed = ((int32_t)r << 16) | ((int32_t)g << 8) | b;
  if (packed == ledShown) return;
  ledShown = packed;
  ledShow(r, g, b);
}

void setup() {
  Serial.begin(115200);
  SerialBM.begin(460800);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  ledTick(millis());  // 依 LED_BOOT_MODE 送第一次(開機後 LED_BOOT_GRACE_MS 內)

  // 四條 PWM 腳先拉低，再設頻率、寫 duty 0。頻率必須在第一次 analogWrite 之前設。
  const uint8_t pwmPins[] = {M1_RPWM, M1_LPWM, M2_RPWM, M2_LPWM};
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW);
  }
  analogWriteFrequency(pwmFreqHz);
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
  lastPidUs = micros();
  nextPidUs = lastPidUs + PID_INTERVAL_US;
  if (pcaReady) updateSteering();

  Serial.println(F("=== car2 BTS7960 PID 版（60 Hz / 10 kHz，bring-up 中）==="));
  Serial.print(F("PCA9685 "));
  Serial.println(pcaReady ? F("OK") : F("沒回應（馬達不會輸出）"));
  printParams();
  printUsbHelp();
}

void loop() {
  uint32_t nowUs = micros();
  {
    static uint32_t prevLoopUs = 0;
    if (prevLoopUs != 0 && nowUs - prevLoopUs > maxLoopGapUs) maxLoopGapUs = nowUs - prevLoopUs;
    prevLoopUs = nowUs;
  }
  pumpSend(bmSender, SerialBM);
  pumpSend(usbSender, Serial);
  pollBM();
  pollUsb();

  uint32_t now = millis();
  pcaRecoverTick(now);
  ledTick(now);

  int cmdL, cmdR;
  bool raw = false;
  if (rawUntilMs != 0) {
    if ((int32_t)(now - rawUntilMs) >= 0) {
      rawUntilMs = 0;
      rawL = rawR = 0;
    } else {
      raw = true;
    }
  }
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

  if ((int32_t)(nowUs - nextPidUs) >= 0) {
    uint32_t dtUs = nowUs - lastPidUs;
    lastPidUs = nowUs;
    nextPidUs += PID_INTERVAL_US;
    if ((int32_t)(nowUs - nextPidUs) > (int32_t)PID_INTERVAL_US) nextPidUs = nowUs + PID_INTERVAL_US;  // 落後太多就重新對齊
    if (dtUs < 1000) dtUs = 1000;
    sampleEncoder(m1, dtUs);
    sampleEncoder(m2, dtUs);
    if (raw) {
      updateMotorRaw(m1, rawL);
      updateMotorRaw(m2, rawR);
    } else {
      updateMotor(m1, cmdL, dtUs);
      updateMotor(m2, cmdR, dtUs);
    }
    if (streamOn && (++streamDiv >= 2)) {
      streamDiv = 0;
      sendStream();
    }
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus(cmdL, cmdR);
  }
}
