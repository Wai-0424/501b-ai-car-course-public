// 排查用：車2實際5kHz PWM下的完整duty(0~250)->RPM開迴路掃描，跟乾淨測試板
// 那次(10kHz)做法一樣：連續斜坡(馬達全程保持轉動、不歸零重啟)，不是
// car2_openloop_coldstart_test.ino那種「每個duty都從死停重新啟動」的
// 冷啟動測試——連續斜坡電流風險低很多，因為除了deadband以下，馬達
// 一直在轉，不會有前次測試那種「堵轉電流撐900ms」的狀況。
//
// 安全機制：一旦偵測到「前一個duty值還有在轉，換到更高的duty卻完全
// 沒動」，代表不是死區(死區只會發生在低duty、且應該是連續的)、而是
// 疑似驅動異常(跟今天稍早car2_openloop_coldstart_test.ino之後懷疑
// 的驅動板/保險絲狀況一樣)，立刻中止整個掃描並切斷輸出，不要傻等
// HOLD_MS跑完、更不會繼續往更高duty測。
//
// 2026-09-17修正：原本[ol]這行50Hz用直接Serial.print()印，跟
// car2_speed_control_test_wip.ino裡找到的病灶是同一種——USB序列埠緩衝區
// 被灌滿、host沒即時讀時會忙等阻塞，凍結updateMotorPWM()，讓這次校正
// 掃描量到的RPM本身就可能被汙染。改成跟smoothness_probe.ino同一套非
// 阻塞佇列(ChunkedSender/queueLine)，這份重新校正的數據才乾淨可信。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---- 非阻塞分段傳送（沿用smoothness_probe.ino已驗證過的做法）----
// 型別定義要放在所有函式之前，否則Arduino自動產生的函式原型（在檔案最
// 前面插入，早於這裡下面任何一個函式定義）會找不到ChunkedSender型別，
// 導致「型別未宣告」的編譯錯誤。
struct ChunkedSender {
  char buf[96];
  uint8_t len;
  uint8_t pos;
  uint32_t nextSendMs;
  bool active;
};

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

const uint32_t PWM_PERIOD_US = 200;  // 5kHz，跟car2正式版一致(不是bench那次的10kHz)
const uint32_t HOLD_MS = 1000;       // 每個duty停留多久
const uint32_t SETTLE_MS = 300;      // 前面這段時間量到的ticks不算，只讓轉速穩定
const uint32_t LOG_PERIOD_MS = 20;   // 50Hz，跟正式版PID頻率一致

// 0~250，每10一階，涵蓋deadband前後到接近滿duty
const uint8_t DUTY_START = 0;
const uint8_t DUTY_END = 250;
const uint8_t DUTY_STEP = 10;

const float TICKS_PER_REV = 1288.86;  // 車2實測修正值(見car2_speed_control_test.ino)

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

uint8_t curDuty = DUTY_START;
uint32_t stepStartMs = 0;
uint32_t lastLogMs = 0;
int32_t stepStartPosM1 = 0;
int32_t stepStartPosM2 = 0;
int32_t settlePosM1 = 0;  // settle結束那一刻的位置，RPM只算settle之後這段，
int32_t settlePosM2 = 0;  // 不要把settle期間的加速爬升也算進穩態轉速裡
bool settleCaptured = false;
bool everMoved = false;  // 一旦量到過真的有轉動就恆為true——只有「已經證實會動、
                          // 之後卻不動了」才算異常，還在死區以下本來就不會動，
                          // 不能拿來當異常判斷基準(這是上一版的bug，duty=10這種
                          // 死區以下的值會被誤判成異常中止)
bool aborted = false;
bool finished = false;

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

// 佇列還在送前一行時，新的一行直接跳過不送，不阻塞等待。
void queueLine(const char* s) {
  if (usbSender.active) return;
  startSend(usbSender, s);
}

void stopAll(const __FlashStringHelper* reason) {
  curDuty = 0;
  pwm.setPin(CH_M1_IN1, 4095);
  pwm.setPin(CH_M1_IN2, 4095);
  pwm.setPin(CH_M2_IN1, 4095);
  pwm.setPin(CH_M2_IN2, 4095);
  Serial.println(reason);  // 掃描已經結束/中止，馬達不再運轉，這裡阻塞無風險
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)curDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < onUs ? HIGH : LOW);
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

  // 2026-09-17新增：燒錄過程/剛開機那瞬間腳位可能短暫浮動，加上如果上一次
  // 測試結束時輪子還有一點殘留慣性沒完全靜止，這裡先等一下讓真正的duty=0
  // (此時PWM還沒開始輸出，腳位維持靜止)穩定下來，再把這個時間點的posM1/
  // posM2當作起始基準，避免這些殘留位移被算進第一階(duty=0)的量測窗口。
  delay(500);

  Serial.println(F("=== 車2 5kHz 完整duty->RPM 開迴路掃描 ==="));
  Serial.println(F("csv,duty,rpmM1,rpmM2"));
  curDuty = DUTY_START;
  stepStartMs = millis();
  stepStartPosM1 = posM1;
  stepStartPosM2 = posM2;
  lastLogMs = stepStartMs;
}

void loop() {
  pumpSend(usbSender, Serial);  // 不管掃描是不是還在跑，都要繼續把佇列裡的
                                  // 資料送完，不能卡在佇列裡送不出去

  if (aborted || finished) {
    updateMotorPWM();
    return;
  }
  updateMotorPWM();
  uint32_t now = millis();
  uint32_t elapsed = now - stepStartMs;

  if (elapsed >= SETTLE_MS && !settleCaptured) {
    settlePosM1 = posM1;
    settlePosM2 = posM2;
    settleCaptured = true;
  }

  // 安全中止判斷：settle時間過後，若這個duty完全沒有新增ticks，
  // 但上一階(較低duty)是有在動的，代表異常，立刻中止。
  if (elapsed >= SETTLE_MS && curDuty > 0 && everMoved) {
    int32_t movedThisStep1 = posM1 - stepStartPosM1;
    int32_t movedThisStep2 = posM2 - stepStartPosM2;
    if (movedThisStep1 == 0 && movedThisStep2 == 0) {
      aborted = true;
      stopAll(F("!!! 異常中止：先前已經證實會轉，這一階完全沒動，懷疑驅動異常，已切斷輸出 !!!"));
      return;
    }
  }

  if (elapsed >= HOLD_MS) {
    int32_t movedM1 = posM1 - settlePosM1;  // 只算settle結束之後這段，避免把
    int32_t movedM2 = posM2 - settlePosM2;  // 加速爬升期間也算進穩態RPM，之前
                                             // 版本錯拿stepStartPos(整段HOLD_MS
                                             // 都算)去除以只有(HOLD_MS-SETTLE_MS)
                                             // 的時間，會把RPM灌水多算約40%+
    float measureS = (HOLD_MS - SETTLE_MS) / 1000.0f;
    float rpmM1 = (movedM1 / TICKS_PER_REV) / measureS * 60.0f;
    float rpmM2 = (-movedM2 / TICKS_PER_REV) / measureS * 60.0f;

    char csvLine[48];
    snprintf(csvLine, sizeof(csvLine), "csv,%u,%.2f,%.2f\n", curDuty, rpmM1, rpmM2);
    queueLine(csvLine);

    // 門檻設20 ticks，濾掉編碼器雜訊/震動誤觸發（duty=10這種死區以下的
    // 值實測過量到過個位數ticks的雜訊，不是真的在轉）
    // 2026-09-17修正：duty=0這階排除在everMoved判斷之外——本來就不該有
    // 任何驅動力矩，量到的任何位移(不管是燒錄/重開機瞬間腳位浮動、還是
    // 上一次測試殘留的慣性)都不代表「這顆馬達被驗證過會動」，反而可能
    // 誤觸發下一階(duty=10，本來就該落在死區完全不轉)的異常中止判斷——
    // 這正是本次實測踩到的狀況。
    const int32_t MOVE_NOISE_FLOOR = 20;
    if (curDuty > 0 && (abs(movedM1) >= MOVE_NOISE_FLOOR || abs(movedM2) >= MOVE_NOISE_FLOOR)) everMoved = true;

    if (curDuty >= DUTY_END) {
      finished = true;
      stopAll(F("=== 掃描完成，已停止 ==="));
      return;
    }
    curDuty = (uint8_t)min((int)curDuty + DUTY_STEP, (int)DUTY_END);
    stepStartMs = now;
    stepStartPosM1 = posM1;
    stepStartPosM2 = posM2;
    settleCaptured = false;
  }

  if (!aborted && !finished && (now - lastLogMs >= LOG_PERIOD_MS)) {
    lastLogMs = now;
    char line[48];
    snprintf(line, sizeof(line), "[ol] duty=%u posM1=%ld posM2=%ld\n",
             curDuty, (long)posM1, (long)posM2);
    queueLine(line);
  }
}
