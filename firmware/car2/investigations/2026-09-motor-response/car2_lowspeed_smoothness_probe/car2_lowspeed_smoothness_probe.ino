// 排查用：驗證「動摩擦維持轉動的門檻是否比靜摩擦啟動門檻低很多」這個
// 假設。做法：先用duty=60把輪子踢動(確定掙脫靜摩擦)，之後逐階往下降
// duty(50→40→30→20→10→0)，每階撐夠久，看輪子是「只是變慢但還在轉」
// (代表動摩擦門檻真的比較低，PID有機會在這個範圍收斂)還是「直接停下來」
// (代表沒有這個維持範圍，回授一樣沒有平衡點可以抓)。
//
// 純開迴路、直接控制duty，不經過PID，才能乾淨看出「這個duty能不能維持
// 已經在轉的輪子」而不被PID的動態行為干擾。
//
// log輸出改成非阻塞分段傳送（沿用car2_speed_control_test_wip.ino裡
// 已經驗證過的ChunkedSender/pumpSend()模式）：這片HT32核心的USB
// Serial，TX緩衝區被灌滿、host沒即時讀取時，Serial.print()會忙等
// 阻塞（見docs/architecture/uart_tx_corruption_bug.md與wip版註解），
// 阻塞期間loop()連帶updateMotorPWM()都不會被呼叫，PWM輸出會凍結在
// 阻塞當下的電位——這份探測本身就是要量「這個duty能不能維持轉動」，
// 如果量測過程自己造成PWM忽有忽無，量到的「停下來」就分不清是真的
// 動摩擦門檻，還是量測工具自己的阻塞假象，整個測試會失去意義。所以
// 這裡跟主控制韌體一樣，log一律走非阻塞佇列，絕不讓Serial.print()
// 卡住loop()。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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

const uint32_t PWM_PERIOD_US = 200;  // 5kHz，跟car2正式版一致
const uint32_t LOG_PERIOD_MS = 20;   // 50Hz，每20ms記一次位置

struct Phase {
  uint8_t duty;
  uint32_t holdMs;
};

// 先用60踢動(確定掙脫靜摩擦)，之後逐階降duty，每階撐夠久看是否還在轉。
const Phase PHASES[] = {
  {60, 600},   // 踢動，確定已經在轉
  {50, 1800},
  {60, 400},   // 中間重新踢一下，避免前一階如果已經停了，後面都測不到東西
  {40, 1800},
  {60, 400},
  {30, 1800},
  {60, 400},
  {20, 1800},
  {60, 400},
  {10, 1800},
  {0, 500},
};
const uint8_t NUM_PHASES = sizeof(PHASES) / sizeof(PHASES[0]);

// ---- 非阻塞分段傳送（跟wip版main firmware的BM傳送用同一套設計）----
// 每次pumpSend()最多送一個5-byte chunk就return，段落間隔靠「下次loop()
// 進來時millis()是否到nextSendMs」自然形成，不會有任何一次呼叫超過
// 一個write()的時間，保證不阻塞updateMotorPWM()的呼叫節奏。
struct ChunkedSender {
  char buf[80];
  uint8_t len;
  uint8_t pos;
  uint32_t nextSendMs;
  bool active;
};

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

// 佇列還在送前一行時，新的一行直接跳過不送——寧可log偶爾漏一筆
// (50Hz的log，漏一筆不影響後續分析)，也不要為了「一定要送出」而
// 阻塞等待，那樣就違背了整套非阻塞設計的目的。
void queueLine(const char* s) {
  if (usbSender.active) return;
  startSend(usbSender, s);
}

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

uint8_t curDuty = 0;
uint8_t phaseIdx = 0;
uint32_t phaseStartMs = 0;
uint32_t lastLogMs = 0;

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

  Serial.println(F("=== 動摩擦維持門檻測試 ==="));
  Serial.println(F("(6秒緩衝，準備擷取序列埠...)"));
  delay(6000);

  phaseIdx = 0;
  curDuty = PHASES[0].duty;
  phaseStartMs = millis();
  lastLogMs = phaseStartMs;

  Serial.print(F("### phase 0 duty="));
  Serial.println(PHASES[0].duty);
}

void loop() {
  // 每次loop()都先pump，不管下面是不是還在跑階梯——確保setup()結尾
  // 或最後一行log排進佇列後，就算測試已經跑完也會繼續被送完，不會卡在
  // 佇列裡送不出去。
  pumpSend(usbSender, Serial);

  if (phaseIdx >= NUM_PHASES) {
    curDuty = 0;
    updateMotorPWM();
    return;
  }

  updateMotorPWM();

  uint32_t now = millis();
  uint32_t elapsed = now - phaseStartMs;

  if (now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    char line[64];
    snprintf(line, sizeof(line), "[ol] phase=%u duty=%u posM1=%ld posM2=%ld\n",
             phaseIdx, curDuty, (long)posM1, (long)posM2);
    queueLine(line);
  }

  if (elapsed >= PHASES[phaseIdx].holdMs) {
    phaseIdx++;
    if (phaseIdx < NUM_PHASES) {
      phaseStartMs = now;
      curDuty = PHASES[phaseIdx].duty;
      char line[32];
      snprintf(line, sizeof(line), "### phase %u duty=%u\n", phaseIdx, PHASES[phaseIdx].duty);
      queueLine(line);
    } else {
      queueLine("=== 測試完成 ===\n");
    }
  }
}
