// 車2：硬體 PWM(analogWrite) + 編碼器回授 —— 最簡單的測試（2026-10-10）
//
// 目的：正式版 car2_car_control.ino 的 D8/D9 是用 micros() 手刻的軟體 PWM，每次
// loop() 被 sendStatus() 之類的長函式佔住，PWM 就跟著凍結。wiring_plan.md 記載
// 早期 analogWrite() 在 D8/D9 上「實測沒有動作」，根因沒查出來。這支程式只回答一件事：
//   現在這片板子、現在這組接線，analogWrite() 驅動 D8/D9 時，編碼器有沒有回授？
//
// 三個彼此獨立的檢查（先做 1，再做 2，最後 3 用來對照）：
//   1. 編碼器本身：閒置時用手轉輪子，序列埠會即時印出計數。數字會變 = 編碼器線與
//      ISR 正常。完全不需要馬達電源，也不會有任何輸出。
//   2. 硬體 PWM（指令 h，預設）：analogWrite() 驅動馬達 2 秒，看編碼器 ticks。
//   3. 軟體 PWM（指令 s）：同樣 duty、同樣 5 kHz，用舊做法驅動，當對照組。
//      s 有動、h 沒動 -> 問題在 analogWrite()；兩個都沒動 -> 問題在接線/電源/驅動板。
//
// 腳位與正式版 car2_car_control.ino 完全一致（不要在這裡改，否則測的不是同一組接線）：
//   M1 左輪：PWM=D9  方向=PCA9685 CH3(高)/CH4(低)=正轉  編碼器 D4/D5   正轉時計數為正
//   M2 右輪：PWM=D8  方向=PCA9685 CH1(高)/CH2(低)=正轉  編碼器 D10/D11  正轉時計數為負
//   下面印出來的 L / R 都已經換成「前進為正」。
//
// 核心對照（HT32 core 1.3.0, variants/BM53A367A/variant.c）：
//   D8 -> PWM0 計時器 CH0；D9 -> MCTM0 計時器 CH0N（互補通道）。兩顆是不同計時器，
//   但 D9 走的是互補通道這條比較少人用的路徑，所以每次驅動結束後會直接把計時器暫存器
//   (重載值 CRR、比較值 CCR) 印出來，不用示波器就能看出 analogWrite() 有沒有把
//   duty 設對。
//
// 安全：
//   - 開機不會自動轉。只有下 l / r / b 指令才轉，一次最多 RUN_MS 毫秒，任何時候下 x 中止。
//   - 輪子一定要離地。L298N 沒有過流保護。
//   - 驅動 STALL_CHECK_MS 後該輪編碼器仍沒動 -> 自動中止（避免堵轉）。
//   - 停止時先把方向腳全部拉低（L298N 兩個 IN 同電位=停），再把 PWM 設 0。所以就算
//     互補通道的極性反了（duty 0 變成全開），馬達也不會轉。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 3;
const uint8_t CH_M1_IN2 = 4;
const uint8_t CH_M2_IN1 = 1;
const uint8_t CH_M2_IN2 = 2;

const uint8_t M1_PWM_PIN = 9;  // 左輪
const uint8_t M2_PWM_PIN = 8;  // 右輪

const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const float TICKS_PER_REV = 1288.86f;      // 跟正式版一樣：48 x 3249/121
const uint32_t RUN_MS = 2000;
const uint32_t STALL_CHECK_MS = 700;
const int32_t STALL_MIN_TICKS = 5;
const int32_t OK_MIN_TICKS = 50;           // 2 秒內前進計數至少這麼多才算「有回授」
const uint32_t SW_PWM_PERIOD_US = 200;     // 5 kHz，跟正式版軟體 PWM 一致
const uint32_t HW_REFRESH_MS = 20;         // 模擬正式版每個 PID 週期(20 ms)呼叫一次 analogWrite

uint8_t duty = 160;          // 14 V L298N 起轉門檻約 90~115，預設取高於它
uint32_t freqHz = 5000;      // 正式版前饋表是在 5 kHz 校正的；核心預設是 1 kHz
bool swMode = false;         // false=硬體 PWM(analogWrite)，true=軟體 PWM(對照組)
bool pcaReady = false;
bool hwTouched8 = false;
bool hwTouched9 = false;

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

// 前進為正的計數
int32_t fwdL() { return posM1; }
int32_t fwdR() { return -posM2; }

// ---- PCA9685 方向腳（回傳 I2C 是否成功，跟正式版 pcaWritePinChecked 同一寫法）----
bool pcaPin(uint8_t ch, bool high) {
  uint8_t r = high ? pwm.setPWM(ch, 4096, 0) : pwm.setPWM(ch, 0, 4096);
  return r == 0;
}
bool dirForward(uint8_t in1, uint8_t in2) {
  bool a = pcaPin(in1, true);
  bool b = pcaPin(in2, false);
  return a && b;
}
bool dirOff(uint8_t in1, uint8_t in2) {
  bool a = pcaPin(in1, false);
  bool b = pcaPin(in2, false);
  return a && b;
}

bool pcaProbe() {
  Wire1.beginTransmission(0x40);
  Wire1.write((uint8_t)0x00);
  return Wire1.endTransmission() == 0;
}

// 先關方向腳（保證不轉），再把 PWM 設 0。
void stopAll() {
  if (pcaReady) {
    dirOff(CH_M1_IN1, CH_M1_IN2);
    dirOff(CH_M2_IN1, CH_M2_IN2);
  }
  if (hwTouched9) analogWrite(M1_PWM_PIN, 0);
  if (hwTouched8) analogWrite(M2_PWM_PIN, 0);
  if (swMode) {
    digitalWrite(M1_PWM_PIN, LOW);
    digitalWrite(M2_PWM_PIN, LOW);
  }
}

// 把某根 PWM 腳對應的計時器暫存器印出來：重載值 CRR、比較值 CCR、計時器是否啟動，
// 以及這個 duty 理論上應該得到的 CCR。數字對得上 = analogWrite() 有把 duty 設對。
void dumpPwm(const char* name, uint8_t pin, uint8_t dutyNow) {
  HT_TM_TypeDef* tm = (HT_TM_TypeDef*)digitalPinToAOUTPort(pin);
  uint32_t chRaw = (uint32_t)digitalPinToAOUTCH(pin);
  bool isN = (chRaw & TM_CH_NBIT) != 0;
  uint32_t ch = chRaw & ~((uint32_t)TM_CH_NBIT);
  uint32_t crr = tm->CRR;
  uint32_t ccr = (&tm->CH0CCR)[ch];
  bool en = (tm->CTR & 1u) != 0;
  uint32_t expect = ((crr + 1) * (uint32_t)dutyNow) / 255u;
  Serial.print(F("  "));
  Serial.print(name);
  Serial.print(F(" 計時器="));
  Serial.print(en ? F("ON") : F("OFF"));
  Serial.print(F(" 通道="));
  Serial.print(ch);
  Serial.print(isN ? F("N(互補)") : F(""));
  Serial.print(F(" CRR="));
  Serial.print(crr);
  Serial.print(F(" CCR="));
  Serial.print(ccr);
  Serial.print(F(" (duty "));
  Serial.print(dutyNow);
  Serial.print(F(" 預期CCR≈"));
  Serial.print(expect);
  Serial.print(F(") 實際頻率≈"));
  Serial.print(crr > 0 ? (uint32_t)(SystemCoreClock / (crr + 1)) : 0u);
  Serial.println(F(" Hz(未計預除頻)"));
}

void printHelp() {
  Serial.println(F("--------------------------------------------------------------"));
  Serial.println(F("指令: l=只轉左輪(D9)  r=只轉右輪(D8)  b=兩輪同時  x=中止"));
  Serial.println(F("      h=硬體PWM(analogWrite)  s=軟體PWM(舊做法,對照用)"));
  Serial.println(F("      +/-=duty +-10   1=1kHz  5=5kHz   ?=顯示設定"));
  Serial.println(F("閒置時用手轉輪子: 計數會變 = 編碼器正常。輪子要離地才下 l/r/b。"));
  Serial.print(F("目前: 模式="));
  Serial.print(swMode ? F("軟體PWM") : F("硬體PWM"));
  Serial.print(F("  duty="));
  Serial.print(duty);
  Serial.print(F("  頻率="));
  Serial.print(freqHz);
  Serial.print(F("Hz  PCA9685="));
  Serial.println(pcaReady ? F("OK") : F("沒回應(不能驅動)"));
  Serial.println(F("--------------------------------------------------------------"));
}

void verdictFor(const __FlashStringHelper* name, int32_t ticks, float seconds) {
  float rpm = (seconds > 0.0f) ? (ticks / TICKS_PER_REV) / seconds * 60.0f : 0.0f;
  Serial.print(F("  "));
  Serial.print(name);
  Serial.print(F(": 前進計數 "));
  Serial.print(ticks);
  Serial.print(F(" ticks  約 "));
  Serial.print(rpm, 1);
  Serial.print(F(" rpm  -> "));
  if (ticks >= OK_MIN_TICKS) {
    Serial.println(F("OK 有回授"));
  } else if (ticks <= -OK_MIN_TICKS) {
    Serial.println(F("有回授但方向相反(編碼器A/B相或馬達極性)"));
  } else {
    Serial.println(F("沒有回授"));
  }
}

void runTest(bool useL, bool useR) {
  if (!pcaReady) {
    Serial.println(F("PCA9685 沒回應 -> 不驅動（沒有方向腳，馬達本來就不會轉）"));
    return;
  }
  bool ok = true;
  if (useL) ok = dirForward(CH_M1_IN1, CH_M1_IN2) && ok;
  if (useR) ok = dirForward(CH_M2_IN1, CH_M2_IN2) && ok;
  if (!ok) {
    pcaReady = false;
    stopAll();
    Serial.println(F("方向腳 I2C 寫入失敗 -> 中止"));
    return;
  }

  Serial.print(F(">>> 驅動 "));
  Serial.print(useL && useR ? F("兩輪") : (useL ? F("左輪(D9)") : F("右輪(D8)")));
  Serial.print(swMode ? F("  軟體PWM") : F("  硬體PWM"));
  Serial.print(F("  duty="));
  Serial.print(duty);
  Serial.print(F("  "));
  Serial.print(freqHz);
  Serial.println(F("Hz  （下 x 可中止）"));

  if (swMode) {
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
    digitalWrite(M1_PWM_PIN, LOW);
    digitalWrite(M2_PWM_PIN, LOW);
  } else {
    analogWriteFrequency(freqHz);
  }

  int32_t l0 = fwdL(), r0 = fwdR();
  int32_t lw = l0, rw = r0;
  uint32_t t0 = millis();
  uint32_t lastWin = t0;
  uint32_t lastRefresh = t0 - HW_REFRESH_MS;
  bool stallChecked = false;
  const char* abortWhy = nullptr;

  while (true) {
    uint32_t now = millis();
    if (now - t0 >= RUN_MS) break;

    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { abortWhy = "使用者中止"; break; }
    }

    if (swMode) {
      uint32_t phase = micros() % SW_PWM_PERIOD_US;
      uint32_t onUs = (uint32_t)duty * SW_PWM_PERIOD_US / 255;
      digitalWrite(M1_PWM_PIN, (useL && phase < onUs) ? HIGH : LOW);
      digitalWrite(M2_PWM_PIN, (useR && phase < onUs) ? HIGH : LOW);
    } else if (now - lastRefresh >= HW_REFRESH_MS) {
      lastRefresh = now;
      if (useL) { analogWrite(M1_PWM_PIN, duty); hwTouched9 = true; }
      if (useR) { analogWrite(M2_PWM_PIN, duty); hwTouched8 = true; }
    }

    if (now - lastWin >= 200) {
      lastWin = now;
      int32_t l = fwdL(), r = fwdR();
      Serial.print(F("  t="));
      Serial.print(now - t0);
      Serial.print(F("ms  L +"));
      Serial.print(l - lw);
      Serial.print(F("  R +"));
      Serial.println(r - rw);
      lw = l;
      rw = r;
    }

    if (!stallChecked && now - t0 >= STALL_CHECK_MS) {
      stallChecked = true;
      bool lStill = useL && (abs(fwdL() - l0) < STALL_MIN_TICKS);
      bool rStill = useR && (abs(fwdR() - r0) < STALL_MIN_TICKS);
      if (lStill || rStill) { abortWhy = "編碼器沒動(防堵轉自動中止)"; break; }
    }
  }

  uint32_t elapsed = millis() - t0;
  int32_t dL = fwdL() - l0;
  int32_t dR = fwdR() - r0;

  if (!swMode) {
    Serial.println(F("計時器暫存器（驅動中的最後狀態）:"));
    if (useL) dumpPwm("D9(左,MCTM0)", M1_PWM_PIN, duty);
    if (useR) dumpPwm("D8(右,PWM0) ", M2_PWM_PIN, duty);
  }

  stopAll();

  if (abortWhy) {
    Serial.print(F("!!! 提前結束: "));
    Serial.println(abortWhy);
  }
  Serial.print(F("結果（"));
  Serial.print(elapsed);
  Serial.println(F(" ms）:"));
  float sec = elapsed / 1000.0f;
  if (useL) verdictFor(F("左輪"), dL, sec);
  if (useR) verdictFor(F("右輪"), dR, sec);
  Serial.println(F("=== 結束，已停止 ==="));
}

int32_t lastPrintL = 0, lastPrintR = 0;
uint32_t lastIdlePrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);

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
    bool a = dirOff(CH_M1_IN1, CH_M1_IN2);
    bool b = dirOff(CH_M2_IN1, CH_M2_IN2);
    pcaReady = a && b;
  }

  Serial.println();
  Serial.println(F("=== car2 硬體PWM + 編碼器回授 測試 ready（開機不會自動轉）==="));
  printHelp();
  lastPrintL = fwdL();
  lastPrintR = fwdR();
}

void loop() {
  uint32_t now = millis();

  // 閒置：編碼器計數有變就印（限速 4 Hz），手轉輪子即可確認編碼器線
  if (now - lastIdlePrintMs >= 250) {
    int32_t l = fwdL(), r = fwdR();
    if (l != lastPrintL || r != lastPrintR) {
      lastIdlePrintMs = now;
      Serial.print(F("[enc] L="));
      Serial.print(l);
      Serial.print(F("  R="));
      Serial.println(r);
      lastPrintL = l;
      lastPrintR = r;
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'l': case 'L': runTest(true, false); break;
      case 'r': case 'R': runTest(false, true); break;
      case 'b': case 'B': runTest(true, true); break;
      case 'h': case 'H': swMode = false; Serial.println(F("模式: 硬體PWM(analogWrite)")); break;
      case 's': case 'S': swMode = true;  Serial.println(F("模式: 軟體PWM(舊做法,對照組)")); break;
      case '+': duty = (duty <= 245) ? duty + 10 : 255; Serial.print(F("duty=")); Serial.println(duty); break;
      case '-': duty = (duty >= 40) ? duty - 10 : 30;   Serial.print(F("duty=")); Serial.println(duty); break;
      case '1': freqHz = 1000; Serial.println(F("頻率=1000Hz(核心預設)")); break;
      case '5': freqHz = 5000; Serial.println(F("頻率=5000Hz(正式版校正頻率)")); break;
      case '?': printHelp(); break;
      default: break;  // 換行、空白、x(閒置時)都忽略
    }
    lastPrintL = fwdL();
    lastPrintR = fwdR();
  }
}
