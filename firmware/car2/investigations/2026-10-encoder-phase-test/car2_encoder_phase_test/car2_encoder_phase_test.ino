// 車2 編碼器 A/B 相接線測試（2026-10-12）——不驅動馬達，用手轉輪子
//
// 目的：車子拆掉重組後，確認四條編碼器線有沒有接對：
//   M1(左輪) 編碼器 A=D4、B=D5      M2(右輪) 編碼器 A=D10、B=D11
// 韌體的前進極性慣例（沿用 car2_car_control）：往前轉時 M1 計數為「正」、M2 計數為「負」
// （updateMotorPID 的 encSign：M1=+1、M2=-1）。
//
// 用法：USB 序列埠 115200。
//   1. 不要給馬達供電（或把驅動板電源關掉）。這份韌體不輸出 PWM、不碰 PCA9685。
//   2. 只轉【左輪】往車子前進的方向轉約 1 圈，停 1.5 秒，看報告。
//   3. 只轉【右輪】往車子前進的方向轉約 1 圈，停 1.5 秒，看報告。
//   4. 輸入 z 歸零、輸入 r 重印說明。
//
// 為了避免驅動板的輸入腳浮接，D2/D3/D8/D9 被設成輸出低電位（BTS7960 與舊 L298N 配置都不會因此轉動）。

#include <Arduino.h>

const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

// 每輸出軸一圈的 4 倍解碼計數：12 PPR × 4 × 3249/121
const float TICKS_PER_REV = 1288.86f;
// 慣例：往前轉時各輪的計數正負號
const int EXPECT_SIGN_M1 = +1;
const int EXPECT_SIGN_M2 = -1;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

struct Enc {
  volatile int32_t pos;
  volatile uint8_t state;
  volatile uint32_t edgesA;
  volatile uint32_t edgesB;
  volatile uint32_t illegal;  // 一次跳兩格（漏掉一個邊緣）或兩相同時變化
};
// (結構要放在所有函式之前，否則 Arduino 自動產生的函式原型會引用到還沒宣告的型別)
struct Snap { int32_t pos; uint32_t ea, eb, ill; };
struct Seg {
  bool in;
  Snap self0;    // 區段開始時，這顆輪子的快照
  Snap other0;   // 區段開始時，另一顆輪子的快照（用來判斷另一輪有沒有跟著動）
  uint32_t lastMoveMs;
};
Enc enc1 = {0, 0, 0, 0, 0};
Enc enc2 = {0, 0, 0, 0, 0};

static inline void encUpdate(Enc& e, uint8_t pinA, uint8_t pinB) {
  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  uint8_t ns = (a << 1) | b;
  if (ns == e.state) return;
  int8_t d = QDEC_TABLE[((e.state << 2) | ns) & 0x0F];
  if (d == 0) e.illegal++;
  e.pos += d;
  e.state = ns;
}
void isrM1A() { enc1.edgesA++; encUpdate(enc1, PIN_M1_A, PIN_M1_B); }
void isrM1B() { enc1.edgesB++; encUpdate(enc1, PIN_M1_A, PIN_M1_B); }
void isrM2A() { enc2.edgesA++; encUpdate(enc2, PIN_M2_A, PIN_M2_B); }
void isrM2B() { enc2.edgesB++; encUpdate(enc2, PIN_M2_A, PIN_M2_B); }

Snap snap(Enc& e) {
  noInterrupts();
  Snap s = {e.pos, e.edgesA, e.edgesB, e.illegal};
  interrupts();
  return s;
}

void printHelp() {
  Serial.println(F("=== 車2 編碼器 A/B 相測試（不輸出 PWM）==="));
  Serial.println(F("只轉左輪往【前進】方向約 1 圈、停 1.5 秒看報告；再只轉右輪同樣做一次。"));
  Serial.println(F("指令：z=歸零  r=重印說明  s=印目前腳位電位"));
  Serial.println(F("慣例：往前轉 M1(左) 應為 +、M2(右) 應為 -。"));
}

void printLevels() {
  Serial.print(F("腳位電位  M1: A(D4)="));
  Serial.print(digitalRead(PIN_M1_A));
  Serial.print(F(" B(D5)="));
  Serial.print(digitalRead(PIN_M1_B));
  Serial.print(F("   M2: A(D10)="));
  Serial.print(digitalRead(PIN_M2_A));
  Serial.print(F(" B(D11)="));
  Serial.println(digitalRead(PIN_M2_B));
}

void setup() {
  Serial.begin(115200);
  const uint8_t holdLow[] = {2, 3, 8, 9};
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(holdLow[i], OUTPUT);
    digitalWrite(holdLow[i], LOW);
  }
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  enc1.state = (digitalRead(PIN_M1_A) << 1) | digitalRead(PIN_M1_B);
  enc2.state = (digitalRead(PIN_M2_A) << 1) | digitalRead(PIN_M2_B);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);
  delay(500);
  printHelp();
  printLevels();
}

// 每顆輪子各自的「手轉」區段：從開始動到停下 1.5 秒（兩輪分開計，先左再右連續轉也不會混在一起）
Seg seg1 = {false, {0, 0, 0, 0}, {0, 0, 0, 0}, 0};
Seg seg2 = {false, {0, 0, 0, 0}, {0, 0, 0, 0}, 0};
Snap last1, last2;
uint32_t lastPrintMs = 0;
Snap zero1 = {0, 0, 0, 0}, zero2 = {0, 0, 0, 0};

void verdictWheel(const char* name, int expectSign, const Snap& s0, const Snap& s1, const Snap& other0, const Snap& other1) {
  int32_t net = s1.pos - s0.pos;
  uint32_t ea = s1.ea - s0.ea, eb = s1.eb - s0.eb, ill = s1.ill - s0.ill;
  int32_t otherNet = other1.pos - other0.pos;
  Serial.print(F("  ["));
  Serial.print(name);
  Serial.print(F("] 淨計數 "));
  Serial.print(net);
  Serial.print(F("（約 "));
  Serial.print(fabsf((float)net) / TICKS_PER_REV, 2);
  Serial.print(F(" 圈）  A邊緣 "));
  Serial.print(ea);
  Serial.print(F("  B邊緣 "));
  Serial.print(eb);
  Serial.print(F("  非法跳變 "));
  Serial.println(ill);

  if (abs(net) < 100 && ea + eb < 100) {
    Serial.println(F("    -> 這輪幾乎沒計數（沒轉，或兩相都沒接）。"));
    return;
  }
  if (ea == 0 || eb == 0) {
    Serial.println(F("    -> 異常：其中一相完全沒有邊緣。檢查那條 A 或 B 線（斷線／接錯腳）。"));
    return;
  }
  float ratio = (float)ea / (float)eb;
  if (ratio < 0.8f || ratio > 1.25f) {
    Serial.println(F("    -> 警告：A、B 邊緣數差很多，一相可能接觸不良。"));
  }
  if (ill * 50 > ea + eb) {
    Serial.println(F("    -> 警告：非法跳變超過 2%，訊號有雜訊或接觸不良（轉太快時也可能發生）。"));
  }
  if (abs(otherNet) > abs(net) / 4 + 20) {
    Serial.println(F("    -> 異常：另一輪也有明顯計數，左右編碼器可能接反（M1／M2 對調），或兩輪機械連動。"));
  }
  int sign = (net > 0) - (net < 0);
  if (sign == expectSign) {
    Serial.println(F("    -> 方向 OK：往前轉的計數正負號符合慣例（A/B 相順序對）。"));
  } else {
    Serial.println(F("    -> 方向相反：若你剛才確實是往前進方向轉，代表 A、B 兩線接反。"));
    Serial.println(F("       處理：對調這顆編碼器的 A、B 線；或在韌體把該輪的 encSign 乘 -1。"));
  }
}

// 檢查一顆輪子的區段；停下 1.5 秒就印報告
void checkSeg(Seg& g, bool moved, const char* name, int expectSign, const Snap& self, const Snap& other,
              const Snap& lastSelf, const Snap& lastOther, uint32_t now) {
  if (moved) {
    if (!g.in) {
      g.in = true;
      g.self0 = lastSelf;  // 動之前的狀態
      g.other0 = lastOther;
    }
    g.lastMoveMs = now;
  } else if (g.in && now - g.lastMoveMs >= 1500) {
    g.in = false;
    int32_t n = self.pos - g.self0.pos;
    uint32_t edges = (self.ea + self.eb) - (g.self0.ea + g.self0.eb);
    if (abs(n) < 10 && edges < 20) return;  // 輕微抖動，不報告
    Serial.println(F("--- 這次手轉的報告 ---"));
    verdictWheel(name, expectSign, g.self0, self, g.other0, other);
    Serial.println(F("---"));
  }
}

void loop() {
  uint32_t now = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      noInterrupts();
      enc1.pos = 0; enc1.edgesA = 0; enc1.edgesB = 0; enc1.illegal = 0;
      enc2.pos = 0; enc2.edgesA = 0; enc2.edgesB = 0; enc2.illegal = 0;
      interrupts();
      seg1.in = false;
      seg2.in = false;
      last1 = zero1;
      last2 = zero2;
      Serial.println(F("已歸零"));
    } else if (c == 'r') {
      printHelp();
    } else if (c == 's') {
      printLevels();
    }
  }

  Snap a = snap(enc1);
  Snap b = snap(enc2);
  bool moved1 = (a.ea + a.eb != last1.ea + last1.eb);
  bool moved2 = (b.ea + b.eb != last2.ea + last2.eb);

  if ((moved1 || moved2) && now - lastPrintMs >= 250) {
    lastPrintMs = now;
    Serial.print(F("M1(左) pos="));
    Serial.print(a.pos);
    Serial.print(F("  M2(右) pos="));
    Serial.println(b.pos);
  }

  checkSeg(seg1, moved1, "M1 左輪", EXPECT_SIGN_M1, a, b, last1, last2, now);
  checkSeg(seg2, moved2, "M2 右輪", EXPECT_SIGN_M2, b, a, last2, last1, now);

  last1 = a;
  last2 = b;
}
