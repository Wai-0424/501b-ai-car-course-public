// Hall 正交編碼器 + PCA9685 馬達驅動 獨立測試工具
//
// 沿用跟 bmduino_car_control 完全一樣的驅動架構（PCA9685 出方向、
// BMduino 軟體PWM 出轉速、D6/D7 讀正交編碼器回授），但拔掉車輛專屬的
// CMD/STA 協定、PID、雙輪，變成一個單馬達的獨立測試工具，方便之後在
// 其他 BM+PCA9685 的專案上，快速驗證「新接的馬達+編碼器」能不能正常
// 驅動、讀轉速。
//
// 目前先只留 M1 這一組（PCA9685 CH1/CH2 出方向、D9 出轉速PWM、
// D6/D7 讀編碼器），之後真的要測第二顆馬達，比照這組再加一份即可。
//
// **注意：這裡測的馬達（見下方「目前測試對象」）只是借用這個韌體架構
// 來驗證，跟車輛專案（bmduino_car_control，JGB37-520馬達，
// TICKS_PER_REV=1550）完全是兩回事，兩邊的校正常數不要搞混、也不要
// 拿這份的 TICKS_PER_REV 去改車輛專案。**
//
// ---- 目前測試對象：Hennkwell PK32K5SP284HE12T-027BBQ ----
// DC 行星齒輪箱直流馬達 + 內建磁性編碼器，datasheet 重點：
//   - 額定電壓 12VDC（6~12VDC 操作範圍）
//   - 減速比 121/3249（馬達本身約轉 26.85 圈，輸出軸才轉 1 圈）
//   - 編碼器：磁性編碼器、12 PPR（每圈脈波數，裝在馬達本體軸、
//     減速箱之前，不是輸出軸）
//   - Hall sensor 相序：B 先於 A（B>A），標示的正轉方向是 CCW
//   - 6P 接頭線色定義（JST PHR-6）：
//       1 黑 Motor-      2 紅 Motor+       3 綠 編碼器Ground
//       4 棕 Channel B   5 黃 Channel A    6 橙 Sensor Voltage+
//     **橙線（Sensor Voltage+）是霍爾感測器自己的供電，範圍
//     2.4V~26V，跟黑/紅的馬達動力線是分開的兩回事，也跟綠線的
//     Ground 不同。**
//
// 除錯記錄（已解決）：一開始 A/B 兩相 edge 數嚴重不對稱（其中一相
// 明顯偏多，且比例會隨轉速改變、甚至高低速時哪相偏多會反過來），
// tick 一直卡在附近打轉不會真的累加。依序排除了 I2C 鬆脫/SDA、SCL
// 接反（跟編碼器無關，但當時同時發生）、INPUT_PULLUP（無效）、
// Sensor Voltage+ 沒接好（有幫助但沒解決）之後，最後發現是**棕線
// (Channel B) 跟綠線(Ground) 接反了**——接對之後 edgeA/edgeB 每次
// 都完全一致、tick 平順累加，duty=120 穩定在馬達軸約4200rpm、
// duty=200 約6500rpm（換算成輸出軸約156/242rpm，跟datasheet的
// no-load 313rpm量級吻合）。這條經驗給以後其他專案接線時參考：
// A/B 兩相 edge 數量級差很多、且比例不穩定，優先懷疑接線腳位對應
// 錯誤（尤其容易跟Ground搞混），不一定是雜訊或供電問題。
//
// TICKS_PER_REV 換算（4倍正交解碼）：
//   馬達軸每圈 tick 數 = 12 PPR x 4 = 48
//   若編碼器讀值要換算成「輸出軸」轉速：48 x (3249/121) ≈ 1288.8
//   這裡先假設你要看的是「馬達軸」轉速（跟編碼器實際裝的位置一致），
//   下面先填 48；如果你要的是輸出軸轉速，改成 1288.8，或直接用手動
//   轉輸出軸整數圈的方式實測校正（比較準，減速箱間隙、實際比值跟
//   標稱值可能有些微差異）。
//
// 用法：
//   1. 先裝函式庫：Arduino IDE -> 工具 -> 管理函式庫 -> 搜尋安裝
//      `Adafruit PWM Servo Driver Library`
//   2. 燒錄後打開 Serial Monitor（115200，換行字元選「Newline」）
//   3. 輸入 -255~255 的整數 + Enter：正數=正轉、負數=反轉、0=停止，
//      會持續印出目前 duty / tick數 / 換算RPM
//   4. 校正 TICKS_PER_REV：先隨便給一個小 duty 轉起來，或手動轉軸，
//      看 tick 數變化是否合理，抓整數圈數比對修正這個常數
//
// 常數（CH_M1_IN1/IN2、M1_PWM_PIN、PIN_M1_A/B）沿用車輛專案 M1 那組
// 接線位置，但 TICKS_PER_REV 已經改成這顆新馬達自己的值，拿去別的
// 專案用的話，全部换成你自己的接線/校正值即可。

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---- PCA9685 方向腳 + BM 軟體PWM 轉速腳 ----
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz 軟體PWM

// ---- 編碼器輸入（四倍正交解碼）----
// PIN_M1_A <- 黃線(Channel A)，PIN_M1_B <- 棕線(Channel B)；
// 綠線(Ground)接BM的GND，橙線(Sensor Voltage+)另外接2.4~26V供電
// （見上方說明，這條線目前懷疑是雜訊根因，優先檢查）。
const uint8_t PIN_M1_A = 6;
const uint8_t PIN_M1_B = 7;

// ---- 校正常數：目前這顆 Hennkwell PK32K5SP284HE12T-027BBQ 的值
// （馬達軸12PPR x 4倍解碼 = 48，見上方換算說明；不是車輛專案的1550）----
const float TICKS_PER_REV = 48.0;
const uint32_t REPORT_PERIOD_MS = 200;

volatile int32_t posM1 = 0;
volatile uint8_t stateM1 = 0;

// 標準正交解碼表（四倍解析度）：index = (上次2-bit狀態<<2)|這次2-bit狀態，
// value = 方向增量(-1/0/+1)。0代表無效跳變（雜訊或漏拍），直接忽略。
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

// 除錯用：分開算 A相、B相各自觸發了幾次中斷（CHANGE），用來排查是不是
// 其中一相根本沒在跳動（斷線/沒接對）——如果馬達真的在轉但tick卻卡在
// 0附近亂跳，很可能就是這裡：其中一相edge數接近0，另一相edge數正常。
volatile uint32_t edgeCountA = 0;
volatile uint32_t edgeCountB = 0;
void isrM1A() { edgeCountA++; updateQuadM1(); }
void isrM1B() { edgeCountB++; updateQuadM1(); }

int m1Duty = 0;  // 0~255，方向另外用 setDirection() 控制

void setDirection(int dir) {
  if (dir > 0) {
    pwm.setPin(CH_M1_IN1, 4095);
    pwm.setPin(CH_M1_IN2, 0);
  } else if (dir < 0) {
    pwm.setPin(CH_M1_IN1, 0);
    pwm.setPin(CH_M1_IN2, 4095);
  } else {
    pwm.setPin(CH_M1_IN1, 0);
    pwm.setPin(CH_M1_IN2, 0);  // 煞車
  }
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
}

int32_t lastPos = 0;
uint32_t lastReportMs = 0;

// 用緩衝逐字元組行再 atoi()，不用 Serial.parseInt()：parseInt() 遇到
// 換行殘留字元（例如 \r\n 只吃掉一個）會在下一輪卡住等下一個數字最多
// 達預設1秒逾時，這段時間 loop() 完全卡住、PWM跟RPM回報都會停擺。
char inputBuf[16];
uint8_t inputLen = 0;

void pollSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLen > 0) {
        inputBuf[inputLen] = '\0';
        int v = atoi(inputBuf);
        v = constrain(v, -255, 255);
        setDirection(v > 0 ? 1 : (v < 0 ? -1 : 0));
        m1Duty = abs(v);
        Serial.print(F("set duty="));
        Serial.println(v);
        inputLen = 0;
      }
    } else if (inputLen < sizeof(inputBuf) - 1) {
      inputBuf[inputLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_PWM_PIN, OUTPUT);
  // 改用INPUT_PULLUP：有些霍爾編碼器輸出是open-collector，沒外接
  // 提升電阻的話，訊號在電晶體關閉時會懸空，读到的就是類似雜訊、
  // 在0/1間亂跳、不會穩定累加的樣子（目前看到的症狀很像）。
  pinMode(PIN_M1_A, INPUT_PULLUP);
  pinMode(PIN_M1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  setDirection(0);

  delay(500);
  Serial.println(F("=== Hall編碼器 + PCA9685馬達驅動 測試工具 ==="));
  Serial.println(F("輸入 -255~255 的整數 + Enter 設定duty"));
  Serial.println(F("（正=正轉、負=反轉、0=停止），會持續印出 tick/RPM。"));

  lastReportMs = millis();
}

void loop() {
  updateMotorPWM();
  pollSerialInput();

  uint32_t now = millis();
  if (now - lastReportMs < REPORT_PERIOD_MS) return;

  float dtSec = (now - lastReportMs) / 1000.0f;
  lastReportMs = now;

  int32_t p = posM1;
  int32_t d = p - lastPos;
  lastPos = p;
  float rpm = (d / TICKS_PER_REV) * (60.0f / dtSec);

  uint32_t eA = edgeCountA;
  uint32_t eB = edgeCountB;
  edgeCountA = 0;
  edgeCountB = 0;

  Serial.print(F("duty="));
  Serial.print(m1Duty);
  Serial.print(F("  tick="));
  Serial.print(p);
  Serial.print(F("  rpm="));
  Serial.print(rpm, 1);
  Serial.print(F("  edgeA="));
  Serial.print(eA);
  Serial.print(F("  edgeB="));
  Serial.print(eB);
  Serial.print(F("  lvlA="));
  Serial.print(digitalRead(PIN_M1_A));
  Serial.print(F("  lvlB="));
  Serial.println(digitalRead(PIN_M1_B));
}
