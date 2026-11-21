// BMduino 車輛控制韌體 — 對應 protocol.md v0.1 + wiring_plan.md
// 跟 bmduino_uart_test.ino（純通訊打通測試用）的差別：這份是實際會拿來
// 跑車的版本。
//
// 馬達控制架構（實測驗證過）：
//   方向腳（IN1~IN4）：直接由 PCA9685 輸出數位 0/1（pwm.setPin()）。
//     PCA9685 輸出高電位是吃 V+ 腳（目前設定 5V），不是吃 VCC 邏輯供電，
//     所以直接對應到驅動板要的 TTL 5V 準位，不需要額外緩衝，也不用
//     反相。
//   轉速腳（ENA/ENB）：BMduino 軟體 PWM（5kHz，見 updateMotorPWM()）
//     → 74LS14 雙級串接（兩個閘口反相兩次 = 正邏輯）→ 驅動板。
//     用雙級是因為 BMduino GPIO 直出只有 3.3V，吃不到驅動板 TTL 門檻，
//     74LS14 用 5V 供電整形成真正 5V；用兩級而非一級是為了讓電路上
//     直接是正邏輯，韌體不用再手動反相 duty。
//     （BMduino 內建 analogWrite() 實測在 D8/D9 上不可靠，改用
//     digitalWrite() + micros() 自己寫軟體 PWM。）
//   軟體 PWM 頻率：實測 loop() 單獨跑約 38kHz、加上編碼器中斷負載後
//     約 40.7kHz（中斷開銷不大），5kHz(200us) 週期時還有 ~7.6 次
//     loop 迭代可以判斷切換時機，是目前數據支持、風險可控的上限；
//     再往上到 8~10kHz 每週期迭代數會降到 4~5 次，duty 誤差會明顯
//     變大，不建議。
//
// 轉速閉環（新增）：兩顆馬達都裝了霍爾正交編碼器（A/B 相），四倍正交
// 解碼，搭配 PID 做速度閉環控制，取代原本「bwl/bwr 直接對應 PWM duty」
// 的開環做法。`bwl`/`bwr` 現在的語意是「目標速度百分比」，不是
// 「PWM duty百分比」，實際 duty 由 PID 根據編碼器回授的實際速度動態調整。
//
// 【2026-09-08 更新：PCA9685 改接 BMCOM1(Wire1)，編碼器腳位/接線全面
// 重新驗證過】車輛馬達（M1/M2，馳名電機 JGB37-520 DC12V 330RPM）換新、
// 驅動板也換過幾次，過程中發現舊驅動板兩顆馬達的正反轉出力嚴重不對稱
// （其中一個方向死區高達 duty≈237），換了健康的驅動板後才恢復正常對稱。
// 另外也發現 M1 編碼器接線本身接觸不良/雜訊（D4曾經時斷時通），改接到
// D2/D3 後才穩定；M2 編碼器一度發現正負接反，修正後才正常收斂。
// 這裡的常數是最終驗證過、兩顆馬達四個方向都能正常閉環收斂的狀態。
//
// PCA9685 channel 對應（走 Wire1/BMCOM1，不是主 Wire）：
//   CH0 → 前輪轉向 servo signal
//   CH1 → M1 驅動板 IN1（方向，數位 0/1）
//   CH2 → M1 驅動板 IN2（方向，數位 0/1）
//   CH3 → M2 驅動板 IN2(IN4)（方向，數位 0/1）
//   CH4 → M2 驅動板 IN1(IN3)（方向，數位 0/1）
// BMduino 腳位對應：
//   D9 → 74LS14(雙級) → M1 驅動板 ENA（轉速 PWM）
//   D8 → 74LS14(雙級) → M2 驅動板 ENB（轉速 PWM）
//   D2 → M1 編碼器 A相     D3 → M1 編碼器 B相
//   D6 → M2 編碼器 A相     D7 → M2 編碼器 B相
// 實際接線測試結果：協定裡的 M1（bwl）實際接的是「車體左輪」，
// M2（bwr）實際接的是「車體右輪」，跟命名直接對應（不是舊文件記錄的
// 對調狀態，換馬達/板子後這個對應可能有變，之後如果再動過硬體建議
// 重新視覺確認一次）。
//
// 74LS14 雙級輸入端（接 BMduino D8/D9 那一級）開機瞬間會浮接，建議
// 加上拉電阻到 5V，確保開機預設狀態安全（ENA/ENB 關閉）。
//
// 如果實際接線跟這裡假設的不同，改下面的 CH_* / M*_PWM_PIN / PIN_* 常數即可。
//
// 【2026-09-14 更新：加入 checksum（沿用車2的 XOR8 概念，見
// docs/architecture/protocol.md「車1專屬擴充」）】CMD/STA 都在封包末端加
// `,cs XX`（兩位大寫hex，`<`跟`>`之間、扣除cs欄位本身之所有字元逐byte
// XOR）。收到 CMD 時先驗證，不符就整包丟棄、不執行動作、也不更新
// lastCmdMs（等同沒收到），避免依據被雜訊改壞的數值執行動作。這裡只加
// checksum這一項，車2另外還有的 seq/ackseq/460800baud/分段傳送不在
// 這次更動範圍內。
//
// 【2026-09-14 再更新、2026-09-15修正：STA 加 unit/upms 欄位，讓Orin端有
// 粗略距離/位移信號可用】`<STA,S1 -1,unit 37,upms 12345,cs XX>`。
// unit 是 uint8_t 循環計數（0~255自動wrap），每累積 UNIT_THRESHOLD_TICKS
// 的位移就 +1，累加來源是獨立讀M1/M2原始encoder位置算出的絕對物理位移
// （不是PID的EMA濾波值，見 updateDistanceUnit()），整數運算、不用float，
// 避免長時間浮點累加誤差。**後退時unit會正確-1（uint8_t下溢自動變成255，
// 這是刻意設計、不是bug）**，但Orin端解析unit時不能用單純的「無號差值」
// (now-prev)&0xFF去累加，那樣後退會被誤判成快繞完一圈；正確做法是
// signed 8-bit差值（raw<128就是raw、否則raw-256）。upms 是開機以來的
// millis()，用來讓Orin端偵測BMduino是否重開機過（重開機後unit會歸零，
// 沒偵測到的話會被誤判成一次大幅移動）。細節見 unit 變數定義處註解跟
// docs/architecture/protocol.md。
//
// 【2026-09-15 再更新：STA改用非阻塞分段傳送，修正Orin端收不到回授的問題】
// 車1的STA封包一直都超過docs/architecture/uart_tx_corruption_bug.md記錄的
// 5byte安全門檻（連最早版本的`<STA,S1 -1>`都已經是12byte），只是這個bug
// 一直沒被抓到：之前Orin端沒有驗證STA內容，途中壞掉的封包被悄悄丟棄也沒人
// 注意到。現在STA變長（加了checksum/unit/upms）、Orin端又開始嚴格驗證
// checksum，「偶爾壞掉」變成「幾乎每包都被判定壞掉、完全收不到回授」。
// 沿用車2已驗證過的ChunkedSender/startSend()/pumpSend()做法：SerialBM
// 那條路改成每次loop()最多送一個5-byte chunk,不再一次整包send整個
// 凍住PWM輸出的風險視窗，見 pumpSend() 定義處註解。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 2026-09-08更新：BMCOM1(Serial1)現在給PCA9685(Wire1/I2C1)用，
// Jetson<->BMduino通訊改走SPI1排針(Serial3: TX3=D27/PA4, RX3=D28/PA5)。
#define SerialBM Serial3

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

// ---- PCA9685 channel 配置 ----
const uint8_t CH_STEER = 0;
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t CH_M2_IN1 = 4;  // 實接驅動板 IN3（CH3/CH4 接線跟 IN3/IN4 對調，這裡對調常數修正）
const uint8_t CH_M2_IN2 = 3;  // 實接驅動板 IN4

// ---- 馬達轉速 PWM（BMduino 軟體 PWM，經 74LS14 雙級正邏輯緩衝）----
const uint8_t M1_PWM_PIN = 9;  // D9 → 74LS14(雙級) → M1 ENA
const uint8_t M2_PWM_PIN = 8;  // D8 → 74LS14(雙級) → M2 ENB

// ---- 編碼器輸入（四倍正交解碼）----
const uint8_t PIN_M1_A = 2;  // M1 編碼器 A 相（原本D4接觸不良，改接到D2）
const uint8_t PIN_M1_B = 3;  // M1 編碼器 B 相（原本D5有雜訊，改接到D3）
const uint8_t PIN_M2_A = 6;
const uint8_t PIN_M2_B = 7;

// 實測 servo（TBSK20）datasheet：500~2500us 對應 0~180°，中立點 1500us
const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

const uint32_t CMD_TIMEOUT_MS = 500;    // 失效保護：超過這麼久沒收到CMD就煞車
const uint32_t STATUS_PERIOD_MS = 100;  // 狀態回報週期。找到PID誤差根因後改回正式值：
                                          // sendStatus() 的 Serial.print() 本身會讓 loop()
                                          // 卡住約1.3~1.4ms，卡住期間軟體PWM會失真，回報越
                                          // 頻繁失真越常發生，100ms 是目前驗證過夠用又影響小的值。

const uint8_t NUM_SENSORS = 1;  // 目前只有1個佔位感測器，之後要加TOF就把這個數字改大

const uint32_t PWM_PERIOD_US = 200;  // 200us = 5kHz

// ---- 編碼器/PID 校正常數 ----
// 一圈 tick 數：手動轉5圈量到 ~1882、驅動定值比對量到 ~1500~1650 之間，
// 目前先用 1550 這個折衷估計值；後來實測 M1 全速 332RPM 幾乎完全對上
// 官方規格 330RPM，算是對這個估計值的旁證。
const float TICKS_PER_REV = 1550.0;
const uint32_t PID_INTERVAL_MS = 20;  // PID 更新率 50Hz

// ---- 距離回報（STA的unit欄位，2026-09-14新增，2026-09-15改名+改整數累積）----
// 現階段不綁定實際跑道一圈的長度，直接拿「一輪轉一圈」當作unit的最小距離
// 單位（沿用已校正過的TICKS_PER_REV，不需要另外量測跑道），單純讓Orin端
// 有個粗略的距離/位移信號可以用（不是真正的圈數——真正第幾圈由Orin端用
// 視覺起終點標記計算，unit只提供防呆用的距離參考，故意不取名lap避免
// 誤會成圈數）。之後如果真的要換成別的距離單位，只要把這個常數換成量出來
// 的tick數即可，不影響下面的累加/wrap邏輯。
// 用int32_t（不是float）：避免長時間浮點累加誤差；累加來源是M1+M2兩輪
// 原始tick「相加」（不是平均，除法在整數運算裡會損失餘數），所以門檻也要
// 對應乘2，才能跟「兩輪平均每輪走TICKS_PER_UNIT」的物理意義一致。
const int32_t TICKS_PER_UNIT = (int32_t)TICKS_PER_REV;
const int32_t UNIT_THRESHOLD_TICKS = 2 * TICKS_PER_UNIT;

// ---- PWM duty -> RPM 實測校正（2026-09-09 重新掃描確認，同一顆馬達，
// 5kHz，R²全部>0.998，跟09-08數據幾乎一致（死區差異1~4，屬量測誤差），
// 這裡改用09-09這次更新鮮的數字，一樣正反轉平均：
//   M1：正轉死區38.17/斜率1.2286(maxRPM271.05)、反轉死區37.84/斜率1.2137(maxRPM267.83)
//   M2：正轉死區43.89/斜率1.4384(maxRPM309.17)、反轉死區45.02/斜率1.4934(maxRPM317.65)
const float FF_DEADBAND_M1 = 38.01;
const float FF_SLOPE_M1 = 1.5851;  // duty per (tick/PID週期)
const float MAX_TICKS_PER_INTERVAL_M1 = 139.21;

const float FF_DEADBAND_M2 = 44.46;
const float FF_SLOPE_M2 = 1.3205;
const float MAX_TICKS_PER_INTERVAL_M2 = 161.94;

// 加上 feedforward 之後，PID 只需要處理殘差（feedforward 沒抓準的部分），
// 增益可以比純 PID（無 feedforward）小很多，起始值仍需要實測微調：
const float PID_KP = 0.5;
const float PID_KI = 0.08;
const float PID_INTEGRAL_LIMIT = 300.0;  // anti-windup 限幅

// 目標速度階躍變化時（例如從0->30%），feedforward+Kp*error 算出來的
// duty 會直接跳到很高（例如140+），這個 duty 會維持好幾個PID週期直到
// 馬達+齒輪箱（有實際轉動慣量、不是瞬間響應）追上來，追上來的過程中
// 累積的角動量會讓實際轉速衝過頭（overshoot），衝過頭之後才被PID往回拉，
// 造成持續震盪。加上輸出斜率限制（slew rate limit）：duty 每個PID週期
// 最多只能變動這麼多，等於強制「軟啟動」，避免一次給太大的duty衝擊。
const float MAX_DUTY_STEP_PER_CYCLE = 15.0;

// ---- 反轉保護（2026-09-09新增）：驅動板是Handson Technology XY-160D
// （7A/160W，SKU DRV-1011），官方資料表的Arduino範例程式明確示範「正轉
// -> 煞車(IN1=IN2=0)停留100ms -> 反轉」，不是直接從正轉訊號跳反轉訊號。
// 鍵盤遙控測試時w/s快速切換方向燒毀驅動板，懷疑就是馬達還在轉動時被
// 瞬間反接、H-bridge承受過大反電動勢/瞬間電流造成的。這裡在韌體最底層
// （updateMotorPID，不管上層是CMD協定還是鍵盤）強制：只要新方向跟上一次
// 真正驅動過的方向相反，就必須先「量到」已經煞車滿 REVERSE_BRAKE_MS
// 才放行——不管上層是直接送反方向、還是先送0再送反方向，都用同一把
// 時間尺量，安全邊際比官方範例的100ms更大。
const uint32_t REVERSE_BRAKE_MS = 150;

// ---- 型別定義（要放在所有函式之前，避免 Arduino 自動產生的函式原型
// 在 struct 定義之前找不到型別）----
struct PidState {
  float integral;
  int32_t lastPos;
  float filteredSpeed;  // 實際速度的低通濾波值，降低編碼器雜訊（PID 快速切換
                         // duty 時觀察到耦合雜訊會讓 tick 數虛高）對 PID 的影響
  int8_t lastDir;  // 上次實際下達給 PCA9685 的方向（2=尚未設定過，強制第一次一定送）。
                   // driveDirection() 每次都無條件送 I2C（pwm.setPin()）的話，
                   // 就算方向沒變也照送，實測會讓 loop() 卡住達 ~2.6ms（遠超過
                   // 200us的軟體PWM週期），卡住期間PWM腳位輸出被凍結、失真，
                   // 導致實際平均duty比命令值高，這是先前PID運作時實際轉速比
                   // 靜態duty校正曲線量到的還快很多的根因。方向沒變就不用重送。
  int8_t lastNonZeroDir;  // 上一次「真正驅動過」的非零方向（0=還沒有過，開機預設）。
                          // 跟lastDir不同：lastDir在targetPercent==0時會被設成0，
                          // lastNonZeroDir會保留，才能在停止之後判斷「接下來這個
                          // 方向跟停止前是不是反的」。
  uint32_t zeroSinceMs;   // 最近一次從非零方向進入duty=0（煞車）狀態的時間點，
                          // 用來算「已經煞了多久」，不管是直接反轉觸發的煞車、
                          // 還是上層先送了個0再送反方向，一律用同一把尺量。
};

struct LinkState {
  Stream* port;
  char buf[80];
  uint8_t len;
  bool active;
  uint32_t byteCount;
};

// 非阻塞分段傳送用的狀態結構（2026-09-15新增，沿用車2已驗證過的做法，見
// docs/architecture/uart_tx_corruption_bug.md）。要放在所有函式之前，否則
// Arduino自動產生的函式原型（startSend/pumpSend簽章用到這個型別）會插在
// 這個定義之前，導致「型別未宣告」的編譯錯誤。
struct ChunkedSender {
  char buf[80];
  uint8_t len;
  uint8_t pos;
  uint32_t nextSendMs;
  bool active;
};

// ---- 目前狀態 ----
int steerAngle = 0;    // -90~90，0=直行、負=左轉、正=右轉
int leftSpeed = 0;     // -100~100，PID 目標速度百分比（不是直接的PWM duty）
int rightSpeed = 0;
uint8_t m1DriverDuty = 0;  // 驅動板端 ENA 目前的 duty（0~255），由 PID 動態調整
uint8_t m2DriverDuty = 0;  // 驅動板端 ENB 目前的 duty（0~255），由 PID 動態調整
uint32_t lastCmdMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastPidMs = 0;
uint32_t cmdChecksumFailCount = 0;  // 除錯用：checksum驗證失敗的CMD累計次數，隨STA debug line印出

// ---- 編碼器計數（四倍正交解碼，ISR 更新）----
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

// ---- PID 狀態 ----
PidState pidM1State = { 0, 0, 0, 2, 0, 0 };
PidState pidM2State = { 0, 0, 0, 2, 0, 0 };

// 除錯用：最近一次 PID 週期量到的實際速度（tick/週期，沿目標方向為正）
float lastActualM1 = 0;
float lastActualM2 = 0;

// ---- 距離回報：unit 是 uint8_t 循環計數（0~255 wrap），每累積
// UNIT_THRESHOLD_TICKS（=2*TICKS_PER_UNIT，兩輪原始tick相加的門檻）就
// +1；後退累積到反方向一樣多就 -1（uint8_t 溢位/下溢自動變成0/255，本身
// 就是正確的wrap，不是bug）。故意不叫lap：這不是真正的跑道圈數（真正第
// 幾圈由Orin端用視覺起終點標記算），只是一個固定距離單位，叫lap容易誤會。
//
// unitAccumTicks 是int32_t（不是float，避免長時間浮點累加誤差）累積的
// 餘數，帶正負號、前進為正。刻意不沿用PID的lastActualM1/M2（那是「相對
// 於目前指令方向」、經過EMA低通濾波、且targetPercent==0/煞車時會被強制
// 清成0的值，拿來做絕對距離累積會漏算煞車時的殘餘滑行），而是用
// unitLastPosM1/M2獨立記錄上次讀到的原始posM1/posM2，每次直接算tick差值
// 乘encSign（M1=-1、M2=+1，跟loop()呼叫updateMotorPID時用的值一致），
// 得到的是不受指令/PID狀態影響的絕對物理位移，累加兩輪（不取平均，整數
// 除法會損失餘數，所以門檻改乘2）比單輪更能代表車體實際移動距離。
//
// 【重要】Orin端解讀unit時不能用簡單的「無號差值」（(now-prev)&0xFF），
// 那個算法假設兩次讀取之間車子淨移動一定是前進，車子倒退時會被誤判成
// 快要繞完一圈。正確做法是signed 8-bit差值：
//   raw = (unit_now - unit_prev) & 0xFF
//   delta = raw if raw < 128 else raw - 256   # 映射到 -128~127
// 只要兩次STA之間淨移動不超過127個unit（幾乎必定成立），這樣才能正確
// 處理前進/後退兩種方向。
//
// 【重開機注意】unit在BMduino重開機時會回到0，STA另外加的upms欄位
// （開機以來的millis()，單位ms）就是給Orin端判斷「這包的開機時間比上一包
// 還短」，代表中間發生了重開機，這時候要重設自己的unit基準值，不能直接
// 拿新值跟舊值做signed-diff，否則會被誤判成一次大幅移動。
uint8_t unit = 0;
int32_t unitAccumTicks = 0;
int32_t unitLastPosM1 = 0;
int32_t unitLastPosM2 = 0;

// 每次 loop() 都呼叫（不依賴PID時序），獨立把M1/M2最新的原始encoder位置
// 換算成絕對物理位移、累加、跨過門檻就讓 unit +1/-1。
void updateDistanceUnit() {
  int32_t currentM1 = posM1;
  int32_t currentM2 = posM2;
  int32_t deltaM1 = currentM1 - unitLastPosM1;
  int32_t deltaM2 = currentM2 - unitLastPosM2;
  unitLastPosM1 = currentM1;
  unitLastPosM2 = currentM2;

  // encSign跟loop()呼叫updateMotorPID時用的值一致：M1=-1、M2=+1。
  unitAccumTicks += deltaM1 * (-1) + deltaM2 * (1);

  while (unitAccumTicks >= UNIT_THRESHOLD_TICKS) {
    unitAccumTicks -= UNIT_THRESHOLD_TICKS;
    unit++;
  }
  while (unitAccumTicks <= -UNIT_THRESHOLD_TICKS) {
    unitAccumTicks += UNIT_THRESHOLD_TICKS;
    unit--;
  }
}

// ---- 每個 serial 埠各自的收訊緩衝 ----
LinkState linkUSB = { &Serial, {0}, 0, false, 0 };
LinkState linkBM = { &SerialBM, {0}, 0, false, 0 };

// ---- 非阻塞分段傳送（2026-09-15新增，取代原本一次整包送出的
// SerialBM.print()）----
// 車1的STA封包一直都超過 docs/architecture/uart_tx_corruption_bug.md
// 記錄的5byte安全門檻（連最早的`<STA,S1 -1>`都已經是12byte），只是之前
// Orin端沒有驗證STA內容，壞掉的封包被悄悄丟棄、沒人注意到；現在STA
// 變長（加了checksum/unit/upms）、Orin端又開始嚴格驗證checksum，
// 「polling內容偶爾壞掉」變成「幾乎每包都被判定壞掉、完全收不到回授」。
// 這裡沿用車2已經實測驗證過的做法：每次pumpSend()最多送一個5-byte
// chunk就return，段落間隔靠「下次loop()進來時millis()是否已到
// nextSendMs」自然形成，不佔用任何一次loop()呼叫超過一個write()的時間，
// 軟體PWM(updateMotorPWM())不會被凍結。ChunkedSender結構定義移到檔案
// 前面，原因見那裡的註解。
ChunkedSender bmSender = { {0}, 0, 0, 0, false };

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
  cs.nextSendMs = millis() + 1;  // 1ms間隔，沿用車2在115200/460800下都已驗證過的保守值
  if (cs.pos >= cs.len) cs.active = false;
}

void applyCommand(const char* key, int value) {
  if (strcmp(key, "fwa") == 0) {
    steerAngle = constrain(value, -60, 60);  // 0=直行、負=左轉、正=右轉（2026-09-08測試放寬到±60，見README已知問題）
  } else if (strcmp(key, "bwl") == 0) {
    // 2026-09-08實測：正值指令實際會讓後輪往後轉，取負號修正成
    // 「正值=前進」，PID/encSign已驗證過不用動，只是這裡的方向語意相反。
    leftSpeed = -constrain(value, -100, 100);
  } else if (strcmp(key, "bwr") == 0) {
    rightSpeed = -constrain(value, -100, 100);
  }
}

void driveDirection(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 0);
  } else if (dir < 0) {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 4095);
  } else {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 0);  // 煞車
  }
}

// driveDirection() 每次呼叫都是兩次 I2C 交易（pwm.setPin() x2），實測
// 這樣的 I2C 交易加起來會讓 loop() 卡住約 1ms 以上，卡住期間軟體PWM
// 完全沒機會切換，嚴重失真。方向沒變其實不需要重送，這裡加一層快取，
// 只有方向真的改變（含第一次呼叫）才實際送 I2C。
void driveDirectionCached(uint8_t in1Ch, uint8_t in2Ch, int dir, int8_t* lastDir) {
  if (*lastDir == dir) return;
  driveDirection(in1Ch, in2Ch, dir);
  *lastDir = (int8_t)dir;
}

// PID 速度閉環（含 feedforward）：targetPercent 是 -100~100 的目標速度
// 百分比。用編碼器這個 PID 週期內累積的 tick 數（沿目標方向為正）當作
// 實際速度，跟目標速度比較算出誤差。
// feedforward（ffDeadband/ffSlope，來自實測 PWM->RPM 曲線的線性回歸）
// 直接根據目標速度算出「大概需要多少 duty」當基準，PID 只需要在這個
// 基準上做小幅修正（補 feedforward 沒抓準的部分），比純 PID 收斂快、
// 震盪小。targetPercent==0 時直接煞車並清空積分項，避免下次啟動時
// 積分飽和暴衝。
void updateMotorPID(int targetPercent, volatile int32_t* pos, PidState* pid,
                     uint8_t* dutyOut, uint8_t in1Ch, uint8_t in2Ch, float* actualOut,
                     float ffDeadband, float ffSlope, float maxTicksPerInterval,
                     int encSign) {
  int32_t currentPos = *pos;
  int32_t deltaTicks = currentPos - pid->lastPos;
  pid->lastPos = currentPos;

  uint32_t now = millis();

  if (targetPercent == 0) {
    if (pid->lastDir != 0) {
      pid->zeroSinceMs = now;  // 剛從有在轉變成煞車，記錄煞車起點
    }
    driveDirectionCached(in1Ch, in2Ch, 0, &pid->lastDir);
    *dutyOut = 0;
    pid->integral = 0;
    pid->filteredSpeed = 0;
    *actualOut = 0;
    return;
  }

  int dir = targetPercent > 0 ? 1 : -1;

  // 反轉保護：不管上層是「直接送反方向」還是「先送0再送反方向」，
  // 只要跟上一次真正驅動過的方向相反，都必須先量到已經煞車滿
  // REVERSE_BRAKE_MS 才放行，不能只憑「有沒有經過0」判斷——先前的版本
  // 只在偵測到「直接翻轉、完全沒經過0」時才強制煞夠時間，但PID只有
  // 50Hz，如果上層剛好在一個PID週期內就從正轉切到0又切到反轉，煞車
  // 時間可能只有幾ms，跟直接翻轉一樣危險，這裡統一用「量到的煞車
  // 時長」把兩種情況合併處理。
  if (pid->lastNonZeroDir != 0 && pid->lastNonZeroDir != dir) {
    if (pid->lastDir != 0) {
      // 還沒進入煞車狀態（這是直接翻轉的情況），先煞車、記錄起點，
      // 這個週期先不送新方向。
      pid->zeroSinceMs = now;
      driveDirectionCached(in1Ch, in2Ch, 0, &pid->lastDir);
      *dutyOut = 0;
      pid->integral = 0;
      pid->filteredSpeed = 0;
      *actualOut = 0;
      return;
    }
    if ((int32_t)(now - pid->zeroSinceMs) < (int32_t)REVERSE_BRAKE_MS) {
      // 已經在煞車，但還沒煞夠久，繼續維持煞車、不送新方向。
      *dutyOut = 0;
      *actualOut = 0;
      return;
    }
    // 煞車時間已經足夠，這個週期開始才允許送出新方向，往下走正常流程。
  }

  pid->lastNonZeroDir = dir;

  // encSign：修正編碼器計數方向跟馬達命令方向的正負號對應，兩顆馬達的
  // 編碼器A/B相接線方向不一樣，指令方向相同時量到的原始tick正負號會
  // 不同（PID算error要看正確符號，不然目標速度是正的但量到的
  // actualSpeed一直是負的，PID會拼命把duty往上衝，越衝越錯）。
  // 2026-09-08實測確認：M1傳encSign=-1、M2傳encSign=+1（見loop()呼叫處）。
  float rawActualSpeed = deltaTicks * dir * encSign;  // 沿著目標方向移動視為正值

  // 低通濾波（指數移動平均）：PID 快速切換 duty 時，編碼器線上觀察到
  // 耦合雜訊會讓瞬間 tick 數虛高，濾波後的值比較能反映真實趨勢。
  // ALPHA 越小濾波越強（但反應變慢），0.3 是折衷起始值。
  const float ALPHA = 0.3f;
  pid->filteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pid->filteredSpeed;
  float actualSpeed = pid->filteredSpeed;
  *actualOut = actualSpeed;

  float targetSpeed = (abs(targetPercent) / 100.0f) * maxTicksPerInterval;
  float error = targetSpeed - actualSpeed;

  float feedforward = ffDeadband + ffSlope * targetSpeed;

  // 位置式 PID：每次直接算出完整輸出（不是疊加在上次 duty 上），
  // feedforward 當基準、PID 項只做修正。
  // anti-windup：先用「還沒加上這次誤差」的積分算一次嘗試輸出，如果會
  // 飽和（超出0~255），就不要再把這次誤差疊加進積分，避免積分越滾越大、
  // 之後要花很久才能退回來。
  float tentativeOutput = feedforward + PID_KP * error + PID_KI * pid->integral;
  bool willSaturate = (tentativeOutput <= 0.0f) || (tentativeOutput >= 255.0f);
  if (!willSaturate) {
    pid->integral = constrain(pid->integral + error, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  }

  float output = feedforward + PID_KP * error + PID_KI * pid->integral;

  // 斜率限制：這次的duty不能跟上次差太多，強制平滑過渡，避免階躍指令
  // 直接把duty打到頂造成馬達轉動慣量帶來的overshoot。
  float prevDuty = (float)(*dutyOut);
  float slewedOutput = constrain(output, prevDuty - MAX_DUTY_STEP_PER_CYCLE, prevDuty + MAX_DUTY_STEP_PER_CYCLE);

  float newDuty = constrain(slewedOutput, 0.0f, 255.0f);
  *dutyOut = (uint8_t)newDuty;

  driveDirectionCached(in1Ch, in2Ch, dir, &pid->lastDir);
}

void updateSteering() {
  // steerAngle -90(左)~90(右) 對應 STEER_MIN_US~STEER_MAX_US。如果實際
  // 裝上車後左右方向跟預期相反，把 STEER_MIN_US/STEER_MAX_US 兩個值
  // 對調即可，不用改這行。
  uint16_t steerUs = map(steerAngle, -90, 90, STEER_MIN_US, STEER_MAX_US);
  pwm.writeMicroseconds(CH_STEER, steerUs);
}

// 軟體 PWM：每次 loop() 都呼叫，靠 micros() 算目前在週期內的相位，
// 決定 BMduino 端這一瞬間該輸出 HIGH 還是 LOW。雙級 74LS14 是正邏輯，
// 驅動板端想要 HIGH（duty 內）的時間，BMduino 端也輸出 HIGH。
void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1DriverDuty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2DriverDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

// 感測器讀值佔位，之後接 TOF 等模組就把這裡換成實際讀取。
// 回傳 -1 代表這個感測器目前無效/未接。
int16_t readSensorMM(uint8_t index) {
  (void)index;
  return -1;
}

// 計算 checksum：將 s[0..len-1] 逐字元進行 XOR 運算（跟車2韌體同一套演算法）。
uint8_t xorChecksum(const char* s, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= (uint8_t)s[i];
  return cs;
}

// 驗證 CMD 末端的 ,cs XX 欄位，驗證通過就把frame在cs欄位前截斷（方便後面
// strtok繼續解析），驗證失敗回傳false，呼叫端要整包丟棄、不執行任何動作。
bool verifyAndStripChecksum(char* frame) {
  char* csPos = strstr(frame, ",cs ");
  if (csPos == nullptr) return false;
  size_t payloadLen = (size_t)(csPos - frame);
  uint8_t expected = (uint8_t)strtoul(csPos + 4, nullptr, 16);
  if (xorChecksum(frame, payloadLen) != expected) return false;
  frame[payloadLen] = '\0';
  return true;
}

void handleFrame(char* frame) {
  if (!verifyAndStripChecksum(frame)) {
    cmdChecksumFailCount++;
    return;
  }

  char* tok = strtok(frame, ",");
  if (tok == nullptr || strcmp(tok, "CMD") != 0) return;

  while ((tok = strtok(nullptr, ",")) != nullptr) {
    char key[8];
    int value;
    if (sscanf(tok, "%7s %d", key, &value) == 2) {
      applyCommand(key, value);
    }
  }

  lastCmdMs = millis();
  updateSteering();
  // 馬達目標速度（leftSpeed/rightSpeed）交給 loop() 裡的週期性 PID 處理，
  // 這裡不用馬上驅動，PID 最慢 PID_INTERVAL_MS 內就會跟上新目標。
}

void pollLink(LinkState& ls) {
  while (ls.port->available()) {
    char c = ls.port->read();
    ls.byteCount++;
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
      // 緩衝溢位，放棄這個封包重新同步
      ls.active = false;
      ls.len = 0;
    }
  }
}

void sendStatus() {
  char payload[160];
  int n = snprintf(payload, sizeof(payload), "STA");
  for (uint8_t i = 1; i <= NUM_SENSORS; i++) {
    int16_t d = readSensorMM(i);
    n += snprintf(payload + n, sizeof(payload) - n, ",S%u %d", i, d);
  }
  n += snprintf(payload + n, sizeof(payload) - n, ",unit %u,upms %lu", unit, (unsigned long)millis());
  uint8_t cs = xorChecksum(payload, strlen(payload));

  char line[176];
  snprintf(line, sizeof(line), "<%s,cs %02X>\n", payload, cs);

  Serial.print(line);        // USB debug保留原樣行為，USB不受Serial1/Serial3那個bug影響
  startSend(bmSender, line);  // 非阻塞排入BM傳送佇列，實際byte由loop()裡的
                               // pumpSend()逐次送出，這裡不blocking

  Serial.print(F("[enc] posM1="));
  Serial.print(posM1);
  Serial.print(F(" posM2="));
  Serial.print(posM2);
  Serial.print(F(" dutyM1="));
  Serial.print(m1DriverDuty);
  Serial.print(F(" dutyM2="));
  Serial.print(m2DriverDuty);
  Serial.print(F(" unit="));
  Serial.print(unit);
  Serial.print(F(" cmdCsFail="));
  Serial.println(cmdChecksumFailCount);
}

void setup() {
  Serial.begin(115200);
  SerialBM.begin(115200);

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

  lastCmdMs = millis();
  lastPidMs = millis();
  updateSteering();
  driveDirection(CH_M1_IN1, CH_M1_IN2, 0);
  driveDirection(CH_M2_IN1, CH_M2_IN2, 0);
}

void loop() {
  updateMotorPWM();
  updateDistanceUnit();
  pumpSend(bmSender, SerialBM);  // 非阻塞：多數次呼叫直接return，真正要送的
                                  // 那一刻也只送5byte、不delay，不影響上面的PWM

  pollLink(linkUSB);
  pollLink(linkBM);

  uint32_t now = millis();

  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  if (now - lastPidMs >= PID_INTERVAL_MS) {
    lastPidMs = now;
    updateMotorPID(leftSpeed, &posM1, &pidM1State, &m1DriverDuty, CH_M1_IN1, CH_M1_IN2, &lastActualM1,
                   FF_DEADBAND_M1, FF_SLOPE_M1, MAX_TICKS_PER_INTERVAL_M1, -1);
    updateMotorPID(rightSpeed, &posM2, &pidM2State, &m2DriverDuty, CH_M2_IN1, CH_M2_IN2, &lastActualM2,
                   FF_DEADBAND_M2, FF_SLOPE_M2, MAX_TICKS_PER_INTERVAL_M2, 1);
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }
}
