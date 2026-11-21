// 第二台車專用：速度閉環控制測試（架構沿用正式版 bmduino_car_control.ino
// 的 PID + feedforward，套用車2實測確認的腳位對應跟新校正常數）。
//
// 跟正式版car1的差異：
//   - PCA9685 I2C 走 Wire1(BMCOM1)，不是主 Wire(A4/A5)
//   - M1(左輪)：PCA9685 CH1/CH2(方向)、D8(轉速PWM)、D4/D5(編碼器)
//   - M2(右輪)：PCA9685 CH3/CH4(方向)、D9(轉速PWM)、D6/D7(編碼器)
//     （車2實測確認 D8/D9、編碼器腳位都跟車1相反，見
//     car2_motor_pwm_sweep_bidir.ino 與 bmduino_d8d9_identify_test.ino）
//   - FF_DEADBAND/FF_SLOPE/MAX_TICKS_PER_INTERVAL 改成車2實測 duty 0~250
//     完整正反轉掃描回歸出來的值（car2_sweep_duty200.csv 前身資料的完整版）
//   - 新增：即時車速換算（RPM -> m/s / km/h），輪徑 10cm，隨 STA 狀態一併印出
//
// 指令格式沿用 protocol.md：<CMD,fwa 0,bwl -60,bwr 60>
// bwl/bwr 語意跟正式版一樣：-100~100 是「目標速度百分比」，不是直接的
// PWM duty，韌體會用編碼器回授 + PID + feedforward 動態調整實際 duty。
//
// TICKS_PER_REV 已改成用馬達規格書(PK32K5SP284HE12T-027BBQ)算出來的正確值：
// 12 PPR 磁性編碼器(裝在馬達軸、減速箱之前) × 4倍正交解碼 × 減速比
// 3249/121，而不是沿用車1的1550估計值。這個常數只用在把原始ticks換算成
// 「人看得懂的RPM/車速」顯示用途，不影響PID控制迴路本身的收斂正確性——
// feedforward(FF_SLOPE/FF_DEADBAND)的校正常數是車2實測duty-vs-ticks回歸
// 直接量出來的，跟這個顯示用常數是分開的兩件事，見下方FF_CALIBRATION_
// TICKS_PER_REV的說明。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Orin走SPI1排針(Serial3)、PCA9685走BMCOM1(Wire1)，兩個各自用不同實體
// 接頭，不會衝突。這片HT32核心的Serial1/Serial3等非USB序列埠，單次
// write()送出較長訊息時有機率整包損毀(root cause分析見
// docs/architecture/uart_tx_corruption_bug.md)，已經用sendChunkedBM()
// 分段傳送繞開，不是Serial3特有的問題，換成Serial1一樣需要這個處理。
#define SerialBM Serial3

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

// ---- PCA9685 channel 配置 ----
const uint8_t CH_STEER = 0;
// 實測發現前進/後退方向反了，把 IN1/IN2 對調修正（不動PID/編碼器邏輯）
const uint8_t CH_M1_IN1 = 2;
const uint8_t CH_M1_IN2 = 1;
const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;

// ---- 馬達轉速 PWM（車2實測確認：D8=M1左輪、D9=M2右輪，跟車1相反）----
const uint8_t M1_PWM_PIN = 8;
const uint8_t M2_PWM_PIN = 9;

// ---- 編碼器輸入（車2實測確認：D4/D5=M1左輪；M2原本用D6/D7，但實測
// 發現D6/D7腳位本身有問題讀不到訊號，已改接到D10/D11，馬達端接線不變）----
const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

const uint32_t CMD_TIMEOUT_MS = 500;
const uint32_t STATUS_PERIOD_MS = 200;
// ENABLE_FAST_LOG：預設關閉，只在需要細看編碼器原始計數/duty時才臨時開啟
// (50Hz debug列印，只走USB不走BM)。
// 2026-09-15關掉：抓到FAST_LOG(10Hz)+sendStatus(5Hz)疊加的印字量會把USB
// 序列埠256-byte緩衝區灌爆，Serial.print()忙等阻塞導致updateMotorPWM()
// 整個loop()被卡住300~500ms，這才是低速時「間歇停頓」的主因之一，不是
// 馬達機械特性——同時把sendStatus()改成精簡CSV格式(見下方)進一步降低
// 印字量。需要細看時再臨時打開，平常維持關閉。
#define ENABLE_FAST_LOG 0
// 2026-09-11起：BM狀態回傳改成非阻塞（見下方ChunkedSender/pumpSend()），
// 不再需要ENABLE_BM_STATUS_SEND這個開關來避免PWM被delay(1)凍結——
// 舊的sendChunkedBM()每200ms連續呼叫3次delay(1)（阻塞約3ms）才是造成
// D8/D9 PWM訊號被凍結、馬達卡頓的原因，不是「送不送」本身的問題。
// 換成每次loop()最多送一個5-byte chunk、從不呼叫delay()之後，這個回傳
// 就可以安全地重新打開。
const uint32_t FAST_LOG_PERIOD_MS = 100;  // 這次改成10Hz，對齊要跟Orin端/cmd_vel
                                           // 同步比對的取樣頻率。平常排查用預設
                                           // 是20(50Hz)，刻意不跟著PID_INTERVAL_MS一起拉高到100Hz
                                          // ——已經驗證過debug列印頻率越高，USB指令
                                          // 接收就越不穩，這個是純診斷用途，維持
                                          // 50Hz、跟PID運算頻率脫鉤即可，只印在USB(不上BM)，
                                          // 專門用來看兩顆馬達編碼器原始計數/duty
                                          // 有沒有「一下有一下沒有」的瞬斷現象——
                                          // 這比STATUS_PERIOD_MS(200ms=5Hz)細10倍，
                                          // 才看得出來是真的電壓被拉塌，還是只是
                                          // PID正常在調整。

const uint32_t PWM_PERIOD_US = 200;  // 5kHz。這次單獨只改PWM頻率(PID/debug列印都
                                      // 沒動)，servo還是會抖，證實問題跟10kHz這個
                                      // 頻率本身有關，不是之前混在一起測時的其他
                                      // 變數。D8/D9(軟體PWM)邏輯上跟PCA9685(I2C)
                                      // 互不相干，但兩倍切換頻率的雜訊很可能透過
                                      // 車2手工接線的共地/電源耦合進PCA9685——這是
                                      // 電氣佈線層面的限制(去耦電容/走線分開)，
                                      // 光改韌體常數沒辦法解決，已退回5kHz。原廠
                                      // 參考韌體(hall_motor_pid_tune.ino)是乾淨的
                                      // 裸測試板，沒有車2實際佈線的耦合問題。

// ---- 編碼器/PID 校正常數（車2實測，duty 0~250 完整正反轉掃描回歸）----
// TICKS_PER_REV：12 PPR磁性編碼器(裝在馬達軸)×4倍正交解碼×減速比(3249/121)
// = 48 × 3249/121 ≈ 1288.86，取代原本沿用車1的1550估計值。這是「修正過的
// 正確值」，只用在把原始ticks換算成人看得懂的RPM/車速(見ticksPerIntervalToRPM)，
// 不影響下面PID/feedforward的校正常數。
const float TICKS_PER_REV = 1288.86;
const uint32_t PID_INTERVAL_MS = 20;  // 實測過拉到10(100Hz)，同時觀察到servo抖動、
                                       // USB指令也變不穩，已退回原本驗證穩定的50Hz。

// 2026-09-11重新校正：car2_openloop_duty_sweep_5khz_full.ino在車2實車、
// 正式版5kHz下直接量到完整duty(0~250)->輸出軸RPM連續斜坡曲線(見
// car2_5khz_duty_rpm_curve.png)，duty>=40的線性段回歸直接得到
// 「duty per RPM」的deadband/slope，不用再像舊版那樣繞一個「歷史
// FF_CALIBRATION_TICKS_PER_REV=1550錨點」才能算——舊版那個錨點只是
// 因為當初回歸用的RPM是拿還沒修正的1550算出來的，是換算過程的歷史
// 包袱，這次直接用現在的TICKS_PER_REV重新量、重新回歸，不需要了。
const float FF_DEADBAND_M1 = 13.26;
const float FF_SLOPE_M1 = 1.5118;  // duty per tick/PID週期(已換算)
const float MAX_TICKS_PER_INTERVAL_M1 = 146.83f * PID_INTERVAL_MS / 20.0f;  // 對應實測全速341.76 RPM

const float FF_DEADBAND_M2 = 13.39;
const float FF_SLOPE_M2 = 1.5099;
const float MAX_TICKS_PER_INTERVAL_M2 = 146.94f * PID_INTERVAL_MS / 20.0f;  // 對應實測全速342.03 RPM

// 實測發現中段目標(約20%)下，duty會爬到接近上限、位置卻同時平滑減速
//到完全靜止(見orin_short_test紀錄)，開迴路測試已證實同一duty區間馬達
// /供電/編碼器本身完全穩定不會斷——問題是PID本身的震盪讓馬達反覆
// 减速到低速區(可能因此更容易撞見機構上的卡點)。降KP減少對瞬間誤差
// 的過度反應、降KI減緩積分把duty往上衝的速度，兩者都是為了讓duty
// 收斂更平緩、不要每次誤差一放大就衝很兇。
// Kp改成0.25，對齊測試報告.md針對同一顆馬達型號(PK32K5SP284HE12T-027BBQ)
// 用網格搜尋(overshoot/穩態誤差/上升時間加權)驗證過的數值，比之前故障
// 排除時從0.5砍半得來的0.3更有根據。Ki/積分限幅/斜率限制原本就已經
// 跟報告一致(0.04/300/15)。
const float PID_KP = 0.25;
const float PID_KI = 0.04;

// 積分上限改成每顆馬達分開設定：M1 FF常數(正反轉不一致、平均值可能偏低)
// 導致追不到目標、積分頂到原本300的上限還不夠，先只加大M1的空間，
// M2 原本收斂得不錯，維持原本300不要跟著動。
const float PID_INTEGRAL_LIMIT_M1 = 300.0;  // 曲線改成線性、feedforward準確很多，積分上限先調回正常值
const float PID_INTEGRAL_LIMIT_M2 = 300.0;
// 2026-09-15從15.0調到6.0：原本15.0時，起步瞬間feedforward+PID理論值算
// 出來動輒90+，斜率限制只夠壓住「每週期最多爬多少」，理論值一直頂著
// 上限，結果大約120ms(6個週期)就衝到duty~90，實車起步感覺很猛、沒辦法
// 靜態輕柔起步。改成6.0後，同樣情境要花約300ms(15個週期)才爬到duty~90，
// 起步明顯平順很多；代價是一般行駛中加減速的反應也會變慢(不只是起步)，
// 如果覺得太鈍可以再往上調。換算成實際時間，原本15.0(20ms週期)是750
// duty/秒的斜率上限，6.0則是300 duty/秒；下面等比例配合PID_INTERVAL_MS
// 縮放的邏輯不變。
const float MAX_DUTY_STEP_PER_CYCLE = 6.0f * PID_INTERVAL_MS / 20.0f;

// ---- 反轉保護：驅動板（Handson Technology XY-160D，7A/160W，SKU
// DRV-1011，跟車1同款）規格頁明確要求：正轉->煞車(IN1=IN2=0)停留0.1秒
// 以上->反轉，不能在馬達還在轉的時候直接切換方向。車1實測時「鍵盤遙控
// w/s快速切換方向」曾經因為這個原因燒毀過一片驅動板，車2用同款驅動板，
// 直接沿用車1驗證過的保護邏輯。
//
// 只憑「這次命令方向跟上次命令方向是否直接翻轉」判斷不夠：如果上層先送
// 0（停止）再送反方向，這裡看到的是「先進入0再進入反方向」，不是直接
// 翻轉，但如果兩個命令間隔太短，馬達可能根本還沒真的停下來，一樣危險。
// 所以用「量到已經處於煞車狀態多久」而不是「有沒有經過0」來判斷是否放行，
// 兩種情況（直接翻轉 / 先0再反向但間隔太短）都會被同一套時間門檻擋住。
const uint32_t REVERSE_BRAKE_MS = 200;

// 2026-09-11移除整套啟動掙脫脈衝機制：car2_openloop_coldstart_test.ino
// 加上car2_openloop_duty_sweep_5khz_full.ino這兩次實測發現，先前判斷
// 「馬達靜摩擦門檻跳躍式、需要脈衝才能掙脫」是誤判——真正原因是車2
// 供電/驅動板電流餘裕不足(這顆馬達最大堵轉電流23A，驅動板XY-160D只有
// 7A額定)，冷啟動測試連續多次讓馬達在中高duty下堵轉超過7A額定持續
// 900ms，就是這樣才把驅動輸出打到完全沒反應。換句話說，先前觀察到的
// 「卡頓300ms~1s後突然暴衝」不是這顆馬達本身的靜摩擦特性，車1同款
// PID架構、乾淨測試板上量這顆馬達也完全不需要脈衝就能順利收斂(見
// hardware/motors/PK32K5SP284HE12T-027BBQ_控制資料/測試報告.md)。
// 車2實車5kHz連續斜坡重新測過(car2_5khz_duty_rpm_curve.png)，死區只在
// duty約35~40，比先前MIN_RUNNING_DUTY=85低很多，改回跟車1一樣單純的
// feedforward+PID+斜率限制，不再用脈衝/duty下限特殊處理，見下方
// updateMotorPID()。

// ---- 低速duty下限 + 死區：真正的補償方法，不是加大PID ----
// 拿掉脈衝後改用純feedforward+PID+斜率限制實測car2_speedcmd_test.py
// 發現：5%/10%目標(duty穩態落在~38~51)完全卡在「動一下停一下」的
// limit cycle裡，不管PID怎麼收斂都逃不掉——這是Stribeck摩擦特性造成
// 的物理缺口(duty<~38完全不轉、duty>~55才能持續穩定轉，中間沒有任何
// duty能讓實際轉速穩定停在這兩者之間)，線性PID沒辦法在不存在穩定平衡
// 點的地方收斂，加大Kp/Ki只會讓極限環震盪得更兇，不會消失。
//
// 2026-09-15：曾經試過佔空比平均(開一段時間、停一段時間交替，開的時候
// 用保證可靠運轉的duty，時間平均湊近低目標)，實測循環週期本身會有規律
// 的小抖動，使用者反應不能接受(車子的抖動比機械卡點的偶發頓挫更容易
// 感覺到)。改成更簡單的做法：死區(LOW_SPEED_DEADZONE_PCT)以下直接視為0，
// 死區以上直接進入連續PID+duty下限(MIN_RUN_DUTY)模式，「開始動」就是
// 「動得順」，不再有循環抖動的過渡帶。代價是死區到MIN_RUN_DUTY自然交叉
// 點這段速度會很接近，但完全平滑、沒有規律抖動。
const float MIN_RUN_DUTY = 60.0f;  // 車2實車5kHz掃描確認「絕對可靠、
                                    // duty>=60即可靠持續轉動」的值。
const float LOW_SPEED_DEADZONE_PCT = 5.0f;  // 低於這個目標%，直接視為0，
                                             // 不嘗試驅動。

// ---- 車速換算（輪徑10cm）----
const float WHEEL_DIAMETER_M = 0.10;
const float WHEEL_CIRCUMFERENCE_M = 3.14159265f * WHEEL_DIAMETER_M;  // ≈0.31416 m/圈

// 非阻塞分段傳送用的狀態結構，要放在所有函式之前，否則Arduino自動產生
// 的函式原型（startSend/pumpSend簽章用到這個型別）會插在這個定義之前，
// 導致「型別未宣告」的編譯錯誤。
struct ChunkedSender {
  char buf[80];
  uint8_t len;
  uint8_t pos;
  uint32_t nextSendMs;
  bool active;
};

struct PidState {
  float integral;
  int32_t lastPos;
  float filteredSpeed;
  int8_t lastDir;
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

int steerAngle = 0;
int leftSpeed = 0;
int rightSpeed = 0;
int lastRxSeq = -1;             // 最後一次checksum驗證通過的CMD seq，回傳給Orin當ackseq
uint32_t cmdChecksumFailCount = 0;  // 收到但checksum不對、被丟棄的CMD frame數
uint8_t m1DriverDuty = 0;
uint8_t m2DriverDuty = 0;
uint32_t lastCmdMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastPidMs = 0;
uint32_t lastFastLogMs = 0;

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

PidState pidM1State = { 0, 0, 0, 2, 0, 0 };
PidState pidM2State = { 0, 0, 0, 2, 0, 0 };

float lastActualM1 = 0;  // tick/PID週期，沿目標方向為正
float lastActualM2 = 0;

LinkState linkUSB = { &Serial, {0}, 0, false, 0 };
LinkState linkBM = { &SerialBM, {0}, 0, false, 0 };

// ---- 非阻塞分段傳送（取代舊的sendChunkedBM()裡的delay(1)忙等）----
// 每次pumpSend()最多送一個5-byte chunk就return，真正的段落間隔靠
// 「下次loop()再進來時millis()是否已經到nextSendMs」自然形成，不佔用
// 任何一次loop()呼叫超過一個write()的時間，PWM輸出(updateMotorPWM())
// 不會被凍結。ChunkedSender結構定義移到檔案前面，原因見那裡的註解。
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
  cs.nextSendMs = millis() + 1;  // 1ms間隔，沿用115200下已驗證過的保守值；
                                  // 460800下單byte時間更短，這個margin更寬鬆
  if (cs.pos >= cs.len) cs.active = false;
}

// ---- checksum：XOR8，算的範圍是frame內容「除了,cs欄位本身」的所有字元，
// 跟Jetson端uart_protocol.py的_checksum()演算法必須完全一致 ----
uint8_t xorChecksum(const char* s, size_t len) {
  uint8_t cs = 0;
  for (size_t i = 0; i < len; i++) cs ^= (uint8_t)s[i];
  return cs;
}

// 就地驗證frame的checksum、驗證通過的話把frame截短成「不含,cs欄位」的
// 版本讓後面的strtok照舊解析；驗證失敗回傳false，frame內容不能信任。
bool verifyAndStripChecksum(char* frame) {
  char* csPos = strstr(frame, ",cs ");
  if (csPos == nullptr) return false;
  size_t payloadLen = (size_t)(csPos - frame);
  uint8_t expected = (uint8_t)strtoul(csPos + 4, nullptr, 16);
  if (xorChecksum(frame, payloadLen) != expected) return false;
  frame[payloadLen] = '\0';
  return true;
}

void applyCommand(const char* key, int value) {
  if (strcmp(key, "seq") == 0) {
    lastRxSeq = value;
  } else if (strcmp(key, "fwa") == 0) {
    steerAngle = constrain(value, -60, 60);  // 這個session已實測±60度轉向機構正常無卡死
  } else if (strcmp(key, "bwl") == 0) {
    leftSpeed = constrain(value, -100, 100);
  } else if (strcmp(key, "bwr") == 0) {
    rightSpeed = constrain(value, -100, 100);
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
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 4095);  // 懸空(coast)，不要硬煞車，避免高轉速時停止產生大電流衝擊
  }
}

void driveDirectionCached(uint8_t in1Ch, uint8_t in2Ch, int dir, int8_t* lastDir) {
  if (*lastDir == dir) return;
  driveDirection(in1Ch, in2Ch, dir);
  *lastDir = (int8_t)dir;
}

void updateMotorPID(int targetPercent, volatile int32_t* pos, PidState* pid,
                     uint8_t* dutyOut, uint8_t in1Ch, uint8_t in2Ch, float* actualOut,
                     float ffDeadband, float ffSlope, float maxTicksPerInterval,
                     int encSign, float integralLimit) {
  int32_t currentPos = *pos;
  int32_t deltaTicks = currentPos - pid->lastPos;
  pid->lastPos = currentPos;

  uint32_t now = millis();

  // 死區：目標值(絕對值)低於LOW_SPEED_DEADZONE_PCT一律視為0處理——這段
  // 目標算出來的爬行速度跟死區以上沒有太大差異，卻要付出額外複雜度，
  // 不值得，見上方常數說明。
  bool inDeadzone = abs(targetPercent) < LOW_SPEED_DEADZONE_PCT;

  if (inDeadzone) {
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

  int effectivePercent = targetPercent;
  int dir = (targetPercent > 0) ? 1 : -1;

  // 反轉保護：不管上層是「直接送反方向」還是「先送0再送反方向」，
  // 只要跟上一次真正驅動過的方向相反，都必須先量到已經煞車滿
  // REVERSE_BRAKE_MS 才放行，不能只憑「有沒有經過0」判斷——PID只有
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

  driveDirectionCached(in1Ch, in2Ch, dir, &pid->lastDir);

  float rawActualSpeed = deltaTicks * dir * encSign;

  // 改回原廠測試馬達參考韌體(hall_motor_pid_tune.ino)用的0.3，PID_INTERVAL_MS
  // 目前固定20沒有再變動，不需要額外的比例換算；如果之後真的要調
  // PID_INTERVAL_MS，記得這裡的濾波時間常數(≈週期/ALPHA)也要跟著重新考慮。
  const float ALPHA = 0.3f;
  pid->filteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pid->filteredSpeed;
  float actualSpeed = pid->filteredSpeed;
  *actualOut = actualSpeed;

  // 2026-09-11改回車1的做法：target% 直接線性對應到目標轉速(ticks)，
  // 不再繞過MIN_RUNNING_DUTY/掙脫脈衝這些特殊處理——見上方常數區塊
  // 移除脈衝機制的說明，車2實測過的死區(duty~35~40)遠低於之前假設的
  // MIN_RUNNING_DUTY=85，單純feedforward+PID+斜率限制就夠用。用
  // effectivePercent(可能是佔空比平均後的on-phase目標)而不是原始
  // targetPercent，PID才會確實往可靠運轉的那個轉速收斂。
  float targetSpeed = (abs(effectivePercent) / 100.0f) * maxTicksPerInterval;
  float error = targetSpeed - actualSpeed;

  float feedforward = ffDeadband + ffSlope * targetSpeed;

  float tentativeOutput = feedforward + PID_KP * error + PID_KI * pid->integral;
  // 下限也算飽和：duty本來就會被夾在MIN_RUN_DUTY，如果PID算出來的值已經
  // 低於這個下限，積分再往下灌也不會讓輸出更低，只會在目標回升時要多花
  // 時間把灌爆的積分還債(windup)，所以跟原本0/255飽和判斷一樣處理。
  bool willSaturate = (tentativeOutput <= MIN_RUN_DUTY) || (tentativeOutput >= 255.0f);
  if (!willSaturate) {
    pid->integral = constrain(pid->integral + error, -integralLimit, integralLimit);
  }

  float output = feedforward + PID_KP * error + PID_KI * pid->integral;

  // 下限套在斜率限制之前，不是套在最終輸出上——這樣起步時(prevDuty=0)
  // 還是會照MAX_DUTY_STEP_PER_CYCLE慢慢爬升過去(不會一開機就跳到
  // MIN_RUN_DUTY)，只有等duty真的爬到/超過MIN_RUN_DUTY之後，才會被夾住
  // 不再往下掉回靜摩擦斷層。如果直接套在最終輸出，起步第一週期就會被
  // 瞬間拉到MIN_RUN_DUTY，等於把平緩起步的斜率限制完全繞過去。
  output = max(output, MIN_RUN_DUTY);

  float prevDuty = (float)(*dutyOut);
  float slewedOutput = constrain(output, prevDuty - MAX_DUTY_STEP_PER_CYCLE, prevDuty + MAX_DUTY_STEP_PER_CYCLE);

  *dutyOut = (uint8_t)constrain(slewedOutput, 0.0f, 255.0f);
}

void updateSteering() {
  uint16_t steerUs = map(steerAngle, -90, 90, STEER_MIN_US, STEER_MAX_US);
  pwm.writeMicroseconds(CH_STEER, steerUs);
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1DriverDuty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2DriverDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

// tick/PID週期 -> RPM -> m/s -> km/h
float ticksPerIntervalToRPM(float ticksPerInterval) {
  return (ticksPerInterval / TICKS_PER_REV) * (60000.0f / PID_INTERVAL_MS);
}
float rpmToMps(float rpm) {
  return (rpm / 60.0f) * WHEEL_CIRCUMFERENCE_M;
}

void handleFrame(char* frame) {
  if (!verifyAndStripChecksum(frame)) {
    cmdChecksumFailCount++;
    return;  // checksum不對，整包丟棄，不更新lastCmdMs/lastRxSeq——
             // 寧可讓CMD_TIMEOUT_MS的失效保護介入，也不要照著壞掉的值動作
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
}

void pollLink(LinkState& ls) {
  while (ls.port->available()) {
    char c = ls.port->read();
    ls.byteCount++;
    // 2026-09-11移除：這段逐byte hex dump(原本是查Jetson開頭遺失問題留下的)
    // 沒有開關保護、永遠執行，Serial(USB debug,115200)寫滿256-byte TX buffer後
    // HardwareSerial::write()會忙等阻塞(見core HardwareSerial.cpp)，BM線這次
    // 拉到460800後，一個frame(~35 byte)印出的debug量可以讓這裡卡住40~50ms，
    // 期間updateMotorPWM()(D8/D9軟體PWM，週期只有200us)完全沒被呼叫、PWM輸出
    // 凍結在原本電位——跟上面ChunkedSender要解決的是同一類「阻塞呼叫凍結PWM
    // 造成馬達卡頓」問題，只是這次藏在這段沒加開關的除錯print裡。低速時
    // duty的on-time佔比小，PWM被凍結幾十ms相對影響比高速時大很多，這很可能
    // 是「低速控制不順暢」的主因，所以直接拿掉，需要查byte遺失時再臨時加回來。
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

void sendStatus() {
  float rpmM1 = ticksPerIntervalToRPM(lastActualM1);
  float rpmM2 = ticksPerIntervalToRPM(lastActualM2);

  char payload[48];
  snprintf(payload, sizeof(payload), "STA,ackseq %d,S1 -1", lastRxSeq);
  uint8_t cs = xorChecksum(payload, strlen(payload));
  char line[64];
  snprintf(line, sizeof(line), "<%s,cs %02X>\n", payload, cs);

  Serial.print(line);        // USB debug保留原樣行為
  startSend(bmSender, line);  // 非阻塞排入BM傳送佇列，實際byte由loop()裡的
                               // pumpSend()逐次送出，這裡不blocking

  // 2026-09-15改成精簡CSV格式，取代原本一大串帶文字標籤的debug輸出——
  // 已經證實印字量會把USB序列埠緩衝區灌爆、造成Serial.print()忙等阻塞
  // 進而凍結PWM輸出。這幾行只有開發者自己在解析，不需要給人看的標籤，
  // 純數字CSV可以把字元數砍到約1/5~1/6，降低再次塞爆緩衝區的機率。
  // 欄位順序：dbg,targetL,targetR,dutyL,dutyR,rpmL,rpmR
  Serial.print(F("dbg,"));
  Serial.print(leftSpeed);
  Serial.print(',');
  Serial.print(rightSpeed);
  Serial.print(',');
  Serial.print(m1DriverDuty);
  Serial.print(',');
  Serial.print(m2DriverDuty);
  Serial.print(',');
  Serial.print(rpmM1, 1);
  Serial.print(',');
  Serial.println(rpmM2, 1);
}

void setup() {
  Serial.begin(115200);       // USB debug monitor，跟Jetson連線無關，維持115200
  SerialBM.begin(460800);  // 2026-09-11起車2專屬拉高到460800（僅車2，車1
                            // 維持115200不動），Jetson端uart_bridge_node的
                            // serial_baud參數要跟著同步改，兩邊沒對齊會變亂碼

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

  Serial.println(F("=== car2 speed control test ready ==="));
  Serial.println(F("send <CMD,fwa 0,bwl 50,bwr 50> style commands"));
}

void loop() {
  updateMotorPWM();
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
                   FF_DEADBAND_M1, FF_SLOPE_M1, MAX_TICKS_PER_INTERVAL_M1, 1, PID_INTEGRAL_LIMIT_M1);  // IN1/IN2對調(方向修正)後，encSign也要跟著翻面
    updateMotorPID(rightSpeed, &posM2, &pidM2State, &m2DriverDuty, CH_M2_IN1, CH_M2_IN2, &lastActualM2,
                   FF_DEADBAND_M2, FF_SLOPE_M2, MAX_TICKS_PER_INTERVAL_M2, -1, PID_INTEGRAL_LIMIT_M2);
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }

#if ENABLE_FAST_LOG
  if (now - lastFastLogMs >= FAST_LOG_PERIOD_MS) {
    lastFastLogMs = now;
    Serial.print(F("[enc] t="));
    Serial.print(now);
    Serial.print(F(" posM1="));
    Serial.print(posM1);
    Serial.print(F(" posM2="));
    Serial.print(posM2);
    Serial.print(F(" dutyM1="));
    Serial.print(m1DriverDuty);
    Serial.print(F(" dutyM2="));
    Serial.println(m2DriverDuty);
  }
#endif
}
