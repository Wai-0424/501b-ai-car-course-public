// 車2主控韌體（正式版）——速度閉環控制：編碼器回授 + 查表前饋(FF_LUT) + PID + 變化率限制。
// 架構沿用車1正式版 bmduino_car_control.ino，腳位對應與校正數據是車2實測的。
//
// 硬體（現行：L298N 驅動板）：
//   - PCA9685 走 Wire1(BMCOM1)：CH0 方向盤、CH1~CH4 馬達方向腳
//   - M1(左輪)：PCA9685 CH3/CH4(方向)、D9(轉速 PWM)、D4/D5(編碼器)
//   - M2(右輪)：PCA9685 CH1/CH2(方向)、D8(轉速 PWM)、D10/D11(編碼器)
//     （換成 L298N 後馬達線與編碼器的對應是交叉的，沒有餘裕重接，改在韌體對調並重新確認極性；
//      M2 編碼器原本接 D6/D7，該腳位讀不到訊號，已改接 D10/D11）
//   - D8/D9 是硬體 PWM(analogWrite，5 kHz)；Orin 走 Serial3(460800)，與 PCA9685 各用不同接頭
//
// 車2專屬的處理機制（車1的馬達沒有這些問題，見 updateMotorPID() 與各常數說明）：
//   * kick-start：從靜止起步先打一個短脈衝掙脫靜摩擦
//   * MIN_RUN_DUTY：已經在轉時 duty 不低於維持下限
//   * 0~100% 外部油門線性映射到內部目標，避免油門行程前段沒反應
//   * 堵轉/編碼器失效切斷、PCA9685 掉線時方向腳互鎖、反轉前強制煞車
//   * turbo：短時間超過 100%（限時，未實測）
//
// 指令格式沿用 protocol.md：<CMD,fwa 0,bwl -60,bwr 60>
// bwl/bwr 是「目標速度百分比」(-100~100，turbo 到 125)，不是 PWM duty，
// 韌體用編碼器回授 + PID + 前饋動態調整實際 duty。
//
// TICKS_PER_REV 用馬達規格書(PK32K5SP284HE12T-027BBQ)算：12 PPR 編碼器(馬達軸、減速箱之前)
// × 4 倍正交解碼 × 減速比 3249/121。它只用來把 ticks 換算成 RPM 顯示，不影響控制迴路；
// 前饋表是直接量 duty-vs-ticks 得到的，與這個常數無關。
//
// 這份檔案升格自 experiments/car2_speed_control_test_wip/。各常數註解保留「量到了什麼、
// 為什麼這樣選」；已被推翻或不再適用的歷史做法不留，要查看 git 歷史與同資料夾的 .bak 備份。

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

// ---- 2026-10-02新增：PCA9685健康狀態與方向腳I2C互鎖 ----
// PCA9685同時負責方向盤(CH0)跟馬達方向腳(CH1~CH4)。之前開機時它沒有電，
// 初始化(pwm.begin/setPWMFreq/readPrescale)全部失敗卻沒人知道，而且方向腳用的
// pwm.setPin()是void、I2C寫入結果被丟掉，方向沒設成功時PWM照樣輸出。改成：
//   * 方向腳改用setPWM()(回傳I2C結果)，寫入失敗不更新方向快取、下個週期重試；
//   * 方向沒設成功(或PCA9685掉線)時，該輪duty一律0——絕不在方向不明時輸出PWM；
//   * 開機沒回應不卡死；健康時完全不碰I2C(沒有週期性探測——每次探測都是一筆I2C交易，
//     匯流排異常時會被Wire的逾時拖住整個loop())，只有「已經不健康」時才每
//     PCA_RETRY_PERIOD_MS重試一次初始化；運作中掉線由方向腳/方向盤寫入失敗(互鎖)或
//     堵轉切斷(編碼器沒動)發現，不靠探測；
//   * STA的f欄位bit2(=4)回報「PCA9685/方向腳I2C故障中」。
bool pcaReady = false;
uint32_t pcaNextCheckMs = 0;
uint32_t dirI2cCallCount = 0;
uint32_t dirI2cErrorCount = 0;
const uint8_t PCA9685_ADDR = 0x40;
const uint32_t PCA_RETRY_PERIOD_MS = 1000;  // 只在pcaReady==false時才重試；此時馬達本來就被互鎖成duty 0，
                                             // 就算這次I2C被拖住也不影響馬達控制

// ---- PCA9685 channel 配置 ----
const uint8_t CH_STEER = 0;
// 實測發現前進/後退方向反了，把 IN1/IN2 對調修正（不動PID/編碼器邏輯）
// 2026-10-02 夜：換成 L298N 後，馬達電源線在驅動板上跟編碼器的對應是交叉的，線沒有餘裕再換，
// 改在韌體對調並重新確認極性(兩顆都量過「前進為正」且兩輪實際都往前滾)：
//   M1(左，編碼器 D4/D5)  -> D9 + PCA CH3(高)/CH4(低)=正轉
//   M2(右，編碼器 D10/D11) -> D8 + PCA CH1(高)/CH2(低)=正轉
const uint8_t CH_M1_IN1 = 3;
const uint8_t CH_M1_IN2 = 4;
const uint8_t CH_M2_IN1 = 1;
const uint8_t CH_M2_IN2 = 2;

// ---- 馬達轉速 PWM（硬體 PWM；M1 左輪=D9、M2 右輪=D8，與車1相反，原因見上面方向腳的說明）----
const uint8_t M1_PWM_PIN = 9;   // 見上面的重新對應說明
const uint8_t M2_PWM_PIN = 8;

// ---- 車燈：WS2812 可定址 LED，資料線接 D13 (=PC11) ----
// 2026-10-14 新增。原本想用 FastLED，但 FastLED 3.10.7 不支援 HT32(編譯失敗：
// "Platform does not have a clockless controller defined")，所以自己寫一個最小的
// WS2812 bit-bang 驅動(ledShow)，時序用直接寫 GPIO 暫存器 + 忙等迴圈(校準方式見 ledShow 上方說明)。
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

// ---- 編碼器輸入（車2實測確認：D4/D5=M1左輪；M2原本用D6/D7，但實測
// 發現D6/D7腳位本身有問題讀不到訊號，已改接到D10/D11，馬達端接線不變）----
const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

const uint32_t CMD_TIMEOUT_MS = 500;
// STA 回報 20 Hz（50 ms）。馬達 PWM 是硬體輸出，不受 loop() 速度影響，sendStatus() 單次耗時(1~3 ms)不再凍結馬達。
// 實測 CMD 60 Hz + STA 20 Hz、馬達轉動下(輪子架空)：10 分鐘 36,005 個 CMD 零遺失、傳送佇列零重疊、
// loop() 相鄰兩次起點間隔從未超過 5 ms、STA 送達 99.4~100%；STA 損毀約 0.4% 只在馬達轉動時出現
// (單字元被換掉，全被 checksum 擋下，成因未確認)。見 investigations/2026-10-sta20-test/README.md。
// USB 的 STA+dbg 合併行每 200 ms 一次(USB_DBG_DIV)。Orin 端的消費者(imu_yaw_node、
// line_follow_control_node)都用時間差積分與 0.8~1.0 s 的新鮮度逾時，沒有假設固定的 5 Hz。
const uint32_t STATUS_PERIOD_MS = 50;

// ENABLE_FAST_LOG：逐週期編碼器/duty 的高頻 USB 列印，預設關閉，只在需要細看瞬斷時臨時開。
// 教訓：高頻 Serial.print() 會灌爆 USB 的 256-byte TX 緩衝，write() 忙等會卡住整個 loop()
// (曾是低速間歇停頓的真正原因，不是馬達機械特性)。開啟時只走 USB、不上 BM，且不要拉太高。
#define ENABLE_FAST_LOG 0
const uint32_t FAST_LOG_PERIOD_MS = 100;

// ---- 馬達 PWM（D8/D9，硬體 PWM）----
// 由計時器自己輸出，不受 loop() 速度影響。頻率是全域設定(analogWriteFrequency)。
// 前饋表(FF_LUT_*)是在 5 kHz 下量的，改頻率就要重量(L298N 時 1 kHz 比 5 kHz 同 duty 快約 40%)。
// D8=PWM0 CH0、D9=MCTM0 CH0N(互補通道)；輪子架空的獨立測試中兩者轉速都與舊軟體 PWM 一致(99.9%/98.5%)、
// CCR 暫存器吻合、D9 極性沒反，見 investigations/2026-10-hw-pwm-test/README.md。
// (早期 wiring_plan.md 記載過 analogWrite 在 D8/D9「完整韌體流程中沒動作」，根因未查出、
// 獨立測試沒重現——完整韌體上要再驗證，不要只信獨立測試。)
// 提高頻率的注意：軟體 PWM 時代 10 kHz 曾讓 servo(PCA9685)異常，研判是切換雜訊經手工接線的
// 共地/電源耦合進 PCA9685，屬電氣佈線問題，不是韌體常數能解；硬體 PWM 提頻時要留意。
const uint32_t PWM_FREQ_HZ = 5000;

// ---- 編碼器/PID 校正常數 ----
// TICKS_PER_REV：12 PPR 磁性編碼器(裝在馬達軸)×4 倍正交解碼×減速比(3249/121) = 48 × 3249/121 ≈ 1288.86。
// 只用在把原始 ticks 換算成 RPM(見 ticksPerIntervalToRPM)，不影響下面 PID/前饋的校正常數。
const float TICKS_PER_REV = 1288.86;
const uint32_t PID_INTERVAL_MS = 20;  // 實測過拉到 10(100 Hz)，servo 抖動、USB 指令也變不穩，維持 50 Hz。
// ---- 前饋查表：目標 ticks/20 ms -> 需要的 duty（表內線性內插）----
// L298N 的 duty->rpm 曲線是凹的(低速每加 1 duty 約 +2.4 rpm、高速約 +1.0 rpm)且有大遲滯，單一直線在
// 低/高兩端都會差 20 rpm 左右，所以用查表。ticks 單位是「每 PID 週期(20 ms)」。
// 資料：輪子架空、約 14 V、硬體 PWM 5 kHz，用 car2_hwpwm_openloop 韌體 + calibrate_hw.py，kick 到 255 後
// 由高往低掃(維持曲線)，每個 duty 保持 1.6 s、取最後 0.7 s，三次 pass 平均(pass 間最大差 3.3 rpm)，
// 保序回歸後取格點；見 investigations/2026-10-hw-pwm-test/ 的 results/hwcal_sustain_20261010_pass*.csv
// 與 host_tools/build_lut.py。
// 這張表和舊軟體 PWM 時期的表差很多(同 duty 慢 12~16 rpm，200~220 duty 差 25~40 rpm)：舊表在軟體 PWM 下校正，
// 軟體 PWM 的有效 duty 比標稱值大；新表和 10/05 診斷韌體(軟體 PWM、loop 很短)的開迴路曲線相符(差 1~4 rpm)，
// 才是真實的 duty->rpm。duty 200~220 轉速幾乎持平(約 243 rpm)再往上又陡升，12 V 的舊資料也有同樣形狀，
// 是這套馬達/驅動板的特性；格點 210 因為在平台區裡重複所以沒放。
// 維持下限：左輪約 duty 64(12.6 rpm)、右輪約 60(9 rpm)，再低就停住——MIN_RUN_DUTY 因此是 62，
// 1% 指令(約 16 rpm)的前饋落在 L 65 / R 63，仍在連續模式。起轉門檻(往上掃，每步 0.5 s)：L 85、R 90。
// 上地面有負載後曲線會右移，最終整定要在實際負載下重新量；換驅動板、換 PWM 頻率或供電電壓時整張表都要重量。
// 舊表(軟體 PWM)備份：car2_car_control.ino.v9_hwpwm_oldlut_20261010.bak / .v8_softpwm_20261010.bak。
const int FF_LUT_N = 27;
const float FF_LUT_DUTY[FF_LUT_N] = {64.0f, 66.0f, 68.0f, 70.0f, 72.0f, 74.0f, 76.0f, 78.0f, 80.0f, 85.0f, 90.0f, 95.0f, 100.0f, 110.0f, 120.0f, 130.0f, 140.0f, 150.0f, 160.0f, 170.0f, 180.0f, 190.0f, 200.0f, 220.0f, 230.0f, 240.0f, 255.0f};
const float FF_LUT_TICKS_M1[FF_LUT_N] = {5.35f, 7.64f, 9.98f, 12.14f, 14.67f, 17.46f, 20.22f, 22.55f, 25.19f, 31.17f, 36.61f, 41.79f, 46.88f, 55.27f, 63.04f, 70.66f, 77.40f, 82.76f, 86.48f, 90.39f, 96.15f, 101.87f, 104.28f, 104.50f, 108.11f, 117.40f, 128.98f};
const float FF_LUT_TICKS_M2[FF_LUT_N] = {8.10f, 11.00f, 14.12f, 16.85f, 19.45f, 21.81f, 24.35f, 26.70f, 29.40f, 35.16f, 40.59f, 45.67f, 50.70f, 59.01f, 66.55f, 73.72f, 80.51f, 85.86f, 89.37f, 92.93f, 98.34f, 104.26f, 106.80f, 106.90f, 110.43f, 119.64f, 131.57f};
// 100% 外部指令 = 116 ticks/20 ms ≈ 270 rpm，需要 duty 約 236~238，距離 255 餘裕約 17~19(約 7%)，留給 PID/負載。
const float MAX_TICKS_PER_INTERVAL_M1 = 116.0f * PID_INTERVAL_MS / 20.0f;
const float MAX_TICKS_PER_INTERVAL_M2 = 116.0f * PID_INTERVAL_MS / 20.0f;

// ---- 2026-10-10：Turbo 模式（短時間超過 100% = 270 rpm）【未實測：寫的時候 BMduino 沒接上】----
// 目的：車2要當「魔王車」，需要短時間衝得比 100%（270 rpm）更快。100% 指令不變；bwl/bwr 在
// 101~TURBO_CMD_MAX_PCT 之間才是 turbo，目標轉速線性延伸（經 CMD_MAP_FLOOR_PCT 映射後每 +1 約 +2.44 rpm），
// 但被前饋表頂端限制（見 TURBO_FF_DUTY_CEILING）：依現行前饋表內插，duty 250 對應 M1 約 291 rpm、
// M2 約 297 rpm，指令約 109 起就頂到上限，所以現表下到不了 300 rpm；要更快得提高上限，或在更高電壓下重量表。
// 預設行為不變：Orin 沒送 >100 的指令（uart_bridge_node 的 max_wheel_speed 目前夾在 100）就完全不會進入 turbo。
// 上板後要驗證的事（電壓、前饋表頂端、電流、溫度、時間限制）見 investigations/2026-10-turbo-14v5/README.md。
//
// 韌體端的保護（不能只靠 Orin 自律）：
//   * 只有正轉（前進）允許 turbo；後退一律 <= 100。
//   * 預算制：連續 turbo 最多 TURBO_MAX_MS；用完就鎖定（>100 的命令一律被夾回 100），要「不要求 turbo」
//     累計到預算回到 TURBO_RESUME_MS 才解除（從 0 充滿要 TURBO_RECHARGE_MS）。鎖定中如果 Orin 還一直要求
//     turbo，不充電也不放行——所以即使上層卡在 112，最多只有開機後那一次 TURBO_MAX_MS 的 turbo，之後一直是
//     100%，直到它降回 <= 100。
//   * turbo 區間的目標轉速上限 = 前饋表在 duty TURBO_FF_DUTY_CEILING 時的轉速，保留 PID 往上修正的 duty
//     餘裕，不會要求「全開才勉強到」的速度。換電壓後前饋表要重量，這個上限會跟著表走。
//   * 其餘保護（堵轉切斷、反轉保護、指令逾時）完全沿用。
const int TURBO_CMD_MAX_PCT = 125;               // 命令值上限（125 約 334 rpm，實際被下面的 duty 上限擋住）
const uint32_t TURBO_MAX_MS = 3000;              // 預算：連續 turbo 最多這麼久
const uint32_t TURBO_RECHARGE_MS = 10000;        // 預算從 0 充滿需要的「不要求 turbo」時間
const uint32_t TURBO_RESUME_MS = TURBO_MAX_MS / 2;  // 鎖定後預算回到這麼多才再放行
const float TURBO_FF_DUTY_CEILING = 250.0f;      // turbo 目標的前饋 duty 上限（255 是硬上限，留 5 給 PID）
uint32_t turboBudgetMs = TURBO_MAX_MS;
bool turboLocked = false;
bool turboActive = false;                         // 這個 PID 週期有沒有放行 turbo（給 dbg 用）



// ---- PID ----
// 2026-10-05 在 14 V/L298N 重新校正後，用車用韌體做步階測試(30->60->30->100->50->15->0，輪子架空，
// results/l298n_14v_pid_*)比較 KP 0.6/1.0/1.5/2.0/3.0：2.0 過衝 1~6%、欠衝 2~6%、整定 0.4~1.6 s 最平衡；
// 3.0 在 30% 起步衝 +18% 且 15% 右輪整定不了；0.6 變差。KI 0.08 過衝變大(13%)，維持 0.04。
// 2026-10-10 硬體 PWM + 新前饋表後再調(輪子架空，50 Hz 遙測、同一個 40 秒階梯計畫比較 26 個變體，
// 見 investigations/2026-10-hw-pwm-test/ 的 results/tune_r{1,2,3}_*.tsv 與 README)：
//   * KP 2.0、KI 0.04 維持不變：KP 1.0 太慢且穩態誤差 3%、KP 3.0 過衝 10.7%；KI 0.08 過衝 17%、
//     KI 0.01~0.02 在前饋偏移 10 duty(模擬負載)時修正太慢(穩態誤差 5.6%)。
//     (模擬：前饋準、馬達 tau 約 0.3 s 時 KI 越大超調越大，0.08 時 30->60 階躍超調 12.8%、0.04 時 7.1%)
//   * 變化率限制 6 -> 12 duty/20 ms：上升時間 0.36 -> 0.29 s、積分誤差 0.26 -> 0.19，過衝沒有變大。
//     起步的平順度現在由 kick 負責。加大限速也加大加速時的瞬間電流(L298N 無過流保護)，電流沒有量測；
//     上地面後如果驅動板或電源有異常發熱/跳電，先把這個值調回 6~8。
//   * 限速器正在限制輸出時暫停積分(見 updateMotorPID 的 slewLimited)：避免大階梯期間積分一直累積。
//   * 試過但不採用「積分分離」(只在誤差小時積分)：前饋偏差大時積分永遠不累積，會留下 P 項的穩態誤差。
//   * 100% 指令的大階梯(90->270 rpm)仍有約 7%(+19 rpm)過衝：P 項把 duty 推到 255 飽和、馬達約
//     0.3 s 的機械延遲讓速度衝過頭，不是積分造成的，KP/KI/限速都救不了，先接受。
// 上地面有負載後要重測。
const float PID_KP = 2.0;
const float PID_KI = 0.04;
const float PID_INTEGRAL_LIMIT_M1 = 300.0;
const float PID_INTEGRAL_LIMIT_M2 = 300.0;
const float MAX_DUTY_STEP_PER_CYCLE = 12.0f * PID_INTERVAL_MS / 20.0f;  // 每個 PID 週期 duty 最多變多少

// ---- 反轉保護 ----
// 驅動板不能在馬達還在轉的時候直接切換方向：正轉 -> 煞車(兩條方向腳同電位)停留一段時間 -> 反轉。
// 車1鍵盤遙控 w/s 快速切換方向曾因此燒毀一片驅動板(XY-160D)。
// 只憑「這次命令方向跟上次是否直接翻轉」判斷不夠：上層先送 0 再送反方向、但間隔太短時，馬達可能根本
// 還沒停下來。所以用「已經處於煞車狀態多久」判斷是否放行，直接翻轉與「先 0 再反向但間隔太短」
// 被同一套時間門檻擋住。
const uint32_t REVERSE_BRAKE_MS = 200;

// ---- 堵轉/編碼器失效切斷 ----
// 這顆馬達最大堵轉電流 23 A，遠高於驅動板額定，專案先前已經燒掉過好幾片(見 docs/architecture/wiring_plan.md
// 第 5 節)；曾把「驅動板被堵轉電流打到沒反應」誤判成馬達靜摩擦。馬達卡住或沒反應時 PID 還是照樣推 duty，
// 所以在「連續模式、指令非 0、編碼器連續 STALL_CUTOFF_MS 一個 tick 都沒動」時強制 duty=0 並鎖定。
// 600 ms 的依據：健康的起步(kick 100 ms + 斜率爬升到可轉 duty)約 300 ms 內就會動；
// 文件記的燒毀門檻是堵轉超過 7 A 持續約 900 ms（XY-160D），600 ms 夾在中間。
// 編碼器失效(線沒接好)也會被同一條擋下來——沒有編碼器回授的閉迴路本來就不該繼續推。
// 鎖定後 Orin 必須先送 0(連續 FAULT_CLEAR_MS)才會解除，狀態從 STA 的 f 欄位回報。
const uint32_t STALL_CUTOFF_MS = 600;
const uint32_t FAULT_CLEAR_MS = 300;
// 常駐速度回報的 EMA 係數，跟 updateMotorPID() 裡的 ALPHA(0.3)一致。
const float SIGNED_SPEED_ALPHA = 0.3f;

// ---- 低速：維持下限 duty 與 kick-start ----
// 低速時馬達有 Stribeck 摩擦特性：duty 低於某個值完全不轉、高於某個值才能持續轉，中間沒有穩定平衡點，
// 線性 PID 在那裡會陷入「動一下停一下」的極限環，加大 Kp/Ki 只會震得更兇。所以：
//   * 起步：從靜止(prevDuty==0)先用 KICK_DUTY 打一個固定時長的開迴路脈衝掙脫靜摩擦，
//     脈衝期間不跑 PID/積分，避免高 duty 被誤當成收斂過程灌進積分。
//   * 續轉：duty 不低於 MIN_RUN_DUTY（已經在轉之後能維持不停的門檻比起轉門檻低很多，有遲滯）。
//     代價是目標低於下限對應的轉速時，會被墊高到下限對應的轉速。
// 教訓：曾以為間歇停頓是 duty 餘裕不夠（反覆調整下限門檻），真正病灶是 sendStatus() 的 USB 輸出用阻塞式
// Serial.print()，以及 updateSteering() 每次都重送 I2C；拿掉這兩個阻塞源之後同樣的 duty 就零停頓。
// 遇到停頓先查 loop() 有沒有被卡住，再考慮調門檻。
const float MIN_RUN_DUTY = 62.0f;  // 硬體 PWM 重量：維持下限 L 約 64(12.6 rpm)/R 約 60(9 rpm)，低於就停住；1% 指令(約 16 rpm)的前饋為 L 65/R 63，仍在連續模式。上地面有負載後最低維持 duty 會上升，需重測

// 起步脈衝：起轉門檻 L 85/R 90(往上掃，每步 0.5 s)，脈衝要高過它但不要太多。
// 起步過衝(8% 指令)：115/150 ms 平均 31%(最大 92%)，100/100 ms 3.3%(8%)，105/100 ms 5.1%(12%)，
// 110/100 ms 7.0%(20%)；上升時間都一樣。選 105/100 ms 是給地面負載多留一點起轉餘裕；
// 如果上地面起不來，調回 115 並把 KICK_DURATION_MS 改 150。
const float KICK_DUTY = 105.0f;
const uint32_t KICK_DURATION_MS = 100;

// 外部命令 -> 內部目標的映射下限：任何非 0 的外部命令都線性映射到 [CMD_MAP_FLOOR_PCT, 100]
// (見 updateMotorPID() 的 effectivePercent)；外部 0 才是停止。否則油門行程前幾 % 會因為低於
// 維持下限而完全沒反應。
const float CMD_MAP_FLOOR_PCT = 5.0f;

// 非阻塞分段傳送用的狀態結構，要放在所有函式之前，否則Arduino自動產生
// 的函式原型（startSend/pumpSend簽章用到這個型別）會插在這個定義之前，
// 導致「型別未宣告」的編譯錯誤。
struct ChunkedSender {
  // 容納 USB 的 STA + dbg 合併行（最長約 250 B）。len 是 uint8_t，上限 255。
  char buf[255];
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
  // 2026-09-15新增：停頓追蹤，取代高頻FAST_LOG——只在driving時(targetPercent!=0)
  // 統計連續幾個PID週期position完全沒變化，用一個精簡摘要取代逐週期印字。
  uint16_t stallStreak;   // 目前連續幾個週期沒動（driving狀態下）
  uint16_t maxStallMs;    // 這段駕駛期間量到的最長單次停頓(ms)
  uint16_t stallEvents;   // 停頓事件次數(連續>=3週期算一次)
  // 2026-09-16新增：kick-start狀態——見updateMotorPID()裡的說明。
  bool kicking;           // 目前是不是正在打靜摩擦掙脫脈衝
  uint32_t kickEndMs;     // 這次脈衝的結束時間點(millis())
  // 2026-10-02新增：常駐速度回報——不管指令是不是0都更新，前進為正(tick/PID週期)，
  // 跟filteredSpeed(沿目標方向為正、指令0時被歸零)分開，見updateMotorPID()開頭。
  float signedSpeed;
  // 2026-10-02新增：堵轉/編碼器失效切斷，見STALL_CUTOFF_MS的說明。
  uint16_t noMotionMs;    // 連續模式驅動中、編碼器連續沒動的累計時間(ms)
  bool fault;             // 已切斷鎖定：輸出強制0，直到指令回到0連續FAULT_CLEAR_MS
  bool faultClearing;     // 鎖定中、指令已是0，正在計FAULT_CLEAR_MS
  uint32_t faultClearSinceMs;
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
// 2026-10-10：STA 20 Hz 之後的健康指標(附在 USB dbg 行尾，前 19 欄順序不變)：
uint32_t staSkipBM = 0;      // 要送 STA 時上一則還沒送完、略過這一次的次數(BM 線)；正常應為 0
uint32_t staSkipUSB = 0;     // 同上，USB 的 STA+dbg 合併行；正常應為 0
uint32_t maxLoopGapUs = 0;   // loop() 相鄰兩次起點的最大間隔(µs)——「有沒有被長呼叫堵住」的直接指標
uint8_t usbDbgDiv = 0;
const uint8_t USB_DBG_DIV = 4;  // USB 合併行每 4 次 STA(=200 ms)才印一次，維持舊的 5 Hz
uint8_t m1DriverDuty = 0;
uint8_t m2DriverDuty = 0;
uint32_t lastCmdMs = 0;
uint8_t ledMode = LED_BOOT_MODE;  // Orin 以 CMD 的 led 欄位指定的車燈模式
bool cmdEverReceived = false;     // 開機後是否收過合法 CMD(handleFrame 設)
int32_t ledShown = -1;            // 目前實際送到 LED 的顏色(0xRRGGBB)，-1=還沒送過；沒變就不重送
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
// 任何一次loop()呼叫超過一個write()的時間。ChunkedSender結構定義移到檔案前面，原因見那裡的註解。
ChunkedSender bmSender = { {0}, 0, 0, 0, false };
// 2026-09-17新增：USB debug輸出專用的非阻塞佇列，見sendStatus()裡的說明。
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

// (turboStep() 的定義放在 ChunkedSender 等結構定義之後的 applyCommand() 前面：Arduino 會把函式原型自動插在
//  第一個函式定義之前，函式放在結構之前會讓 startSend/pumpSend 的原型引用到還沒宣告的型別。)
// 每個 PID 週期呼叫一次。wantTurbo：任一輪的命令 > 100。回傳：這個週期是否放行 turbo。
bool turboStep(bool wantTurbo, uint32_t dtMs) {
  if (wantTurbo) {
    if (turboLocked) return false;                // 鎖定中：不放行、也不充電
    if (turboBudgetMs > dtMs) {
      turboBudgetMs -= dtMs;
      return true;
    }
    turboBudgetMs = 0;
    turboLocked = true;
    return false;
  }
  uint32_t add = dtMs * TURBO_MAX_MS / TURBO_RECHARGE_MS;
  turboBudgetMs = (turboBudgetMs + add > TURBO_MAX_MS) ? TURBO_MAX_MS : turboBudgetMs + add;
  if (turboLocked && turboBudgetMs >= TURBO_RESUME_MS) turboLocked = false;
  return false;
}

void applyCommand(const char* key, int value) {
  if (strcmp(key, "seq") == 0) {
    lastRxSeq = value;
  } else if (strcmp(key, "fwa") == 0) {
    steerAngle = constrain(value, -60, 60);  // 這個session已實測±60度轉向機構正常無卡死
  } else if (strcmp(key, "bwl") == 0) {
    leftSpeed = constrain(value, -100, TURBO_CMD_MAX_PCT);  // 2026-10-10：>100 是 turbo，由 turboStep() 限時
  } else if (strcmp(key, "bwr") == 0) {
    rightSpeed = constrain(value, -100, TURBO_CMD_MAX_PCT);
  } else if (strcmp(key, "led") == 0) {
    if (value >= 0 && value < LED_MODE_COUNT) ledMode = value;  // 不認得的值忽略，維持原模式
  }
}

// 單一方向腳寫入，回傳I2C是否成功。等同pwm.setPin(ch, high ? 4095 : 0)：4095=整條
// 全開(setPWM(ch,4096,0))、0=整條全關(setPWM(ch,0,4096))，差別只在這裡拿得到回傳值。
bool pcaWritePinChecked(uint8_t ch, bool high) {
  uint8_t result = high ? pwm.setPWM(ch, 4096, 0) : pwm.setPWM(ch, 0, 4096);
  dirI2cCallCount++;
  if (result != 0) {
    dirI2cErrorCount++;
    pcaReady = false;  // 交給loop()裡的pcaRecoverTick()每PCA_RETRY_PERIOD_MS重試初始化
    return false;
  }
  return true;
}

// 回傳兩條方向腳是否都寫入成功。兩條都會嘗試寫，不因為第一條失敗就跳過第二條。
bool driveDirection(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  bool a, b;
  if (dir > 0) {
    a = pcaWritePinChecked(in1Ch, true);
    b = pcaWritePinChecked(in2Ch, false);
  } else if (dir < 0) {
    a = pcaWritePinChecked(in1Ch, false);
    b = pcaWritePinChecked(in2Ch, true);
  } else {
    a = pcaWritePinChecked(in1Ch, true);
    b = pcaWritePinChecked(in2Ch, true);  // 懸空(coast)，不要硬煞車，避免高轉速時停止產生大電流衝擊
  }
  return a && b;
}

// 寫入失敗時不更新快取，下個PID週期會再試一次；回傳目前方向是否已確實設定好。
bool driveDirectionCached(uint8_t in1Ch, uint8_t in2Ch, int dir, int8_t* lastDir) {
  if (*lastDir == dir) return true;
  if (!driveDirection(in1Ch, in2Ch, dir)) return false;
  *lastDir = (int8_t)dir;
  return true;
}

// (放在 ChunkedSender/PidState 結構定義之後：Arduino 會把函式原型自動插在第一個函式定義之前，
//  放在結構之前會造成「型別未宣告」的編譯錯誤。)
// 目標 ticks/20ms -> 需要的 duty(前饋)。表內線性內插；低於表頭時沿第一段斜率往下外插
// (會落在 MIN_RUN_DUTY 以下，輸出仍被夾在 MIN_RUN_DUTY)；高於表尾夾在最後一點。
float ffDutyForTicks(const float* lut, float ticks) {
  if (ticks <= lut[0]) {
    float slope = (FF_LUT_DUTY[1] - FF_LUT_DUTY[0]) / max(lut[1] - lut[0], 0.1f);
    return max(0.0f, FF_LUT_DUTY[0] - (lut[0] - ticks) * slope);
  }
  for (int i = 1; i < FF_LUT_N; i++) {
    if (ticks <= lut[i]) {
      float span = lut[i] - lut[i - 1];
      float f = span > 0.01f ? (ticks - lut[i - 1]) / span : 0.0f;
      return FF_LUT_DUTY[i - 1] + f * (FF_LUT_DUTY[i] - FF_LUT_DUTY[i - 1]);
    }
  }
  return FF_LUT_DUTY[FF_LUT_N - 1];
}

// duty -> ticks/20ms(內插，反向查表)。turbo 用它算前饋 duty 上限對應的目標轉速。
float ffTicksForDuty(const float* lut, float duty) {
  if (duty <= FF_LUT_DUTY[0]) return lut[0] * duty / FF_LUT_DUTY[0];
  for (int i = 1; i < FF_LUT_N; i++) {
    if (duty <= FF_LUT_DUTY[i]) {
      float f = (duty - FF_LUT_DUTY[i - 1]) / (FF_LUT_DUTY[i] - FF_LUT_DUTY[i - 1]);
      return lut[i - 1] + f * (lut[i] - lut[i - 1]);
    }
  }
  return lut[FF_LUT_N - 1];
}

void updateMotorPID(int targetPercent, volatile int32_t* pos, PidState* pid,
                     uint8_t* dutyOut, uint8_t in1Ch, uint8_t in2Ch, float* actualOut,
                     const float* ffLutTicks, float maxTicksPerInterval,
                     int encSign, float integralLimit) {
  int32_t currentPos = *pos;
  int32_t deltaTicks = currentPos - pid->lastPos;
  pid->lastPos = currentPos;

  uint32_t now = millis();

  // 常駐速度回報：每個PID週期都更新，不看指令是不是0、也不看是不是在反轉煞車
  // 保護期間。沿「前進為正」(deltaTicks*encSign)。下面各分支在指令0或煞車保護時會
  // 把*actualOut設成0，Orin會看不到手轉/滑行/被推，這個值補上那個缺口。
  pid->signedSpeed = SIGNED_SPEED_ALPHA * (deltaTicks * encSign) +
                     (1.0f - SIGNED_SPEED_ALPHA) * pid->signedSpeed;

  // 堵轉切斷的鎖定狀態：鎖定中一律輸出0；指令回到0連續FAULT_CLEAR_MS才解除。
  if (pid->fault) {
    if (targetPercent == 0) {
      if (!pid->faultClearing) {
        pid->faultClearing = true;
        pid->faultClearSinceMs = now;
      }
      if ((int32_t)(now - pid->faultClearSinceMs) >= (int32_t)FAULT_CLEAR_MS) {
        pid->fault = false;
        pid->faultClearing = false;
        pid->noMotionMs = 0;
      }
    } else {
      pid->faultClearing = false;
    }
    if (pid->fault) {
      driveDirectionCached(in1Ch, in2Ch, 0, &pid->lastDir);
      *dutyOut = 0;
      pid->integral = 0;
      pid->filteredSpeed = 0;
      pid->kicking = false;
      *actualOut = 0;
      return;
    }
  }

  // 只有外部命令字面上就是 0 才算死區：目標 % 的線性映射(見下面 effectivePercent)已經確保任何
  // 非零目標都落在 CMD_MAP_FLOOR_PCT 以上，這裡不該再擋，否則 1~4% 的命令會先被打成 0。
  bool inDeadzone = (targetPercent == 0);

  if (inDeadzone) {
    if (pid->lastDir != 0) {
      pid->zeroSinceMs = now;  // 剛從有在轉變成煞車，記錄煞車起點
    }
    driveDirectionCached(in1Ch, in2Ch, 0, &pid->lastDir);
    *dutyOut = 0;
    pid->integral = 0;
    pid->filteredSpeed = 0;
    pid->kicking = false;  // 進死區視同重新歸零，下次開始驅動要重新掙脫靜摩擦
    pid->noMotionMs = 0;
    *actualOut = 0;
    return;
  }

  int dir = (targetPercent > 0) ? 1 : -1;

  // 外部命令的 [0,100] 線性映射到內部目標的 [CMD_MAP_FLOOR_PCT,100]：外部 100% 仍對應內部 100%，
  // 外部 0 對應停止，其間任何非零命令都有反應，油門行程沒有死的一段。
  // (內部目標本身做不出真線性——duty 低於維持下限沒有穩態平衡點，見 MIN_RUN_DUTY 的說明。)
  float absTargetPercent = (float)abs(targetPercent);
  float effectivePercent = CMD_MAP_FLOOR_PCT +
      (absTargetPercent / 100.0f) * (100.0f - CMD_MAP_FLOOR_PCT);

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
      pid->kicking = false;  // 直接翻轉觸發的煞車，等同重新歸零
      pid->noMotionMs = 0;
      *actualOut = 0;
      return;
    }
    if ((int32_t)(now - pid->zeroSinceMs) < (int32_t)REVERSE_BRAKE_MS) {
      // 已經在煞車，但還沒煞夠久，繼續維持煞車、不送新方向。
      pid->noMotionMs = 0;
      *dutyOut = 0;
      *actualOut = 0;
      return;
    }
    // 煞車時間已經足夠，這個週期開始才允許送出新方向，往下走正常流程。
  }

  pid->lastNonZeroDir = dir;

  // 2026-10-02：方向沒確實設成功、或PCA9685掉線(快取的方向可能已經失效)時絕不輸出PWM。
  if (!pcaReady || !driveDirectionCached(in1Ch, in2Ch, dir, &pid->lastDir)) {
    *dutyOut = 0;
    pid->integral = 0;
    pid->filteredSpeed = 0;
    pid->kicking = false;
    pid->noMotionMs = 0;
    *actualOut = 0;
    return;
  }

  float prevDuty = (float)(*dutyOut);

  // target% 直接線性對應到目標轉速(ticks)。
  float targetSpeed = (effectivePercent / 100.0f) * maxTicksPerInterval;
  if (targetSpeed > maxTicksPerInterval) {
    // 2026-10-10 turbo 區間(>100%)：目標不超過「前饋 duty = TURBO_FF_DUTY_CEILING」對應的轉速，也不低於 100% 的目標
    float turboCap = ffTicksForDuty(ffLutTicks, TURBO_FF_DUTY_CEILING);
    if (targetSpeed > turboCap) targetSpeed = max(turboCap, maxTicksPerInterval);
  }
  float naturalDuty = ffDutyForTicks(ffLutTicks, targetSpeed);  // 純feedforward算出來、
                                                             // 還沒加PID修正的理論duty

  // naturalDuty 低於 MIN_RUN_DUTY（目標低於維持下限對應的轉速）時沒有特別處理：下面輸出仍夾在
  // MIN_RUN_DUTY，目標會被墊高到下限對應的轉速。以現行前饋表，最小的非零命令（1%，約 16 rpm）
  // 前饋為 L 65/R 63，不會走到這種情況。

  // 停頓追蹤（取代高頻 FAST_LOG 印字）：連續 3 個週期(60 ms)沒動算一次事件，濾掉單週期雜訊。
  if (deltaTicks == 0) {
    pid->stallStreak++;
    if (pid->stallStreak == 3) pid->stallEvents++;
    uint16_t stallMs = pid->stallStreak * PID_INTERVAL_MS;
    if (stallMs > pid->maxStallMs) pid->maxStallMs = stallMs;
  } else {
    pid->stallStreak = 0;
  }

  // 堵轉/編碼器失效切斷，依據與數值見STALL_CUTOFF_MS。只有走到這裡(連續模式、
  // 指令非0、不在煞車保護期間)才累計；任何會提前return的分支都已經把計時歸零。
  if (deltaTicks == 0) {
    if (pid->noMotionMs < 60000) pid->noMotionMs += PID_INTERVAL_MS;
  } else {
    pid->noMotionMs = 0;
  }
  if (pid->noMotionMs >= STALL_CUTOFF_MS) {
    pid->fault = true;
    pid->faultClearing = false;
    pid->zeroSinceMs = now;  // 切斷等同進入煞車保護，之後的反轉判斷從這一刻算起
    driveDirectionCached(in1Ch, in2Ch, 0, &pid->lastDir);
    *dutyOut = 0;
    pid->integral = 0;
    pid->filteredSpeed = 0;
    pid->kicking = false;
    *actualOut = 0;
    return;
  }

  // 2026-09-16新增：kick-start，見上方KICK_DUTY/KICK_DURATION_MS常數說明。
  // 偵測條件用prevDuty==0(上一週期輸出是0，代表真的是從靜止開始驅動，不是
  // 死區/煞車期間)，而不是用targetPercent從0變化，因為煞車期間targetPercent
  // 可能早就是非零(等REVERSE_BRAKE_MS)，那種情況上面的反轉保護已經處理過，
  // 不該再疊加一次脈衝。脈衝期間直接開迴路輸出KICK_DUTY，不跑PID/積分，
  // 避免脈衝這段時間的高duty被誤當成正常收斂過程灌進積分裡。
  if (prevDuty == 0.0f && !pid->kicking) {
    pid->kicking = true;
    pid->kickEndMs = now + KICK_DURATION_MS;
  }
  if (pid->kicking) {
    if ((int32_t)(now - pid->kickEndMs) < 0) {
      *dutyOut = (uint8_t)KICK_DUTY;
      pid->integral = 0;
      pid->filteredSpeed = 0;
      *actualOut = 0;
      return;
    }
    pid->kicking = false;
  }

  float rawActualSpeed = deltaTicks * dir * encSign;

  // 改回原廠測試馬達參考韌體(hall_motor_pid_tune.ino)用的0.3，PID_INTERVAL_MS
  // 目前固定20沒有再變動，不需要額外的比例換算；如果之後真的要調
  // PID_INTERVAL_MS，記得這裡的濾波時間常數(≈週期/ALPHA)也要跟著重新考慮。
  const float ALPHA = 0.3f;
  pid->filteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pid->filteredSpeed;
  float actualSpeed = pid->filteredSpeed;
  *actualOut = actualSpeed;

  float error = targetSpeed - actualSpeed;
  float feedforward = naturalDuty;  // 查表前饋

  float tentativeOutput = feedforward + PID_KP * error + PID_KI * pid->integral;
  // 下限也算飽和：duty本來就會被夾在MIN_RUN_DUTY，如果PID算出來的值已經
  // 低於這個下限，積分再往下灌也不會讓輸出更低，只會在目標回升時要多花
  // 時間把灌爆的積分還債(windup)，所以跟原本0/255飽和判斷一樣處理。
  // 2026-10-10：限速器(MAX_DUTY_STEP_PER_CYCLE)正在限制輸出時也算飽和、暫停積分，
  // 避免大階梯期間(輸出被限速卡住、誤差很大)積分一直累積，到達後造成過衝。
  bool slewLimited = (tentativeOutput > prevDuty + MAX_DUTY_STEP_PER_CYCLE) ||
                     (tentativeOutput < prevDuty - MAX_DUTY_STEP_PER_CYCLE);
  bool willSaturate = (tentativeOutput <= MIN_RUN_DUTY) || (tentativeOutput >= 255.0f) || slewLimited;
  if (!willSaturate) {
    pid->integral = constrain(pid->integral + error, -integralLimit, integralLimit);
  }

  float output = feedforward + PID_KP * error + PID_KI * pid->integral;

  // 下限 MIN_RUN_DUTY；起步的靜摩擦掙脫由上面的開迴路脈衝處理，進到這裡時 prevDuty 已經是 KICK_DUTY。
  output = max(output, MIN_RUN_DUTY);

  float slewedOutput = constrain(output, prevDuty - MAX_DUTY_STEP_PER_CYCLE, prevDuty + MAX_DUTY_STEP_PER_CYCLE);

  *dutyOut = (uint8_t)constrain(slewedOutput, 0.0f, 255.0f);
}

// 2026-09-14診斷用計數器：追蹤 updateSteering() 底層 I2C 寫入的成功/失敗次數，
// 用來檢驗「拉高D8/D9頻率時servo異常」是否伴隨真的I2C傳輸失敗（setPWM()回傳非0），
// 而不是單純類比雜訊。不影響任何控制邏輯。
uint32_t i2cSteerCallCount = 0;
uint32_t i2cSteerErrorCount = 0;

// 2026-09-17新增：pulselength換算因子快取——這個值只跟PWM頻率(50Hz，
// setup()裡pwm.setPWMFreq(50)之後就固定不變)有關，原本updateSteering()
// 每次呼叫都用pwm.readPrescale()+pwm.getOscillatorFrequency()重新問一次
// PCA9685，等於每次轉向都多一次不必要的I2C讀取（讀比純寫慢，因為要等
// 從屬裝置回傳資料）。開機時算一次存起來，之後直接用，砍掉這個I2C read。
double steerPulselengthUs = 0;

// 2026-09-17新增：角度沒變就不重送——跟driveDirectionCached()同樣的
// 概念。上層(不管是CMD協定重送同一個fwa、還是心跳包)常常會用同一個角度
// 反覆呼叫updateSteering()，之前是每次都無條件送一次I2C；現在只有角度
// 真的變了才送，砍掉大部分不必要的I2C transaction。-1000是超出合法
// 角度範圍(±90 map範圍、實際限制±60)的哨兵值，保證開機第一次一定送。
int lastSentSteerAngle = -1000;

// 逐段計時：用 micros() 量 loop() 每一段實際花多少時間，找出哪一段最耗時。
// 只記錄「目前為止量到的最大值」，不是每次印，避免計時本身又製造新的印字負擔。
uint32_t maxUs_pwmPump = 0;
uint32_t maxUs_poll = 0;
uint32_t maxUs_pid = 0;
uint32_t maxUs_status = 0;

void updateSteering() {
  if (!pcaReady) return;  // PCA9685還沒初始化好/掉線：steerPulselengthUs可能是0，不能算；
                          // 重新初始化時會把lastSentSteerAngle重設，回來之後會補送一次。
  if (steerAngle == lastSentSteerAngle) return;  // 角度沒變，不用再送一次I2C
  lastSentSteerAngle = steerAngle;

  uint16_t steerUs = map(steerAngle, -90, 90, STEER_MIN_US, STEER_MAX_US);

  // 手動展開 pwm.writeMicroseconds() 內部算法（跟 Adafruit_PWMServoDriver.cpp
  // 完全一致），藉此拿到 setPWM() 的回傳值——原本 writeMicroseconds() 是 void，
  // 會把這個資訊丟掉。PWM輸出的實際行為跟呼叫 writeMicroseconds() 100%相同。
  // 2026-09-17改成用開機時快取好的steerPulselengthUs，不再每次都重新
  // 用I2C讀prescale/oscillator frequency（這兩個值本來就不會變）。
  double pulse = (double)steerUs / steerPulselengthUs;

  uint8_t i2cResult = pwm.setPWM(CH_STEER, 0, (uint16_t)pulse);
  i2cSteerCallCount++;
  if (i2cResult != 0) {
    i2cSteerErrorCount++;
    pcaReady = false;
  }
}

// ---- PCA9685探測/初始化/健康檢查（2026-10-02新增，說明見pcaReady宣告處）----
bool pcaProbe() {
  // 寫一個位元組的暫存器指標(MODE1=0x00)：PCA9685在線就ACK、離線就NACK，跟函式庫自己的
  // write8()同一類一般傳輸。只在開機(pcaInit)與「已經不健康」的重試時才會用到。
  // (2026-10-02曾懷疑零長度探測會讓韌體卡死而改成這樣，後來發現倉庫裡的Wire1掃描
  // 韌體就是零長度探測、9月用過沒問題，所以那個懷疑多半不成立；當時卡死更像是
  // PCA9685那一側整個斷電、I2C線被拉低造成的。保留一位元組寫法只是更保守。)
  Wire1.beginTransmission(PCA9685_ADDR);
  Wire1.write((uint8_t)0x00);
  return Wire1.endTransmission() == 0;
}

bool pcaInit() {
  if (!pcaProbe()) return false;
  pwm.begin();
  pwm.setPWMFreq(50);
  // 開機時算一次pulselength換算因子存起來，updateSteering()之後直接用快取值。
  uint16_t steerPrescale = pwm.readPrescale();
  steerPulselengthUs = 1000000.0;  // 1,000,000 us per second
  steerPulselengthUs *= (double)(steerPrescale + 1);
  steerPulselengthUs /= (double)pwm.getOscillatorFrequency();
  // 方向腳回到待機態(懸空)，並讓PID那邊的方向快取跟實際輸出一致。
  bool a = driveDirection(CH_M1_IN1, CH_M1_IN2, 0);
  bool b = driveDirection(CH_M2_IN1, CH_M2_IN2, 0);
  if (!(a && b)) return false;
  pidM1State.lastDir = 0;
  pidM2State.lastDir = 0;
  lastSentSteerAngle = -1000;  // 強制重送一次方向盤角度
  return true;
}

// 健康(pcaReady)時直接return，完全不碰I2C。不健康時才每PCA_RETRY_PERIOD_MS試一次重新初始化。
void pcaRecoverTick(uint32_t now) {
  if (pcaReady) return;
  if ((int32_t)(now - pcaNextCheckMs) < 0) return;
  pcaNextCheckMs = now + PCA_RETRY_PERIOD_MS;
  if (pcaInit()) {
    pcaReady = true;
    updateSteering();
  }
}

// 2026-10-10：硬體 PWM。duty 只會在 PID 週期(updateMotorPID)被改，所以只需要在那之後呼叫，
// 不用每圈 loop() 都呼叫。每個 PID 週期(20 ms)重寫一次、即使 duty 沒變，跟獨立測試
// (car2_hwpwm_encoder_test，每 20 ms 呼叫 analogWrite)的驗證條件一致。
void updateMotorPWM() {
  analogWrite(M1_PWM_PIN, m1DriverDuty);
  analogWrite(M2_PWM_PIN, m2DriverDuty);
}

// tick/PID週期 -> RPM
float ticksPerIntervalToRPM(float ticksPerInterval) {
  return (ticksPerInterval / TICKS_PER_REV) * (60000.0f / PID_INTERVAL_MS);
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
  cmdEverReceived = true;
  updateSteering();
}

void pollLink(LinkState& ls) {
  while (ls.port->available()) {
    char c = ls.port->read();
    ls.byteCount++;
    // 不在這裡逐 byte 印 hex dump：USB(115200)的 256-byte TX 緩衝寫滿後 write() 會忙等，
    // BM 線 460800 下一個 frame(~35 byte)的 debug 量就能讓這裡卡住 40~50 ms。
    // 需要查 byte 遺失時再臨時加回來。
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

  // STA 欄位：rpmL/rpmR = 輪速回授(沿目標方向為正，來自 updateMotorPID() 的 EMA 濾波值，只做單位換算)；
  //   vL/vR = 常駐速度(整數 rpm、前進為正，指令 0 時也有值，手轉/滑行/被推都看得到)；
  //   f     = bit0 M1 左輪堵轉切斷、bit1 M2 右輪堵轉切斷、bit2 PCA9685/方向腳 I2C 故障。
  // vL/vR/f 刻意用整數 %d、不用 %.1f：嵌入式 printf 的浮點格式化最貴。
  float vM1 = ticksPerIntervalToRPM(pidM1State.signedSpeed);
  float vM2 = ticksPerIntervalToRPM(pidM2State.signedSpeed);
  int vL = (int)(vM1 + (vM1 >= 0 ? 0.5f : -0.5f));
  int vR = (int)(vM2 + (vM2 >= 0 ? 0.5f : -0.5f));
  int faultFlags = (pidM1State.fault ? 1 : 0) | (pidM2State.fault ? 2 : 0) | (pcaReady ? 0 : 4);

  // buffer大小抓最壞情況(負值三位數帶小數)算過：
  // "STA,ackseq -32768,S1 -1,rpmL -345.9,rpmR -345.9,vL -345,vR -345,f 3,led 3" = 75字元，
  // payload[96]留了充分餘裕(就算rpm異常飆到四位數也不會被截斷)；line再包上
  // "<...,cs XX>\n"約多9字元，line[112]同樣留餘裕。傳送走非阻塞分段(pumpSend每次
  // loop最多5byte)，多出來的約20字元只是多幾個1ms的chunk，不佔用任何一次loop。
  char payload[96];
  snprintf(payload, sizeof(payload), "STA,ackseq %d,S1 -1,rpmL %.1f,rpmR %.1f,vL %d,vR %d,f %d,led %d",
           lastRxSeq, rpmM1, rpmM2, vL, vR, faultFlags, ledEffectiveMode(millis()));
  uint8_t cs = xorChecksum(payload, strlen(payload));
  char line[112];
  snprintf(line, sizeof(line), "<%s,cs %02X>\n", payload, cs);

  // 2026-10-10：上一則 STA 還沒送完(約要 15 ms，loop() 被拖住超過 ~35 ms 才會發生)就略過這一次並計數，
  // 不要像舊版直接覆蓋緩衝——那會把送到一半的 STA 截斷。
  if (bmSender.active) {
    staSkipBM++;
  } else {
    startSend(bmSender, line);  // 非阻塞排入BM傳送佇列，實際byte由loop()裡的
                               // pumpSend()逐次送出，這裡不blocking
  }

  // USB debug 行：精簡 CSV（純數字，字元數約是帶文字標籤的 1/5，降低灌爆 USB 緩衝的機率）。
  // 走跟 BM 線同一套非阻塞佇列：STA 與 dbg CSV 兩行組成一則訊息一起排隊，絕不在這裡阻塞。
  // 欄位順序：dbg,targetL,targetR,dutyL,dutyR,rpmL,rpmR,i2cSteerCalls,i2cSteerErr,
  //           stallEvM1,stallMaxM1,stallEvM2,stallMaxM2,pwmPump,poll,pid,status,
  //           dirI2cCalls,dirI2cErr,pcaReady,staSkipBM,staSkipUSB,maxLoopGapUs,turbo(0/1/2=鎖定),turboBudgetMs
  //   (dirI2cCalls/Err 是方向腳 I2C 寫入次數/失敗次數；欄位只往尾端追加，現有解析不受影響。)
  // 合併行(STA + dbg)每 USB_DBG_DIV 次 STA 才印一次(5 Hz)，並加重疊保護：合併行最長約 250 B，
  // 要 50 ms 左右才排得完，不能每 50 ms 都覆蓋。
  if (++usbDbgDiv < USB_DBG_DIV) return;
  usbDbgDiv = 0;
  if (usbSender.active) {
    staSkipUSB++;
    return;
  }
  char dbgLine[200];
  snprintf(dbgLine, sizeof(dbgLine),
      "dbg,%d,%d,%u,%u,%.1f,%.1f,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%d,%lu,%lu,%lu,%d,%lu\n",
      leftSpeed, rightSpeed, m1DriverDuty, m2DriverDuty, rpmM1, rpmM2,
      (unsigned long)i2cSteerCallCount, (unsigned long)i2cSteerErrorCount,
      pidM1State.stallEvents, pidM1State.maxStallMs,
      pidM2State.stallEvents, pidM2State.maxStallMs,
      (unsigned long)maxUs_pwmPump, (unsigned long)maxUs_poll,
      (unsigned long)maxUs_pid, (unsigned long)maxUs_status,
      (unsigned long)dirI2cCallCount, (unsigned long)dirI2cErrorCount, pcaReady ? 1 : 0,
      (unsigned long)staSkipBM, (unsigned long)staSkipUSB, (unsigned long)maxLoopGapUs,
      turboLocked ? 2 : (turboActive ? 1 : 0), (unsigned long)turboBudgetMs);

  // line(STA，約 75 B) + dbgLine(約 100~125 B) 合併；ChunkedSender.buf 是 255。
  char usbCombined[255];
  snprintf(usbCombined, sizeof(usbCombined), "%s%s", line, dbgLine);
  startSend(usbSender, usbCombined);
}

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
  Serial.begin(115200);      // USB debug monitor，跟Jetson連線無關，維持115200
  SerialBM.begin(460800);  // 2026-09-11起車2專屬拉高到460800（僅車2，車1
                            // 維持115200不動），Jetson端uart_bridge_node的
                            // serial_baud參數要跟著同步改，兩邊沒對齊會變亂碼

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  ledTick(millis());  // 依 LED_BOOT_MODE 送第一次(開機到收到第一個 CMD 之前)

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  // 2026-10-10：硬體 PWM。先設頻率再寫 duty 0，讓兩顆計時器在第一個非零 duty 之前就已經
  // 設好重載值(analogWrite 內部用 CRR 換算比較值)。
  analogWriteFrequency(PWM_FREQ_HZ);
  updateMotorPWM();

  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  // PCA9685 的探測/初始化(pwm.begin、setPWMFreq、pulselength 換算因子、方向腳待機態)在 pcaInit()。
  // 開機時它沒回應也不卡死，loop() 裡 pcaRecoverTick() 只在「不健康」時每 PCA_RETRY_PERIOD_MS 重試
  // (健康時完全不碰 I2C)，期間馬達一律不輸出 PWM，STA 的 f 欄位 bit2 回報。
  pcaReady = pcaInit();

  lastCmdMs = millis();
  lastPidMs = millis();
  if (pcaReady) updateSteering();

  Serial.println(F("=== car2 主控韌體 ready (L298N 設定: 常駐速度回報+堵轉切斷+方向腳I2C互鎖+硬體PWM 5kHz+STA 20Hz+Turbo(未實測)+車燈WS2812) ==="));
}

void loop() {
  uint32_t t0 = micros();
  {  // 2026-10-10：loop() 相鄰兩次起點的間隔(µs)，最大值附在 USB dbg 行尾
    static uint32_t prevLoopUs = 0;
    if (prevLoopUs != 0 && t0 - prevLoopUs > maxLoopGapUs) maxLoopGapUs = t0 - prevLoopUs;
    prevLoopUs = t0;
  }
  pumpSend(bmSender, SerialBM);  // 非阻塞：多數次呼叫直接 return，真正要送時也只送 5 byte、不 delay
  pumpSend(usbSender, Serial);   // USB debug 輸出走同一套非阻塞機制
  uint32_t t1 = micros();
  if (t1 - t0 > maxUs_pwmPump) maxUs_pwmPump = t1 - t0;

  pollLink(linkUSB);
  pollLink(linkBM);
  uint32_t t2 = micros();
  if (t2 - t1 > maxUs_poll) maxUs_poll = t2 - t1;

  uint32_t now = millis();

  pcaRecoverTick(now);
  ledTick(now);

  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  if (now - lastPidMs >= PID_INTERVAL_MS) {
    lastPidMs = now;
    // 2026-10-10：Turbo 預算。只有正向命令 > 100 才算要求 turbo；沒放行就把兩輪夾回 100。
    int effLeft = leftSpeed;
    int effRight = rightSpeed;
    bool wantTurbo = (leftSpeed > 100) || (rightSpeed > 100);
    turboActive = turboStep(wantTurbo, PID_INTERVAL_MS);
    if (wantTurbo && !turboActive) {
      effLeft = min(effLeft, 100);
      effRight = min(effRight, 100);
    }
    updateMotorPID(effLeft, &posM1, &pidM1State, &m1DriverDuty, CH_M1_IN1, CH_M1_IN2, &lastActualM1,
                   FF_LUT_TICKS_M1, MAX_TICKS_PER_INTERVAL_M1, 1, PID_INTEGRAL_LIMIT_M1);  // IN1/IN2對調(方向修正)後，encSign也要跟著翻面
    updateMotorPID(effRight, &posM2, &pidM2State, &m2DriverDuty, CH_M2_IN1, CH_M2_IN2, &lastActualM2,
                   FF_LUT_TICKS_M2, MAX_TICKS_PER_INTERVAL_M2, -1, PID_INTEGRAL_LIMIT_M2);
    updateMotorPWM();  // 把這個週期算出來的 duty 寫進硬體 PWM
  }
  uint32_t t3 = micros();
  if (t3 - t2 > maxUs_pid) maxUs_pid = t3 - t2;

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }
  uint32_t t4 = micros();
  if (t4 - t3 > maxUs_status) maxUs_status = t4 - t3;

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
