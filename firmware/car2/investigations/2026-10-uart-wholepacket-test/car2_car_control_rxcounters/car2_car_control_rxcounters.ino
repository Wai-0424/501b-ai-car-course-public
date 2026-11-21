// ============================================================================
// [TEST 2026-10-08] 整包送 CMD 實驗用韌體：car2_car_control.ino 的複本，只多了BM線(Serial3)
// 接收統計，並追加到USB dbg行尾端(7欄)。不改任何控制/PWM/PID邏輯。
// 原版備份與雜湊見 ../backup_before_flash/ 。實驗結束後請燒回原版 car2_car_control.ino 。
// ============================================================================
// 車2主控韌體（正式版）——速度閉環控制，架構沿用車1正式版
// bmduino_car_control.ino 的 PID + feedforward，套用車2實測確認的腳位
// 對應、校正常數，以及車2專屬的低速處理機制（kick-start、佔空比時間
// 平均、0-100%油門映射，見下方）。
//
// 這份檔案是從 firmware/car2/experiments/car2_speed_control_test_wip/
// car2_speed_control_test_wip.ino 升格而來，2026-09 一系列實車測試
// （低速間歇停頓排查、feedforward重新校正、油門線性化）都在那份 WIP
// 檔案上完成、逐步驗證，這裡是收斂後的最終版本，過程中的推理/誤判/
// 實測數據都保留在下方各常數的註解裡（含歷史上曾經試過又推翻的做法），
// 沒有為了「正式版看起來乾淨」而刪掉，之後遇到類似問題可以回頭查當初
// 怎麼想、怎麼量出來的。
//
// 跟正式版car1的差異：
//   - PCA9685 I2C 走 Wire1(BMCOM1)，不是主 Wire(A4/A5)
//   - M1(左輪)：PCA9685 CH1/CH2(方向)、D8(轉速PWM)、D4/D5(編碼器)
//   - M2(右輪)：PCA9685 CH3/CH4(方向)、D9(轉速PWM)、D10/D11(編碼器)
//     （車2實測確認 D8/D9、編碼器腳位都跟車1相反，見
//     car2_motor_pwm_sweep_bidir.ino 與 bmduino_d8d9_identify_test.ino；
//     M2編碼器原本接D6/D7，實測發現腳位本身讀不到訊號，已改接D10/D11）
//   - FF_DEADBAND/FF_SLOPE/MAX_TICKS_PER_INTERVAL 是車2實測 duty 0~250
//     完整正反轉掃描回歸出來的值（見car2_openloop_duty_sweep_5khz_full.ino
//     跟下方常數註解裡2026-09-17重新校正的說明）
//   - 車2專屬的低速控制機制（車1的馬達沒有這個問題，見updateMotorPID()
//     跟MIN_RUN_DUTY/KICK_DUTY/LOW_SPEED_*常數的說明）：
//       * kick-start：從靜止起步先打一個短脈衝掙脫靜摩擦
//       * 低速佔空比時間平均：naturalDuty低於MIN_RUN_DUTY時，改用
//         25Hz開關交替讓平均轉速連續對應目標，取代單純墊高到下限
//       * 0-100%外部油門線性映射到內部可用範圍，避免死區浪費行程
//   - 新增：即時車速換算（RPM -> m/s），輪徑 10cm
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
// 直接量出來的，跟這個顯示用常數是分開的兩件事。

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
//     匯流排異常時會被Wire的逾時拖住、凍結軟體PWM)，只有「已經不健康」時才每
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

// ---- 馬達轉速 PWM（車2實測確認：D8=M1左輪、D9=M2右輪，跟車1相反）----
const uint8_t M1_PWM_PIN = 9;   // 見上面的重新對應說明
const uint8_t M2_PWM_PIN = 8;

// ---- 編碼器輸入（車2實測確認：D4/D5=M1左輪；M2原本用D6/D7，但實測
// 發現D6/D7腳位本身有問題讀不到訊號，已改接到D10/D11，馬達端接線不變）----
const uint8_t PIN_M1_A = 4;
const uint8_t PIN_M1_B = 5;
const uint8_t PIN_M2_A = 10;
const uint8_t PIN_M2_B = 11;

const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

const uint32_t CMD_TIMEOUT_MS = 500;
const uint32_t STATUS_PERIOD_MS = 200;  // 2026-09-15改回200：sendStatus()已經
                                         // 改成精簡CSV格式(字元數砍到約1/5)，
                                         // 測試看看原本的頻率夠不夠用。
// ENABLE_FAST_LOG：預設關閉，只在需要細看編碼器原始計數/duty時才臨時開啟
// (50Hz debug列印，只走USB不走BM)。
// 2026-09-15關掉：抓到FAST_LOG(10Hz)+sendStatus(5Hz)疊加的印字量會把USB
// 序列埠256-byte緩衝區灌爆，Serial.print()忙等阻塞導致updateMotorPWM()
// 整個loop()被卡住300~500ms，這才是低速時「間歇停頓」的真正原因，不是
// 馬達機械特性(見investigations/2026-09-motor-response/car2_lowspeed_
// smoothness_probe.ino的對照測試)。改用下面STALL_TRACK機制在停下來的
// 時候印一次精簡摘要，取代逐週期的高頻印字。
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

const uint32_t PWM_PERIOD_US = 200;  // 2026-09-14改回5kHz：10kHz頻率測試+I2C鎖死
                                      // 調查已經告一段落(結論：10kHz在目前接線下
                                      // 不可靠，需要先處理電氣佈線，見investigations/
                                      // 2026-09-motor-response/相關掃描結果)，這份
                                      // _wip接下來要驗證的是起步duty衝太快的問題，
                                      // 用回已驗證穩定的5kHz。原始檔沒動。
                                      // 這次單獨只改PWM頻率(PID/debug列印都
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
//
// 2026-09-17重新校正：發現當初這次09-11的回歸，原始sweep韌體
// (car2_openloop_duty_sweep_5khz_full.ino)本身的[ol]除錯印字也是用會
// 阻塞的Serial.print()——跟這份檔案自己在別處抓到的病灶(sendStatus()、
// updateSteering())是同一類問題，量測過程可能被自己的印字污染。修好那份
// 掃描韌體的阻塞問題後重新掃一次(result_2026-09-17_clean_sweep_raw2.log)，
// 對duty>=40的線性段重新回歸：R²約0.98(不是舊資料聲稱的水準，這次沒有
// 印字阻塞干擾，這個R²比較可信，也證實duty>=40這段本身還帶一點真實的
// 弱凸曲率，不是純線性——feedforward只能盡量貼合，殘差交給PID修正)。
// 死區從13.26/13.39回歸成16.66/16.28，比較貼近實際但依然遠低於真正的
// 靜摩擦門檻(~55~60)，這就是為什麼還是要靠KICK_DUTY/MIN_RUN_DUTY處理
// 起步/續轉，不是這裡重新校正就能取代的。
// ---- 2026-10-02 夜：換成 L298N 之後的前饋查表（取代舊 XY-160D 的「死區+斜率」直線）----
// 資料：輪子架空、12 V、5 kHz，往下掃(轉起來之後降 duty)的穩態曲線(<=200)接往上掃(>200)的穩態曲線，
// 見 firmware/car2/investigations/2026-10-driver-debug/car2_hw_sweep_per_wheel/l298n_duty_rpm_20261002.csv。
// L298N 的曲線是凹的(低速每加1 duty 約+2.4 rpm、高速約+1.0 rpm)且有大遲滯(起轉約 100~110、維持到約 65~70)，
// 單一直線在低/高兩端都會差 20 rpm 左右，所以改用查表內插。ticks 單位是「每 PID 週期(20 ms)」。
// 上地面有負載之後曲線會往右移，最終整定要在實際負載下重新量。
const int FF_LUT_N = 35;
const float FF_LUT_DUTY[FF_LUT_N] = {50.0f, 52.0f, 54.0f, 56.0f, 58.0f, 60.0f, 62.0f, 64.0f, 66.0f, 68.0f, 70.0f, 72.0f, 74.0f, 76.0f, 78.0f, 80.0f, 85.0f, 90.0f, 95.0f, 100.0f, 110.0f, 120.0f, 130.0f, 140.0f, 150.0f, 160.0f, 170.0f, 180.0f, 190.0f, 200.0f, 210.0f, 220.0f, 230.0f, 240.0f, 255.0f};
const float FF_LUT_TICKS_M1[FF_LUT_N] = {2.19f, 3.41f, 4.64f, 5.87f, 7.09f, 8.51f, 10.77f, 12.77f, 14.02f, 15.77f, 18.45f, 19.78f, 23.20f, 25.98f, 28.54f, 30.15f, 36.67f, 42.21f, 46.66f, 52.01f, 63.49f, 69.40f, 75.71f, 82.35f, 88.03f, 93.42f, 99.20f, 105.04f, 110.39f, 115.69f, 119.04f, 121.90f, 124.75f, 127.61f, 131.90f};
const float FF_LUT_TICKS_M2[FF_LUT_N] = {3.26f, 5.38f, 7.49f, 8.67f, 10.51f, 12.00f, 13.61f, 14.98f, 15.76f, 18.62f, 20.55f, 21.77f, 26.68f, 29.70f, 31.55f, 34.66f, 41.93f, 47.42f, 52.61f, 57.63f, 65.57f, 72.03f, 79.19f, 85.51f, 90.22f, 95.12f, 101.13f, 107.03f, 112.59f, 117.56f, 120.33f, 123.09f, 125.85f, 128.62f, 132.76f};
// 2026-10-05：供電拉高到約 14 V(L298N 5 V 跳線帽已拔、外接 5 V)後重新量的前饋查表(取代上面 12 V 的版本；12 V 版備份在
// car2_car_control.ino.v5_12V_backup_20261005.bak)。資料：輪子架空，診斷韌體開迴路往下掃(<=200)接高 duty 的掃描，
// 見 results/l298n_14v_down_both_20261005.csv。注意：同一個 duty 下車用韌體實測比診斷韌體轉得快(12 V 時高端約 +20 rpm)，
// 所以高 duty 端這張表要用車用韌體自己的閉迴路資料再修正一次(第二輪校正)。
// 第二輪(2026-10-05)：>=73 duty 的點改用車用韌體閉迴路穩態實測的 (duty, rpm) 配對。
// 第三輪(2026-10-05)：把 MIN_RUN_DUTY 暫降到 45 在車用韌體上掃指令 16%~1%(results/l298n_14v_lowspeed_minrun45_20261005.tsv)，
// 連續模式可一路到 1%(約 15 rpm)，兩輪都 0.2 s 內起轉、無堵轉；低端點改用這份實測(單調化後內插)，>=15% 用第二輪閉迴路資料，255 用開迴路滿速。
// 100% 外部指令 <-> 這個速度(116 ticks/20ms ≈ 270 rpm)，14 V 開迴路滿速約 308 rpm，留約 12% 的 duty 餘裕給 PID/負載。
const float MAX_TICKS_PER_INTERVAL_M1 = 116.0f * PID_INTERVAL_MS / 20.0f;
const float MAX_TICKS_PER_INTERVAL_M2 = 116.0f * PID_INTERVAL_MS / 20.0f;


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
const float PID_KP = 2.0;  // 2026-10-05：14 V/L298N 重新校正後在車用韌體上做步階測試(30->60->30->100->50->15->0，輪子架空，results/l298n_14v_pid_*)，KP 0.6/1.0/1.5/2.0/3.0 比較：2.0 過衝1~6%、欠衝2~6%、整定0.4~1.6 s最平衡；3.0 在30%起步衝+18%且15%右輪整定不了；0.6 變差。KI 0.08 過衝變大(13%)，維持0.04；放寬 duty 變化速率限制(6->10)沒有更好。上地面有負載後要重測
const float PID_KI = 0.04;  // 維持舊值：模擬(前饋準、馬達tau約0.3s)顯示KI越大超調越大(0.08時30->60階躍超調12.8%、0.04時7.1%)

// 積分上限改成每顆馬達分開設定：M1 FF常數(正反轉不一致、平均值可能偏低)
// 導致追不到目標、積分頂到原本300的上限還不夠，先只加大M1的空間，
// M2 原本收斂得不錯，維持原本300不要跟著動。
const float PID_INTEGRAL_LIMIT_M1 = 300.0;  // 曲線改成線性、feedforward準確很多，積分上限先調回正常值
const float PID_INTEGRAL_LIMIT_M2 = 300.0;
// 原本15.0時，起步瞬間feedforward+PID理論值算出來動輒90+，斜率限制只夠
// 壓住「每週期最多爬多少」，理論值一直頂著上限，結果大約120ms(6個週期)
// 就衝到duty~90，實車起步感覺很猛、沒辦法靜態輕柔起步。改成6.0後，同樣
// 情境要花約300ms(15個週期)才爬到duty~90，起步應該明顯平順很多；代價是
// 一般行駛中加減速的反應也會變慢(不只是起步)，如果覺得太鈍可以再往上調。
// 換算成實際時間，原本15.0(20ms週期)是750 duty/秒的斜率上限，6.0則是
// 300 duty/秒；下面等比例配合PID_INTERVAL_MS縮放的邏輯不變。
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

// ---- 2026-10-02新增：堵轉/編碼器失效切斷 ----
// 驅動板XY-160D只有7A額定，這顆馬達最大堵轉電流23A，專案先前已經燒掉過
// 好幾片(見docs/architecture/wiring_plan.md第5節)：馬達卡住或沒反應時，PID
// 還是照樣推duty，堵轉電流超過額定幾百ms就會燒。這裡在「連續模式、指令非0、
// 編碼器連續STALL_CUTOFF_MS一個tick都沒動」時強制duty=0並鎖定。
// 600ms的依據：健康的起步(kick 150ms + 斜率爬升到可轉duty)約300ms內就會動，
// 文件記的燒毀門檻是堵轉超過7A持續約900ms，600ms夾在中間。編碼器失效(線沒接好)
// 也會被同一條擋下來——沒有編碼器回授的閉迴路本來就不該繼續推。
// 只在連續模式(naturalDuty>=MIN_RUN_DUTY)統計；低速佔空比模式的OFF phase本來
// 就會完全不動，不是堵轉，那個模式也限制在duty 60間歇輸出，不納入。
// 鎖定後Orin必須先送0(連續FAULT_CLEAR_MS)才會解除，狀態從STA的f欄位回報。
const uint32_t STALL_CUTOFF_MS = 600;
const uint32_t FAULT_CLEAR_MS = 300;
// 常駐速度回報的EMA係數，跟updateMotorPID()裡的ALPHA(0.3)一致。
const float SIGNED_SPEED_ALPHA = 0.3f;

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

// ---- 低速duty下限：取代原本的佔空比平均(duty-cycling)機制 ----
// 拿掉脈衝後改用純feedforward+PID+斜率限制實測car2_speedcmd_test.py
// 發現：5%/10%目標(duty穩態落在~38~51)完全卡在「動一下停一下」的
// limit cycle裡，不管PID怎麼收斂都逃不掉——這是Stribeck摩擦特性造成
// 的物理缺口(duty<~38完全不轉、duty>~55才能持續穩定轉，中間沒有任何
// duty能讓實際轉速穩定停在這兩者之間)，線性PID沒辦法在不存在穩定平衡
// 點的地方收斂，加大Kp/Ki只會讓極限環震盪得更兇，不會消失。
//
// 第一次補償方法是佔空比平均(開一段時間、停一段時間交替)，但實測發現
// on-phase時間被MIN_ON_MS地板撐住時，會跟起步斜率限制(MAX_DUTY_STEP_
// PER_CYCLE)打架——on-phase時間太短，duty還來不及爬到可靠掙脫的水準
// 就被切回0，5%這種低目標實測完全動不了。
//
// 2026-09-14改成更直接的做法：不試著讓平均速度湊近很低的目標，單純
// 不讓輸出duty掉到MIN_RUN_DUTY以下——只要有在驅動，duty就夾在
// [MIN_RUN_DUTY,255]。代價是低於這個下限對應轉速的目標沒辦法精準
// 跟到，會被墊高到下限對應的轉速，換取完全不再有開關造成的頓挫。
//
// 2026-09-16重新拆成兩個門檻，不再共用同一個值：上面這段2026-09-14的
// 60是「從靜止啟動」的門檻，car2_lowspeed_smoothness_probe.ino的開迴路
// 階梯測試（見firmware/car2/investigations/2026-09-motor-response/
// car2_lowspeed_smoothness_probe/result_2026-09-16_raw.log）證實「已經在
// 轉」之後能維持不停的門檻明顯更低：duty=40穩定持續轉(~300~345 ticks/s，
// 400ms後不再衰減)，duty=30則在<1秒內衰減到完全停止。所以改成
// KICK_DUTY(60，啟動用)+MIN_RUN_DUTY(續轉用)兩段，續轉門檻原本選50——比實測
// 剛好能撐住的40多留一點餘裕(電壓/機構耗損變化時40可能不夠)，實測duty=50
// 的持續轉速(~625 ticks/s)約是duty=60(~900 ticks/s)的七成，把原本「不管
// 目標多小都墊高到60對應轉速」的下限往低速端拉了一截。
//
// 2026-09-17再從50調到55，後來證實是誤診：50/55時觀察到的間歇停頓
// (result_2026-09-17_full_sweep_1to100.log、result_2026-09-17_
// full_sweep_1s_duty55.log)真正病灶是sendStatus()的USB debug輸出
// 還在用阻塞式Serial.print()——跟這份檔案自己在別處診斷過的病灶一模
// 一樣，只是漏改了USB這條線。改成非阻塞(usbSender/pumpSend())之後，
// 同樣的1~100%完整掃描(result_2026-09-17_full_sweep_1s_nonblocking_
// usb.log)在duty=55下stallEvents全程0，證實跟duty餘裕無關。
// 2026-09-17完整測試過40/45/50/55(修完sendStatus()的USB阻塞問題之後)：
// 40有5次真正停頓(target 4,5,6,7,13)、45跟50都各只剩1次、而且剛好卡在
// 同一個target=8——研判是齒輪箱裡固定的機構卡點，累積轉到那個角度附近
// 才會踩到，不是隨機的。50時還額外發現：卡住當下PID算出來的修正量
// (~46~48)低於50這個地板，直接被下限蓋掉、duty完全沒反應，只能被動等
// 轉速自己爬回來；45時同樣的修正量會超過45，看得到duty主動往上頂。
// 55是唯一完整100階掃描零停頓的值(result_2026-09-17_full_sweep_1s_
// nonblocking_usb.log)。2026-09-17再加上updateSteering()的I2C快取化
// (拿掉每次CMD都重送的readPrescale/setPWM)之後，又多砍了一個潛在阻塞
// 源，45重測後也變成零停頓(result_2026-09-17_full_sweep_1s_duty45_
// i2copt.log)，證實之前45的那次殘留停頓也是I2C阻塞推過邊界的。接著測
// 40，兩個阻塞源都拔掉之後，最早探測到的真正機構下限也守住零停頓——
// 正式版定案用40，貼著實測到的物理下限，把地板轉速壓到最低。
const float MIN_RUN_DUTY = 50.0f;  // 14 V L298N、車用韌體實測：連續模式最低維持 duty 約 L 56/R 52(指令 1% = 約 15 rpm)，50 是 PID 往下修的下限(略低於實測，避免夾住 FF)；上地面有負載後最低維持 duty 會上升，需重測

// 2026-09-16新增：kick-start，取代單純duty下限造成的「目標再小也墊高到
// MIN_RUN_DUTY對應轉速」問題。做法：偵測到從靜止(prevDuty==0)開始驅動時，
// 先用duty=60(沿用原本驗證過的可靠靜摩擦掙脫值)打一個固定時長的開迴路
// 脈衝，脈衝結束後才交給連續PID+MIN_RUN_DUTY。KICK_DURATION_MS=150：
// probe測試裡duty=60幾乎第一個20ms樣本就已經在動，150ms(約7~8個PID週期)
// 足夠可靠掙脫、又不會拖太久感覺像啟動遲滯；跟舊版「靠斜率限制爬到60」
// 的耗時(MAX_DUTY_STEP_PER_CYCLE=6.0時約200ms)同一個量級，起步手感應該
// 差不多，不是新的暴衝。
const float KICK_DUTY = 115.0f;  // 14 V L298N：起轉門檻 L 100/R 90(往上掃)，起轉脈衝要高過它但不要太多(太高起步會衝)
const uint32_t KICK_DURATION_MS = 150;

// ---- 2026-09-17重新加回：低速佔空比時間平均 ----
// 之前(2026-09-15)拿掉這套機制，是因為那次的on-phase走的是有斜率限制
// (MAX_DUTY_STEP_PER_CYCLE)的PID路徑，duty還沒爬到能掙脫靜摩擦的水準
// 就被切回0，5%這種低目標實測完全動不了。現在改成on-phase直接開迴路
// 輸出LOW_SPEED_ON_DUTY(沿用kick-start已驗證可靠的60)，沒有斜率限制
// 的問題，讓「平均轉速」能跟著目標%連續爬升，而不是一進到這個區間就
// 墊高到MIN_RUN_DUTY對應的固定轉速。
// 代價(使用者已知情並接受)：ON/OFF交替本身有規律性的小抖動，觸感跟
// 純PID模式不同，之前team覺得比偶發頓挫更難接受才改用duty下限法；
// 這次用戶明確要求整條0-100%油門都要有連續反應，接受這個代價換取
// 低速段也有連續、可分辨的轉速。
// 只在naturalDuty(純feedforward算出來、還沒加PID修正的理論duty)低於
// MIN_RUN_DUTY時才啟用這個模式，naturalDuty>=MIN_RUN_DUTY的範圍完全
// 不受影響，繼續用原本驗證過零停頓的連續PID+kick-start路徑。
const float LOW_SPEED_ON_DUTY = 115.0f;  // 14 V L298N：ON 階段要高過起轉門檻(L 100/R 90)
// 2026-09-17實測：第一版100ms(10Hz)循環，使用者實際感受是「像引擎一頓
// 一頓」，人能感覺出一次一次的脈衝，不是連續震動——10Hz太慢了。改成
// 40ms(25Hz)，是目前架構下能做到的最高切換頻率：updateMotorPID()本身
// 只有PID_INTERVAL_MS(20ms)才被呼叫一次，一個週期只能容納2個PID tick，
// 這代表on-phase只能是「這次tick開、下次tick關」這種交替，onFraction
// 實際上只有0%/50%/100%三種可能，沒辦法比這更細——要更細的量化就得縮短
// PID_INTERVAL_MS本身，但那個已經實測過會讓servo抖動、USB指令不穩(見
// PID_INTERVAL_MS常數說明)，不值得為了這裡犧牲。40ms讓觸感從「一頓一頓」
// 變成可以接受的震動，使用者實測確認過，代價是低速段的平滑漸變只有3級。
const uint32_t LOW_SPEED_CYCLE_MS = 100;  // L298N：ON 階段 duty 120 時，週期40ms的最小平均速度會有約50%；拉長到100ms(輪子tau約0.3s，速度漣波很小)
const uint32_t LOW_SPEED_MIN_ON_MS = 20;   // 1個PID tick，配合上面40ms週期；
                                            // 實測duty=60幾乎第一個20ms樣本
                                            // 就已經在動，1個tick足夠。

// 2026-09-17：這個常數的角色是「外部命令只要不是0，內部目標最低會被
// 線性映射到這個值」的映射下限（不是「低於這個%就視為0」的死區判斷，
// 那個判斷只看targetPercent是不是字面上的0，見updateMotorPID()裡
// inDeadzone）。原因：實測發現如果目標<5%維持原樣直接進死區，等於
// 使用者油門行程的0~5%這一段完全沒有反應，體感是「推了油門車子沒
// 反應」；改成映射之後，油門只要離開0就有東西，行程沒有一段是死的，
// 見effectivePercent的映射公式。
const float LOW_SPEED_DEADZONE_PCT = 5.0f;

// ---- 車速換算（輪徑10cm）----
const float WHEEL_DIAMETER_M = 0.10;
const float WHEEL_CIRCUMFERENCE_M = 3.14159265f * WHEEL_DIAMETER_M;  // ≈0.31416 m/圈

// 非阻塞分段傳送用的狀態結構，要放在所有函式之前，否則Arduino自動產生
// 的函式原型（startSend/pumpSend簽章用到這個型別）會插在這個定義之前，
// 導致「型別未宣告」的編譯錯誤。
struct ChunkedSender {
  // 2026-09-17從80加大到200：sendStatus()改用這套機制送USB debug之後，
  // STA行+dbg CSV行合併成一則訊息一起排隊，長度會超過原本80這個只夠裝
  // BM線STA短訊息的大小。
  // 2026-09-17再加大到240：STA加了rpmL/rpmR兩個欄位給Orin做輪速回授，
  // usbCombined(STA+dbg CSV合併)的長度跟著變長，見sendStatus()裡的說明。
  char buf[255];  // [TEST 2026-10-08] 240->255(len是uint8_t，上限255)，容納dbg行多出的7欄
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
// [TEST 2026-10-08] 只統計來自BM線(Serial3，Orin)的CMD frame，用來判斷Orin整包送是否安全：
uint32_t bmCmdOkCount = 0;          // BM線上checksum驗證通過的CMD frame數(應等於Orin送出包數)
uint32_t bmCsFailCount = 0;         // BM線上checksum不符被丟棄的frame數
uint32_t bmFrameAbortCount = 0;     // 收到'<'時上一個frame還沒收到'>'（結尾遺失或被打斷）
uint32_t bmFrameOverflowCount = 0;  // frame超過接收緩衝(80)被丟棄
bool rxFromBM = false;              // handleFrame當下的frame來自BM線還是USB
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
// 目標 ticks/20ms -> 需要的 duty(前饋)。表內線性內插；低於表頭時沿第一段斜率往下外插(會落在
// MIN_RUN_DUTY 以下，交給低速佔空比模式)；高於表尾夾在最後一點。
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

// duty -> ticks/20ms(內插，反向查表)。低速佔空比模式用它算「ON 階段」的速度。
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

  // 2026-09-17改成只有外部命令字面上就是0才算死區——目標%的線性重新映射
  // (見下面effectivePercent)已經確保任何非零目標都會落在LOW_SPEED_
  // DEADZONE_PCT以上，這裡不用再額外擋一次，也不該擋，否則1~4%這種
  // 命令會先被這裡打成0，映射就沒意義了。
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

  // 2026-09-17新增：把外部命令的完整[0,100]映射到內部目標的
  // [LOW_SPEED_DEADZONE_PCT,100]——car2_lowspeed_smoothness_probe.ino的
  // 探測已經證實duty<40沒有穩態平衡點，物理上做不出「內部目標本身」的
  // 真線性(見上面kick-start那段的討論)，但可以做「外部命令→內部目標」
  // 這一層映射的線性：外部100%仍然對應內部100%，外部0%對應內部0(=完全
  // 停止)，中間任何非零外部命令都線性落在[LOW_SPEED_DEADZONE_PCT,100]，
  // 不會再有一段命令(原本1~4%)因為低於死區被直接打成0、白白浪費行程。
  float absTargetPercent = (float)abs(targetPercent);
  float effectivePercent = LOW_SPEED_DEADZONE_PCT +
      (absTargetPercent / 100.0f) * (100.0f - LOW_SPEED_DEADZONE_PCT);

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

  // 2026-09-11改回車1的做法：target% 直接線性對應到目標轉速(ticks)。
  float targetSpeed = (effectivePercent / 100.0f) * maxTicksPerInterval;
  float naturalDuty = ffDutyForTicks(ffLutTicks, targetSpeed);  // 純feedforward算出來、
                                                             // 還沒加PID修正的理論duty

  // 2026-09-17重新加回：低速佔空比時間平均，只在naturalDuty低於MIN_RUN_DUTY
  // 時啟用——見上方LOW_SPEED_ON_DUTY等常數的說明。這個範圍以外(naturalDuty
  // >=MIN_RUN_DUTY)完全不受影響，繼續走下面驗證過零停頓的連續PID路徑。
  if (naturalDuty < MIN_RUN_DUTY) {
    pid->kicking = false;  // 不使用連續模式的kick-start狀態機
    pid->integral = 0;     // 避免離開這個模式、回到連續PID時積分還帶著舊值
    pid->noMotionMs = 0;   // 低速佔空比的OFF phase本來就會不動，不算堵轉

    float rawActualSpeed = deltaTicks * dir * encSign;
    const float ALPHA = 0.3f;
    pid->filteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pid->filteredSpeed;
    *actualOut = pid->filteredSpeed;

    // onFraction：用LOW_SPEED_ON_DUTY對應的理論轉速當基準，算出「開多久
    // 比例才能讓平均轉速等於目標」——跟kick-start共用同一條feedforward
    // 反函數，一階近似，實際準不準要靠實測驗證，抓到系統性偏差可以之後
    // 加修正係數。
    float onSpeedPerInterval = ffTicksForDuty(ffLutTicks, LOW_SPEED_ON_DUTY);
    float onFraction = constrain(targetSpeed / onSpeedPerInterval, 0.0f, 1.0f);
    uint32_t onMs = (uint32_t)(onFraction * LOW_SPEED_CYCLE_MS);
    if (onMs > 0 && onMs < LOW_SPEED_MIN_ON_MS) onMs = LOW_SPEED_MIN_ON_MS;
    if (onMs > LOW_SPEED_CYCLE_MS) onMs = LOW_SPEED_CYCLE_MS;

    uint32_t phase = now % LOW_SPEED_CYCLE_MS;
    *dutyOut = (phase < onMs) ? (uint8_t)LOW_SPEED_ON_DUTY : 0;
    return;
  }

  // ---- naturalDuty >= MIN_RUN_DUTY：維持原本連續PID+kick-start路徑 ----

  // 2026-09-15新增：停頓追蹤，取代逐週期高頻印字(FAST_LOG)。滿3個週期
  // (60ms)沒動才算一次事件，濾掉單週期雜訊。2026-09-17移到這裡(只在
  // 連續模式下統計)：低速佔空比模式的OFF phase本來就該是完全不動，不是
  // 真正的停頓，混在一起統計會製造大量假警報。
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
  float feedforward = naturalDuty;  // 就是上面已經算過的ffDeadband+ffSlope*targetSpeed

  float tentativeOutput = feedforward + PID_KP * error + PID_KI * pid->integral;
  // 下限也算飽和：duty本來就會被夾在MIN_RUN_DUTY，如果PID算出來的值已經
  // 低於這個下限，積分再往下灌也不會讓輸出更低，只會在目標回升時要多花
  // 時間把灌爆的積分還債(windup)，所以跟原本0/255飽和判斷一樣處理。
  bool willSaturate = (tentativeOutput <= MIN_RUN_DUTY) || (tentativeOutput >= 255.0f);
  if (!willSaturate) {
    pid->integral = constrain(pid->integral + error, -integralLimit, integralLimit);
  }

  float output = feedforward + PID_KP * error + PID_KI * pid->integral;

  // 2026-09-14新增下限，2026-09-16改成搭配kick-start：起步的靜摩擦掙脫已經
  // 交給上面的開迴路脈衝處理，這裡進入PID時prevDuty已經是KICK_DUTY(60)，
  // 不會是0，斜率限制在這之後只負責脈衝結束後續轉duty的平滑收斂，不用再
  // 兼顧「起步不要一次跳到底」這件事。
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

// 2026-09-15新增：逐段計時，找出loop()裡到底是哪一段吃掉最多時間造成PWM
// 凍結——降低印字量之後停頓有變輕但沒完全消失，與其繼續憑經驗猜，直接
// 用micros()量每一段實際花多少時間。只記錄「目前為止量到的最大值」，
// 不是每次印，避免計時本身又製造新的印字負擔。
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
    if (rxFromBM) bmCsFailCount++;
    return;  // checksum不對，整包丟棄，不更新lastCmdMs/lastRxSeq——
             // 寧可讓CMD_TIMEOUT_MS的失效保護介入，也不要照著壞掉的值動作
  }

  if (rxFromBM) bmCmdOkCount++;
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
  bool isBM = (ls.port == (Stream*)&SerialBM);  // [TEST 2026-10-08]
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
      if (isBM && ls.active && ls.len > 0) bmFrameAbortCount++;  // [TEST]
      ls.len = 0;
      ls.active = true;
      continue;
    }
    if (!ls.active) continue;
    if (c == '>') {
      ls.buf[ls.len] = '\0';
      rxFromBM = isBM;  // [TEST]
      handleFrame(ls.buf);
      ls.active = false;
      ls.len = 0;
      continue;
    }
    if (ls.len < sizeof(ls.buf) - 1) {
      ls.buf[ls.len++] = c;
    } else {
      if (isBM) bmFrameOverflowCount++;  // [TEST]
      ls.active = false;
      ls.len = 0;
    }
  }
}

void sendStatus() {
  float rpmM1 = ticksPerIntervalToRPM(lastActualM1);
  float rpmM2 = ticksPerIntervalToRPM(lastActualM2);
  float mpsM1 = rpmToMps(rpmM1);
  float mpsM2 = rpmToMps(rpmM2);

  // 2026-09-17新增：rpmL/rpmR給Orin做輪速回授，沿目標方向為正——沿用
  // sendStatus()原本就在算的rpmM1/rpmM2(來自updateMotorPID()的EMA濾波
  // 值，這裡只是單位換算，不是重新計算)，之前只印到USB debug的CSV，
  // 沒有真的送進發給Orin的STA封包，這次接上去。
  // 2026-10-02新增 vL/vR/f：
  //   vL/vR = 常駐速度(整數rpm、前進為正，指令0時也有值，手轉/滑行/被推都看得到)；
  //   f     = 堵轉/編碼器失效切斷旗標(bit0=M1左輪、bit1=M2右輪)。
  // 刻意用整數%d、不用%.1f：嵌入式printf的浮點格式化最貴，而sendStatus()就是loop()
  // 裡會凍結軟體PWM的那一段(maxUs_status)，多兩個整數欄位的成本可以忽略。
  float vM1 = ticksPerIntervalToRPM(pidM1State.signedSpeed);
  float vM2 = ticksPerIntervalToRPM(pidM2State.signedSpeed);
  int vL = (int)(vM1 + (vM1 >= 0 ? 0.5f : -0.5f));
  int vR = (int)(vM2 + (vM2 >= 0 ? 0.5f : -0.5f));
  int faultFlags = (pidM1State.fault ? 1 : 0) | (pidM2State.fault ? 2 : 0) | (pcaReady ? 0 : 4);

  // buffer大小抓最壞情況(負值三位數帶小數)算過：
  // "STA,ackseq -32768,S1 -1,rpmL -345.9,rpmR -345.9,vL -345,vR -345,f 3" = 67字元，
  // payload[96]留了充分餘裕(就算rpm異常飆到四位數也不會被截斷)；line再包上
  // "<...,cs XX>\n"約多9字元，line[112]同樣留餘裕。傳送走非阻塞分段(pumpSend每次
  // loop最多5byte)，多出來的約20字元只是多幾個1ms的chunk，不佔用任何一次loop。
  char payload[96];
  snprintf(payload, sizeof(payload), "STA,ackseq %d,S1 -1,rpmL %.1f,rpmR %.1f,vL %d,vR %d,f %d",
           lastRxSeq, rpmM1, rpmM2, vL, vR, faultFlags);
  uint8_t cs = xorChecksum(payload, strlen(payload));
  char line[112];
  snprintf(line, sizeof(line), "<%s,cs %02X>\n", payload, cs);

  startSend(bmSender, line);  // 非阻塞排入BM傳送佇列，實際byte由loop()裡的
                               // pumpSend()逐次送出，這裡不blocking

  // 2026-09-15改成精簡CSV格式，取代原本一大串帶文字標籤的debug輸出——
  // 已經證實印字量會把USB序列埠緩衝區灌爆、造成Serial.print()忙等阻塞
  // 進而凍結PWM輸出。這幾行只有我自己在解析，不需要給人看的標籤，純數字
  // CSV可以把字元數砍到約1/5~1/6，降低再次塞爆緩衝區的機率。
  // 欄位順序：dbg,targetL,targetR,dutyL,dutyR,rpmL,rpmR,i2cCalls,i2cErr,
  //           stallEvM1,stallMaxM1,stallEvM2,stallMaxM2,pwmPump,poll,pid,status
  //
  // 2026-09-17修正：上面這段分析本身完全正確，卻只套用在BM線
  // （bmSender在2026-09-11就已經改非阻塞），USB debug這邊當時漏改，
  // 還是維持一長串直接Serial.print()——也就是說，這份檔案精準診斷過
  // 「Serial.print()忙等阻塞會凍結PWM」這個病灶，卻在USB線自己重蹈
  // 覆轍。低速間歇停頓很可能就是這裡造成的：host（電腦端）沒有連續在讀
  // USB序列埠時，這一串印字會把256-byte緩衝區灌爆、卡住loop()。改成跟
  // BM線同一套非阻塞佇列：STA跟dbg CSV兩行組成一則訊息一起排隊，
  // usbSender/pumpSend()負責逐次送出，絕不在這裡阻塞。
  // 2026-10-02：欄位尾端追加 dirI2cCalls,dirI2cErr,pcaReady（方向腳I2C寫入次數/失敗次數/
  // PCA9685是否就緒）。加在最後，前面16欄的順序不變，現有的解析不受影響。
  // [TEST 2026-10-08] 欄位尾端再追加7欄（前面19欄順序不變）：
  //   bmCmdOk,bmCsFail,bmAbort,bmOverflow,bmBytes,feSerial3,rxdrSerial3
  char dbgLine[200];
  snprintf(dbgLine, sizeof(dbgLine),
      "dbg,%d,%d,%u,%u,%.1f,%.1f,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
      leftSpeed, rightSpeed, m1DriverDuty, m2DriverDuty, rpmM1, rpmM2,
      (unsigned long)i2cSteerCallCount, (unsigned long)i2cSteerErrorCount,
      pidM1State.stallEvents, pidM1State.maxStallMs,
      pidM2State.stallEvents, pidM2State.maxStallMs,
      (unsigned long)maxUs_pwmPump, (unsigned long)maxUs_poll,
      (unsigned long)maxUs_pid, (unsigned long)maxUs_status,
      (unsigned long)dirI2cCallCount, (unsigned long)dirI2cErrorCount, pcaReady ? 1 : 0,
      (unsigned long)bmCmdOkCount, (unsigned long)bmCsFailCount, (unsigned long)bmFrameAbortCount,
      (unsigned long)bmFrameOverflowCount, (unsigned long)linkBM.byteCount,
      (unsigned long)gDbgFeCount[SERIAL3_BUFFER_INDEX], (unsigned long)gDbgRxdrCount[SERIAL3_BUFFER_INDEX]);

  // 2026-09-17：line加了rpmL/rpmR之後變長，usbCombined(=line+dbgLine)
  // 跟著加大，維持在ChunkedSender.buf(240)以內留餘裕。
  char usbCombined[254];  // [TEST] 232->254，仍在ChunkedSender.buf(255)以內
  snprintf(usbCombined, sizeof(usbCombined), "%s%s", line, dbgLine);
  startSend(usbSender, usbCombined);
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
  // 2026-10-02：PCA9685的探測/初始化(pwm.begin、setPWMFreq、pulselength換算因子、
  // 方向腳待機態)抽進pcaInit()。開機時它沒回應也不卡死，loop()裡pcaRecoverTick()只在
  // 「不健康」時每PCA_RETRY_PERIOD_MS重試(健康時完全不碰I2C)，期間馬達一律不輸出PWM，
  // STA的f回報bit2。
  pcaReady = pcaInit();

  lastCmdMs = millis();
  lastPidMs = millis();
  if (pcaReady) updateSteering();

  Serial.println(F("=== [TEST 2026-10-08 rxcounters] car2 主控韌體 ready (L298N 設定: 常駐速度回報+堵轉切斷+方向腳I2C互鎖) ==="));
}

void loop() {
  uint32_t t0 = micros();
  updateMotorPWM();
  pumpSend(bmSender, SerialBM);  // 非阻塞：多數次呼叫直接return，真正要送的
                                   // 那一刻也只送5byte、不delay，不影響上面的PWM
  pumpSend(usbSender, Serial);   // 2026-09-17新增：USB debug輸出也改走
                                   // 同一套非阻塞機制，理由見sendStatus()
  uint32_t t1 = micros();
  if (t1 - t0 > maxUs_pwmPump) maxUs_pwmPump = t1 - t0;

  pollLink(linkUSB);
  pollLink(linkBM);
  uint32_t t2 = micros();
  if (t2 - t1 > maxUs_poll) maxUs_poll = t2 - t1;

  uint32_t now = millis();

  pcaRecoverTick(now);

  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  if (now - lastPidMs >= PID_INTERVAL_MS) {
    lastPidMs = now;
    updateMotorPID(leftSpeed, &posM1, &pidM1State, &m1DriverDuty, CH_M1_IN1, CH_M1_IN2, &lastActualM1,
                   FF_LUT_TICKS_M1, MAX_TICKS_PER_INTERVAL_M1, 1, PID_INTEGRAL_LIMIT_M1);  // IN1/IN2對調(方向修正)後，encSign也要跟著翻面
    updateMotorPID(rightSpeed, &posM2, &pidM2State, &m2DriverDuty, CH_M2_IN1, CH_M2_IN2, &lastActualM2,
                   FF_LUT_TICKS_M2, MAX_TICKS_PER_INTERVAL_M2, -1, PID_INTEGRAL_LIMIT_M2);
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
