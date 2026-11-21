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
const float FF_DEADBAND_M1 = 16.66;
const float FF_SLOPE_M1 = 1.4827;  // duty per tick/PID週期(已換算)
const float MAX_TICKS_PER_INTERVAL_M1 = 148.17f * PID_INTERVAL_MS / 20.0f;  // 對應實測全速344.89 RPM

const float FF_DEADBAND_M2 = 16.28;
const float FF_SLOPE_M2 = 1.4922;
const float MAX_TICKS_PER_INTERVAL_M2 = 147.86f * PID_INTERVAL_MS / 20.0f;  // 對應實測全速344.16 RPM

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
const float MIN_RUN_DUTY = 40.0f;

// 2026-09-16新增：kick-start，取代單純duty下限造成的「目標再小也墊高到
// MIN_RUN_DUTY對應轉速」問題。做法：偵測到從靜止(prevDuty==0)開始驅動時，
// 先用duty=60(沿用原本驗證過的可靠靜摩擦掙脫值)打一個固定時長的開迴路
// 脈衝，脈衝結束後才交給連續PID+MIN_RUN_DUTY。KICK_DURATION_MS=150：
// probe測試裡duty=60幾乎第一個20ms樣本就已經在動，150ms(約7~8個PID週期)
// 足夠可靠掙脫、又不會拖太久感覺像啟動遲滯；跟舊版「靠斜率限制爬到60」
// 的耗時(MAX_DUTY_STEP_PER_CYCLE=6.0時約200ms)同一個量級，起步手感應該
// 差不多，不是新的暴衝。
const float KICK_DUTY = 60.0f;
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
const float LOW_SPEED_ON_DUTY = 60.0f;
// 2026-09-17實測：第一版100ms(10Hz)循環，使用者實際感受是「像引擎一頓
// 一頓」，人能感覺出一次一次的脈衝，不是連續震動——10Hz太慢了。改成
// 40ms(25Hz)，是目前架構下能做到的最高切換頻率：updateMotorPID()本身
// 只有PID_INTERVAL_MS(20ms)才被呼叫一次，一個週期只能容納2個PID tick，
// 這代表on-phase只能是「這次tick開、下次tick關」這種交替，onFraction
// 實際上只有0%/50%/100%三種可能，沒辦法比這更細——要更細的量化就得縮短
// PID_INTERVAL_MS本身，但那個已經實測過會讓servo抖動、USB指令不穩(見
// PID_INTERVAL_MS常數說明)，不值得為了這裡犧牲。40ms讓觸感從「一頓一頓」
// 變成可以接受的震動，使用者實測確認過，代價是低速段的平滑漸變只有3級。
const uint32_t LOW_SPEED_CYCLE_MS = 40;
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
  char buf[240];
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

  float prevDuty = (float)(*dutyOut);

  // 2026-09-11改回車1的做法：target% 直接線性對應到目標轉速(ticks)。
  float targetSpeed = (effectivePercent / 100.0f) * maxTicksPerInterval;
  float naturalDuty = ffDeadband + ffSlope * targetSpeed;  // 純feedforward算出來、
                                                             // 還沒加PID修正的理論duty

  // 2026-09-17重新加回：低速佔空比時間平均，只在naturalDuty低於MIN_RUN_DUTY
  // 時啟用——見上方LOW_SPEED_ON_DUTY等常數的說明。這個範圍以外(naturalDuty
  // >=MIN_RUN_DUTY)完全不受影響，繼續走下面驗證過零停頓的連續PID路徑。
  if (naturalDuty < MIN_RUN_DUTY) {
    pid->kicking = false;  // 不使用連續模式的kick-start狀態機
    pid->integral = 0;     // 避免離開這個模式、回到連續PID時積分還帶著舊值

    float rawActualSpeed = deltaTicks * dir * encSign;
    const float ALPHA = 0.3f;
    pid->filteredSpeed = ALPHA * rawActualSpeed + (1.0f - ALPHA) * pid->filteredSpeed;
    *actualOut = pid->filteredSpeed;

    // onFraction：用LOW_SPEED_ON_DUTY對應的理論轉速當基準，算出「開多久
    // 比例才能讓平均轉速等於目標」——跟kick-start共用同一條feedforward
    // 反函數，一階近似，實際準不準要靠實測驗證，抓到系統性偏差可以之後
    // 加修正係數。
    float onSpeedPerInterval = (LOW_SPEED_ON_DUTY - ffDeadband) / ffSlope;
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
  float mpsM1 = rpmToMps(rpmM1);
  float mpsM2 = rpmToMps(rpmM2);

  // 2026-09-17新增：rpmL/rpmR給Orin做輪速回授，沿目標方向為正——沿用
  // sendStatus()原本就在算的rpmM1/rpmM2(來自updateMotorPID()的EMA濾波
  // 值，這裡只是單位換算，不是重新計算)，之前只印到USB debug的CSV，
  // 沒有真的送進發給Orin的STA封包，這次接上去。
  // buffer大小抓最壞情況(負值三位數帶小數)算過：
  // "STA,ackseq -32768,S1 -1,rpmL -345.9,rpmR -345.9" = 47字元，
  // payload[72]留了充分餘裕；line再包上"<...,cs XX>\n"約多9字元，
  // line[88]同樣留餘裕。
  char payload[72];
  snprintf(payload, sizeof(payload), "STA,ackseq %d,S1 -1,rpmL %.1f,rpmR %.1f",
           lastRxSeq, rpmM1, rpmM2);
  uint8_t cs = xorChecksum(payload, strlen(payload));
  char line[88];
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
  char dbgLine[136];
  snprintf(dbgLine, sizeof(dbgLine),
      "dbg,%d,%d,%u,%u,%.1f,%.1f,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%lu,%lu\n",
      leftSpeed, rightSpeed, m1DriverDuty, m2DriverDuty, rpmM1, rpmM2,
      (unsigned long)i2cSteerCallCount, (unsigned long)i2cSteerErrorCount,
      pidM1State.stallEvents, pidM1State.maxStallMs,
      pidM2State.stallEvents, pidM2State.maxStallMs,
      (unsigned long)maxUs_pwmPump, (unsigned long)maxUs_poll,
      (unsigned long)maxUs_pid, (unsigned long)maxUs_status);

  // 2026-09-17：line加了rpmL/rpmR之後變長，usbCombined(=line+dbgLine)
  // 跟著加大，維持在ChunkedSender.buf(240)以內留餘裕。
  char usbCombined[232];
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
  pwm.begin();
  pwm.setPWMFreq(50);

  // 2026-09-17新增：開機時算一次pulselength換算因子存起來，updateSteering()
  // 之後直接用快取值，不用每次轉向都重新I2C讀prescale/oscillator frequency。
  uint16_t steerPrescale = pwm.readPrescale();
  steerPulselengthUs = 1000000.0;  // 1,000,000 us per second
  steerPulselengthUs *= (double)(steerPrescale + 1);
  steerPulselengthUs /= (double)pwm.getOscillatorFrequency();

  lastCmdMs = millis();
  lastPidMs = millis();
  updateSteering();
  driveDirection(CH_M1_IN1, CH_M1_IN2, 0);
  driveDirection(CH_M2_IN1, CH_M2_IN2, 0);

  Serial.println(F("=== car2 主控韌體 ready ==="));
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
