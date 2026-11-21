# 車2 專用韌體

第二台車，PCA9685 走 Wire1(BMCOM1)，腳位/編碼器對應跟車1不同（見各檔案開頭註解）。

[接線與迭代](../../hardware/car2/README.md) 依驅動板整理接線、量測與歷史版本。

- `car2_bts7960_pid/` — **BTS7960 車用韌體**。2026-10-14 紀錄確認已刷寫並完成架高驗證，詳見下方車頭燈紀錄。
- `car2_bts7960_basic/`、`car2_bts7960_brake/` — BTS7960 基本動作與煞車階段測試。
- `car2_car_control/` — **L298N 車用版本（2026-10-02 起；2026-10-05 為 14 V 校正版）**：常駐速度回報 `vL/vR`、堵轉切斷與故障旗標 `f`、方向腳 I2C 互鎖、查表前饋，沿用車1 PID＋feedforward 架構。目錄內的 `*.bak` 是 12 V 與中間版本的備份。2026-10-07 由 `experiments/car2_car_control_always_speed/` 改名升格。**2026-10-10 起 D8/D9 馬達 PWM 改為硬體 PWM（`analogWrite`，5 kHz）**，編譯結果 50,644 B；軟體 PWM 舊版備份為 `car2_car_control.ino.v8_softpwm_20261010.bak`（49,168 B）。前饋表是軟體 PWM 校正的，硬體 PWM 下同轉速約需多 6~8 duty（由 PID 補償），驗證與比較見 [investigations/2026-10-hw-pwm-test/](investigations/2026-10-hw-pwm-test/README.md)。
- `legacy_xy160d/car2_car_control_xy160d/` — **舊驅動板（XY-160D）時期的整合韌體**（2026-09-17 從 `experiments/car2_speed_control_test_wip/` 升格），含 kick-start、低速佔空比時間平均、0-100% 油門線性映射。驅動板換成 L298N 後已不適用，只保留備查。
- `calibration/motor_pwm_sweep_bidir/` — 車2馬達 duty→RPM 雙向掃描，含曲線與 CSV 原始數據
- `experiments/` — 通往整合韌體路上的階段性測試（`car2_speed_control_test/`、已升格的 `car2_speed_control_test_wip/`），不是現行版
- `investigations/2026-09-motor-response/` — 開迴路、編碼器、冷啟動、低速摩擦特性與馬達反應排查
- `investigations/2026-09-pca9685/` — PCA9685／I2C／轉向 servo 排查
- `investigations/2026-10-driver-debug/` — 驅動板排查與 L298N 校正：診斷韌體（逐輪/保持/掃描/階躍響應）、電腦端工具 `host_tools/`、量測資料 `results/`、圖表，見該資料夾 `README.md`

## 車體物理校正常數（實測，2026-09-17）

詳細量測方式與原始數據見 vision_line_following_plan.md（本機 `書面資料/技術文件/architecture/vision_line_following_plan.md`） 車2專屬章節「已驗證的前提」與「`fwa` ↔ 實際輪角校正」。

- **軸距（wheelbase）**：288mm（前軸中心到後軸中心）
- **輪距（track width）**：前輪 303mm、後輪 292mm（前後不同，前輪因轉向機構外張較寬，不能只用一個值）
- **輪徑**：10cm（半徑5cm）——搭配 `bwl`/`bwr`=100 時實測輪速（約335~342RPM）換算最大線速度約 **1.77 m/s（~6.4 km/h）**
- **`fwa` ↔ 實際輪角**（靜態量角度實測，servo 40~140 對應 `fwa` -50~+50）：
  - 右轉（`fwa` ≥ 0）：`實際輪角 = 0.5 × fwa`，線性無偏移
  - 左轉（`fwa` < 0）：`實際輪角 = 0.5 × fwa − 5`，轉向機構左轉方向有約5°背隙(backlash)，右轉沒有
  - 反推（要讓輪子轉到目標角度該送的 `fwa`）：右轉 `fwa = 2 × 實際輪角`；左轉 `fwa = 2 × 實際輪角 + 10`
  - 這組公式是目前唯一的實測依據；轉向機構若重新調整/鎖緊（背隙可能改變），需要重新驗證
  - 已套用進 Orin 端的視覺循線控制（曲率前饋，`_fwa_from_wheel_angle_deg()`），取代原本沒有真實軸距支撐的線性猜測值
  - 上面「最大線速度約 1.77 m/s」是**舊驅動板（XY-160D）**的數字；換成 L298N 後滿速約 270 rpm（≈ 1.41 m/s），100% 指令對應約 244 rpm（≈ 1.28 m/s），見下節。

## 2026-10-10：L298N 版硬體 PWM、重測前饋表、重調 PID

- **D8/D9 馬達 PWM 改為硬體 PWM**（`analogWrite`，5 kHz；D8 = PWM0 CH0、D9 = MCTM0 CH0N）。5 kHz 必須維持：同 duty 下 1 kHz 比 5 kHz 快約 40%。
- **前饋表重量**（`FF_LUT_*`，27 格點，約 14 V、輪子架空）：新表比舊表（軟體 PWM 校正）同 duty 慢 12~16 rpm，原因是舊表的軟體 PWM 有效 duty 偏大；新曲線和 10/05 診斷韌體的開迴路曲線相符。起轉門檻 左 85 / 右 90，維持下限 左約 64 / 右約 60（`MIN_RUN_DUTY` 50 → 62），全速（duty 255）左約 300 / 右約 306 rpm，**100% 指令 = 270 rpm 需要 duty 約 237**。
- **PID：** `KP 2.0`、`KI 0.04` 不變；限速 6 → 12 duty / 20 ms、限速時暫停積分、`KICK_DUTY` 115 → 105、`KICK_DURATION_MS` 150 → 100。
- 結果與完整數據：[investigations/2026-10-hw-pwm-test/](investigations/2026-10-hw-pwm-test/README.md)。**全部在輪子架空下完成；上地面後要重量。**
- 舊版備份（`car2_car_control/`）：`.v8_softpwm_20261010.bak`（軟體 PWM）→ `.v9_hwpwm_oldlut_20261010.bak`（硬體 PWM、舊前饋表）→ `.v10_hwpwm_newlut_oldpid_20261010.bak`（硬體 PWM、新前饋表、舊 PID）。
- **STA 回報頻率 5 Hz → 20 Hz（`STATUS_PERIOD_MS` 200 → 50）已加入 L298N 車用版本**：測試（CMD 60 Hz + STA 20 Hz，馬達轉動、輪子架空）CMD 零遺失、傳送佇列零重疊、`loop()` 間隔從未超過 5 ms；STA 損毀約 0.4% 只在馬達轉動時出現、全被 checksum 擋下，成因未確認。新增 `sendStatus()` 傳送佇列重疊保護，USB 的 STA+dbg 合併行仍維持每 200 ms 一次、格式不變，dbg 行尾追加 3 欄 `staSkipBM, staSkipUSB, maxLoopGapUs`（共 22 欄）。**STA 協議格式沒變**（沒有加序號欄位）。測試與驗證：[investigations/2026-10-sta20-test/](investigations/2026-10-sta20-test/README.md)。搬遷前備份：`car2_car_control.ino.v11_pre_sta20_20261010.bak`。
- **Turbo 模式（短時間超過 100% = 270 rpm）：原始碼已寫好、編譯通過，但尚未燒錄、尚未實測**（寫的時候 BMduino 沒接上）。`bwl/bwr` 正向放寬到 +125（101 以上是 turbo，後退仍 -100），韌體端預算制限時（連續 3 s、鎖定後約 5 s 解除、10 s 充滿），turbo 目標被限制在「前饋表 duty 250 的轉速」；命令 ≤100 時行為完全不變，`TURBO_CMD_MAX_PCT` 改 100 即完全關閉。目前（約 14 V）的表 turbo 上限只有左 291 / 右 297 rpm，到不了 300；14.5 V 估計剛好在邊緣。上板檢查清單、風險、預期值與驗證腳本：[investigations/2026-10-turbo-14v5/](investigations/2026-10-turbo-14v5/README.md)。備份：`car2_car_control.ino.v12_pre_turbo_20261010.bak`。Orin 端 `uart_bridge_node` 的 `max_wheel_speed` 夾在 100，要另外改才送得出 turbo。
- **建置依賴本機修改過的核心：** 本機 `HT32` 核心的 `HardwareSerial.cpp` 相對 `HardwareSerial.cpp.orig_backup` 有兩行 `USART_ClearFlag(..., USART_FLAG_TXDE)` 被註解掉（另外有 `gDbgFeCount` / `gDbgRxdrCount` 除錯計數器，STA 測試版會用到）。這個修改不在本倉庫裡；換電腦或重裝核心後編譯出的韌體行為可能不同。

## 2026-10-14：車頭燈（D13，16 顆 WS2812B，由 Orin 以 CMD 的 `led` 欄位控制）

- 模式 0 熄滅 / 1 白色常亮 / 2 橘色爆閃（停車、輪速 0） / 3 紅色呼吸（失去 Orin，韌體自己切）。協議、Orin 端對應與接線注意事項見 protocol.md（本機 `書面資料/技術文件/architecture/protocol.md`）「`CMD` 加 `led`」。
- **FastLED 不支援這塊 HT32 板子**（編譯失敗：Platform does not have a clockless controller defined），所以韌體裡自己寫了 WS2812 bit-bang（`ledShow()`，每顆 LED 關中斷約 30 µs）。常數 `LED_COUNT`、`LED_BRIGHTNESS`、`LED_BOOT_MODE`、`LED_LINKLOST_BREATH` 與時序在 `PIN_LED` 附近。橘色爆閃 2 Hz（500 ms 週期，亮 250 ms，`LED_STROBE_ON_MS` 可調）。
- **時序教訓（2026-10-14）**：第一版用 `GPIO_SetOutBits/ClearOutBits` + SysTick 計時，常亮白燈看起來正常，但熄滅/橘/紅全都變成白色——那兩個 GPIO 函式是真的函式呼叫，一次 set→clear 的高電位就 1.3 µs，WS2812 的 0 碼（~0.35 µs）全被讀成 1，而全 1 的白光剛好不受影響；讀 `SysTick->VAL` 本身約 0.4 µs，也不能當計時基準。現在改成直接寫 `port->SRR / port->RR` 加固定次數的忙等迴圈（每次 3 個時脈週期 = 50 ns，`LED_T0H_LOOPS` 等常數）。換編譯器/核心版本或 CPU 時脈要重新校準。
- **兩版韌體都有燈**：`car2_car_control`（L298N 版）與 `car2_bts7960_pid`（BTS7960 版，**車2目前實際使用的是這版**，燈的程式碼與常數兩邊一致；BTS 版的 USB `dbg` 行尾追加了 `led` 欄位）。2026-10-14 一開始誤把 L298N 版刷進了 BTS 車（馬達完全不動，因為 BTS 的致能腳在 PCA9685 上、L298N 版不會拉起），已改刷 BTS 版（備份 `car2_bts7960_pid.ino.v2_pre_led_20261014.bak`）。**刷韌體前先確認板子上原本是哪一版。**
- **已驗證（2026-10-14，BTS 版，BM 接 Orin，車架空）**：0 熄滅 / 1 白 / 2 橘 2 Hz 閃 / 3 紅呼吸（沒有 Orin 時韌體自己切）外觀都正確，STA 的 `led` 與 CMD 一致。
- **燈對馬達控制的影響（2026-10-14，BTS 版，40% 約 129 rpm 架空，9 個階段各 12 s：熄滅/白/橘閃/紅呼吸/熄滅/紅呼吸/橘閃/白/熄滅）**：輪速標準差 0.24~0.43 rpm（四種模式沒有差別）、平均 129.0~129.2 rpm；STA 20 Hz 不掉、CMD 與 ack 差 0、`staSkipBM` 0、故障旗標 0；編碼器非法跳變每秒約 0.3 次（燈熄滅時的基準是 0.48/s，開燈時沒有更多）；checksum 錯誤約 0.5% 與開不開燈無關。唯一可見的差別是爆閃/呼吸時 `loop()` 最大間隔從 1.8 ms 升到 2.4 ms（一次更新 16 顆約 0.5 ms，60 Hz PID 週期 16.7 ms，不影響）。**沒測**：地面負載、高轉速、100% 指令、實際循線。

- 全亮白 16 顆最多約 1 A，供電要夠；開機重啟或亮度偏黃就先調低 `LED_BRIGHTNESS`。

## 2026-10-02：驅動板改為標準 L298N

**為什麼換**：XY-160D 在 10/1~10/2 連續損壞/不穩（至少換了 3 片）；同一段排查中發現 **PCA9685 到驅動板之間有一條斷線**，使其中一路從頭到尾不動，修好後換上標準 L298N 兩路都正常。先前每片 XY-160D 損壞的確切原因不能完全確定（堵轉無保護，可能還有接線問題）。**每個欄位的來源與量測資料見** [hardware/car2_hardware_summary_2026-10-05.md](../../hardware/car2/car2_hardware_summary_2026-10-05.md) **與** [investigations/2026-10-driver-debug/README.md](investigations/2026-10-driver-debug/README.md)。

**L298N 版腳位對應（驅動板接線與編碼器交叉，韌體已對調）**：

| 邏輯馬達 | 編碼器 | PWM | 方向腳（PCA9685） | 正轉 |
|---|---|---|---|---|
| M1（左） | D4/D5 | D9 | CH3、CH4 | CH3 高、CH4 低 |
| M2（右） | D10/D11 | D8 | CH1、CH2 | CH1 高、CH2 低 |

**L298N 校正結果（12 V、輪子架空，PWM 5 kHz）**：
- 起轉門檻（往上掃）：左 duty 110、右 duty 100，起轉瞬間跳到約 60~70 rpm；維持門檻（往下掃）：左 70、右 65（約 10~15 rpm）；遲滯約 40 duty。
- 滿速（duty 255）：左 269 rpm、右 274 rpm；維持曲線是凹的（低速約 2.4 rpm/duty，高速約 1.0 rpm/duty）。
- 階躍響應：τ ≈ 0.26~0.42 s，死時間 ≈ 0.06~0.08 s。
- 前饋改用**查表**（維持曲線），舊的「死區 + 斜率」直線在低/高兩端會差約 20 rpm。
- 新常數：`KICK_DUTY` 130、`MIN_RUN_DUTY` 75、`LOW_SPEED_ON_DUTY` 120、`LOW_SPEED_CYCLE_MS` 100、`PID_KP` 1.0、`PID_KI` 0.04（模擬顯示 KI 0.08 超調更大）、100% 指令 = 105 ticks/20 ms（≈ 244 rpm）。
- 閉迴路準確度（Orin、架空）：15~75% 誤差 ±2% 內、100% +3%、反向 +2%、左右差 ±1%；圈數驗證 30 秒 25%：理論 35.13 圈、編碼器 35.29/35.28 圈。
- 指令 < 約 7% 輪子不會動；起轉有一個短暫的速度突波（起轉脈衝）。
- 看門狗停車約 1.0 秒（70 rpm）。

**注意**：這些是無負載數據，上地面後曲線會右移，需要重新量；L298N 每通道約 2 A，馬達額定負載 2.6 A、堵轉 23 A，**沒有過流保護**，地面測試要注意發熱；`ENA`/`ENB` 跳線帽必須拔掉。

**新增的韌體功能（STA 欄位，詳見 protocol.md（本機 `書面資料/技術文件/architecture/protocol.md`））**：`vL/vR`（指令為 0 時也有值的帶號速度）、`f`（bit0/1 左右輪堵轉或編碼器失效切斷、bit2 PCA9685/方向腳 I2C 故障）。Orin 端以馬達故障旗標進行對應處理。
