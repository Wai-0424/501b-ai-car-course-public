# 2026-10-10：BM 收 CMD 60 Hz + 回 STA 20 Hz，會不會堵塞？（已測試，結果見最下方）

**問題：** 車2 目前 BM 每 200 ms（5 Hz）回一次 STA；Orin 端 CMD 預設 30 Hz。想試「BM 收 CMD 60 Hz、回 STA 給 Orin 20 Hz」，看會不會堵塞。

**事前評估（只讀程式與既有數據，2026-10-10）：** UART 線路不會塞（CMD 60 Hz ≈ 460800 baud 的 5~6.5%，STA 20 Hz ≈ 3%；核心 RX/TX ring buffer 各 256 B；STA 分段排空約 14~18 ms < 50 ms 週期）。真正要驗證的是：
1. `sendStatus()` 單次耗時 1~3 ms，20 Hz 下會讓 `loop()` 每秒被占住 20 次。**D8/D9 已在 10/10 改成硬體 PWM，所以不再凍結馬達輸出**，但仍可能拖慢 `pollLink()` 收 CMD 與 PID 週期。
2. 正式版 `startSend()` 沒有「上一則還沒送完就覆蓋」的保護；USB 的 STA+dbg 合併行要 29~45 ms 才排得完，20 Hz 下接近週期。
3. STA 壞包（10/08 馬達轉動時 0.07~0.59%，成因未驗證）在 20 Hz 下絕對數量 4 倍。

## 這個資料夾的內容

| 檔案 | 用途 |
|---|---|
| `host_tools/make_sta20.py` | 從**目前的**正式版 `car2_car_control.ino` 自動產生測試版韌體（PID 定案後重新產生即可） |
| `car2_car_control_sta20/car2_car_control_sta20.ino` | 產生出來的測試版（STA 週期 50 ms，編譯 51,036 B；**測試用，不是行駛用**） |
| `host_tools/sta20_orin.py` | Orin 端：以固定頻率**整包**送 CMD，量 STA 的到達率、到達間隔、序號缺口、壞包樣本 |
| `host_tools/sta20_pc.py` | 電腦端總控：讀 USB dbg、SSH 呼叫 Orin 端、逐輪比對並判定、存 JSON 與原始 dbg |
| `results/` | 執行結果（執行後才會有） |

## 測試版韌體相對正式版改了什麼（不改任何控制/PID/PWM 邏輯）

- `STATUS_PERIOD_MS` 200 → 50（STA 20 Hz）。
- **傳送佇列重疊保護：** `sendStatus()` 時上一則還沒送完就略過這一次，並計 `staSkipBM` / `staSkipUSB`。
- **STA 加 `sn` 欄位**（第幾次呼叫 `sendStatus()`）：Orin 端用序號缺口精確算出遺失+損毀的 STA 數。STA 因此多約 11 byte。
- **USB 只印 dbg、每 4 次 STA 才印一次**（5 Hz），dbg 尾端追加 16 欄：`ackseq, bmCmdOk, bmCsFail, bmAbort, bmOverflow, bmBytes, feS3, rxdrS3, staCalls, staSentBM, staSkipBM, staSkipUSB, maxLoopGapUs, gap1ms, gap2ms, gap5ms`（前 19 欄與正式版相同）。
- **BM 線 CMD 接收統計**（沿用 10/08 rxcounters 的做法）。
- **`maxLoopGapUs`：** `loop()` 相鄰兩次起點的最大間隔與 >1/2/5 ms 的次數——「會不會堵塞」的直接指標。
- 開機訊息加 `[TEST sta20 STA20Hz]`。

## 執行步驟（需要你在場：輪子離地、Orin 上沒有 ROS 在用 UART）

```bash
# 1. 產生並編譯（正式版有改就重新產生）
python firmware/car2/investigations/2026-10-sta20-test/host_tools/make_sta20.py
arduino-cli compile --fqbn HT32:HT32:BM53A367A firmware/car2/investigations/2026-10-sta20-test/car2_car_control_sta20
# 2. 燒錄測試版（會覆蓋板上的正式版）
arduino-cli upload  --fqbn HT32:HT32:BM53A367A -p COM10 firmware/car2/investigations/2026-10-sta20-test/car2_car_control_sta20
# 3. 冒煙測試（首次加 --deploy，會在 Orin 新建 ~/uart_sta20_test_20261010/ 並複製 sta20_orin.py）
python firmware/car2/investigations/2026-10-sta20-test/host_tools/sta20_pc.py --port COM10 --plan smoke --i-confirm-wheels-lifted --deploy
# 4. 主要比較（約 8 分鐘）與長時間（10 分鐘）
python firmware/car2/investigations/2026-10-sta20-test/host_tools/sta20_pc.py --port COM10 --plan M --i-confirm-wheels-lifted
python firmware/car2/investigations/2026-10-sta20-test/host_tools/sta20_pc.py --port COM10 --plan long --i-confirm-wheels-lifted
# 5. 測完燒回正式版
arduino-cli compile --fqbn HT32:HT32:BM53A367A firmware/car2/car2_car_control
arduino-cli upload  --fqbn HT32:HT32:BM53A367A -p COM10 firmware/car2/car2_car_control
```

執行前請設定 SSH 帳號 `ORIN_USER` 及主機 `ORIN_HOST`。STA 測試可用 `--orin-host` 覆寫位址，`prod_sta20_check.py` 則用 `--host`。開啟 USB 序列埠會讓 BMduino 重開機，計數器因此歸零，每次執行都是乾淨的起點。

## 計畫與判定

| 計畫 | 內容 |
|---|---|
| `smoke` | CMD 60 Hz、輪速 40%、10 秒 |
| `M` | CMD 60 Hz 120 s；CMD 30 Hz 120 s（目前預設，當對照）；CMD 60 Hz + Orin 4 個 CPU 忙碌行程 120 s；CMD 100 Hz 60 s（壓力） |
| `long` | CMD 60 Hz 600 s |

輪速在 0 → 40 → 60 → 40 → 0% 之間每 1.5 秒換階（上限寫死 60%，只往前轉），前輪 ±8° 擺動，CMD 一律**整包**單一 `write()`。

**HARD（回答「會不會堵塞」，全部成立才 PASS）：**
- CMD 完整性：Δ`bmCmdOk` = 送出 + 5（停止指令）；Δ`bmBytes` = 送出 byte；Δ`rxdrS3` = Δ`bmBytes`；Δ`csFail` / `abort` / `overflow` / `FE` = 0
- 傳送不重疊：Δ`staSkipBM` = 0 且 Δ`staSkipUSB` = 0
- 不堵塞：Δ`gap5ms` = 0（`loop()` 起點間隔 > 5 ms 的次數）
- I2C：Δ`i2cSteerErr` = 0、Δ`dirI2cErr` = 0、`pcaReady` = 1
- STA 送達：Orin 端送達率 ≥ 0.95，序號重複 = 0

**SOFT（只報告）：** STA 壞包率與序號缺口率、STA 到達間隔 p99 / max、`maxLoopGapUs` 與 >1/2 ms 次數、ack 年齡。

## 這個測試**不會**回答的

- 馬達在地面帶負載時的電氣雜訊（輪子離地，輪速 ≤ 60%，只往前轉）。
- 真實感知負載（只有 CPU 忙碌行程）；`uart_bridge_node` 本身的讀取效能（用的是測試腳本，不是 ROS 節點）。
- STA 壞包的**成因**：這個測試能量出 20 Hz 下的壞包率，但不能區分是馬達雜訊、傳送端還是接收端。
- 控制效益：CMD 60 Hz 對控制品質幾乎沒有幫助（控制節點本身 20 Hz，10/08 的分析顯示 20→50 Hz 只差 ±0.3~1.4 cm）；這個測試只回答「會不會堵塞」。


---

# 實測結果（2026-10-10 13:08~13:50，輪子架空，輪速 ≤60%、只往前轉，BM 燒的是 STA 20 Hz 測試版，Orin 端用測試腳本不是 ROS 節點）

原始資料在 `results/`（`sta20_*.json`、`sta20_dbg_raw_*.log`）。**結論：CMD 60 Hz + STA 20 Hz 在這個條件下不會堵塞。**

| 輪次 | CMD | CPU 負載 | 時間 | CMD 收到/送出 | STA 收到 | STA 序號缺口（遺失+損毀） | `staSkip` | `loop()` 最大間隔 / >5 ms 次數 | 判定 |
|---|---|---|---|---|---|---|---|---|---|
| 馬達不轉 | 60 Hz | 無 | 30 s | 1805 / 1805 | 600 / 600 | 0 | 0 | 1.3 ms / 0 | PASS |
| 馬達不轉 | 60 Hz | 無 | 120 s | 7205 / 7205 | 2400 / 2400 | 0 | 0 | 2.3 ms / 0 | PASS |
| 馬達轉 | 60 Hz | 無 | 30 s | 1805 / 1805 | 597 / 600 | 2（0.33%） | 0 | 2.9 ms / 0 | PASS |
| 馬達轉 | 60 Hz | 無 | 120 s | 7205 / 7205 | 2393 / 2400 | 7（0.29%） | 0 | 3.7 ms / 0 | PASS |
| 馬達轉 | 30 Hz（目前預設，對照） | 無 | 120 s | 3605 / 3605 | 2388 / 2400 | 12（0.50%） | 0 | 3.7 ms / 0 | PASS |
| 馬達轉 | 60 Hz | Orin 4 個 CPU 忙碌行程 | 120 s | 7205 / 7205 | 2389 / 2400 | 11（0.46%） | 0 | 3.7 ms / 0 | PASS |
| 馬達轉 | 100 Hz（壓力） | 無 | 60 s | 6005 / 6005 | 1193 / 1200 | 6（0.50%） | 0 | 3.7 ms / 0 | PASS |
| 馬達轉 | 60 Hz | 無 | **600 s** | **36005 / 36005** | 11953 / 12000 | 46（0.38%） | 0 | 3.9 ms / 0 | PASS |
| 冒煙 × 5 | 60 Hz | 無 | 10 s × 5 | 605 / 605（每輪） | 每輪 198~200 / 200 | 0~1 | 0 | — | PASS ×5 |
| 冒煙（第一次，剛燒完韌體） | 60 Hz | 無 | 10 s | 605 / 605 | 197 / 200 | 3（1.5%） | 0 | 3.9 ms / 0 | **FAIL**（FE 28，多收 38 byte） |

- **CMD 方向：** 每一輪（除第一次冒煙）收到數 = 送出數、byte 數吻合、`csFail` / `abort` / `overflow` / `FE` 全為 0、`rxdr` = `bytes`。60 Hz 與 30 Hz 沒有差異，100 Hz 也沒事。
- **傳送不重疊：** `staSkipBM` 與 `staSkipUSB` 全程 0，即使 STA 比正式版多 11 byte（`sn` 欄位）、每秒 20 次。
- **不堵塞：** `loop()` 相鄰兩次起點的最大間隔 3.7~3.9 ms（馬達轉）、1.3~2.3 ms（馬達不轉），**從沒超過 5 ms**；馬達轉時 >2 ms 的次數約等於 `sendStatus()` 呼叫次數的 16%（單次 2.4~2.7 ms，含編碼器中斷搶佔）。因為 PWM 已改硬體，這些間隔不再影響馬達輸出。PID、`poll`、`pid` 區段最大值與 STA 5 Hz 時相同量級（poll ≤1.4 ms、pid ≤2.9 ms）。
- **STA 方向（BM → Orin）：** 送達 99.4~100%，到達間隔 p50 50.0 ms、p99 52~53 ms，最大 100~152 ms（= 掉 1~2 個 STA）；`ack` 年齡平均 21~30 ms、最大 44 ms；沒有重複；Orin 端每秒收到 18~20 個。
- **沒有看到 CMD 頻率、Orin 負載造成的差異：** 30 / 60 / 100 Hz 的 STA 損毀率是 0.50 / 0.29 / 0.50%，滿載 0.46%，在抽樣誤差內。

## STA 損毀：只在馬達轉動時出現，成因沒有確認

- **馬達不轉：3000 個 STA 零損毀**（600 + 2400）；加上 10/08 的 0 / 1078，共 0 / 4078。**馬達轉動：約 22000 個 STA，損毀約 0.4%**。如果損毀與馬達無關，3000 個裡一個都沒有的機率約百萬分之三，所以與馬達運轉有關。
- **型態：** 全是單一字元被換成另一個字元（例如 `vL`→`pL`、`cs 4F`→`csv4F`、`sn`→`1n`、`rpmL 1`→`rpmLe1`），被換上去的字元常常在同一個封包的別處出現過。**全部被 checksum 擋下**，沒有一個假陽性（通過 checksum 的 STA 都沒問題）。
- **兩個假設，沒有區分：** (a) 馬達電氣雜訊 / EMI；(b) 編碼器中斷搶佔核心 `write()` 的非原子環形緩衝操作（單字元被錯放，這個型態比較像 (b)）。要區分：在測試版裡把每個 5 byte 分段寫入包在 `noInterrupts()` 裡再跑一次，損毀歸零就是 (b)；但這會讓編碼器丟邊緣，只能當診斷，不能上正式版。
- **控制上的影響：** 轉向控制不用 STA；故障閘門是事件式。20 Hz 下掉一個 STA 只缺 50 ms，5 Hz 時缺 200 ms，所以 20 Hz 對掉包反而比較強健。

## 第一次冒煙測試的異常（FE 28、多收 38 byte、STA 損毀 1.5%）

燒完測試版後的第一個 10 秒冒煙測試，Orin→BM 方向出現 28 個 framing error、多收 38 個雜訊 byte，但 600 個 CMD 全部通過 checksum（雜訊 byte 落在 frame 之間）。**之後 14 次不同長度的執行（含 5 次相同條件的重複）FE 全為 0，沒有重現，成因不明。** 可能是第一次由 Orin 開啟序列埠時的線路擾動，沒有證據。若日後再出現，先記錄是不是「第一次執行」。

## 這個測試沒涵蓋的

- 輪子離地、輪速 ≤60%、只往前轉；**地面負載、反轉、全速的電氣雜訊沒測**（STA 損毀率可能更高）。
- Orin 端是測試腳本，**不是 `uart_bridge_node`**；STA 每包要發 5 個 ROS 訊息（20 Hz = 100 個/秒）的負擔、CMD 60 Hz 在 ROS 節點上的效能都沒測。
- 沒做 STA 5 Hz 的同條件對照（診斷用的 `sta5_final` 版本已能產生並編譯，沒有燒）。
- 控制效益沒有評估：控制節點本身是 20 Hz，CMD 60 Hz 只是重送最新指令。

## 把 STA 20 Hz 搬進正式版需要做的事（還沒做）

1. `STATUS_PERIOD_MS` 200 → 50。
2. `sendStatus()` 加重疊保護（上一則沒送完就略過並計數）——正式版沒有。
3. USB 的 STA+dbg 合併行（143~225 B，需 29~45 ms 排空）降頻或拆開，否則 20 Hz 下會互相覆蓋。
4. 先決定要不要加 `sn` 這類序號欄位（協議書目前沒有）。


---

# 已搬進正式版（2026-10-10 13:40~13:55）

**改動（`car2_car_control.ino`，備份 `.v11_pre_sta20_20261010.bak`）：** `STATUS_PERIOD_MS` 200 → 50；`sendStatus()` 對 BM 線加重疊保護（上一則沒送完就略過並累計 `staSkipBM`）；USB 的 STA+dbg 合併行每 4 次（200 ms）才印一次並加重疊保護（`staSkipUSB`），**格式與頻率與舊版相同**，所以現有電腦端工具（`usb_drive.py` 等）不用改；dbg 行尾追加 `staSkipBM, staSkipUSB, maxLoopGapUs`（19 → 22 欄）；`ChunkedSender` 緩衝 240 → 255；開機訊息尾端「+STA 20Hz」。**沒有**加 `sn` 序號欄位（STA 協議格式不變），也沒有 CMD 接收計數器（那些只在測試版）。編譯 50,724 B（搬遷前 50,604 B）。

**Orin 端消費者的相容性（讀 10/06 snapshot，沒有改任何東西）：** `imu_yaw_node`（輪速閘門）與 `line_follow_control_node`（輪速里程）都用「時間差 × 最後一筆速度」積分，並用 0.8~1.0 s 的新鮮度逾時判斷，**沒有假設固定的 5 Hz**；STA 變快反而讓取樣更貼近真實、掉一包的缺口從 400 ms 降到 100 ms。`MotorFaultGate` 是事件式。

**驗證（輪子架空、輪速 ≤60%）：**

| 項目 | 結果 |
|---|---|
| USB 輸出 | STA 行與 dbg 行仍是 5 Hz（間隔 0.2 s）、格式與舊版相同；dbg 22 欄（前 19 欄不變） |
| 閉迴路迴歸（`usb_drive.py`，同一個計畫） | 無中止、無故障旗標；1%~100% 穩態誤差與搬遷前相同（25%：+0.7/−0.2 vs +0.7/−0.3；50%：+0.2/+0.2 vs +0.8/+0.2；100%：+0.3/+0.3 vs +0.3/+0.4）。資料 `../2026-10-hw-pwm-test/results/closedloop_sta20_prod_20261010.tsv` |
| Orin 端 120 s、CMD 60 Hz、輪速 60%（`prod_sta20_check.py`） | STA 2390 個 = 19.92 Hz（送達率 0.996）、到達間隔 p50 50.0 / p99 52.0 / max 102 ms、每秒 19~20 個；`staSkipBM` 0、`staSkipUSB` 0、`maxLoopGap` 2.8 ms、I2C 錯誤 0、堵轉事件 0、故障旗標 0 → **PASS**（`results/prod_check_20261010_134945.json`） |
| 真正的 `uart_bridge_node`（ROS，1 分鐘，30 Hz，輪速 40%） | 兩個方向 UP、STA 1298 個（約 20 Hz）、checksum 失敗 2 個（0.15%）、`motor_fault_flags` 0、`fault_gate` OK、ack 落後 2 包、結束後無殘留行程 |

**沒驗證的：** 地面負載、反轉、全速；`uart_bridge_node` 搭配真正感知負載的長時間運轉；CMD 在正式版上的接收完整性（沒有計數器，靠測試版的結果，接收路徑沒有改）。

**測試版產生器現況：** `make_sta20.py` 的補丁錨點對應「搬遷前」的正式版，現在改以備份 `.v11_pre_sta20_20261010.bak` 為底（`PROD` 路徑），所以還能產生同樣的測試版；日後若要在新版上重做計數器測試，請先確認錨點是否仍存在。
