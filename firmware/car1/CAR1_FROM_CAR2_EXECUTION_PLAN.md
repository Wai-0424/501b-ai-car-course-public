# 車一：以車二架構建立獨立 L298N 專案

更新：2026-10-08。目標是建立可獨立版本控制的車一專案，不覆蓋目前的
`bmduino_car_control` 或 Orin 上既有 `ai_car_bringup`。

## 已確認的硬體基準

| 項目 | 實測結果 |
|---|---|
| BMduino 上傳 | e-Link32 Pro，FQBN `HT32:HT32:BM53A367A` |
| UART | Orin 端 `Serial3`，現階段保留 115200，通訊重構後再決定是否升速 |
| PCA9685 | `Wire1`、位址 `0x40`，I2C ACK 正常 |
| M1 | CH1→IN1、CH2→IN2、D8→ENA、encoder D2/D3 |
| M2 | CH3→IN3、CH4→IN4、D9→ENB、encoder D6/D7 |
| encoder raw sign | M1 F/R=`+7623/-7774`；M2 F/R=`-6543/+6475`（255 duty、1 s） |
| D415 | Orin 已辨識 Intel RealSense D415，serial `821212062359`，深度與彩色串流正常 |

完整量測與端子圖見 [CAR1_CURRENT_WIRING_MAP.md](../../hardware/car1/CAR1_CURRENT_WIRING_MAP.md)。

## 專案切分

```text
firmware/car1/
  bmduino_car_control/             # 舊驅動板正式版；只作回退
  car1_l298n_diagnostic/           # 已驗證接線與 encoder 的安全診斷版
  bmduino_car_control_l298n/       # 新正式韌體（本計畫建立）

car1_ws/src/car1_bringup/          # Orin 上獨立 Git 專案（以 car2_bringup 借鑑）
```

## 分階段工作

### 0. 建立可回退基線

1. 以今日已驗證的接線寫入 `bmduino_car_control_l298n`，不修改舊正式韌體。
2. 新韌體上電必須是 ENA=ENB=0、方向輸入 LOW；保留 USB `stop` 指令。
3. 所有 BMduino 編譯都使用本機 Arduino CLI 與 e-Link32 Pro；每次刷寫後先讀
   `status`、PCA ACK，才送馬達指令。

### 1. L298N 韌體：借鏡車二的安全與協定設計

先建立下列最小集合，暫不開啟 PID：

- 車二風格的 `seq/ackseq`、XOR checksum、傳輸逾時 watchdog 與固定格式 STA。
- 每輪獨立 `motor_direction_sign` 與 `encoder_sign`；以今日 raw sign 先配置，
  車體前進方向在目視確認後定義。
- `motor_fault` 欄位：encoder 無 tick、堵轉、I2C 寫入失敗、命令逾時時切 EN。
- USB 診斷與 Orin `Serial3` 共用同一個封包解析器，避免兩份控制邏輯漂移。
- telemetry 至少含：seq、ackseq、raw encoder delta、目標 duty、實際 duty、fault。

驗證：兩輪正反短脈衝、timeout 歸零、checksum 壞包不執行、fault 會切 EN。

### 2. L298N 馬達曲線與 PID

L298N 的壓降與死區不能沿用舊驅動板數值。先用架高測試量測每輪、每方向：

1. duty 20→120、每 10 一級，取得 duty/tick/RPM/電流/溫度 CSV。
2. 量出起轉門檻、維持門檻與正反差異後建立前饋查表。
3. 先 feedforward + P，KI=0；穩定後才逐步加 I 與輸出 slew-rate。
4. 設定堵轉與 encoder-loss 門檻後再進行地面測試。

### 3. Orin：建立 `car1_bringup`

從車二只移植可共用的骨架，而非直接複製整車參數：

| 車二模組 | 車一處理 |
|---|---|
| `uart_protocol.py`、`uart_bridge_node.py` | 移植結構，改為車一 port、baud、frame 與 telemetry |
| `motor_fault.py` | 移植故障閘門，門檻由車一實測設定 |
| `manual_steer_tool.py`、`cmd_tcp_bridge_node.py` | 移植並保留限速、零速預設 |
| `line_follow_control_node.py`、speed profile | 最後才移植，使用車一幾何與新 PID 上限 |
| Orbbec depth/IMU 節點 | 不直接複製；車一改接 D415 的 RealSense pipeline |

第一個 launch 只啟動 UART bridge、狀態監控與手動控制；STA/fault 全數正常後才加
D415 感知與循線。

### 4. 驗收順序

1. USB BMduino：兩輪正反、encoder、stop、fault。
2. Orin UART：checksum、seq/ack、timeout、反覆重開機。
3. 架高閉環：10%→20%→40%→60%，兩輪同步。
4. D415：彩色、深度、點雲/BEV 與控制隔離。
5. 地面慢速手動，最後才進入循線。

## 建議立即執行的三件事

1. 建立 `bmduino_car_control_l298n` 骨架，固定今日接線與安全停機行為。
2. 在 Orin 建立獨立 `~/car1_ws/src/car1_bringup` Git 專案，先放 UART bridge、
   protocol、fault 與手動控制。
3. 完成 L298N duty→RPM CSV 前，不讓自動循線節點送出非零輪速。
