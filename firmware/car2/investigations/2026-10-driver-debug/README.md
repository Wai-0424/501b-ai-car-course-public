# car2 驅動板排查與 L298N 校正（2026-10-02）

[車 2 L298N 校正及驗證報告](car2_l298n_calibration_2026-10-02.md) 整理開迴路掃描、階躍響應與 PID 驗證。

診斷韌體 `car2_hw_sweep_per_wheel/`（等 USB 指令才輸出，逐輪/保持/掃描/階躍響應）、電腦端工具 `host_tools/`
（需要 pyserial；BMduino 的 USB 是 Holtek CMSIS-DAP `04D9:802F` 的虛擬 COM 埠，Linux 的 cdc_acm 不支援，要在
Windows 上讀）、量測結果 `results/`。圖表在診斷韌體資料夾裡：`l298n_duty_rpm_20261002.png/.csv`（PWM→rpm）、
`l298n_step_response_20261002.png`（階躍響應）、`l298n_pid_sim_20261002.png`（PID 模擬）。

車用韌體（L298N 設定）：`../../car2_car_control/`。重點：
- 腳位對調（換 L298N 後馬達線與編碼器對應交叉）：M1 = D9 + CH3/CH4，M2 = D8 + CH1/CH2。
- 前饋改查表（維持曲線），`KICK_DUTY` 130、`MIN_RUN_DUTY` 75、`LOW_SPEED_ON_DUTY` 120、`LOW_SPEED_CYCLE_MS` 100、
  `PID_KP` 1.0、`PID_KI` 0.04。
- 完整說明與假設見該韌體檔案裡的註解與 `書面資料/技術文件/architecture/protocol.md`。

USB 閉迴路測試（輪子架空、12 V）：25%→71.8/71.3 rpm、35%→93.2/93.2、50%→131、100%→250、-25%→-70.7，
穩態誤差 ≤3%（>=15%），低於約 7% 指令輪子不會動（起轉死區）。上地面有負載後曲線會移，需要重新量。
