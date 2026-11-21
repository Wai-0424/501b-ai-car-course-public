# 跨車硬體除錯紀錄（2026-09-01 ～ 09-09）

這批是車1、車2接線調機期間產生的一次性診斷小程式，多數彼此「承接」關係（見各檔案開頭註解的「承接 xxx」字樣），依時間序解決問題，不是穩定可重複使用的工具。**部分明確標註是為車2排查寫的（例如 `bmduino_motor_ch1ch3_low_pwm50_test`、`bmduino_motor_pwm50_encoder_check`），但也有被車1接線驗證重用過（見 `bmduino_car1_closedloop_wire1_test` 開頭提到用了 `bmduino_motor_dir_test.ino`），所以沒有拆進 car1/car2，歸在這裡。如果你記得某支其實是專屬車1或車2，可以再手動搬過去。**

大致分三條調查線：

1. **PCA9685 換接 I2C 匯流排**：`bmduino_wire1_i2c_scan`、`bmduino_wire2_i2c_scan`、`bmduino_pca9685_wire1_servo_center`、`bmduino_pca9685_ch_toggle_probe`
2. **馬達方向/PWM/編碼器接線排查**：`bmduino_motor_dir_test`、`bmduino_motor_static_high_debug`、`bmduino_motor_coast_pwm50_debug`、`bmduino_motor_ch1ch3_low_pwm50_test`、`bmduino_motor_pwm50_encoder_check`（+`_rev`）、`bmduino_motor_highduty_sustain_test`、`bmduino_d8d9_identify_test`、`bmduino_m1_phase_check`、`bmduino_m1_fwd_raw_encoder_check`、`bmduino_m1_m2_simultaneous_check`、`bmduino_m2_raw177_test`、`bmduino_m2_pwm_only_probe`、`bmduino_both_50pct_pwm`
3. **HT32 UART 資料損毀調查**（結論見 docs/architecture/uart_tx_corruption_bug.md（本機 `書面資料/技術文件/architecture/uart_tx_corruption_bug.md`）、docs/architecture/HT32_uart_bug_report.md（本機 `書面資料/技術文件/architecture/HT32_uart_bug_report.md`），根因是 74125 緩衝器沒供電，非軟體問題）：`bmduino_serial1_length_test`、`bmduino_serial1_ram_vs_f_test`、`bmduino_uart_loopback_diag`、`bmduino_uart_crosslink_diag`
   - `rx_captures/` — 對應這條調查線的原始序列埠接收 log（原本放在 repo 根目錄）
