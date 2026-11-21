# 車1 專用韌體

車 1 的 [接線與迭代](../../hardware/car1/README.md) 記錄 L298N 腳位和遷移計畫。燒錄前先核對車上驅動板與目前燒錄版本。

- [`bmduino_car_control_l298n/`](bmduino_car_control_l298n/README.md) 有 2026-10-10 刷寫及架高測試紀錄，並記錄 2026-11-17 的速度設定更新。
- [`car1_l298n_diagnostic/`](car1_l298n_diagnostic/README.md) 是 L298N 接線與控制診斷。`car1_led_test/` 用於車燈測試。
- [`orin_tools/`](orin_tools/README.md) 放車 1 的上位機測試工具。`tools/` 放 PID、煞車、PWM 和逾時停車測試。

- `bmduino_car_control/` — **既有實機驗證版（版本 2）**：PCA9685 方向、BMduino 軟體 PWM 與霍爾編碼器 PID 閉環
- `bmduino_car_control_v1_bm_gpio/` — **版本1**（demo 備用）：方向/轉速都走 BMduino GPIO
- `bmduino_car1_closedloop_wire1_test/` — 車1改接 PCA9685 走 Wire1(BMCOM1) 後的第一次閉環上機測試
- `bmduino_car1_fwdrev_readback_compare/` — 車1馬達正反轉 RPM 對照測試（排查同一顆馬達正反轉出力不對稱）
- `car1_motor_pwm_sweep_bidir_wire1/` — 車1新馬達（JGB37-520）duty→RPM 雙向掃描,套用 Wire1 新接線
- `car_motor_pwm_sweep/` — 車1原始（車2出現前）duty→RPM 單向掃描工具，含 [左右輪 PWM 與轉速掃描圖](car_motor_pwm_sweep/motor_sweep_chart.pdf)
- `test_logs/` — 車 1 歷次測試序列埠 log 與事件紀錄
