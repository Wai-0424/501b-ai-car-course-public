# 通用工具（與車輛編號無關）

跟特定車輛接線/協定解耦，可直接沿用或搬到其他專案的工具。

- `bmduino_uart_test/` — 純通訊打通測試（`protocol.md` v0.1），不含馬達驅動邏輯，車1、車2初期都可用來驗證通訊
- `hall_encoder_feedback_test/` — 通用霍爾正交編碼器測試工具，跟車輛協定/PCA9685完全解耦
- `hall_motor_pwm_sweep/` — 針對**測試用馬達**（Hennkwell PK32K5SP284HE12T-027BBQ）的 duty→RPM 掃描，腳位/常數是那顆測試馬達專屬，不能直接套用到車輛本體
- `hall_motor_pid_tune/` — 同一顆測試馬達的 PID 調參工具，搭配 [PK32 馬達速度閉迴路教學](hall_motor_pid_tune/motor_PK32K5SP284HE12T-027BBQ_report.md)及 [PDF](hall_motor_pid_tune/motor_PK32K5SP284HE12T-027BBQ_report.pdf)
- `esp32_uart_test/` — ESP32 UART 測試（跟 BMduino 車輛專案無直接關聯）

## 與馬達測試台資料包的關係

`hardware/motors/PK32K5SP284HE12T-027BBQ_控制資料/firmware/` 裡有同名的 `hall_encoder_feedback_test`、`hall_motor_pid_tune`、`hall_motor_pwm_sweep`，那是隨 2026-08-09 測試報告打包的**快照**。兩邊是各自獨立的檔案，修改其中一份不會同步到另一份：

- `hall_motor_pid_tune`、`hall_motor_pwm_sweep`：去掉註解後程式碼完全相同，只有這裡的開頭註解較完整。
- `hall_encoder_feedback_test`：**編碼器腳位不同**。資料包是舊接法 `D2/D3`，這裡是現行的 `D6/D7`（見車1已知問題）。照資料包報告重現實驗時，要依實際接線選用對應版本。
- 平常以這裡為準；資料包保持報告當時的原貌，用來對照報告內容。
