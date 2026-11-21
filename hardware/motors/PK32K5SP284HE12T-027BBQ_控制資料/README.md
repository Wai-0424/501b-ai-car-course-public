# PK32K5SP284HE12T-027BBQ 馬達測試教學包

直流馬達＋磁性編碼器 速度閉環 PID 控制的完整範例，從特性量測到自動調參。

## 先看這個

**[PK32 PID 教學 PDF](../../../firmware/common_tools/hall_motor_pid_tune/motor_PK32K5SP284HE12T-027BBQ_report.pdf)**。本硬體包另有 [原始量測時的 Markdown 紀錄](測試報告.md)。

目前維護中的報告與韌體入口在 [維護中的 Markdown 教學](../../../firmware/common_tools/hall_motor_pid_tune/motor_PK32K5SP284HE12T-027BBQ_report.md) 和 [firmware/common_tools](../../../firmware/common_tools/README.md)。本資料夾的三支韌體保留原始量測時的接線設定；尤其編碼器腳位與現行工具可能不同，兩套不可未確認就交叉燒錄。

## 資料夾結構

原廠 PDF 存於本機的「書面資料／技術文件／廠商資料表」。公開規格可參考 [Hennkwell PK32 系列](https://www.hennkwell.com.tw/en/product/PK32.html)，系列頁的數值須依實際料號核對。

```
firmware/      — 三份 Arduino 韌體（依報告裡的順序使用）
  hall_encoder_feedback_test/  — Step1: 驗證馬達驅動＋編碼器回授
  hall_motor_pwm_sweep/        — Step2: PWM duty→RPM 特性掃描
  hall_motor_pid_tune/         — Step3~5: PID 速度閉環（Serial指令即時調參）
scripts/       — 電腦端 Python 腳本（配合上面的 hall_motor_pwm_sweep / hall_motor_pid_tune 韌體使用）
  capture_sweep.py          — 抓 hall_motor_pwm_sweep 韌體輸出的 CSV
  plot_hall_motor_sweep.py  — 畫圖＋線性回歸，算 feedforward 常數
  pid_autotune.py           — 對 hall_motor_pid_tune 韌體自動掃 Kp/Ki 網格，找最佳參數
  rpm_target_sweep.py       — 全範圍驗證，測多個目標轉速的收斂準確度
  example_data/             — 本次測試（PK32K5SP284HE12T-027BBQ）留下的範例數據，供對照
```

## 換一顆新馬達要改哪裡

1. 韌體裡的 `CH_M1_IN1`/`CH_M1_IN2`/`M1_PWM_PIN`/`PIN_M1_A`/`PIN_M1_B` — 改成你實際接線的腳位
2. `TICKS_PER_REV` — 換算方式是「編碼器 PPR × 4」，PPR 查新馬達的 datasheet
3. 減速比 — 查新馬達的 datasheet，如果編碼器裝在馬達軸（減速箱之前）才需要這個換算
4. `hall_motor_pid_tune.ino` 裡的 `FF_DEADBAND`/`FF_SLOPE`/`MAX_TICKS_PER_INTERVAL` — 這幾個要跑完 Step2 的 PWM 掃描才能算出來，不能沿用舊馬達的值

其餘的流程、程式邏輯、腳本都可以直接照搬，細節看報告內文。
