# 車 1 L298N 低風險診斷

此 sketch 是新 L298N 的第一階段確認工具。它不開 Orin UART，開機時兩個
Enable PWM 都是 LOW，馬達不會自動動作。

使用前：

1. 將車輪架高，拔除 L298N 的 ENA 與 ENB 跳線帽。
2. 確認 BMduino、PCA9685 與 L298N 共地，並以限流電源或保險絲保護馬達電源。
3. 使用本機的 e-Link32 Pro（CMSIS-DAP）與 Arduino CLI。BMduino-UNO 的 FQBN
   是 `HT32:HT32:BM53A367A`，不可使用 `_IAP` 版本。先編譯：

   ```powershell
   arduino-cli compile --fqbn HT32:HT32:BM53A367A --build-path build/car1_l298n_diagnostic firmware/car1/car1_l298n_diagnostic
   ```

   確認編譯成功後，使用 e-Link32 Pro 上傳。此模式不使用 `COM25`；COM25 只用於
   燒錄後的 USB Serial Monitor：

   ```powershell
   arduino-cli upload --fqbn HT32:HT32:BM53A367A --input-dir build/car1_l298n_diagnostic
   ```

   燒錄後開啟 USB Serial Monitor，設定 115200 baud 與 Newline。
4. 先輸入 status；接著只測單一、低 duty 的短脈衝，例如 pulse M1 F 20 250。
5. 每次測試確認輪子、方向與 encoder_delta。若馬達未轉，才依序提高到 duty
   40、60、80；不要超出 sketch 限制。

## D8／D9 PWM 訊號量測

輸入 `pwm_sweep start` 後，D8 與 D9 會以 5 kHz 同步輸出 duty 0→255→0 的
連續掃描，每級 20 ms。兩組 L298N 方向輸入均保持 LOW，因此這不是馬達運轉
測試；可用示波器確認 ENA／ENB 是否有 PWM 訊號。輸入 `pwm_sweep stop` 或
`stop` 即可將 D8、D9 拉 LOW 並停止掃描。

測試時輸入 ! 可中止目前脈衝，輸入 stop 可關閉兩路輸出。

此版本刻意不測 L298N 主動煞車；停止行為是 ENA/ENB=LOW 的輸出停用。確認
方向與編碼器後，才進入 duty→RPM 掃描與 PID 校正。
