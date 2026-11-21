# 車2 轉向角度量測（2026-09-15）

- `car2_uart_bridge_angle_test.log`、`throttle_rpm_calibration_console.log`：量測當時的主控台紀錄（主機帳號名稱已改成 `<orin-user>`）。
- `angle_test/`：每個 `fwa` 設定（+0、+20、+35、+50、−20、−35、−50）左右兩個相機的擷取照片，檔名 `NN_fwa<p|n><值>_<left|right>.jpg`。

**照片只保存在本機，沒有納入 git。** 其中數張拍到經過的人（腿、鞋子），而這個儲存庫是公開的，所以 `.gitignore` 排除了 `angle_test/*.jpg`。由照片得到的結論（`fwa` ↔ 實際輪角公式）寫在 vision_line_following_plan.md（本機 `書面資料/技術文件/architecture/vision_line_following_plan.md`），不需要照片也能使用。若要重新取得照片，請向保管這台電腦的人索取。
