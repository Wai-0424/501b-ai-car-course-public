# AI 自走車（韌體與硬體接線）

兩台自走車使用 Jetson Orin Nano Super 和 BMduino（HT32）。車輛各自維護控制韌體、驅動板接線、校正與測試紀錄。

| 要找的資料 | 入口 |
| --- | --- |
| 車 1 韌體、L298N 診斷與測試 | [車 1 韌體](firmware/car1/README.md) |
| 車 2 BTS7960、L298N 與 XY-160D 版本 | [車 2 韌體](firmware/car2/README.md) |
| 兩台車各自的接線與迭代 | [硬體導覽](hardware/README.md) |
| 共用 UART、編碼器與馬達測試工具 | [韌體工具](firmware/common_tools/README.md) |
| 用電腦 USB 測 BMduino 序列埠 | [主機端測試工具](host/README.md) |
| 跨車 UART 或硬體排查 | [調查紀錄](firmware/investigations/README.md) |

## 版本與操作

- 燒錄或接線前，從對應車輛的韌體與接線入口比對驅動板、腳位、協定和驗證日期。
- 車 1、車 2 的 UART 組態分開維護。車 2 的文件記錄了 460800 baud、checksum 和分段傳送，實際設定以要使用的韌體版本為準。
- 同一個序列埠一次只由一個程式占用。

## 本機資料

- `書面資料/技術文件/` 保存 UART 協議書、接線歷史、技術調查與早期測速報告。整個 `書面資料/` 同時收錄海報、競賽資料、實驗報告、專案紀錄和資料稽核。
- `電路規劃圖/` 保存 KiCad 主工程及歷史備份。
- `build/`、`tmp/` 和 `_trash_for_review/` 保存編譯產物、文件工作檔和待覆核資料。

本專案程式碼採用 [MIT License](LICENSE)。資料表和其他第三方材料適用各自的授權條款。
