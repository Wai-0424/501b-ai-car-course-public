# 主機端工具導覽

本資料夾只保留一支不經 ROS 的診斷工具：

- [pc_test_client.py](pc_test_client.py) 是直接接序列埠的診斷 CLI，用來測試韌體的 UART 通訊，可由電腦經 USB 序列埠測試 BMduino，預設為 115200 baud。測試流程見本機的 `書面資料/技術文件/architecture/uart_bringup_guide.md`。車 2 現行韌體的協定相容性尚未驗證。

使用前先確認對應車輛的韌體已燒錄；同一個序列埠不可同時被多個程式占用。

遠端手動控制（teleop）、駕駛客戶端與影像接收端都是 ROS bridge 的客戶端，屬於 ROS 專案，已不在本 repo。
