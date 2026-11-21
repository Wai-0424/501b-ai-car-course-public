#!/usr/bin/env python3
"""UART 通訊測試用客戶端，對應 protocol.md v0.1。
現在可以直接對 BMduino 的 USB COM port 用；之後接 Jetson 時，
把 port 換成 /dev/ttyTHS1（Jetson UART1）即可，指令格式不用改。

2026-09-14：車1韌體加入 checksum 驗證後，build_cmd() 自動在末端附加
`,cs XX`（沿用車2的 XOR8 演算法），CLI 用法不用變，只是底層送出去的
封包多了這個欄位；沒有這個欄位韌體會直接丟棄整包指令。

2026-09-15：車1韌體的STA改用分段傳送修好了長訊息傳輸損毀問題
（本機技術文件 uart_tx_corruption_bug.md）後，CMD這個方向也對稱地
改用 send_chunked() 送出（沿用車2 car2_bringup/uart_protocol.py 的做法），
不再一次 ser.write() 整包送出。
"""

import argparse
import threading
import time

import serial


def _checksum(payload: str) -> int:
    cs = 0
    for ch in payload.encode("ascii"):
        cs ^= ch
    return cs


def build_cmd(fwa: int, bwl: int, bwr: int) -> bytes:
    fwa = max(-90, min(90, fwa))
    bwl = max(-100, min(100, bwl))
    bwr = max(-100, min(100, bwr))
    payload = f"CMD,fwa {fwa},bwl {bwl},bwr {bwr}"
    cs = _checksum(payload)
    return f"<{payload},cs {cs:02X}>\n".encode()


def send_chunked(ser: serial.Serial, data: bytes, chunk_size: int = 5, delay: float = 0.002) -> None:
    for i in range(0, len(data), chunk_size):
        ser.write(data[i:i + chunk_size])
        ser.flush()
        time.sleep(delay)


def reader_thread(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            chunk = ser.readline()
        except serial.SerialException:
            break
        if not chunk:
            continue
        line = chunk.decode(errors="replace").strip()
        if line:
            print(f"[RX {time.strftime('%H:%M:%S')}] {line}")


def main() -> None:
    parser = argparse.ArgumentParser(description="BMduino UART protocol test client")
    parser.add_argument("port", help="序列埠，例如 COM5 或 /dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    print("已連線。指令：")
    print("  s <fwa -90..90(0=直行,負=左轉,正=右轉)> <bwl -100..100> <bwr -100..100>   送出控制封包")
    print("  q                                                離開")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line == "q":
                break
            parts = line.split()
            if parts[0] == "s" and len(parts) == 4:
                try:
                    fwa, bwl, bwr = (int(p) for p in parts[1:])
                except ValueError:
                    print("用法: s <fwa> <bwl> <bwr>")
                    continue
                cmd = build_cmd(fwa, bwl, bwr)
                send_chunked(ser, cmd)
                print(f"[TX] {cmd.decode().strip()}")
            else:
                print("未知指令")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        stop_event.set()
        ser.close()


if __name__ == "__main__":
    main()
