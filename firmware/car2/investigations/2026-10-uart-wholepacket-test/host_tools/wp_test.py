#!/usr/bin/env python3
"""Orin 端：CMD 送法比較實驗（2026-10-08，v2：可帶馬達轉速與 CPU 負載）。

以固定頻率送 CMD，記錄送出端與 STA 回報端的統計，輸出一行 JSON。
BM 端的接收統計由 USB dbg 行提供，由電腦端 pc_driver.py 擷取比對。

安全設計：
  - 預設 bwl / bwr 恆為 0。要讓輪子轉，必須同時帶 --wheel-max N 與 --wheels-lifted
    （使用者確認輪子已離地）；N 硬上限 WHEEL_CAP(60)。
  - 輪速在 0 -> 2/3N -> N -> 2/3N -> 0 之間每 STEP_S 秒換一階，只往前轉，不反轉。
  - fwa 只在 +-FWA_AMP_MAX 以內每 0.5 秒切換一次。
  - 結束時一定以最保守的分段方式送 5 包全 0 的停止指令（輪速、轉向都歸零）。
  - 啟動前檢查 /dev/ttyTHS1 沒有被 uart_bridge_node 等程式占用。
"""
import argparse
import glob
import json
import os
import statistics
import subprocess
import sys
import threading
import time
from typing import Optional

import serial

PORT = "/dev/ttyTHS1"
BAUD = 460800
FWA_AMP_MAX = 10          # 硬上限：即使參數給更大也會被夾住
WHEEL_CAP = 60            # 硬上限（目標速度百分比）
FLIP_PERIOD_S = 0.5       # 轉向切換週期
STEP_S = 1.5              # 輪速每階停留時間
PROFILE = [0.0, 2 / 3, 1.0, 2 / 3, 0.0]


def checksum(payload: str) -> int:
    cs = 0
    for ch in payload.encode("ascii"):
        cs ^= ch
    return cs


def build_cmd(seq: int, fwa: int, bwl: int, bwr: int) -> bytes:
    payload = f"CMD,seq {seq % 65536},fwa {fwa},bwl {bwl},bwr {bwr}"
    return f"<{payload},cs {checksum(payload):02X}>\n".encode()


def parse_sta(line: bytes) -> Optional[dict]:
    try:
        text = line.decode("ascii", errors="strict").strip()
    except UnicodeDecodeError:
        return None
    if not (text.startswith("<") and text.endswith(">")):
        return None
    parts = text[1:-1].split(",")
    if not parts or parts[0] != "STA":
        return None
    cs_value = None
    kv = []
    for part in parts[1:]:
        tok = part.strip().split(" ", 1)
        if len(tok) != 2:
            return None
        if tok[0] == "cs":
            try:
                cs_value = int(tok[1], 16)
            except ValueError:
                return None
        else:
            kv.append((tok[0], tok[1]))
    if cs_value is None:
        return None
    if checksum("STA," + ",".join(f"{k} {v}" for k, v in kv)) != cs_value:
        return None
    return dict(kv)


def write_packet(ser, data: bytes, mode: str, chunk: int, delay_s: float, flush: bool) -> None:
    if mode == "whole":
        ser.write(data)
    else:
        n = len(data)
        for i in range(0, n, chunk):
            ser.write(data[i:i + chunk])
            if i + chunk < n:           # 最後一段後不再睡
                time.sleep(delay_s)
    if flush:
        ser.flush()                      # tcdrain：等硬體真的送完（Orin ttyTHS1 上每次約 8 ms）


def port_in_use() -> list:
    hits = []
    target = os.path.realpath(PORT)
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.path.realpath(fd) == target:
                hits.append(fd.split("/")[2])
        except OSError:
            continue
    try:
        out = subprocess.run(
            ["pgrep", "-af", "uart_bridge_node|car2_bringup|line_follow_control|raw_serial_test"],
            capture_output=True, text=True, timeout=5,
        ).stdout.strip()
        for ln in out.splitlines():
            if "pgrep" not in ln and str(os.getpid()) != ln.split()[0]:
                hits.append(ln[:100])
    except Exception:
        pass
    return hits


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--mode", choices=["chunk", "whole"], required=True)
    ap.add_argument("--chunk", type=int, default=5)
    ap.add_argument("--delay-ms", type=float, default=2.0)
    ap.add_argument("--flush", action="store_true", help="每包寫完後 ser.flush()")
    ap.add_argument("--rate", type=float, required=True, help="每秒送幾次(每次 burst 包)")
    ap.add_argument("--burst", type=int, default=1, help="每個週期連續送幾包")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--seq-start", type=int, default=1000)
    ap.add_argument("--fwa-amp", type=int, default=8)
    ap.add_argument("--wheel-max", type=int, default=0, help="輪速上限(百分比)，需搭配 --wheels-lifted")
    ap.add_argument("--wheels-lifted", action="store_true", help="使用者確認輪子已離地")
    ap.add_argument("--load", type=int, default=0, help="在 Orin 上開幾個 CPU 忙碌行程模擬感知負載")
    ap.add_argument("--no-port-check", action="store_true")
    args = ap.parse_args()

    amp = max(0, min(FWA_AMP_MAX, abs(args.fwa_amp)))
    wheel_max = max(0, min(WHEEL_CAP, args.wheel_max))
    if wheel_max > 0 and not args.wheels_lifted:
        print(json.dumps({"label": args.label, "error": "wheel_max>0 requires --wheels-lifted"}, ensure_ascii=False))
        return 4

    if not args.no_port_check:
        hits = port_in_use()
        if hits:
            print(json.dumps({"label": args.label, "error": "port_in_use", "hits": hits}, ensure_ascii=False))
            return 3

    workers = []
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = PORT, BAUD, 0.05
    ser.rtscts = ser.dsrdtr = ser.xonxoff = False
    ser.open()
    ser.reset_input_buffer()

    stop = threading.Event()
    sent_log = []
    sent_time = {}
    running = [True]
    sta = {"ok": 0, "bad": 0, "ack": [], "lag": [], "age": [], "fault": 0}
    lock = threading.Lock()

    def reader():
        buf = b""
        while not stop.is_set():
            try:
                chunk = ser.read(256)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                parsed = parse_sta(line + b"\n")
                now = time.monotonic()
                with lock:
                    if parsed is None:
                        sta["bad"] += 1
                        continue
                    sta["ok"] += 1
                    if parsed.get("f", "0") not in ("0", ""):
                        sta["fault"] += 1
                    try:
                        ack = int(parsed.get("ackseq", "-1"))
                    except ValueError:
                        continue
                    sta["ack"].append(ack)
                    t_ack = sent_time.get(ack)
                    if t_ack is not None and running[0]:
                        sta["age"].append((now - t_ack) * 1000.0)
                    latest = None
                    for t, s in reversed(sent_log[-300:]):
                        if t <= now:
                            latest = s
                            break
                    if latest is not None and ack >= 0 and running[0]:
                        sta["lag"].append((latest - ack) % 65536)

    th = threading.Thread(target=reader, daemon=True)
    th.start()

    period = 1.0 / args.rate
    seq = args.seq_start
    durations = []
    late = 0
    bytes_sent = 0
    sent = 0
    stop_bytes = 0
    err = None
    loadavg0 = os.getloadavg()[0]
    t_start = t_end = time.monotonic()
    try:
        for _ in range(max(0, args.load)):
            workers.append(subprocess.Popen([sys.executable, "-c", "while True: pass"]))
        if workers:
            time.sleep(1.0)          # 讓負載先拉起來

        t_start = time.monotonic()
        next_t = t_start
        while True:
            now = time.monotonic()
            if now - t_start >= args.duration:
                break
            if now < next_t:
                dt = next_t - now
                if dt > 0.0015:
                    time.sleep(dt - 0.001)
                continue
            if now - next_t > period:
                late += 1
                next_t = now          # 落後超過一個週期就重新對時，不補送（不製造人為突發）
            rel = now - t_start
            fwa = amp if int(rel / FLIP_PERIOD_S) % 2 == 0 else -amp
            wl = round(wheel_max * PROFILE[int(rel / STEP_S) % len(PROFILE)])
            for _b in range(max(1, args.burst)):
                data = build_cmd(seq, fwa, wl, wl)
                t0 = time.monotonic()
                write_packet(ser, data, args.mode, args.chunk, args.delay_ms / 1000.0, args.flush)
                t1 = time.monotonic()
                with lock:
                    sent_log.append((t0, seq))
                    sent_time[seq] = t0
                durations.append((t1 - t0) * 1000.0)
                bytes_sent += len(data)
                sent += 1
                seq = (seq + 1) % 65536
            next_t += period
        t_end = time.monotonic()
    except Exception as e:           # noqa: BLE001
        err = repr(e)
        t_end = time.monotonic()
    finally:
        running[0] = False
        loadavg1 = os.getloadavg()[0]
        for w in workers:
            try:
                w.kill()
            except Exception:       # noqa: BLE001
                pass
        for _ in range(5):          # 停止指令：用最保守的 chunk5/2ms 送，確保一定送達
            try:
                data = build_cmd(seq, 0, 0, 0)
                with lock:
                    sent_log.append((time.monotonic(), seq))
                    sent_time[seq] = time.monotonic()
                write_packet(ser, data, "chunk", 5, 0.002, False)
                stop_bytes += len(data)
            except Exception as e:  # noqa: BLE001
                err = (err or "") + " stopfail:" + repr(e)
            seq = (seq + 1) % 65536
            time.sleep(0.05)
        time.sleep(0.6)
        stop.set()
        th.join(timeout=1.0)
        ser.close()

    with lock:
        lag = list(sta["lag"])
        age = list(sta["age"])
        acks = list(sta["ack"])
    ds = sorted(durations)

    def pct(p):
        return round(ds[min(len(ds) - 1, int(len(ds) * p))], 3) if ds else None

    elapsed = max(1e-6, t_end - t_start)
    result = {
        "label": args.label, "mode": args.mode,
        "chunk": args.chunk if args.mode == "chunk" else None,
        "delay_ms": args.delay_ms if args.mode == "chunk" else None,
        "flush": args.flush, "rate_hz": args.rate, "burst": args.burst, "duration_s": args.duration,
        "fwa_amp": amp, "wheel_max": wheel_max, "load_workers": args.load,
        "loadavg_before": round(loadavg0, 2), "loadavg_after": round(loadavg1, 2),
        "seq_first": args.seq_start,
        "sent": sent, "bytes_sent": bytes_sent, "stop_cmds": 5, "stop_cmd_bytes": stop_bytes,
        "achieved_rate_pkt_s": round(sent / elapsed, 2),
        "send_ms_mean": round(statistics.mean(durations), 3) if durations else None,
        "send_ms_p50": pct(0.5), "send_ms_p99": pct(0.99), "send_ms_p999": pct(0.999),
        "send_ms_max": round(max(durations), 3) if durations else None,
        "late_periods": late,
        "sta_ok": sta["ok"], "sta_bad": sta["bad"], "sta_fault_nonzero": sta["fault"],
        "ack_first": acks[0] if acks else None, "ack_last": acks[-1] if acks else None,
        "ack_lag_pkts_mean": round(statistics.mean(lag), 2) if lag else None,
        "ack_lag_pkts_max": max(lag) if lag else None,
        "ack_age_ms_mean": round(statistics.mean(age), 1) if age else None,
        "ack_age_ms_max": round(max(age), 1) if age else None,
        "error": err,
    }
    print(json.dumps(result, ensure_ascii=False))
    return 0 if err is None else 2


if __name__ == "__main__":
    sys.exit(main())
