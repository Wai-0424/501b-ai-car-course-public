#!/usr/bin/env python3
"""Orin 端：以固定頻率「整包」送 CMD，同時量 STA 回報的到達率與到達間隔。輸出一行 JSON。

用途：驗證「BM 收 CMD 60 Hz + 回 STA 20 Hz」會不會堵塞（見 ../README.md）。
BM 端的接收/傳送計數由 USB dbg 行提供，由電腦端 sta20_pc.py 擷取並比對。

安全設計（沿用 2026-10-08 的 wp_test.py）：
  - 預設 bwl / bwr 恆為 0。要讓輪子轉，必須同時帶 --wheel-max N 與 --wheels-lifted（使用者確認輪子已離地）；
    N 硬上限 WHEEL_CAP(60)。
  - 輪速在 0 -> 2/3N -> N -> 2/3N -> 0 之間每 STEP_S 秒換一階，只往前轉，不反轉。
  - fwa 只在 +-FWA_AMP_MAX 以內每 0.5 秒切換。
  - 結束時一定用最保守的分段方式送 5 包全 0 的停止指令。
  - 啟動前檢查 /dev/ttyTHS1 沒有被 uart_bridge_node 等程式占用。
量測細節：
  - 讀取執行緒用「先阻塞等 1 byte、再把已到的全部讀走」，讓到達時間戳的誤差約 1 ms；若用固定大小 read 加逾時，
    50 ms 的 STA 週期會被讀取批次量化。時間戳是「收到換行(STA 結尾)」的時間。
  - STA 統計只算 [開始送 CMD, 停止送 CMD] 這段時間內到達的。
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
FWA_AMP_MAX = 10
WHEEL_CAP = 60
FLIP_PERIOD_S = 0.5
STEP_S = 1.5
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


def write_packet(ser, data: bytes, mode: str) -> None:
    if mode == "whole":
        ser.write(data)
    else:                                   # 5 byte / 2 ms：舊做法與停止指令用
        for i in range(0, len(data), 5):
            ser.write(data[i:i + 5])
            if i + 5 < len(data):
                time.sleep(0.002)


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
            capture_output=True, text=True, timeout=5).stdout.strip()
        for ln in out.splitlines():
            if "pgrep" not in ln and str(os.getpid()) != ln.split()[0]:
                hits.append(ln[:100])
    except Exception:
        pass
    return hits


def pct(sorted_vals, p):
    return round(sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * p))], 3) if sorted_vals else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--rate", type=float, default=60.0, help="CMD 每秒送幾包")
    ap.add_argument("--mode", choices=["whole", "chunk"], default="whole")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--seq-start", type=int, default=1000)
    ap.add_argument("--fwa-amp", type=int, default=8)
    ap.add_argument("--wheel-max", type=int, default=0)
    ap.add_argument("--wheels-lifted", action="store_true")
    ap.add_argument("--sta-hz", type=float, default=20.0, help="預期的 STA 頻率，用來算到達率")
    ap.add_argument("--load", type=int, default=0, help="在 Orin 上開幾個 CPU 忙碌行程")
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

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = PORT, BAUD, 0.05
    ser.rtscts = ser.dsrdtr = ser.xonxoff = False
    ser.open()
    ser.reset_input_buffer()

    stop = threading.Event()
    lock = threading.Lock()
    sent_time = {}
    arrivals = []                    # (monotonic, ackseq, rpmL, rpmR, f)
    bad_times = []
    bad_samples = []
    state = {"running": False}

    def handle(line: bytes, t: float):
        parsed = parse_sta(line)
        with lock:
            if parsed is None:
                if line.strip():
                    bad_times.append(t)
                    if len(bad_samples) < 8:
                        bad_samples.append(repr(line[:120]))
                return
            try:
                ack = int(parsed.get("ackseq", "-1"))
                rl = float(parsed.get("rpmL", "0"))
                rr = float(parsed.get("rpmR", "0"))
                f = int(parsed.get("f", "0"))
                sn = int(parsed.get("sn", "-1"))
            except ValueError:
                bad_times.append(t)
                return
            arrivals.append((t, ack, rl, rr, f, sn))

    def reader():
        buf = b""
        while not stop.is_set():
            try:
                b = ser.read(1)
                if not b:
                    continue
                n = ser.in_waiting
                if n:
                    b += ser.read(n)
            except (serial.SerialException, OSError):
                break
            t = time.monotonic()
            buf += b
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                handle(line + b"\n", t)

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
    workers = []
    loadavg0 = os.getloadavg()[0]
    t_start = t_end = time.monotonic()
    try:
        for _ in range(max(0, args.load)):
            workers.append(subprocess.Popen([sys.executable, "-c", "while True: pass"]))
        if workers:
            time.sleep(1.0)
        t_start = time.monotonic()
        state["running"] = True
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
                next_t = now
            rel = now - t_start
            fwa = amp if int(rel / FLIP_PERIOD_S) % 2 == 0 else -amp
            wl = round(wheel_max * PROFILE[int(rel / STEP_S) % len(PROFILE)])
            data = build_cmd(seq, fwa, wl, wl)
            t0 = time.monotonic()
            write_packet(ser, data, args.mode)
            t1 = time.monotonic()
            with lock:
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
        state["running"] = False
        loadavg1 = os.getloadavg()[0]
        for w in workers:
            try:
                w.kill()
            except Exception:       # noqa: BLE001
                pass
        for _ in range(5):          # 停止指令：最保守的分段送法，確保送達
            try:
                data = build_cmd(seq, 0, 0, 0)
                write_packet(ser, data, "chunk")
                stop_bytes += len(data)
            except Exception as e:  # noqa: BLE001
                err = (err or "") + " stopfail:" + repr(e)
            seq = (seq + 1) % 65536
            time.sleep(0.05)
        time.sleep(0.6)
        stop.set()
        th.join(timeout=1.0)
        ser.close()

    elapsed = max(1e-6, t_end - t_start)
    with lock:
        win = [a for a in arrivals if t_start <= a[0] <= t_end]
        bad_win = [t for t in bad_times if t_start <= t <= t_end]
        st = dict(sent_time)
    ts = [a[0] for a in win]
    gaps = sorted((b - a) * 1000.0 for a, b in zip(ts, ts[1:]))
    exp_period_ms = 1000.0 / args.sta_hz
    age = []
    for t, ack, *_ in win:
        t0 = st.get(ack)
        if t0 is not None:
            age.append((t - t0) * 1000.0)
    per_sec = {}
    for t in ts:
        per_sec[int(t - t_start)] = per_sec.get(int(t - t_start), 0) + 1
    per_sec_vals = [per_sec.get(i, 0) for i in range(int(elapsed))]
    ds = sorted(durations)
    rpm_max = max([max(abs(a[2]), abs(a[3])) for a in win] or [0.0])
    acks = [a[1] for a in win]
    ok_n, bad_n = len(win), len(bad_win)
    sns = [a[5] for a in win if a[5] >= 0]
    sn_span = (sns[-1] - sns[0] + 1) if sns else 0
    sn_missing = sn_span - len(sns) if sns else None          # 序號缺口 = 遺失 + 損毀（含 BM 端略過的）
    sn_dups = len(sns) - len(set(sns)) if sns else None
    result = {
        "label": args.label, "mode": args.mode, "rate_hz": args.rate, "duration_s": args.duration,
        "wheel_max": wheel_max, "load_workers": args.load,
        "loadavg_before": round(loadavg0, 2), "loadavg_after": round(loadavg1, 2),
        "seq_first": args.seq_start, "sent": sent, "bytes_sent": bytes_sent, "stop_cmds": 5, "stop_cmd_bytes": stop_bytes,
        "achieved_rate_pkt_s": round(sent / elapsed, 2), "late_periods": late,
        "send_ms_mean": round(statistics.mean(durations), 3) if durations else None,
        "send_ms_p99": pct(ds, 0.99), "send_ms_max": round(max(durations), 3) if durations else None,
        "elapsed_s": round(elapsed, 3),
        "sta_expected_hz": args.sta_hz,
        "sta_ok": ok_n, "sta_bad": bad_n,
        "sta_ok_rate_hz": round(ok_n / elapsed, 3),
        "sta_delivery_ratio": round(ok_n / (args.sta_hz * elapsed), 4),
        "sta_bad_pct": round(100.0 * bad_n / max(1, ok_n + bad_n), 3),
        "sta_gap_ms_mean": round(statistics.mean(gaps), 2) if gaps else None,
        "sta_gap_ms_p50": pct(gaps, 0.5), "sta_gap_ms_p95": pct(gaps, 0.95), "sta_gap_ms_p99": pct(gaps, 0.99),
        "sta_gap_ms_min": round(gaps[0], 2) if gaps else None, "sta_gap_ms_max": round(gaps[-1], 2) if gaps else None,
        "sta_gaps_over_1p5_period": sum(1 for g in gaps if g > 1.5 * exp_period_ms),
        "sta_gaps_over_2p5_period": sum(1 for g in gaps if g > 2.5 * exp_period_ms),
        "sta_per_sec_min": min(per_sec_vals) if per_sec_vals else None,
        "sta_per_sec_max": max(per_sec_vals) if per_sec_vals else None,
        "ack_first": acks[0] if acks else None, "ack_last": acks[-1] if acks else None,
        "ack_age_ms_mean": round(statistics.mean(age), 1) if age else None,
        "ack_age_ms_max": round(max(age), 1) if age else None,
        "sn_first": sns[0] if sns else None, "sn_last": sns[-1] if sns else None, "sn_span": sn_span,
        "sn_missing": sn_missing, "sn_missing_pct": round(100.0 * sn_missing / sn_span, 3) if sns and sn_span else None,
        "sn_dups": sn_dups,
        "fault_nonzero": sum(1 for a in win if a[4] != 0), "rpm_max_seen": round(rpm_max, 1),
        "bad_samples": bad_samples,
        "error": err,
    }
    print(json.dumps(result, ensure_ascii=False))
    return 0 if err is None else 2


if __name__ == "__main__":
    sys.exit(main())
