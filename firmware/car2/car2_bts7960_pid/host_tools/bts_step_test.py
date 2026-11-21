#!/usr/bin/env python3
"""PID 步階測試（對 car2_bts7960_pid 韌體的 m 指令 + t 1 串流）。輪子必須離地。

  python bts_step_test.py <tag> [KP KI]      # 帶 KP KI 會先用 k 指令設定
計畫：0 -> 30 -> 60 -> 30 -> 100 -> 50 -> 15 -> 0（%），每階 STEP_S 秒。目標 rpm = (5 + 0.95*pct)% x 270。
輸出 results/bts_step_<tag>.csv，並印每一階的上升時間(10~90%)、過衝、穩態誤差(最後 0.7 s 平均)。
"""
import csv
import os
import statistics as st
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

TICKS_PER_REV = 1288.86
MAX_RPM_100 = 270.0
FLOOR = 5.0
STEP_S = 2.2
PLAN = [30, 60, 30, 100, 50, 15, 0]
OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)


def target_rpm(pct):
    return 0.0 if pct == 0 else (FLOOR + abs(pct) / 100.0 * (100 - FLOOR)) / 100.0 * MAX_RPM_100


def main():
    tag = sys.argv[1]
    port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 0.01
    ser.dtr = ser.rts = False
    ser.open()
    time.sleep(1.0)
    ser.reset_input_buffer()
    # 環境變數 PRE：開頭先送的指令，以分號分隔，例如 PRE="r 300;k 0.8 0.8"
    for _c in filter(None, os.environ.get("PRE", "").split(";")):
        ser.write((_c.strip() + "\n").encode())
        time.sleep(0.05)
    ser.write(b"s\nt 1\n")
    samples = []   # (ms, posL, posR, dutyL, dutyR, step_idx, pct)
    flags = [0]

    def pump(secs, idx, pct):
        t_end = time.time() + secs
        while time.time() < t_end:
            line = ser.readline()
            if not line:
                continue
            txt = line.decode("utf-8", "replace").strip()
            if txt.startswith("e,"):
                p = txt.split(",")
                try:
                    samples.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5]), idx, pct))
                except (ValueError, IndexError):
                    pass
            elif txt.startswith("dbg,"):
                try:
                    flags[0] = int(txt.split(",")[11])
                except (ValueError, IndexError):
                    pass

    try:
        pump(0.6, -1, 0)
        for i, pct in enumerate(PLAN):
            t_end = time.time() + STEP_S
            while time.time() < t_end:
                ser.write(f"m {pct} {pct}\n".encode())
                pump(0.4, i, pct)
                if flags[0] & 3:
                    print("韌體回報堵轉切斷 f=", flags[0])
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"s\ns\ns\nt 0\n")
        time.sleep(0.2)
        ser.close()

    # 轉速：用 4 個樣本(約 130 ms)的位置差
    rows = []
    for k in range(len(samples)):
        j = max(0, k - 3)
        a, b = samples[j], samples[k]
        dt = (b[0] - a[0]) / 1000.0
        rl = rr = 0.0
        if dt > 0:
            rl = (b[1] - a[1]) / TICKS_PER_REV / dt * 60
            rr = (b[2] - a[2]) / TICKS_PER_REV / dt * 60
        rows.append((b[0], rl, rr, b[3], b[4], b[5], b[6]))
    with open(OUT / f"bts_step_{tag}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["ms", "rpmL", "rpmR", "dutyL", "dutyR", "step", "pct"])
        w.writerows(rows)

    print(f"{'階':>2} {'指令%':>5} {'目標rpm':>7} | {'輪':>2} {'上升10-90%(s)':>13} {'過衝%':>6} {'穩態rpm':>8} {'誤差%':>6} {'穩態duty':>8}")
    prev_target = 0.0
    for i, pct in enumerate(PLAN):
        seg = [r for r in rows if r[5] == i]
        if len(seg) < 10:
            continue
        t0 = seg[0][0]
        tgt = target_rpm(pct)
        for name, col, dcol in (("L", 1, 3), ("R", 2, 4)):
            ys = [(r[0] - t0) / 1000.0 for r in seg], [r[col] for r in seg]
            ts, vs = ys
            tail = [v for t, v in zip(ts, vs) if t >= STEP_S - 0.8]
            dtail = [r[dcol] for r in seg if (r[0] - t0) / 1000.0 >= STEP_S - 0.8]
            ss = st.mean(tail) if tail else 0
            span = tgt - prev_target
            rise = None
            if abs(span) > 5:
                lo, hi = prev_target + 0.1 * span, prev_target + 0.9 * span
                t10 = next((t for t, v in zip(ts, vs) if (v - lo) * span >= 0), None)
                t90 = next((t for t, v in zip(ts, vs) if (v - hi) * span >= 0), None)
                if t10 is not None and t90 is not None:
                    rise = t90 - t10
            over = None
            if abs(span) > 5 and tgt > 0:
                peak = max(vs) if span > 0 else min(vs)
                over = (peak - tgt) / abs(span) * 100.0 * (1 if span > 0 else -1)
            err = (ss - tgt) / tgt * 100.0 if tgt > 0 else float("nan")
            print(f"{i:>2} {pct:>5} {tgt:>7.1f} | {name:>2} {('%.2f' % rise) if rise is not None else '-':>13} "
                  f"{('%.1f' % over) if over is not None else '-':>6} {ss:>8.1f} {err:>6.1f} {st.mean(dtail) if dtail else 0:>8.0f}")
        prev_target = tgt


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
