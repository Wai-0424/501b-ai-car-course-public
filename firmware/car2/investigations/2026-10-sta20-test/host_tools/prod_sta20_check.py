#!/usr/bin/env python3
"""驗證「已搬進正式版」的 STA 20 Hz（不是測試版）：Orin 端以整包送 CMD、量 STA 到達；電腦端讀 USB dbg 的健康指標。

  python prod_sta20_check.py --port COM10 --duration 120 --rate 60 --wheel 60 --i-confirm-wheels-lifted

前提：BMduino 燒的是 car2_car_control（STA 20 Hz 版，開機訊息有「STA 20Hz」），dbg 行為 24 欄（2026-10-10 加 turbo 後；STA 20 Hz 搬進時是 22 欄）；
Orin 上已有 ~/uart_sta20_test_20261010/sta20_orin.py（sta20_pc.py --deploy 會複製）且沒有 ROS 在用 UART。
判定（全部成立才 PASS）：
  Orin 端 STA 送達率 >= 0.95 且到達間隔 p99 <= 60 ms
  Δ staSkipBM == 0 且 Δ staSkipUSB == 0
  maxLoopGapUs < 5000（開機到現在的累計最大值）
  I2C 錯誤 == 0、pcaReady == 1、馬達堵轉事件(stallEv)不增加
正式版沒有 CMD 接收計數器（那是測試版才有），CMD 完整性靠測試版的結果，這裡不重複。
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import serial

ORIN_USER = os.environ.get("ORIN_USER")
REMOTE = "~/uart_sta20_test_20261010/sta20_orin.py"
NAMES = ["targetL", "targetR", "dutyL", "dutyR", "rpmL", "rpmR", "i2cSteerCalls", "i2cSteerErr", "stallEvM1", "stallMaxM1",
         "stallEvM2", "stallMaxM2", "maxUs_pwmPump", "maxUs_poll", "maxUs_pid", "maxUs_status", "dirI2cCalls", "dirI2cErr",
         "pcaReady", "staSkipBM", "staSkipUSB", "maxLoopGapUs", "turboState", "turboBudgetMs"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--duration", type=float, default=120)
    ap.add_argument("--rate", type=float, default=60)
    ap.add_argument("--wheel", type=int, default=60)
    ap.add_argument("--host", default=os.environ.get("ORIN_HOST"), help="Orin 位址，可用 ORIN_HOST 設定")
    ap.add_argument("--load", type=int, default=0)
    ap.add_argument("--i-confirm-wheels-lifted", action="store_true")
    args = ap.parse_args()
    if not ORIN_USER or not args.host:
        ap.error("請設定 ORIN_USER，並設定 ORIN_HOST 或傳入 --host")
    if args.wheel > 0 and not args.i_confirm_wheels_lifted:
        sys.exit("馬達會轉，需要 --i-confirm-wheels-lifted")

    ser = serial.Serial(args.port, 115200, timeout=0.3)
    state = {"last": None, "banner": [], "n": 0, "bad": 0}
    stop = threading.Event()

    def reader():
        buf = b""
        while not stop.is_set():
            try:
                c = ser.read(512)
            except serial.SerialException:
                return
            buf += c
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                t = ln.decode("utf-8", "replace").strip()
                if t.startswith("==="):
                    state["banner"].append(t)
                elif t.startswith("dbg,"):
                    p = t.split(",")[1:]
                    if len(p) != len(NAMES):
                        state["bad"] += 1
                        continue
                    try:
                        state["last"] = dict(zip(NAMES, [float(x) for x in p]))
                        state["n"] += 1
                    except ValueError:
                        state["bad"] += 1

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    t0 = time.time()
    while state["last"] is None and time.time() - t0 < 10:
        time.sleep(0.1)
    if state["last"] is None:
        sys.exit("沒收到 24 欄的 dbg 行：確認板上是搬進 STA 20 Hz 的正式版")
    time.sleep(2.0)
    banner = " | ".join(state["banner"])
    print("[pc] 開機訊息:", banner)
    a = dict(state["last"])

    cmd = (f"python3 {REMOTE} --label prod_check --rate {args.rate} --duration {args.duration} --seq-start 1000 "
           f"--wheel-max {args.wheel} --load {args.load} --sta-hz 20" + (" --wheels-lifted" if args.wheel > 0 else ""))
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", f"{ORIN_USER}@{args.host}", cmd],
                       capture_output=True, text=True, timeout=args.duration + 90)
    js = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
    if not js:
        stop.set()
        sys.exit("Orin 端沒有回傳: " + (r.stdout + r.stderr)[-400:])
    o = json.loads(js[-1])
    time.sleep(1.5)
    b = dict(state["last"])
    stop.set()
    th.join(timeout=1)
    ser.close()

    d = lambda k: b[k] - a[k]
    checks = {
        "staDelivery>=0.95": o["sta_delivery_ratio"] >= 0.95,
        "staGap p99<=60ms": (o["sta_gap_ms_p99"] or 1e9) <= 60,
        "staSkipBM==0": d("staSkipBM") == 0,
        "staSkipUSB==0": d("staSkipUSB") == 0,
        "maxLoopGap<5ms": b["maxLoopGapUs"] < 5000,
        "i2cErr==0": d("i2cSteerErr") == 0 and d("dirI2cErr") == 0 and b["pcaReady"] == 1,
        "stallEv不增加": d("stallEvM1") == 0 and d("stallEvM2") == 0,
        "Orin無錯誤": o.get("error") is None,
    }
    print(f"  Orin 送出 {o['sent']} 包 CMD（{o['rate_hz']} Hz 整包，單包 {o['send_ms_mean']} ms，落後週期 {o['late_periods']}）；"
          f"馬達最大 {o['rpm_max_seen']} rpm；故障旗標非 0: {o['fault_nonzero']}")
    print(f"  Orin 收 STA {o['sta_ok']} 個 = {o['sta_ok_rate_hz']} Hz（送達率 {o['sta_delivery_ratio']}）；壞包 {o['sta_bad']}（{o['sta_bad_pct']}%，"
          f"含開啟序列埠時抓到的半行）；到達間隔 p50 {o['sta_gap_ms_p50']} p99 {o['sta_gap_ms_p99']} max {o['sta_gap_ms_max']} ms；"
          f"每秒收到 {o['sta_per_sec_min']}~{o['sta_per_sec_max']} 個；ack 年齡 mean {o['ack_age_ms_mean']} max {o['ack_age_ms_max']} ms")
    print(f"  BM  Δ略過(BM) {d('staSkipBM'):.0f}  Δ略過(USB) {d('staSkipUSB'):.0f}  maxLoopGap {b['maxLoopGapUs']:.0f} µs（累計）；"
          f"poll {b['maxUs_poll']:.0f} pid {b['maxUs_pid']:.0f} status {b['maxUs_status']:.0f} µs（累計）；ΔI2C錯誤 {d('i2cSteerErr') + d('dirI2cErr'):.0f}；"
          f"Δ堵轉事件 L {d('stallEvM1'):.0f} R {d('stallEvM2'):.0f}")
    ok = all(checks.values())
    print("  判定：" + ("PASS" if ok else "FAIL → " + ", ".join(k for k, v in checks.items() if not v)))
    out = Path(__file__).resolve().parent.parent / "results" / f"prod_check_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps({"banner": banner, "orin": o, "dbg_before": a, "dbg_after": b, "checks": checks, "pass": ok},
                              ensure_ascii=False, indent=2), encoding="utf-8")
    print("  ->", out.name)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
