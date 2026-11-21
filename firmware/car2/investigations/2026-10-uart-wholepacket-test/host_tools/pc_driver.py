#!/usr/bin/env python3
"""電腦端總控 v2：讀 BMduino USB dbg 計數 + 透過 SSH 在 Orin 跑 wp_test.py，逐輪比對送收數量並排名。

前提：BMduino 已燒上 car2_car_control_rxcounters（USB dbg 行尾多 7 欄），USB 序列埠接在這台電腦。
輪速不為 0 的計畫需要 --i-confirm-wheels-lifted（輪子已離地）。

計畫：
  smoke  10 秒冒煙測試（整包 50 Hz，馬達轉）
  A1/A2  各種送法在 50 Hz、馬達轉動下比較（每種 60 秒）
  B      頻率天花板：整包 100/200/400/600 Hz、分段 0.5 ms 100/200 Hz、分段 1 ms 100 Hz（每輪 20 秒，失敗就停止往上推）
  C      Orin 上 4 個 CPU 忙碌行程 + 馬達轉動，複驗前幾名（每種 60 秒）

判準（每一輪，全部成立才 PASS）：
  Δ bmCmdOk == sent + 5(停止指令)；Δ bmBytes == 送出 byte（含停止指令實際長度）；Δ rxdr == Δ bytes
  Δ bmCsFail / bmAbort / bmOverflow / FE == 0
  Δ i2cSteerErr == 0 且 Δ dirI2cErr == 0 且 pcaReady == 1（馬達雜訊干擾 I2C 的指標）
  ack 對應指令的年齡(ms) 最大值 <= 250
資訊（不影響 PASS）：Orin 端 STA 壞包數、故障旗標、馬達實際最大轉速、單包送出耗時、達成的送包速率。
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

ORIN_USER = os.environ.get("ORIN_USER")
ORIN_HOST = os.environ.get("ORIN_HOST")
REMOTE_SCRIPT = "~/uart_wholepacket_test_20261008/wp_test.py"

DBG_NAMES = ["dbg", "targetL", "targetR", "dutyL", "dutyR", "rpmL", "rpmR", "i2cSteerCalls", "i2cSteerErr",
             "stallEvM1", "stallMaxM1", "stallEvM2", "stallMaxM2", "maxUs_pwmPump", "maxUs_poll", "maxUs_pid",
             "maxUs_status", "dirI2cCalls", "dirI2cErr", "pcaReady",
             "bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3"]

WHEEL = 60

METHODS = {   # label -> 送法參數
    "chunk5_2ms":    dict(mode="chunk", chunk=5, delay_ms=2.0),
    "chunk5_1ms":    dict(mode="chunk", chunk=5, delay_ms=1.0),
    "chunk5_0.5ms":  dict(mode="chunk", chunk=5, delay_ms=0.5),
    "chunk5_0.25ms": dict(mode="chunk", chunk=5, delay_ms=0.25),
    "chunk8_0.5ms":  dict(mode="chunk", chunk=8, delay_ms=0.5),
    "chunk16_0.5ms": dict(mode="chunk", chunk=16, delay_ms=0.5),
    "whole":         dict(mode="whole"),
    "whole_flush":   dict(mode="whole", flush=True),
}


def T(method, rate, duration, seq, wheel=WHEEL, load=0, burst=1, group=None, tag=""):
    label = f"{method}_{rate}Hz" + (f"x{burst}" if burst > 1 else "") + (f"_load{load}" if load else "") + tag
    d = dict(label=label, rate=rate, duration=duration, seq=seq, wheel=wheel, load=load, burst=burst, group=group)
    d.update(METHODS[method])
    return d


def build_plans():
    a_methods = ["chunk5_2ms", "chunk5_1ms", "chunk5_0.5ms", "chunk5_0.25ms", "chunk8_0.5ms", "chunk16_0.5ms", "whole", "whole_flush"]
    plans = {
        "smoke": [T("whole", 50, 10, 300, wheel=40)],
        "A1": [T(m, 50, 60, 1000 + i * 4000) for i, m in enumerate(a_methods[:4])],
        "A2": [T(m, 50, 60, 20000 + i * 4000) for i, m in enumerate(a_methods[4:])],
        "B": (
            [T("whole", r, 20, 40000 + i * 3000, group="whole") for i, r in enumerate([100, 200, 400, 600])]
            + [T("chunk5_0.5ms", r, 20, 52000 + i * 3000, group="c05") for i, r in enumerate([100, 200])]
            + [T("chunk5_1ms", 100, 20, 58000, group="c1")]
        ),
        "C": [
            T("chunk5_2ms", 50, 60, 1000, load=4),
            T("chunk5_0.5ms", 50, 60, 5000, load=4),
            T("whole", 50, 60, 9000, load=4),
            T("whole", 100, 60, 13000, load=4),
            T("whole", 10, 60, 17000, load=4, burst=5),
        ],
    }
    return plans


class DbgReader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.latest = None
        self.latest_t = 0.0
        self.history = []          # (monotonic, vals)
        self.raw = []
        self.stop_flag = threading.Event()
        self.lock = threading.Lock()
        self.bad_lines = 0

    def run(self):
        buf = b""
        while not self.stop_flag.is_set():
            try:
                chunk = self.ser.read(512)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if not text.startswith("dbg,"):
                    continue
                parts = text.split(",")
                if len(parts) != len(DBG_NAMES):
                    self.bad_lines += 1
                    continue
                try:
                    vals = dict(zip(DBG_NAMES[1:], [float(x) for x in parts[1:]]))
                except ValueError:
                    self.bad_lines += 1
                    continue
                now = time.monotonic()
                with self.lock:
                    self.latest, self.latest_t = vals, now
                    self.history.append((now, vals))
                    self.raw.append((datetime.now().isoformat(timespec="milliseconds"), text))

    def snapshot_after(self, t_after, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if self.latest is not None and self.latest_t > t_after:
                    return dict(self.latest), self.latest_t
            time.sleep(0.05)
        return None, None

    def window(self, t0, t1):
        with self.lock:
            return [v for t, v in self.history if t0 <= t <= t1]

    def close(self):
        self.stop_flag.set()
        self.join(timeout=1.0)
        self.ser.close()


def run_remote(tr, timeout):
    cmd = (f"python3 {REMOTE_SCRIPT} --label {tr['label']} --mode {tr['mode']} --rate {tr['rate']} "
           f"--duration {tr['duration']} --seq-start {tr['seq']} --burst {tr['burst']} "
           f"--wheel-max {tr['wheel']} --load {tr['load']}")
    if tr["wheel"] > 0:
        cmd += " --wheels-lifted"
    if tr["mode"] == "chunk":
        cmd += f" --chunk {tr['chunk']} --delay-ms {tr['delay_ms']}"
    if tr.get("flush"):
        cmd += " --flush"
    proc = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", f"{ORIN_USER}@{ORIN_HOST}", cmd],
        capture_output=True, text=True, timeout=timeout,
    )
    last = [ln for ln in proc.stdout.strip().splitlines() if ln.startswith("{")]
    if not last:
        return None, proc.stdout[-400:] + proc.stderr[-400:]
    return json.loads(last[-1]), None


def evaluate(res, a, b):
    keys = ("bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3", "i2cSteerErr", "dirI2cErr")
    d = {k: b[k] - a[k] for k in keys}
    exp_cmds = res["sent"] + res["stop_cmds"]
    exp_bytes = res["bytes_sent"] + res["stop_cmd_bytes"]
    age = res.get("ack_age_ms_max")
    checks = {
        "cmdOk==sent+5": d["bmCmdOk"] == exp_cmds,
        "bytes==sent": d["bmBytes"] == exp_bytes,
        "rxdr==bytes": d["rxdrS3"] == d["bmBytes"],
        "csFail==0": d["bmCsFail"] == 0,
        "abort==0": d["bmAbort"] == 0,
        "overflow==0": d["bmOverflow"] == 0,
        "FE==0": d["feS3"] == 0,
        "i2cErr==0": d["i2cSteerErr"] == 0 and d["dirI2cErr"] == 0 and b["pcaReady"] == 1,
        "ackAge<=250ms": age is not None and age <= 250,
    }
    return d, exp_cmds, exp_bytes, checks


def ranking(summary):
    rows = [s for s in summary if "orin" in s and "pass_" in s]
    rows.sort(key=lambda s: (not s["pass_"], s["orin"]["send_ms_mean"] or 1e9))
    print("\n=== 排名（先看是否 PASS，再看單包送出耗時）===")
    print(f"{'label':32} {'PASS':5} {'send ms mean/p99/max':24} {'pkt/s':>7} {'late':>5} {'ackAge max':>10} {'rpm max':>8}")
    for s in rows:
        o = s["orin"]
        print(f"{s['label']:32} {'yes' if s['pass_'] else 'NO':5} "
              f"{o['send_ms_mean']:>7}/{o['send_ms_p99']}/{o['send_ms_max']:<10} {o['achieved_rate_pkt_s']:>7} "
              f"{o['late_periods']:>5} {o['ack_age_ms_max']!s:>10} {s.get('rpm_max', 0):>8.0f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="BMduino USB 序列埠，例如 COM10")
    ap.add_argument("--plan", required=True, choices=["smoke", "A1", "A2", "B", "C"])
    ap.add_argument("--i-confirm-wheels-lifted", action="store_true", help="輪子已離地才可執行輪速不為 0 的計畫")
    ap.add_argument("--gap", type=float, default=3.0, help="每輪之間休息秒數")
    args = ap.parse_args()
    if not ORIN_USER or not ORIN_HOST:
        ap.error("請設定 ORIN_USER 和 ORIN_HOST")

    trials = build_plans()[args.plan]
    if any(t["wheel"] > 0 for t in trials) and not args.i_confirm_wheels_lifted:
        sys.exit("此計畫會讓馬達轉動，需要 --i-confirm-wheels-lifted（輪子已離地）")

    out_dir = Path(__file__).resolve().parent.parent / "results"
    out_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    rd = DbgReader(args.port)
    rd.start()
    print(f"[pc] 讀取 {args.port}（開啟序列埠會讓 BMduino 重開機），等待 dbg 行 ...")
    first, _ = rd.snapshot_after(0, timeout=8.0)
    if first is None:
        rd.close()
        sys.exit("[pc] 8 秒內沒有收到 dbg 行：確認韌體是 rxcounters 版、序列埠沒被其他程式占用")
    time.sleep(2.0)
    print(f"[pc] OK，pcaReady={first['pcaReady']:.0f}")

    summary, failed_groups = [], set()
    for tr in trials:
        if tr["group"] and tr["group"] in failed_groups:
            print(f"\n=== {tr['label']}: 略過（同組較低頻率已失敗）===")
            continue
        print(f"\n=== {tr['label']}: {tr['mode']}"
              + (f" chunk{tr['chunk']}/{tr['delay_ms']}ms" if tr["mode"] == "chunk" else "")
              + (" +flush" if tr.get("flush") else "")
              + f" {tr['rate']} Hz×{tr['burst']} × {tr['duration']} s, 輪速上限 {tr['wheel']}%, CPU 負載 {tr['load']} ===")
        time.sleep(args.gap)
        a, ta = rd.snapshot_after(time.monotonic())
        res, err = run_remote(tr, timeout=tr["duration"] + 60)
        if res is None or res.get("error"):
            print("[pc] Orin 端錯誤/無回傳：", err or res)
            summary.append(dict(label=tr["label"], error=err or res))
            break
        b, tb = rd.snapshot_after(time.monotonic())
        if a is None or b is None:
            print("[pc] 取不到 dbg 快照")
            summary.append(dict(label=tr["label"], orin=res, error="no dbg snapshot"))
            break
        d, exp_cmds, exp_bytes, checks = evaluate(res, a, b)
        ok = all(checks.values())
        win = rd.window(ta, tb)
        rpm_max = max([max(abs(v["rpmL"]), abs(v["rpmR"])) for v in win] or [0])
        o = res
        print(f"  Orin：{o['sent']} 包 / {o['bytes_sent']} B，達成 {o['achieved_rate_pkt_s']} 包/秒；單包送出 mean {o['send_ms_mean']} "
              f"p99 {o['send_ms_p99']} p99.9 {o['send_ms_p999']} max {o['send_ms_max']} ms；落後週期 {o['late_periods']}；"
              f"loadavg {o['loadavg_before']}→{o['loadavg_after']}")
        print(f"  Orin：STA ok {o['sta_ok']} bad {o['sta_bad']} 故障旗標非0 {o['sta_fault_nonzero']}；"
              f"ack 對應指令年齡 mean {o['ack_age_ms_mean']} max {o['ack_age_ms_max']} ms")
        print(f"  BM  ：Δ合法CMD {d['bmCmdOk']:.0f}（預期 {exp_cmds}）  Δbytes {d['bmBytes']:.0f}（預期 {exp_bytes}）  "
              f"ΔcsFail {d['bmCsFail']:.0f}  Δabort {d['bmAbort']:.0f}  Δoverflow {d['bmOverflow']:.0f}  ΔFE {d['feS3']:.0f}  "
              f"ΔI2C錯誤 {d['i2cSteerErr'] + d['dirI2cErr']:.0f}")
        print(f"  BM  ：馬達實際最大轉速 {rpm_max:.0f} rpm；loop 區段最大耗時 µs（累計）poll {b['maxUs_poll']:.0f} "
              f"pid {b['maxUs_pid']:.0f} status {b['maxUs_status']:.0f}")
        if tr["wheel"] > 0 and rpm_max < 20:
            print("  ！警告：馬達最大轉速 < 20 rpm，馬達可能沒有真的轉（雜訊測試無效）")
        print("  判定：" + ("PASS" if ok else "FAIL  → " + ", ".join(k for k, v in checks.items() if not v)))
        summary.append(dict(label=tr["label"], pass_=ok, checks=checks, deltas=d, expected=dict(cmds=exp_cmds, bytes=exp_bytes),
                            rpm_max=rpm_max, orin=res, dbg_before=a, dbg_after=b, trial=tr))
        if not ok and tr["group"]:
            failed_groups.add(tr["group"])

    ranking(summary)
    (out_dir / f"results_{args.plan}_{stamp}.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    with open(out_dir / f"dbg_raw_{args.plan}_{stamp}.log", "w", encoding="utf-8") as f:
        for ts, line in rd.raw:
            f.write(f"{ts} {line}\n")
    print(f"\n[pc] 結果已存到 {out_dir}（bad dbg lines: {rd.bad_lines}）")
    rd.close()


if __name__ == "__main__":
    main()
