#!/usr/bin/env python3
"""電腦端總控：驗證「BM 收 CMD 60 Hz + 回 STA 20 Hz」會不會堵塞。

前提：
  - BMduino 已燒上 make_sta20.py 產生的 car2_car_control_sta20（開機訊息有 [TEST sta20]），USB 接在這台電腦。
  - Orin 上沒有 uart_bridge_node 等程式在用 /dev/ttyTHS1；Orin 端腳本用 --deploy 複製過去
    （只會新建 ~/uart_sta20_test_20261010/，不動 Orin 上既有檔案）。
  - 會讓馬達轉動的計畫必須帶 --i-confirm-wheels-lifted（輪子已離地），輪速上限寫死 60%。
注意：開啟 USB 序列埠會讓 BMduino 重開機，計數器歸零（這是預期的，每次執行是一個乾淨的起點）。

計畫：
  smoke  10 秒冒煙（CMD 60 Hz，輪速 40%）
  M      CMD 60 Hz 120 s / CMD 30 Hz 120 s / CMD 60 Hz + Orin 4 個 CPU 忙碌行程 120 s / CMD 100 Hz 60 s
  long   CMD 60 Hz 600 s

判定（HARD = 回答「會不會堵塞」；全部成立才 PASS）：
  CMD 完整性：Δ bmCmdOk == 送出 + 5(停止指令)；Δ bmBytes == 送出 byte；Δ rxdr == Δ bytes；
              Δ csFail / abort / overflow / FE == 0
  傳送不重疊：Δ staSkipBM == 0 且 Δ staSkipUSB == 0（上一則 STA / dbg 還沒送完就又要送 = 佇列塞住）
  不堵塞    ：Δ gap5ms == 0（loop() 相鄰兩次起點間隔 > 5 ms 的次數）
  I2C       ：Δ i2cSteerErr == 0 且 Δ dirI2cErr == 0 且 pcaReady == 1
  STA 送達  ：Orin 端 STA 送達率 >= 0.95（收到的 / (20 Hz x 秒數)），且序號重複數 == 0
SOFT（只報告）：STA 壞包率、序號缺口率(遺失+損毀)、STA 到達間隔 p99/max、loop 間隔最大值與 >1/2 ms 次數、ack 年齡。
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
REMOTE_DIR = "~/uart_sta20_test_20261010"
REMOTE_SCRIPT = f"{REMOTE_DIR}/sta20_orin.py"

DBG_NAMES = ["dbg", "targetL", "targetR", "dutyL", "dutyR", "rpmL", "rpmR", "i2cSteerCalls", "i2cSteerErr",
             "stallEvM1", "stallMaxM1", "stallEvM2", "stallMaxM2", "maxUs_pwmPump", "maxUs_poll", "maxUs_pid",
             "maxUs_status", "dirI2cCalls", "dirI2cErr", "pcaReady",
             "ackseq", "bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3",
             "staCalls", "staSentBM", "staSkipBM", "staSkipUSB", "maxLoopGapUs", "gap1ms", "gap2ms", "gap5ms"]

WHEEL = 60
STA_HZ = 20.0


def T(label, rate, duration, seq, wheel=WHEEL, load=0):
    return dict(label=label, rate=rate, duration=duration, seq=seq, wheel=wheel, load=load)


PLANS = {
    "smoke": [T("smoke_cmd60", 60, 10, 300, wheel=40)],
    "M": [
        T("cmd60_sta20", 60, 120, 1000),
        T("cmd30_sta20", 30, 120, 9000),
        T("cmd60_sta20_load4", 60, 120, 17000, load=4),
        T("cmd100_sta20", 100, 60, 25000),
    ],
    "long": [T("cmd60_sta20_long", 60, 600, 1000)],
}


class DbgReader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.latest = None
        self.latest_t = 0.0
        self.history = []
        self.raw = []
        self.banner = []
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
                text = line.decode("utf-8", errors="replace").strip()
                if text.startswith("==="):
                    self.banner.append(text)
                    continue
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


def ssh_base(host):
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", f"{ORIN_USER}@{host}"]


def deploy(host):
    here = Path(__file__).resolve().parent / "sta20_orin.py"
    print(f"[pc] 部署 Orin 端腳本：ssh {ORIN_USER}@{host} mkdir -p {REMOTE_DIR}; scp sta20_orin.py")
    r = subprocess.run(ssh_base(host) + [f"mkdir -p {REMOTE_DIR}"], capture_output=True, text=True, timeout=30)
    if r.returncode:
        sys.exit("[pc] mkdir 失敗：" + (r.stderr or r.stdout))
    r = subprocess.run(["scp", "-o", "BatchMode=yes", str(here), f"{ORIN_USER}@{host}:{REMOTE_DIR}/sta20_orin.py"],
                       capture_output=True, text=True, timeout=60)
    if r.returncode:
        sys.exit("[pc] scp 失敗：" + (r.stderr or r.stdout))
    print("[pc] 部署完成")


def run_remote(host, tr, timeout):
    cmd = (f"python3 {REMOTE_SCRIPT} --label {tr['label']} --rate {tr['rate']} --duration {tr['duration']} "
           f"--seq-start {tr['seq']} --wheel-max {tr['wheel']} --load {tr['load']} --sta-hz {STA_HZ}")
    if tr["wheel"] > 0:
        cmd += " --wheels-lifted"
    proc = subprocess.run(ssh_base(host) + [cmd], capture_output=True, text=True, timeout=timeout)
    last = [ln for ln in proc.stdout.strip().splitlines() if ln.startswith("{")]
    if not last:
        return None, proc.stdout[-400:] + proc.stderr[-400:]
    return json.loads(last[-1]), None


def evaluate(res, a, b):
    keys = ("bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3", "i2cSteerErr", "dirI2cErr",
            "staCalls", "staSentBM", "staSkipBM", "staSkipUSB", "gap1ms", "gap2ms", "gap5ms")
    d = {k: b[k] - a[k] for k in keys}
    exp_cmds = res["sent"] + res["stop_cmds"]
    exp_bytes = res["bytes_sent"] + res["stop_cmd_bytes"]
    checks = {
        "cmdOk==sent+5": d["bmCmdOk"] == exp_cmds,
        "bytes==sent": d["bmBytes"] == exp_bytes,
        "rxdr==bytes": d["rxdrS3"] == d["bmBytes"],
        "csFail==0": d["bmCsFail"] == 0,
        "abort==0": d["bmAbort"] == 0,
        "overflow==0": d["bmOverflow"] == 0,
        "FE==0": d["feS3"] == 0,
        "staSkipBM==0": d["staSkipBM"] == 0,
        "staSkipUSB==0": d["staSkipUSB"] == 0,
        "gap>5ms==0": d["gap5ms"] == 0,
        "i2cErr==0": d["i2cSteerErr"] == 0 and d["dirI2cErr"] == 0 and b["pcaReady"] == 1,
        "staDelivery>=0.95": res["sta_delivery_ratio"] >= 0.95,
        "snDups==0": (res.get("sn_dups") or 0) == 0,
    }
    return d, exp_cmds, exp_bytes, checks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="BMduino USB 序列埠，例如 COM10")
    ap.add_argument("--plan", required=True, choices=list(PLANS) + ["custom"])
    ap.add_argument("--trial", action="append", default=[],
                    help="--plan custom 用，可重複：label,rate,duration,wheel,load（例如 off_cmd60,60,30,0,0）")
    ap.add_argument("--i-confirm-wheels-lifted", action="store_true")
    ap.add_argument("--orin-host", default=os.environ.get("ORIN_HOST"),
                    help="Orin 位址，可用 ORIN_HOST 設定")
    ap.add_argument("--deploy", action="store_true", help="先把 sta20_orin.py 複製到 Orin（新建資料夾）")
    ap.add_argument("--gap", type=float, default=3.0)
    ap.add_argument("--sta-hz", type=float, default=20.0, help="板上韌體的 STA 頻率（STA 5 Hz 對照版用 5），決定送達率的分母")
    args = ap.parse_args()
    if not ORIN_USER or not args.orin_host:
        ap.error("請設定 ORIN_USER，並設定 ORIN_HOST 或傳入 --orin-host")
    global STA_HZ
    STA_HZ = args.sta_hz

    if args.plan == "custom":
        trials = []
        for i, s in enumerate(args.trial):
            lab, rate, dur, wh, ld = s.split(",")
            trials.append(T(lab, float(rate), float(dur), 40000 + i * 3000, wheel=int(wh), load=int(ld)))
        if not trials:
            sys.exit("--plan custom 需要至少一個 --trial")
    else:
        trials = PLANS[args.plan]
    if any(t["wheel"] > 0 for t in trials) and not args.i_confirm_wheels_lifted:
        sys.exit("此計畫會讓馬達轉動，需要 --i-confirm-wheels-lifted（輪子已離地）")
    if args.deploy:
        deploy(args.orin_host)

    out_dir = Path(__file__).resolve().parent.parent / "results"
    out_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    rd = DbgReader(args.port)
    rd.start()
    print(f"[pc] 讀取 {args.port}（開啟序列埠會讓 BMduino 重開機），等待 dbg 行 ...")
    first, _ = rd.snapshot_after(0, timeout=10.0)
    if first is None:
        rd.close()
        sys.exit("[pc] 10 秒內沒有收到 35 欄的 dbg 行：確認板上是 car2_car_control_sta20、序列埠沒被其他程式占用")
    time.sleep(2.0)
    banner = " | ".join(rd.banner)
    print(f"[pc] 開機訊息：{banner}")
    if "[TEST sta20" not in banner:
        print("[pc] ！警告：開機訊息沒有 [TEST sta20]（可能沒抓到開機那一行，或燒錯韌體）")

    summary = []
    for tr in trials:
        print(f"\n=== {tr['label']}: CMD {tr['rate']} Hz 整包 x {tr['duration']} s, 輪速上限 {tr['wheel']}%, CPU 負載 {tr['load']} ===")
        time.sleep(args.gap)
        a, ta = rd.snapshot_after(time.monotonic())
        res, err = run_remote(args.orin_host, tr, timeout=tr["duration"] + 90)
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
        print(f"  Orin  送出：{o['sent']} 包（{o['achieved_rate_pkt_s']} 包/秒，落後週期 {o['late_periods']}），單包送出 mean {o['send_ms_mean']} "
              f"p99 {o['send_ms_p99']} max {o['send_ms_max']} ms；loadavg {o['loadavg_before']}→{o['loadavg_after']}")
        print(f"  Orin  收 STA：ok {o['sta_ok']}（{o['sta_ok_rate_hz']} Hz，送達率 {o['sta_delivery_ratio']}）  壞包 {o['sta_bad']}（{o['sta_bad_pct']}%）  "
              f"序號缺口 {o['sn_missing']}/{o['sn_span']}（{o['sn_missing_pct']}%）  重複 {o['sn_dups']}")
        print(f"  Orin  STA 到達間隔 ms：mean {o['sta_gap_ms_mean']} p50 {o['sta_gap_ms_p50']} p95 {o['sta_gap_ms_p95']} p99 {o['sta_gap_ms_p99']} "
              f"min {o['sta_gap_ms_min']} max {o['sta_gap_ms_max']}；> 1.5 週期 {o['sta_gaps_over_1p5_period']} 次、> 2.5 週期 {o['sta_gaps_over_2p5_period']} 次；"
              f"每秒收到 {o['sta_per_sec_min']}~{o['sta_per_sec_max']} 個；ack 年齡 mean {o['ack_age_ms_mean']} max {o['ack_age_ms_max']} ms")
        if o.get("bad_samples"):
            print("  Orin  壞包樣本：" + " ; ".join(o["bad_samples"][:4]))
        print(f"  BM    CMD：Δ合法 {d['bmCmdOk']:.0f}（預期 {exp_cmds}）  Δbytes {d['bmBytes']:.0f}（預期 {exp_bytes}）  "
              f"ΔcsFail {d['bmCsFail']:.0f}  Δabort {d['bmAbort']:.0f}  Δoverflow {d['bmOverflow']:.0f}  ΔFE {d['feS3']:.0f}")
        print(f"  BM    STA：Δ呼叫 {d['staCalls']:.0f}  Δ排入BM {d['staSentBM']:.0f}  Δ略過(BM) {d['staSkipBM']:.0f}  Δ略過(USB) {d['staSkipUSB']:.0f}")
        print(f"  BM    不堵塞：maxLoopGap {b['maxLoopGapUs']:.0f} µs（累計）  Δ>1ms {d['gap1ms']:.0f}  Δ>2ms {d['gap2ms']:.0f}  Δ>5ms {d['gap5ms']:.0f}；"
              f"區段最大 µs（累計）poll {b['maxUs_poll']:.0f} pid {b['maxUs_pid']:.0f} status {b['maxUs_status']:.0f}；"
              f"ΔI2C錯誤 {d['i2cSteerErr'] + d['dirI2cErr']:.0f}；馬達最大 {rpm_max:.0f} rpm")
        if tr["wheel"] > 0 and rpm_max < 20:
            print("  ！警告：馬達最大轉速 < 20 rpm，馬達可能沒有真的轉（雜訊/負載條件無效）")
        print("  判定：" + ("PASS" if ok else "FAIL  → " + ", ".join(k for k, v in checks.items() if not v)))
        summary.append(dict(label=tr["label"], pass_=ok, checks=checks, deltas=d, expected=dict(cmds=exp_cmds, bytes=exp_bytes),
                            rpm_max=rpm_max, orin=res, dbg_before=a, dbg_after=b, trial=tr, banner=banner))

    (out_dir / f"sta20_{args.plan}_{stamp}.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    with open(out_dir / f"sta20_dbg_raw_{args.plan}_{stamp}.log", "w", encoding="utf-8") as f:
        for ts, line in rd.raw:
            f.write(f"{ts} {line}\n")
    print(f"\n[pc] 結果已存到 {out_dir}（bad dbg lines: {rd.bad_lines}）")
    print("[pc] 提醒：測試韌體不是行駛用，測完請燒回 car2_car_control.ino")
    rd.close()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
